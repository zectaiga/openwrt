// SPDX-License-Identifier: GPL-2.0-only
/*
 * DSA switch driver for the Realtek RTL9607C / RTL8198D switch core.
 *
 * The RTL960x GMAC driver (rtl960x_gmac, the DSA conduit) does the real
 * datapath and translates between the hardware descriptor CPU-tag and the
 * rtl_otto trailer tag protocol. This driver registers the switch with DSA so
 * per-port netdevs are created and the conduit/tagger are wired up.
 *
 * It also owns the switch-core register window (phys 0x1b000000) to:
 *  - run the one-time PHY/port bring-up (analog patch + CPU-port force-link +
 *    per-port PHY power-up), recovered from the U-Boot "apro" recipe, and
 *  - drive an MDIO bus over the switch SMI indirect access so phylib/phylink
 *    track real per-port link state (carrier follows the cable).
 *
 * The bring-up must run before the MDIO bus is scanned (PHY ID reads only work
 * after the analog patch), so it lives in setup() rather than in the conduit's
 * open().
 */

#include <linux/delay.h>
#include <linux/if_bridge.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <net/dsa.h>
#include <net/switchdev.h>

/* Switch ports: 0..3 = user (LAN/WAN), 9/10 = CPU (GMAC0/GMAC1). */
#define RTL960X_NUM_PORTS	11
#define RTL960X_CPU_PORT	9

/*
 * Switch-core registers (offsets from phys 0x1b000000), used by the PHY/port
 * bring-up and the SMI-indirect MDIO access. Mirrors the U-Boot Lan_RXENABLE /
 * Lan_{READ,WRITE}GPHY "apro" sequences (re8670poll.c).
 */
#define SW_WINDOW_SZ		0x24000		/* covers up to SW_PORTABLE 0x23024 */
#define SW_PHY_DATA		0x000		/* MDIO write data (low 16 bits) */
#define SW_PHY_CMD		0x004		/* MDIO command */
#define SW_PHY_STS		0x008		/* bit16 = busy, low16 = read data */
#define SW_PORT_DISABLE		0x04c
#define SW_PHY_PATCH		0x114		/* PHY analog patch trigger */
#define SW_CPU_PORT_FORCE	0x1f0		/* CPU-port MAC force link */
#define SW_CPU_PORT_MASK	0x25c
#define SW_PORTABLE		0x23024		/* disabled-port bitmap source */

#define SW_PHY_BMCR_UP		0x1140		/* AN enable + 1000/full, powered up */
#define SW_CPU_FORCE_VAL	0x96		/* FORCE|1000|LINK|TXPF */
#define SW_MAX_PHY_PORT		5

/*
 * Port isolation. PISO_PORT[port] is a per-port 32-bit register; its low 11 bits
 * are the egress-allowed portmask over ports 0..10 (bits 11..28 are the unused
 * extension portmask, bits 29..31 reserved). Writing a restricted mask is what
 * enables isolation -- there is no separate enable bit. The power-on default is
 * all-ones (flat forwarding). (RTL9607C SDK: RTL9607C_PISO_PORTr @ 0x27000,
 * stride 4, field PORTMASK lsp 0 len 29.)
 */
#define SW_PISO_PORT(p)		(0x27000 + (p) * 4)
#define SW_PISO_PORTMASK	0x7ff

/*
 * Per-port spanning-tree state. MSTI_CTRL is a per-port 32-bit register holding
 * four 2-bit MSTI states; for single (C)STP we only use MSTI 0 (bits [1:0]).
 * Same global LUT/STP register block as LUT_SYS_LRN_LIMITNO @ 0x17038.
 * (RTL9607C SDK: RTL9607C_MSTI_CTRLr @ 0x1704C, stride 4, field STATE lsp 0 len 2,
 * array index = msti.) State values match rtk_stp_state_t.
 */
#define SW_MSTI_CTRL(p)		(0x1704C + (p) * 4)
#define SW_MSTI0_STATE_MASK	0x3
#define SW_STP_DISABLED		0
#define SW_STP_BLOCKING		1
#define SW_STP_LEARNING		2
#define SW_STP_FORWARDING	3

/*
 * L2 FDB flush. FLUSH_CTRL selects what to flush; writing the per-port bit in
 * FLUSH_EN starts the flush; FLUSH_STATUS (bit 0 of FLUSH_CTRL) reads back busy.
 * (RTL9607C SDK: L2_TBL_FLUSH_CTRL @ 0x17044, L2_TBL_FLUSH_EN @ 0x17048 where
 * each port maps to one bit; flush mode 0 = per-port, bit3 static, bit4 dynamic.)
 */
#define SW_L2_FLUSH_CTRL	0x17044
#define SW_L2_FLUSH_EN		0x17048
#define SW_FLUSH_STATUS_BUSY	BIT(0)
#define SW_FLUSH_MODE_PORT	(0 << 1)
#define SW_FLUSH_STATIC		BIT(3)
#define SW_FLUSH_DYNAMIC	BIT(4)
#define SW_FLUSH_TIMEOUT_US	100000

/*
 * VLAN. The switch is kept VLAN-aware at all times with a transparent default
 * VLAN (all ports member + untagged, per-port PVID = default), and per-port
 * ingress filtering toggled by DSA's port_vlan_filtering -- the same model as
 * the rtl83xx DSA driver. Egress tag/untag follows the per-VID untag mask
 * (egress mode "original"). All offsets/fields from the RTL9607C SDK.
 *
 * The 4K VLAN table is reached through the shared indirect table engine
 * (TBL_ACCESS_*): write the data word(s), then a CTRL word carrying the entry
 * address, command (read/write) and table type, and poll the STS busy flag.
 * The VLAN entry is a single 32-bit word: member mask [10:0], untag mask
 * [21:11] (ports 0..10), FID/MSTI [23:22], ...
 */
#define SW_TBL_CTRL		0x12000
#define SW_TBL_STS		0x12004
#define SW_TBL_WR_DATA0		0x12008		/* word 0 (datareg_num = 1 for VLAN) */
#define SW_TBL_RD_DATA0		0x1201C
#define TBL_CTRL_ADDR(v)	(((v) & 0xfff) << 12)
#define TBL_CTRL_CMD_WRITE	(1 << 3)	/* CMD_TYPE = write */
#define TBL_CTRL_TYPE_VLAN	1		/* TBL_TYPE for the 4K VLAN table */
#define TBL_STS_BUSY		BIT(13)
#define TBL_TIMEOUT_US		10000

#define VLAN_MBR_MASK		0x7ff		/* ports 0..10 */
#define VLAN_UNTAG_SHIFT	11

#define SW_VLAN_ACCEPT		0x13000		/* 2 bits/port, 0 = accept all */
#define SW_VLAN_INGRESS		0x13004		/* 1 bit/port, ingress filtering */
#define SW_VLAN_CTRL		0x13008
#define SW_VLAN_CTRL_EN		BIT(0)		/* global VLAN_FILTERING enable */
#define SW_VLAN_PB_VID		0x1300C		/* per-port PVID, 12 bits, 2 ports/word */
#define SW_VLAN_PVID_BITS	12
#define SW_VLAN_PVID_MASK	0xfff
#define SW_VLAN_EGR_TAG(p)	(0x2A000 + (p) * 4)	/* bits [1:0] = egress mode */
#define SW_VLAN_EGR_MODE_ORI	0		/* tag/untag per the VID untag mask */
#define RTL960X_DEFAULT_VID	1
#define RTL960X_NUM_VLANS	4096

/*
 * Per-port standalone VLAN. A standalone (non-bridged) user port must exchange
 * untagged frames with the CPU: the rtl_otto tagger carries the source port in
 * the descriptor trailer, not in an 802.1Q tag, and an L3 netdev on the port
 * cannot handle a VLAN tag. But the CPU port has to be a *tagged* member of any
 * VLAN-aware bridge's VLANs (so the per-port netdev can demux them), and a
 * single CPU untag bit per VID cannot satisfy both. Give each standalone port
 * its own reserved VID with the CPU as an untagged member, used as the port's
 * PVID, so its untagged traffic reaches the CPU untagged without colliding with
 * the bridge VLANs (which keep the CPU tagged). Bridge join/leave switches the
 * port between this VID and the bridge-managed VLANs.
 */
#define RTL960X_STANDALONE_VID(p)	(RTL960X_NUM_VLANS - RTL960X_NUM_PORTS + (p))

/* MDIO command/status fields in SW_PHY_CMD / SW_PHY_STS. */
#define MDIO_CMD_READ		(1 << 21)
#define MDIO_CMD_WRITE		(3 << 21)
#define MDIO_CMD_PHYID(p)	(((p) & 0x1f) << 16)
#define MDIO_STS_BUSY		BIT(16)
#define MDIO_STS_DATA(v)	((v) & 0xffff)
#define MDIO_TIMEOUT_US		10000

/* SoC system-status register: bit1 = PHY patch ready, bit0 = patch applied. */
#define SOC_SYS_STATUS_PA	0x18000044
#define SOC_SYS_PATCH_READY	BIT(1)
#define SOC_SYS_PATCH_DONE	BIT(0)

struct rtl960x_dsa {
	struct dsa_switch *ds;
	struct device *dev;
	void __iomem *sw;		/* switch-core register window */
	struct mii_bus *mbus;
	u16 pvid[RTL960X_NUM_PORTS];	/* shadow of each port's PVID */
};

/*
 * Map a clause-22 register number to the switch GPHY OCP address, following the
 * U-Boot Lan_{READ,WRITE}MIIM_apro() mapping. genphy only touches regs 0..6,9,10
 * (all < 16), which map to 0xa400 + reg*2; the paged 16..23 window uses page 0.
 */
static int rtl960x_mdio_ocp(int reg)
{
	if (reg < 16)
		return 0xa400 + reg * 2;
	if (reg < 24)
		return (reg - 16) * 2;		/* page 0 */
	if (reg < 30)
		return 0xa400 + reg * 2;
	return -EINVAL;
}

static int rtl960x_mdio_wait(struct rtl960x_dsa *priv)
{
	u32 v;
	int i;

	for (i = 0; i < MDIO_TIMEOUT_US; i++) {
		v = __raw_readl(priv->sw + SW_PHY_STS);
		if (!(v & MDIO_STS_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int rtl960x_mdio_read(struct mii_bus *bus, int addr, int reg)
{
	struct rtl960x_dsa *priv = bus->priv;
	int ocp = rtl960x_mdio_ocp(reg);
	int ret;

	if (ocp < 0)
		return 0xffff;		/* unsupported reg: read as all-ones */

	__raw_writel(MDIO_CMD_READ | MDIO_CMD_PHYID(addr) | ocp,
		     priv->sw + SW_PHY_CMD);
	ret = rtl960x_mdio_wait(priv);
	if (ret)
		return ret;

	return MDIO_STS_DATA(__raw_readl(priv->sw + SW_PHY_STS));
}

static int rtl960x_mdio_write(struct mii_bus *bus, int addr, int reg, u16 val)
{
	struct rtl960x_dsa *priv = bus->priv;
	int ocp = rtl960x_mdio_ocp(reg);
	u32 data;

	if (ocp < 0)
		return 0;		/* unsupported reg: ignore the write */

	/* The data register's upper 16 bits are unrelated; preserve them. */
	data = __raw_readl(priv->sw + SW_PHY_DATA) & 0xffff0000;
	__raw_writel(data | val, priv->sw + SW_PHY_DATA);

	__raw_writel(MDIO_CMD_WRITE | MDIO_CMD_PHYID(addr) | ocp,
		     priv->sw + SW_PHY_CMD);

	return rtl960x_mdio_wait(priv);
}

/*
 * One-time switch-core PHY/port bring-up. Mirrors the U-Boot Lan_RXENABLE_apro()
 * sequence: wait for the analog patch to be ready, mask the disabled ports,
 * trigger the analog patch, force the CPU-port MAC link, and power up each
 * enabled port's PHY. Without this no cable links and MDIO reads return garbage.
 */
static int rtl960x_switch_bringup(struct rtl960x_dsa *priv)
{
	void __iomem *sys;
	u32 disport;
	int i;

	sys = ioremap(SOC_SYS_STATUS_PA, 4);
	if (!sys)
		return -ENOMEM;

	/* Wait for the PHY analog patch to be ready (already set post-boot). */
	for (i = 0; i < 1000; i++) {
		if (__raw_readl(sys) & SOC_SYS_PATCH_READY)
			break;
		usleep_range(1000, 2000);
	}

	disport = (__raw_readl(priv->sw + SW_PORTABLE) & 0x3e) >> 1;

	__raw_writel((__raw_readl(priv->sw + SW_PORT_DISABLE) & ~(0x1f << 10)) |
		     (disport << 10), priv->sw + SW_PORT_DISABLE);
	__raw_writel(0x1, priv->sw + SW_PHY_PATCH);
	msleep(800);

	__raw_writel(SW_CPU_FORCE_VAL, priv->sw + SW_CPU_PORT_FORCE);
	__raw_writel(0xffff, priv->sw + SW_CPU_PORT_MASK);

	for (i = 0; i < SW_MAX_PHY_PORT; i++) {
		if (disport & BIT(i))
			continue;
		__raw_writel(SW_PHY_BMCR_UP, priv->sw + SW_PHY_DATA);
		__raw_writel(MDIO_CMD_WRITE | MDIO_CMD_PHYID(i) | 0xa400,
			     priv->sw + SW_PHY_CMD);
	}

	__raw_writel(0x1, priv->sw + SW_PHY_PATCH);
	__raw_writel(__raw_readl(sys) | SOC_SYS_PATCH_DONE, sys);

	iounmap(sys);

	dev_info(priv->dev,
		 "switch core PHY/port init done (disabled ports 0x%x)\n",
		 disport);

	return 0;
}

static int rtl960x_mdio_register(struct rtl960x_dsa *priv)
{
	struct device_node *mnp;
	int ret;

	mnp = of_get_child_by_name(priv->dev->of_node, "mdio");
	if (!mnp)
		return 0;		/* no MDIO: ports stay fixed-link */

	priv->mbus = devm_mdiobus_alloc(priv->dev);
	if (!priv->mbus) {
		of_node_put(mnp);
		return -ENOMEM;
	}

	priv->mbus->name = "rtl960x-mdio";
	priv->mbus->read = rtl960x_mdio_read;
	priv->mbus->write = rtl960x_mdio_write;
	priv->mbus->priv = priv;
	priv->mbus->parent = priv->dev;
	snprintf(priv->mbus->id, MII_BUS_ID_SIZE, "%s", dev_name(priv->dev));

	ret = devm_of_mdiobus_register(priv->dev, priv->mbus, mnp);
	of_node_put(mnp);
	if (ret)
		return ret;

	priv->ds->user_mii_bus = priv->mbus;

	return 0;
}

/*
 * Program every port's isolation portmask from the current Linux bridge
 * membership (HW bridge offload):
 *   - every user port may always reach the CPU port,
 *   - user ports in the same bridge may reach each other (forwarded in HW by
 *     the switch's own L2 engine, learning enabled),
 *   - the CPU port may reach every user port.
 * Standalone user ports, and ports in a different bridge (e.g. WAN), stay
 * CPU-only and are therefore isolated in hardware. Called at setup() (no
 * bridges yet -> every user port isolated) and on every bridge join/leave.
 */
static void rtl960x_recalc_isolation(struct dsa_switch *ds)
{
	struct rtl960x_dsa *priv = ds->priv;
	int p, q;

	for (p = 0; p < ds->num_ports; p++) {
		u32 mask;

		if (dsa_is_cpu_port(ds, p)) {
			mask = 0;
			for (q = 0; q < ds->num_ports; q++)
				if (dsa_is_user_port(ds, q))
					mask |= BIT(q);
		} else if (dsa_is_user_port(ds, p)) {
			mask = BIT(RTL960X_CPU_PORT);
			for (q = 0; q < ds->num_ports; q++) {
				if (q == p || !dsa_is_user_port(ds, q))
					continue;
				if (dsa_port_bridge_same(dsa_to_port(ds, p),
							 dsa_to_port(ds, q)))
					mask |= BIT(q);
			}
		} else {
			continue;
		}

		__raw_writel(mask & SW_PISO_PORTMASK, priv->sw + SW_PISO_PORT(p));
	}
}

/* Flush dynamically-learned L2 entries on a port (STP topology change). */
static void rtl960x_dsa_port_fast_age(struct dsa_switch *ds, int port)
{
	struct rtl960x_dsa *priv = ds->priv;
	int i;

	__raw_writel(SW_FLUSH_MODE_PORT | SW_FLUSH_DYNAMIC,
		     priv->sw + SW_L2_FLUSH_CTRL);
	__raw_writel(BIT(port), priv->sw + SW_L2_FLUSH_EN);

	for (i = 0; i < SW_FLUSH_TIMEOUT_US; i += 10) {
		if (!(__raw_readl(priv->sw + SW_L2_FLUSH_CTRL) &
		      SW_FLUSH_STATUS_BUSY))
			return;
		udelay(10);
	}

	dev_warn(priv->dev, "L2 flush timed out on port %d\n", port);
}

static void rtl960x_dsa_port_stp_state_set(struct dsa_switch *ds, int port,
					   u8 state)
{
	struct rtl960x_dsa *priv = ds->priv;
	u32 hw, v;

	switch (state) {
	case BR_STATE_DISABLED:
		hw = SW_STP_DISABLED;
		break;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		hw = SW_STP_BLOCKING;
		break;
	case BR_STATE_LEARNING:
		hw = SW_STP_LEARNING;
		break;
	case BR_STATE_FORWARDING:
		hw = SW_STP_FORWARDING;
		break;
	default:
		return;
	}

	v = __raw_readl(priv->sw + SW_MSTI_CTRL(port));
	v = (v & ~SW_MSTI0_STATE_MASK) | hw;
	__raw_writel(v, priv->sw + SW_MSTI_CTRL(port));
}

/* Indirect VLAN-table access through the shared TBL_ACCESS engine. */
static int rtl960x_vlan_tbl_wait(struct rtl960x_dsa *priv)
{
	int i;

	for (i = 0; i < TBL_TIMEOUT_US; i++) {
		if (!(__raw_readl(priv->sw + SW_TBL_STS) & TBL_STS_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int rtl960x_vlan_tbl_write(struct rtl960x_dsa *priv, u16 vid, u32 word)
{
	__raw_writel(word, priv->sw + SW_TBL_WR_DATA0);
	__raw_writel(TBL_CTRL_ADDR(vid) | TBL_CTRL_CMD_WRITE | TBL_CTRL_TYPE_VLAN,
		     priv->sw + SW_TBL_CTRL);
	return rtl960x_vlan_tbl_wait(priv);
}

static int rtl960x_vlan_tbl_read(struct rtl960x_dsa *priv, u16 vid, u32 *word)
{
	int ret;

	__raw_writel(TBL_CTRL_ADDR(vid) | TBL_CTRL_TYPE_VLAN,
		     priv->sw + SW_TBL_CTRL);
	ret = rtl960x_vlan_tbl_wait(priv);
	if (ret)
		return ret;

	*word = __raw_readl(priv->sw + SW_TBL_RD_DATA0);
	return 0;
}

/* Replace a VID's member and untagged port masks. */
static int rtl960x_vlan_set_masks(struct rtl960x_dsa *priv, u16 vid,
				  u32 member, u32 untag)
{
	u32 word = (member & VLAN_MBR_MASK) |
		   ((untag & VLAN_MBR_MASK) << VLAN_UNTAG_SHIFT);

	return rtl960x_vlan_tbl_write(priv, vid, word);
}

static int rtl960x_vlan_get_masks(struct rtl960x_dsa *priv, u16 vid,
				  u32 *member, u32 *untag)
{
	u32 word;
	int ret;

	ret = rtl960x_vlan_tbl_read(priv, vid, &word);
	if (ret)
		return ret;

	*member = word & VLAN_MBR_MASK;
	*untag = (word >> VLAN_UNTAG_SHIFT) & VLAN_MBR_MASK;
	return 0;
}

static void rtl960x_set_pvid(struct rtl960x_dsa *priv, int port, u16 vid)
{
	u32 off = SW_VLAN_PB_VID + (port / 2) * 4;
	u32 shift = (port % 2) * SW_VLAN_PVID_BITS;
	u32 v;

	v = __raw_readl(priv->sw + off);
	v &= ~(SW_VLAN_PVID_MASK << shift);
	v |= (vid & SW_VLAN_PVID_MASK) << shift;
	__raw_writel(v, priv->sw + off);

	priv->pvid[port] = vid;
}

/*
 * Bring the VLAN engine up with a transparent default: every port is an
 * untagged member of the default VLAN with that VID as its PVID and ingress
 * filtering off, so until DSA programs real VLANs the switch forwards exactly
 * as in the VLAN-unaware case (forwarding is still gated by the bridge/PISO
 * isolation masks). DSA then drives per-port filtering and membership.
 */
static void rtl960x_vlan_setup(struct dsa_switch *ds)
{
	struct rtl960x_dsa *priv = ds->priv;
	u32 all = 0;
	int p, v;

	for (p = 0; p < ds->num_ports; p++)
		if (dsa_is_user_port(ds, p) || dsa_is_cpu_port(ds, p))
			all |= BIT(p);

	/*
	 * Unconfigured VLAN entries power on with every port as a member (flat
	 * forwarding). Clear them so a VID only reaches ports DSA explicitly
	 * adds; otherwise port_vlan_add's read-modify-write would keep the
	 * all-ones default and never isolate VLANs.
	 */
	for (v = 1; v < RTL960X_NUM_VLANS; v++)
		rtl960x_vlan_set_masks(priv, v, 0, 0);

	rtl960x_vlan_set_masks(priv, RTL960X_DEFAULT_VID, all, all);

	/* No ingress filtering yet; egress follows the per-VID untag mask. */
	__raw_writel(0, priv->sw + SW_VLAN_INGRESS);
	for (p = 0; p < ds->num_ports; p++) {
		if (!dsa_is_user_port(ds, p) && !dsa_is_cpu_port(ds, p))
			continue;
		__raw_writel(SW_VLAN_EGR_MODE_ORI, priv->sw + SW_VLAN_EGR_TAG(p));
		rtl960x_set_pvid(priv, p, RTL960X_DEFAULT_VID);
	}

	__raw_writel(__raw_readl(priv->sw + SW_VLAN_CTRL) | SW_VLAN_CTRL_EN,
		     priv->sw + SW_VLAN_CTRL);
}

/*
 * Put a user port into standalone mode: program its reserved per-port VID with
 * the port and the CPU port as untagged members, and make it the port's PVID.
 * Untagged ingress is then tagged internally with this VID and egresses the CPU
 * untagged, so an L3 netdev on the port sees plain (untagged) frames.
 */
static void rtl960x_port_setup_standalone(struct dsa_switch *ds, int port)
{
	struct rtl960x_dsa *priv = ds->priv;
	u16 vid = RTL960X_STANDALONE_VID(port);
	u32 mask = BIT(port) | BIT(RTL960X_CPU_PORT);

	rtl960x_vlan_set_masks(priv, vid, mask, mask);
	rtl960x_set_pvid(priv, port, vid);
}

static int rtl960x_dsa_port_vlan_filtering(struct dsa_switch *ds, int port,
					   bool vlan_filtering,
					   struct netlink_ext_ack *extack)
{
	struct rtl960x_dsa *priv = ds->priv;
	u32 v;

	/*
	 * Toggle per-port ingress VLAN filtering. The CPU port is left
	 * unfiltered so CPU-bound and CPU-injected frames are never dropped.
	 */
	if (dsa_is_cpu_port(ds, port))
		return 0;

	v = __raw_readl(priv->sw + SW_VLAN_INGRESS);
	if (vlan_filtering)
		v |= BIT(port);
	else
		v &= ~BIT(port);
	__raw_writel(v, priv->sw + SW_VLAN_INGRESS);

	return 0;
}

static int rtl960x_dsa_port_vlan_add(struct dsa_switch *ds, int port,
				     const struct switchdev_obj_port_vlan *vlan,
				     struct netlink_ext_ack *extack)
{
	struct rtl960x_dsa *priv = ds->priv;
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	u32 member, untag;
	int ret;

	if (!vlan->vid)
		return 0;

	ret = rtl960x_vlan_get_masks(priv, vlan->vid, &member, &untag);
	if (ret)
		return ret;

	member |= BIT(port);
	if (untagged)
		untag |= BIT(port);
	else
		untag &= ~BIT(port);

	/*
	 * This kernel has no separate host-VLAN callback, so keep the CPU port
	 * a (tagged) member of every VLAN that has user members, so VLAN traffic
	 * destined to the CPU (the bridge / routing) is forwarded there.
	 */
	member |= BIT(RTL960X_CPU_PORT);

	ret = rtl960x_vlan_set_masks(priv, vlan->vid, member, untag);
	if (ret)
		return ret;

	/* The CPU port keeps the default PVID (tagging is driven by DSA). */
	if (!dsa_is_cpu_port(ds, port) && pvid)
		rtl960x_set_pvid(priv, port, vlan->vid);

	return 0;
}

static int rtl960x_dsa_port_vlan_del(struct dsa_switch *ds, int port,
				     const struct switchdev_obj_port_vlan *vlan)
{
	struct rtl960x_dsa *priv = ds->priv;
	u32 member, untag;
	int ret;

	if (!vlan->vid)
		return 0;

	ret = rtl960x_vlan_get_masks(priv, vlan->vid, &member, &untag);
	if (ret)
		return ret;

	member &= ~BIT(port);
	untag &= ~BIT(port);

	/* Drop the CPU port once the VLAN has no user members left. */
	if (!(member & ~BIT(RTL960X_CPU_PORT)))
		member &= ~BIT(RTL960X_CPU_PORT);

	ret = rtl960x_vlan_set_masks(priv, vlan->vid, member, untag);
	if (ret)
		return ret;

	/* Removing the port's current PVID falls back to the default VLAN. */
	if (!dsa_is_cpu_port(ds, port) && priv->pvid[port] == vlan->vid)
		rtl960x_set_pvid(priv, port, RTL960X_DEFAULT_VID);

	return 0;
}

static enum dsa_tag_protocol rtl960x_dsa_get_tag_protocol(struct dsa_switch *ds,
							  int port,
							  enum dsa_tag_protocol mp)
{
	return DSA_TAG_PROTO_RTL_OTTO;
}

static int rtl960x_dsa_setup(struct dsa_switch *ds)
{
	struct rtl960x_dsa *priv = ds->priv;
	int ret, p;

	/*
	 * Bring up the switch-core PHYs/ports before registering the MDIO bus:
	 * PHY ID reads only succeed after the analog patch has run.
	 */
	ret = rtl960x_switch_bringup(priv);
	if (ret)
		return ret;

	/* No bridges yet: every user port starts isolated (CPU-only). */
	rtl960x_recalc_isolation(ds);
	dev_info(priv->dev, "port isolation initialised (HW bridge offload)\n");

	/* VLAN-aware with a transparent default VLAN (see rtl960x_vlan_setup). */
	rtl960x_vlan_setup(ds);

	/* Every user port starts standalone: untagged traffic to/from the CPU. */
	for (p = 0; p < ds->num_ports; p++)
		if (dsa_is_user_port(ds, p))
			rtl960x_port_setup_standalone(ds, p);

	/* Program VLANs even before a bridge turns on VLAN filtering. */
	ds->configure_vlan_while_not_filtering = true;

	return rtl960x_mdio_register(priv);
}

static int rtl960x_dsa_port_bridge_join(struct dsa_switch *ds, int port,
					struct dsa_bridge bridge,
					bool *tx_fwd_offload,
					struct netlink_ext_ack *extack)
{
	/* Open HW forwarding among the ports of this bridge. */
	rtl960x_recalc_isolation(ds);

	return 0;
}

static void rtl960x_dsa_port_bridge_leave(struct dsa_switch *ds, int port,
					  struct dsa_bridge bridge)
{
	/* The port is already unbridged here; reflect the new membership. */
	rtl960x_recalc_isolation(ds);

	/* Back to standalone: untagged exchange with the CPU on its own VID. */
	rtl960x_port_setup_standalone(ds, port);
}

static void rtl960x_dsa_phylink_get_caps(struct dsa_switch *ds, int port,
					 struct phylink_config *config)
{
	config->mac_capabilities = MAC_SYM_PAUSE | MAC_ASYM_PAUSE |
				   MAC_10 | MAC_100 | MAC_1000FD;

	__set_bit(PHY_INTERFACE_MODE_INTERNAL, config->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_GMII, config->supported_interfaces);
}

static void rtl960x_dsa_mac_config(struct phylink_config *config,
				   unsigned int mode,
				   const struct phylink_link_state *state)
{
}

static void rtl960x_dsa_mac_link_down(struct phylink_config *config,
				      unsigned int mode,
				      phy_interface_t interface)
{
}

static void rtl960x_dsa_mac_link_up(struct phylink_config *config,
				    struct phy_device *phydev,
				    unsigned int mode,
				    phy_interface_t interface,
				    int speed, int duplex,
				    bool tx_pause, bool rx_pause)
{
	/*
	 * The internal GPHYs and the switch MAC sync speed/duplex in hardware,
	 * so nothing to program here; phylink uses the PHY link state to drive
	 * the per-port netdev carrier.
	 */
}

static const struct phylink_mac_ops rtl960x_dsa_phylink_mac_ops = {
	.mac_config	= rtl960x_dsa_mac_config,
	.mac_link_down	= rtl960x_dsa_mac_link_down,
	.mac_link_up	= rtl960x_dsa_mac_link_up,
};

static const struct dsa_switch_ops rtl960x_dsa_ops = {
	.get_tag_protocol	= rtl960x_dsa_get_tag_protocol,
	.setup			= rtl960x_dsa_setup,
	.phylink_get_caps	= rtl960x_dsa_phylink_get_caps,
	.port_bridge_join	= rtl960x_dsa_port_bridge_join,
	.port_bridge_leave	= rtl960x_dsa_port_bridge_leave,
	.port_stp_state_set	= rtl960x_dsa_port_stp_state_set,
	.port_fast_age		= rtl960x_dsa_port_fast_age,
	.port_vlan_filtering	= rtl960x_dsa_port_vlan_filtering,
	.port_vlan_add		= rtl960x_dsa_port_vlan_add,
	.port_vlan_del		= rtl960x_dsa_port_vlan_del,
};

static int rtl960x_dsa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtl960x_dsa *priv;
	struct device_node *sw_np;
	struct dsa_switch *ds;
	struct resource res;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ds = devm_kzalloc(dev, sizeof(*ds), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

	priv->dev = dev;
	priv->ds = ds;

	/* Map the switch-core register window (for bring-up and MDIO). */
	sw_np = of_parse_phandle(dev->of_node, "realtek,switchcore", 0);
	if (!sw_np)
		return dev_err_probe(dev, -EINVAL,
				     "missing realtek,switchcore phandle\n");
	ret = of_address_to_resource(sw_np, 0, &res);
	of_node_put(sw_np);
	if (ret)
		return ret;

	priv->sw = devm_ioremap(dev, res.start, SW_WINDOW_SZ);
	if (!priv->sw)
		return -ENOMEM;

	ds->dev = dev;
	ds->num_ports = RTL960X_NUM_PORTS;
	ds->ops = &rtl960x_dsa_ops;
	ds->phylink_mac_ops = &rtl960x_dsa_phylink_mac_ops;
	ds->priv = priv;

	platform_set_drvdata(pdev, priv);

	return dsa_register_switch(ds);
}

static void rtl960x_dsa_remove(struct platform_device *pdev)
{
	struct rtl960x_dsa *priv = platform_get_drvdata(pdev);

	if (priv)
		dsa_unregister_switch(priv->ds);
}

static void rtl960x_dsa_shutdown(struct platform_device *pdev)
{
	struct rtl960x_dsa *priv = platform_get_drvdata(pdev);

	if (priv)
		dsa_switch_shutdown(priv->ds);

	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id rtl960x_dsa_of_match[] = {
	{ .compatible = "realtek,rtl9607c-switch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtl960x_dsa_of_match);

static struct platform_driver rtl960x_dsa_driver = {
	.probe = rtl960x_dsa_probe,
	.remove = rtl960x_dsa_remove,
	.shutdown = rtl960x_dsa_shutdown,
	.driver = {
		.name = "rtl960x-switch",
		.of_match_table = rtl960x_dsa_of_match,
	},
};
module_platform_driver(rtl960x_dsa_driver);

MODULE_DESCRIPTION("Realtek RTL9607C/RTL8198D DSA switch driver");
MODULE_AUTHOR("Taiga Ogawa");
MODULE_LICENSE("GPL");
