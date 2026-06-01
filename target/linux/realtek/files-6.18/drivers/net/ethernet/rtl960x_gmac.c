// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL960x/RTL8198D "LUNA" GMAC datapath driver.
 *
 * The register layout, descriptor format and bring-up sequence were
 * reconstructed from the Realtek re8686/rtl86900 NIC driver (Linux 5.10) and
 * the matching U-Boot poll-mode driver (re8670poll.c). See
 * local-notes/wsr-1500ax2s/re8686-register-map.md for the full analysis.
 *
 * This implements a minimal single-RX-ring + single-TX-ring NAPI datapath for
 * GMAC0 (the CPU port), validated bidirectionally on a Buffalo WSR-1500AX2S.
 *
 * Three things are required and were the main findings (see the notes doc):
 *  1. The GMAC IP clock is gated; it must be ungated via the reset controller
 *     before any register access (gmac1/2 reboot the SoC otherwise).
 *  2. Registers are native big-endian; use the __raw_* accessors (no swap) to
 *     match the vendor's volatile-pointer access.
 *  3. The physical PHY ports power up disabled. The CPU-port GMAC drives a
 *     minimal switch-core (phys 0x1b000000) PHY/port bring-up so cables link.
 *
 * The switch is otherwise left in its power-on config (all ports forward to the
 * CPU port flat, no VLAN/port separation yet).
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <net/dsa.h>

/* GMAC register offsets (from the per-GMAC base, e.g. 0x18012000) */
#define GMAC_IDR0		0x00	/* MAC address [0..3] (32-bit) */
#define GMAC_IDR4		0x04	/* MAC address [4..5] (32-bit) */
#define GMAC_CMD		0x3b	/* (8-bit) */
#define GMAC_IMR		0x3c	/* interrupt mask (16-bit) */
#define GMAC_ISR		0x3e	/* interrupt status, W1C (16-bit) */
#define GMAC_TCR		0x40	/* TX control (32-bit) */
#define GMAC_RCR		0x44	/* RX control / accept bits (32-bit) */
#define GMAC_CPUTAGCR		0x48	/* CPU tag control (32-bit) */
#define GMAC_CONFIG		0x4c	/* (32-bit) */
#define GMAC_CPUTAG1CR		0x50	/* CPU tag1 control (32-bit) */
#define GMAC_MSR		0x58	/* MAC status / force (8-bit) */
#define GMAC_IMR0		0xd0	/* per-TX-ring interrupt mask (32-bit) */
#define GMAC_ISR1		0xd8	/* per-TX-ring interrupt status (32-bit) */
#define GMAC_TXFDP1		0x1300	/* TX ring1 base (32-bit) */
#define GMAC_TXCDO1		0x1304	/* TX ring1 current desc offset (16-bit) */
#define GMAC_RXFDP		0x13f0	/* RX ring1 base (32-bit) */
#define GMAC_RXCDO		0x13f4	/* RX ring1 current desc offset (16-bit) */
#define GMAC_RXRINGSIZE		0x13f6	/* RX ring1 size - 1 (8-bit) */
#define GMAC_RXCPU_DES_NUM	0x1430	/* (8-bit) */
#define GMAC_RX_PSE_DES_THRES	0x1432	/* RX flow-control threshold (8-bit) */
#define GMAC_IO_CMD		0x1434	/* command / enable (32-bit) */
#define GMAC_IO_CMD1		0x1438	/* command1 / desc format (32-bit) */

/* CMD register bits */
#define CMD_RXCHKSUM		BIT(1)
#define CMD_RXJUMBO		BIT(3)

/* Shared descriptor opts1 bits */
#define DESC_OWN		BIT(31)	/* owned by NIC */
#define DESC_EOR		BIT(30)	/* end of ring */
#define DESC_FS			BIT(29)	/* first segment */
#define DESC_LS			BIT(28)	/* last segment */
#define DESC_LEN_MASK		0xfff
/* TX-only opts1 bits */
#define TX_DESC_CRC		BIT(23)	/* append FCS */

/*
 * CPU-tag fields carried in the descriptor sideband (not inline in the frame).
 * RX opts3: source switch port in bits 19:16. RX opts2: trap reason in 28:21.
 * TX opts2: cputag enable (bit31) + destination port bitmap in bits 26:16.
 * These are the basis of the future DSA tagger (see dsa-roadmap.md).
 */
#define RX_OPTS3_SRC_PORT(o3)	(((o3) >> 16) & 0xf)
#define RX_OPTS2_REASON(o2)	(((o2) >> 21) & 0xff)
#define RX_REASON_FLOOD		0xca	/* CPU_REASON_FLOOD: switch HW flooded it */
#define TX_OPTS2_CPUTAG		BIT(31)
#define TX_OPTS2_PORTMASK(pm)	(((pm) & 0x7ff) << 16)
#define TX_OPTS3_KEEP		BIT(23)
#define TX_OPTS3_DISLRN		BIT(21)	/* don't learn the CPU-port SA */
#define TX_OPTS3_L34_KEEP	BIT(17)	/* direct portmask packets must not be rewritten */
#define RTL960X_CPU_PORT	9

/*
 * DSA glue. The switch tags the ingress port in the descriptor, not inline, so
 * the conduit translates between the descriptor and the rtl_otto trailer tag
 * (DSA_TAG_PROTO_RTL_OTTO) the same way the rtl83xx driver does: on RX the FCS
 * is overwritten with a 4-byte trailer [0x80, port, 0x10, 0x00]; on TX a trailer
 * with that shape selects the egress port.
 *
 * trailer[1] bit6 (0x40) maps to skb->offload_fwd_mark in the rtl_otto tagger:
 * set it when the switch already L2-forwarded the frame to a non-CPU port, so
 * the Linux bridge does not software-forward (and thus duplicate) it.
 */
#define DSA_TRAILER_LEN		4
#define DSA_TAG_OFFLOAD_FWD	0x40

/* ISR / IMR bits (16-bit legacy block) */
#define ISR_RX_OK		BIT(0)
#define ISR_RER_RUNT		BIT(2)
#define ISR_RER_OVF		BIT(4)
#define ISR_RDU			BIT(5)	/* RX descriptor unavailable */
#define ISR_TOK			BIT(6)
#define ISR_SW_INT		BIT(10)
#define ISR_RX_ALL		(ISR_SW_INT | ISR_RX_OK | ISR_RER_RUNT | \
				 ISR_RER_OVF | ISR_RDU)

/* RCR accept bits */
#define RCR_ACCEPT_BROADCAST	BIT(3)
#define RCR_ACCEPT_MULTICAST	BIT(2)
#define RCR_ACCEPT_MYPHYS	BIT(1)
#define RCR_ACCEPT_ALLPHYS	BIT(0)
/*
 * This GMAC is a DSA conduit (CPU port): the switch already decides which frames
 * to deliver here (forward/flood/trap), and they carry many destination MACs --
 * the per-port netdev addresses plus anything the switch floods to the CPU on a
 * lookup miss. Accept all unicast (ALLPHYS); MYPHYS alone would silently drop
 * frames the switch delivers that are not addressed to the conduit itself.
 */
#define RCR_DEFAULT		(RCR_ACCEPT_BROADCAST | RCR_ACCEPT_MULTICAST | \
				 RCR_ACCEPT_MYPHYS | RCR_ACCEPT_ALLPHYS)

/* MSR force/flow-control bits */
#define MSR_FORCE_TX		BIT(7)
#define MSR_RXFCE		BIT(6)
#define MSR_TXFCE		BIT(5)

/*
 * Single-ring configuration values, computed from the U-Boot poll driver
 * constants and the Linux re8686 driver (see re8686-register-map.md section 9):
 *   IO_CMD  = RE<<5|TE<<4|RX_MIT(3)<<8|RX_FIFO(2)<<11|RX_TIMER(1)<<13|
 *             TX_MIT(7)<<16|TX_FIFO(1)<<19|1<<30
 *   IO_CMD1 = DESC_FORMAT(3)<<28 | RXRING1(1)<<16
 *   CPUTAGCR= CTEN_RX|2<<16|2<<27|8<<18|0xff<<8|0x04
 *   CPUTAG1CR= CT1_SID (64 << 8)
 *   TCR     = IFG(3)<<10 | NORMAL(0)
 */
#define IO_CMD_CONFIG		0x400f3330
#define IO_CMD1_CONFIG		0x30010000
#define IO_CMD_TX_POLL		BIT(0)
#define CPUTAGCR_CONFIG		0x9022ff04
#define CPUTAG1CR_CONFIG	0x00004000
#define TCR_CONFIG		(0x3 << 10)
#define RX_PSE_DES_THRES_VAL	8

#define RX_RING_SIZE		128	/* must be a power of two, 16..256 */
#define TX_RING_SIZE		128
#define RX_BUF_SIZE		1536	/* 0x600, per U-Boot RX_DESC_BUFFER_SIZE */
#define RX_SHIFT		2	/* HW writes the frame at buf + 2 bytes */
#define TX_MIN_LEN		60

/*
 * Switch-core (phys 0x1b000000) registers used by the minimal PHY/port
 * bring-up, from the U-Boot "apro" recipe (Lan_RXENABLE_apro). The physical
 * ports come up powered-down, so without this no cable gets link.
 */
#define SW_WINDOW_SZ		0x24000		/* covers up to SW_PORTABLE 0x23024 */
#define SW_PHY_DATA		0x000		/* indirect PHY write data (BMCR) */
#define SW_PHY_CMD		0x004		/* indirect PHY access command */
#define SW_PORT_DISABLE		0x04c
#define SW_PHY_PATCH		0x114		/* PHY analog patch trigger */
#define SW_CPU_PORT_FORCE	0x1f0		/* CPU port MAC force link */
#define SW_CPU_PORT_MASK	0x25c
#define SW_PORTABLE		0x23024		/* disabled-port bitmap source */
#define SW_PHY_BMCR_UP		0x1140		/* AN enable + 1000/full, powered up */
#define SW_PHY_WRITE(port)	(0x60a400 | ((port) << 16))
#define SW_CPU_FORCE_VAL	0x96		/* FORCE|1000|LINK|TXPF */
#define SW_MAX_PHY_PORT		5

/* SoC system-status register: bit1 = PHY patch ready, bit0 = patch applied */
#define SOC_SYS_STATUS_PA	0x18000044
#define SOC_SYS_PATCH_READY	BIT(1)
#define SOC_SYS_PATCH_DONE	BIT(0)

#define RTL960X_GMAC_REGS_DUMP_LEN	0x100

struct rtl960x_desc {
	u32 opts1;	/* own/eor/fs/ls + length */
	u32 addr;	/* DMA buffer address */
	u32 opts2;
	u32 opts3;
	u32 opts4;	/* TX descriptors are 5 words; RX uses the first 4 */
} __packed;

#define RX_DESC_WORDS	4
#define TX_DESC_WORDS	5
#define RX_DESC_SZ	(RX_DESC_WORDS * sizeof(u32))	/* 16 bytes */
#define TX_DESC_SZ	(TX_DESC_WORDS * sizeof(u32))	/* 20 bytes */

struct rtl960x_gmac {
	struct net_device *ndev;
	struct device *dev;
	void __iomem *base;
	int irq;
	struct napi_struct napi;
	struct reset_control *rst;

	/* RX ring */
	void *rx_ring;			/* RX_RING_SIZE * RX_DESC_SZ */
	dma_addr_t rx_ring_dma;
	struct sk_buff *rx_skb[RX_RING_SIZE];
	dma_addr_t rx_buf_dma[RX_RING_SIZE];
	u32 rx_head;

	/* TX ring */
	void *tx_ring;			/* TX_RING_SIZE * TX_DESC_SZ */
	dma_addr_t tx_ring_dma;
	struct sk_buff *tx_skb[TX_RING_SIZE];
	u32 tx_head;			/* next slot to fill */
	u32 tx_tail;			/* next slot to reclaim */
	spinlock_t tx_lock;		/* protects tx_head/tx_tail and the ring */
};

/*
 * The GMAC registers are native (big-endian on this SoC); the vendor code
 * accesses them with plain volatile pointers, so use the raw, non-swapping
 * accessors to match.
 */
static inline u32 gmac_r32(struct rtl960x_gmac *g, u32 reg)
{
	return __raw_readl(g->base + reg);
}

static inline void gmac_w32(struct rtl960x_gmac *g, u32 reg, u32 val)
{
	__raw_writel(val, g->base + reg);
}

static inline u16 gmac_r16(struct rtl960x_gmac *g, u32 reg)
{
	return __raw_readw(g->base + reg);
}

static inline void gmac_w16(struct rtl960x_gmac *g, u32 reg, u16 val)
{
	__raw_writew(val, g->base + reg);
}

static inline void gmac_w8(struct rtl960x_gmac *g, u32 reg, u8 val)
{
	__raw_writeb(val, g->base + reg);
}

static inline struct rtl960x_desc *rx_desc(struct rtl960x_gmac *g, u32 i)
{
	return (struct rtl960x_desc *)(g->rx_ring + i * RX_DESC_SZ);
}

static inline struct rtl960x_desc *tx_desc(struct rtl960x_gmac *g, u32 i)
{
	return (struct rtl960x_desc *)(g->tx_ring + i * TX_DESC_SZ);
}

static void rtl960x_gmac_stop_hw(struct rtl960x_gmac *g)
{
	gmac_w32(g, GMAC_IO_CMD, 0);
	gmac_w32(g, GMAC_IO_CMD1, 0);
	gmac_w16(g, GMAC_IMR, 0);
	gmac_w32(g, GMAC_IMR0, 0);
	gmac_w16(g, GMAC_ISR, 0xffff);		/* W1C */
	gmac_w32(g, GMAC_ISR1, 0xffffffff);
	usleep_range(10, 20);
}

static void rtl960x_gmac_set_hwaddr(struct rtl960x_gmac *g)
{
	const u8 *a = g->ndev->dev_addr;

	gmac_w32(g, GMAC_IDR0,
		 (a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3]);
	gmac_w32(g, GMAC_IDR4, (a[4] << 24) | (a[5] << 16));
}

static void rtl960x_gmac_free_rx(struct rtl960x_gmac *g)
{
	int i;

	for (i = 0; i < RX_RING_SIZE; i++) {
		if (!g->rx_skb[i])
			continue;
		dma_unmap_single(g->dev, g->rx_buf_dma[i], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);
		dev_kfree_skb(g->rx_skb[i]);
		g->rx_skb[i] = NULL;
	}
}

static int rtl960x_gmac_alloc_rings(struct rtl960x_gmac *g)
{
	int i;

	g->rx_ring = dma_alloc_coherent(g->dev, RX_RING_SIZE * RX_DESC_SZ,
					&g->rx_ring_dma, GFP_KERNEL);
	if (!g->rx_ring)
		return -ENOMEM;

	g->tx_ring = dma_alloc_coherent(g->dev, TX_RING_SIZE * TX_DESC_SZ,
					&g->tx_ring_dma, GFP_KERNEL);
	if (!g->tx_ring)
		goto err_free_rx_ring;

	/* Populate RX ring with buffers; HW owns each filled descriptor. */
	for (i = 0; i < RX_RING_SIZE; i++) {
		struct rtl960x_desc *d = rx_desc(g, i);
		struct sk_buff *skb;
		dma_addr_t dma;

		skb = netdev_alloc_skb(g->ndev, RX_BUF_SIZE);
		if (!skb)
			goto err_free_rx_bufs;

		dma = dma_map_single(g->dev, skb->data, RX_BUF_SIZE,
				     DMA_FROM_DEVICE);
		if (dma_mapping_error(g->dev, dma)) {
			dev_kfree_skb(skb);
			goto err_free_rx_bufs;
		}

		g->rx_skb[i] = skb;
		g->rx_buf_dma[i] = dma;
		d->addr = dma;
		d->opts2 = 0;
		d->opts3 = 0;
		d->opts1 = DESC_OWN | RX_BUF_SIZE |
			   (i == RX_RING_SIZE - 1 ? DESC_EOR : 0);
	}

	/* TX ring starts idle (CPU-owned); only EOR is preset on the last. */
	for (i = 0; i < TX_RING_SIZE; i++) {
		struct rtl960x_desc *d = tx_desc(g, i);

		d->addr = 0;
		d->opts2 = 0;
		d->opts3 = 0;
		d->opts4 = 0;
		d->opts1 = (i == TX_RING_SIZE - 1) ? DESC_EOR : 0;
	}

	g->rx_head = 0;
	g->tx_head = 0;
	g->tx_tail = 0;
	dma_wmb();

	return 0;

err_free_rx_bufs:
	rtl960x_gmac_free_rx(g);
	dma_free_coherent(g->dev, TX_RING_SIZE * TX_DESC_SZ, g->tx_ring,
			  g->tx_ring_dma);
	g->tx_ring = NULL;
err_free_rx_ring:
	dma_free_coherent(g->dev, RX_RING_SIZE * RX_DESC_SZ, g->rx_ring,
			  g->rx_ring_dma);
	g->rx_ring = NULL;
	return -ENOMEM;
}

static void rtl960x_gmac_free_rings(struct rtl960x_gmac *g)
{
	rtl960x_gmac_free_rx(g);

	if (g->tx_ring) {
		dma_free_coherent(g->dev, TX_RING_SIZE * TX_DESC_SZ,
				  g->tx_ring, g->tx_ring_dma);
		g->tx_ring = NULL;
	}
	if (g->rx_ring) {
		dma_free_coherent(g->dev, RX_RING_SIZE * RX_DESC_SZ,
				  g->rx_ring, g->rx_ring_dma);
		g->rx_ring = NULL;
	}
}

static void rtl960x_gmac_init_hw(struct rtl960x_gmac *g)
{
	rtl960x_gmac_stop_hw(g);

	gmac_w8(g, GMAC_CMD, CMD_RXCHKSUM | CMD_RXJUMBO);
	gmac_w32(g, GMAC_TCR, TCR_CONFIG);

	/* CPU tag: parsed into the descriptor opts fields, not inline. */
	gmac_w32(g, GMAC_CPUTAGCR, CPUTAGCR_CONFIG);
	gmac_w32(g, GMAC_CPUTAG1CR, CPUTAG1CR_CONFIG);

	/* Program the RX/TX ring bases. */
	gmac_w32(g, GMAC_RXFDP, g->rx_ring_dma);
	gmac_w16(g, GMAC_RXCDO, 0);
	gmac_w8(g, GMAC_RXRINGSIZE, RX_RING_SIZE - 1);
	gmac_w8(g, GMAC_RXCPU_DES_NUM, RX_RING_SIZE - 1);
	gmac_w8(g, GMAC_RX_PSE_DES_THRES, RX_PSE_DES_THRES_VAL);

	gmac_w32(g, GMAC_TXFDP1, g->tx_ring_dma);
	gmac_w16(g, GMAC_TXCDO1, 0);

	rtl960x_gmac_set_hwaddr(g);
	gmac_w32(g, GMAC_RCR, RCR_DEFAULT);

	/* Enable the datapath. */
	gmac_w32(g, GMAC_IO_CMD1, IO_CMD1_CONFIG);
	gmac_w32(g, GMAC_IO_CMD, IO_CMD_CONFIG);

	/* Clear and unmask RX interrupts. */
	gmac_w16(g, GMAC_ISR, 0xffff);
	gmac_w32(g, GMAC_ISR1, 0xffffffff);
	gmac_w16(g, GMAC_IMR, ISR_RX_ALL);
}

static void rtl960x_gmac_mask_rx(struct rtl960x_gmac *g)
{
	gmac_w16(g, GMAC_IMR, gmac_r16(g, GMAC_IMR) & ~ISR_RX_ALL);
}

static void rtl960x_gmac_unmask_rx(struct rtl960x_gmac *g)
{
	gmac_w16(g, GMAC_IMR, gmac_r16(g, GMAC_IMR) | ISR_RX_ALL);
}

static int rtl960x_gmac_rx(struct rtl960x_gmac *g, int budget)
{
	struct net_device *ndev = g->ndev;
	bool dsa = netdev_uses_dsa(ndev);
	int done = 0;

	while (done < budget) {
		struct rtl960x_desc *d = rx_desc(g, g->rx_head);
		bool last = (g->rx_head == RX_RING_SIZE - 1);
		struct sk_buff *skb, *new_skb;
		dma_addr_t new_dma;
		u8 src_port;
		u32 opts1, opts2, opts3;
		int len;

		opts1 = d->opts1;
		if (opts1 & DESC_OWN)		/* still owned by HW */
			break;
		dma_rmb();

		opts2 = d->opts2;
		opts3 = d->opts3;
		src_port = RX_OPTS3_SRC_PORT(opts3);

		/*
		 * Without DSA, strip the FCS. With DSA, keep the 4 FCS bytes and
		 * overwrite them with the rtl_otto trailer carrying src_port.
		 */
		len = opts1 & DESC_LEN_MASK;
		if (!dsa)
			len -= ETH_FCS_LEN;

		/* Refill with a fresh buffer before handing the old one up. */
		new_skb = netdev_alloc_skb(ndev, RX_BUF_SIZE);
		if (!new_skb) {
			ndev->stats.rx_dropped++;
			goto rearm_same;
		}
		new_dma = dma_map_single(g->dev, new_skb->data, RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
		if (dma_mapping_error(g->dev, new_dma)) {
			dev_kfree_skb(new_skb);
			ndev->stats.rx_dropped++;
			goto rearm_same;
		}

		skb = g->rx_skb[g->rx_head];
		dma_unmap_single(g->dev, g->rx_buf_dma[g->rx_head], RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		if (len < ETH_ZLEN || len > RX_BUF_SIZE) {
			/* Bad length: drop, keep the fresh buffer. */
			ndev->stats.rx_errors++;
			dev_kfree_skb(skb);
		} else {
			skb_reserve(skb, RX_SHIFT);
			skb_put(skb, len);
			if (dsa) {
				/* overwrite the FCS with the rtl_otto trailer */
				u8 *t = skb->data + len - DSA_TRAILER_LEN;
				u8 tag = src_port;

				/*
				 * If the switch HW-flooded this frame to other
				 * ports (CPU_REASON_FLOOD), mark it offloaded so
				 * the Linux bridge does not forward it again
				 * (avoids duplicate broadcast/multicast). Trapped
				 * frames (BCAST_TRAP, IGMP, ...) keep a different
				 * reason and are left for the bridge/stack.
				 */
				bool fwd = RX_OPTS2_REASON(opts2) ==
					   RX_REASON_FLOOD;

				if (fwd)
					tag |= DSA_TAG_OFFLOAD_FWD;

				t[0] = 0x80;
				t[1] = tag;
				t[2] = 0x10;
				t[3] = 0x00;
			}
			skb->protocol = eth_type_trans(skb, ndev);
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
			napi_gro_receive(&g->napi, skb);
		}

		g->rx_skb[g->rx_head] = new_skb;
		g->rx_buf_dma[g->rx_head] = new_dma;
		d->addr = new_dma;

rearm_same:
		d->opts2 = 0;
		d->opts3 = 0;
		dma_wmb();
		d->opts1 = DESC_OWN | RX_BUF_SIZE | (last ? DESC_EOR : 0);

		g->rx_head = (g->rx_head + 1) % RX_RING_SIZE;
		done++;
	}

	return done;
}

static int rtl960x_gmac_poll(struct napi_struct *napi, int budget)
{
	struct rtl960x_gmac *g = container_of(napi, struct rtl960x_gmac, napi);
	int done;

	done = rtl960x_gmac_rx(g, budget);

	if (done < budget && napi_complete_done(napi, done))
		rtl960x_gmac_unmask_rx(g);

	return done;
}

static irqreturn_t rtl960x_gmac_isr(int irq, void *dev_id)
{
	struct rtl960x_gmac *g = dev_id;
	u16 status = gmac_r16(g, GMAC_ISR) & ISR_RX_ALL;

	if (!status)
		return IRQ_NONE;

	gmac_w16(g, GMAC_ISR, status);		/* W1C ack */

	rtl960x_gmac_mask_rx(g);
	napi_schedule(&g->napi);

	return IRQ_HANDLED;
}

/* Reclaim TX descriptors the HW has finished with. Caller holds tx_lock. */
static void rtl960x_gmac_tx_reclaim(struct rtl960x_gmac *g)
{
	while (g->tx_tail != g->tx_head) {
		struct rtl960x_desc *d = tx_desc(g, g->tx_tail);
		struct sk_buff *skb;

		if (d->opts1 & DESC_OWN)
			break;

		skb = g->tx_skb[g->tx_tail];
		if (skb) {
			dma_unmap_single(g->dev, d->addr, skb->len,
					 DMA_TO_DEVICE);
			g->ndev->stats.tx_packets++;
			g->ndev->stats.tx_bytes += skb->len;
			dev_consume_skb_any(skb);
			g->tx_skb[g->tx_tail] = NULL;
		}
		g->tx_tail = (g->tx_tail + 1) % TX_RING_SIZE;
	}
}

static netdev_tx_t rtl960x_gmac_start_xmit(struct sk_buff *skb,
					   struct net_device *ndev)
{
	struct rtl960x_gmac *g = netdev_priv(ndev);
	struct rtl960x_desc *d;
	dma_addr_t dma;
	int dest_port = -1;
	bool last;
	u32 opts1;
	u32 next;
	int len;

	/*
	 * DSA egress: a frame from a user port carries the rtl_otto trailer
	 * [0x80, port, 0x10, 0x00]. Pull it off and steer the frame to that
	 * switch port via the TX descriptor instead of normal L2 lookup.
	 */
	if (netdev_uses_dsa(ndev) && skb->len >= DSA_TRAILER_LEN) {
		const u8 *t = skb->data + skb->len - DSA_TRAILER_LEN;

		if (t[0] == 0x80 && t[1] < RTL960X_CPU_PORT &&
		    t[2] == 0x10 && t[3] == 0x00) {
			dest_port = t[1];
			skb_trim(skb, skb->len - DSA_TRAILER_LEN);
		}
	}

	if (skb_put_padto(skb, TX_MIN_LEN))
		return NETDEV_TX_OK;		/* skb freed by skb_put_padto */
	len = skb->len;

	spin_lock(&g->tx_lock);

	rtl960x_gmac_tx_reclaim(g);

	next = (g->tx_head + 1) % TX_RING_SIZE;
	if (next == g->tx_tail) {
		netif_stop_queue(ndev);
		spin_unlock(&g->tx_lock);
		return NETDEV_TX_BUSY;
	}

	dma = dma_map_single(g->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(g->dev, dma)) {
		spin_unlock(&g->tx_lock);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	last = (g->tx_head == TX_RING_SIZE - 1);
	d = tx_desc(g, g->tx_head);
	g->tx_skb[g->tx_head] = skb;
	d->addr = dma;
	if (dest_port >= 0) {
		d->opts2 = TX_OPTS2_CPUTAG | TX_OPTS2_PORTMASK(BIT(dest_port));
		d->opts3 = TX_OPTS3_KEEP | TX_OPTS3_DISLRN | TX_OPTS3_L34_KEEP;
	} else {
		d->opts2 = 0;
		d->opts3 = 0;
	}
	d->opts4 = 0;
	opts1 = DESC_OWN | DESC_FS | DESC_LS | TX_DESC_CRC | len |
		(last ? DESC_EOR : 0);
	dma_wmb();
	d->opts1 = opts1;
	dma_wmb();

	g->tx_head = next;

	/* Kick the TX ring. */
	gmac_w32(g, GMAC_IO_CMD, gmac_r32(g, GMAC_IO_CMD) | IO_CMD_TX_POLL);

	spin_unlock(&g->tx_lock);

	return NETDEV_TX_OK;
}

static void rtl960x_gmac_free_tx(struct rtl960x_gmac *g)
{
	int i;

	for (i = 0; i < TX_RING_SIZE; i++) {
		struct rtl960x_desc *d = tx_desc(g, i);

		if (!g->tx_skb[i])
			continue;
		dma_unmap_single(g->dev, d->addr, g->tx_skb[i]->len,
				 DMA_TO_DEVICE);
		dev_kfree_skb(g->tx_skb[i]);
		g->tx_skb[i] = NULL;
	}
}

/*
 * Bring up the switch-core PHYs and force the CPU-port link. Only the GMAC
 * that carries the "realtek,switchcore" phandle (gmac0) runs this, and it runs
 * once globally. Mirrors the U-Boot Lan_RXENABLE_apro() sequence.
 */
static int rtl960x_switch_init(struct rtl960x_gmac *g)
{
	static bool done;
	struct device_node *sw_np;
	void __iomem *sw, *sys;
	struct resource res;
	u32 disport;
	int i, ret;

	if (done)
		return 0;

	sw_np = of_parse_phandle(g->dev->of_node, "realtek,switchcore", 0);
	if (!sw_np)
		return 0;	/* not the CPU-port GMAC */

	ret = of_address_to_resource(sw_np, 0, &res);
	of_node_put(sw_np);
	if (ret)
		return ret;

	sw = ioremap(res.start, SW_WINDOW_SZ);
	if (!sw)
		return -ENOMEM;
	sys = ioremap(SOC_SYS_STATUS_PA, 4);
	if (!sys) {
		iounmap(sw);
		return -ENOMEM;
	}

	/* Wait for the PHY analog patch to be ready (already set post-boot). */
	for (i = 0; i < 1000; i++) {
		if (__raw_readl(sys) & SOC_SYS_PATCH_READY)
			break;
		usleep_range(1000, 2000);
	}

	disport = (__raw_readl(sw + SW_PORTABLE) & 0x3e) >> 1;

	__raw_writel((__raw_readl(sw + SW_PORT_DISABLE) & ~(0x1f << 10)) |
		     (disport << 10), sw + SW_PORT_DISABLE);
	__raw_writel(0x1, sw + SW_PHY_PATCH);
	msleep(800);

	__raw_writel(SW_CPU_FORCE_VAL, sw + SW_CPU_PORT_FORCE);
	__raw_writel(0xffff, sw + SW_CPU_PORT_MASK);

	for (i = 0; i < SW_MAX_PHY_PORT; i++) {
		if (disport & BIT(i))
			continue;
		__raw_writel(SW_PHY_BMCR_UP, sw + SW_PHY_DATA);
		__raw_writel(SW_PHY_WRITE(i), sw + SW_PHY_CMD);
	}

	__raw_writel(0x1, sw + SW_PHY_PATCH);
	__raw_writel(__raw_readl(sys) | SOC_SYS_PATCH_DONE, sys);

	iounmap(sys);
	iounmap(sw);
	done = true;

	dev_info(g->dev, "switch core PHY/port init done (disabled ports 0x%x)\n",
		 disport);

	return 0;
}

static int rtl960x_gmac_open(struct net_device *ndev)
{
	struct rtl960x_gmac *g = netdev_priv(ndev);
	int ret;

	ret = rtl960x_switch_init(g);
	if (ret)
		netdev_warn(ndev, "switch core init failed: %d\n", ret);

	ret = rtl960x_gmac_alloc_rings(g);
	if (ret)
		return ret;

	ret = request_irq(g->irq, rtl960x_gmac_isr, 0, ndev->name, g);
	if (ret) {
		netdev_err(ndev, "failed to request irq %d: %d\n", g->irq, ret);
		goto err_free_rings;
	}

	napi_enable(&g->napi);
	rtl960x_gmac_init_hw(g);

	netif_start_queue(ndev);
	/*
	 * Link state is managed by the switch core, which we don't drive yet;
	 * force the carrier on so the stack will pass traffic for bring-up.
	 */
	netif_carrier_on(ndev);

	netdev_info(ndev, "datapath up (RX ring %d, TX ring %d)\n",
		    RX_RING_SIZE, TX_RING_SIZE);

	return 0;

err_free_rings:
	rtl960x_gmac_free_rings(g);
	return ret;
}

static int rtl960x_gmac_stop(struct net_device *ndev)
{
	struct rtl960x_gmac *g = netdev_priv(ndev);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	rtl960x_gmac_stop_hw(g);

	napi_disable(&g->napi);
	free_irq(g->irq, g);

	rtl960x_gmac_free_tx(g);
	rtl960x_gmac_free_rings(g);

	return 0;
}

static int rtl960x_gmac_set_mac_address(struct net_device *ndev, void *p)
{
	struct rtl960x_gmac *g = netdev_priv(ndev);
	int ret;

	ret = eth_mac_addr(ndev, p);
	if (ret)
		return ret;

	if (netif_running(ndev))
		rtl960x_gmac_set_hwaddr(g);

	return 0;
}

static const struct net_device_ops rtl960x_gmac_netdev_ops = {
	.ndo_open		= rtl960x_gmac_open,
	.ndo_stop		= rtl960x_gmac_stop,
	.ndo_start_xmit		= rtl960x_gmac_start_xmit,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= rtl960x_gmac_set_mac_address,
};

static int rtl960x_gmac_get_regs_len(struct net_device *ndev)
{
	return RTL960X_GMAC_REGS_DUMP_LEN;
}

static void rtl960x_gmac_get_regs(struct net_device *ndev,
				  struct ethtool_regs *regs, void *p)
{
	struct rtl960x_gmac *g = netdev_priv(ndev);
	u32 *buf = p;
	int i;

	regs->version = 1;
	memset(p, 0, RTL960X_GMAC_REGS_DUMP_LEN);

	for (i = 0; i < RTL960X_GMAC_REGS_DUMP_LEN / sizeof(u32); i++)
		buf[i] = gmac_r32(g, i * sizeof(u32));
}

static const struct ethtool_ops rtl960x_gmac_ethtool_ops = {
	.get_regs_len	= rtl960x_gmac_get_regs_len,
	.get_regs	= rtl960x_gmac_get_regs,
};

static int rtl960x_gmac_probe(struct platform_device *pdev)
{
	struct rtl960x_gmac *g;
	struct net_device *ndev;
	struct resource *res;
	int irq;
	int ret;

	ndev = devm_alloc_etherdev(&pdev->dev, sizeof(*g));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->netdev_ops = &rtl960x_gmac_netdev_ops;
	ndev->ethtool_ops = &rtl960x_gmac_ethtool_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = RX_BUF_SIZE - ETH_HLEN - ETH_FCS_LEN;

	eth_hw_addr_random(ndev);
	netif_carrier_off(ndev);

	g = netdev_priv(ndev);
	g->ndev = ndev;
	g->dev = &pdev->dev;
	spin_lock_init(&g->tx_lock);

	g->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(g->base))
		return PTR_ERR(g->base);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/*
	 * Ungate the GMAC IP clock before any register access. GMAC0 is left
	 * enabled by the bootloader, but GMAC1/GMAC2 are gated and reading
	 * their windows while gated hangs the LX bus and reboots the SoC.
	 */
	g->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(g->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(g->rst),
				     "failed to get reset control\n");
	ret = reset_control_deassert(g->rst);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to ungate GMAC IP clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_assert_reset;
	}
	g->irq = irq;

	netif_napi_add(ndev, &g->napi, rtl960x_gmac_poll);

	platform_set_drvdata(pdev, ndev);

	ret = register_netdev(ndev);
	if (ret)
		goto err_del_napi;

	netdev_info(ndev, "RTL960x/LUNA GMAC %pR irq %d\n", res, g->irq);

	return 0;

err_del_napi:
	netif_napi_del(&g->napi);
err_assert_reset:
	reset_control_assert(g->rst);
	return ret;
}

static void rtl960x_gmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct rtl960x_gmac *g = netdev_priv(ndev);

	unregister_netdev(ndev);
	netif_napi_del(&g->napi);
	reset_control_assert(g->rst);
}

static const struct of_device_id rtl960x_gmac_of_match[] = {
	{ .compatible = "realtek,rtl8198d-gmac" },
	{ .compatible = "realtek,rtl9607-gmac" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtl960x_gmac_of_match);

static struct platform_driver rtl960x_gmac_driver = {
	.probe = rtl960x_gmac_probe,
	.remove = rtl960x_gmac_remove,
	.driver = {
		.name = "rtl960x-gmac",
		.of_match_table = rtl960x_gmac_of_match,
	},
};
module_platform_driver(rtl960x_gmac_driver);

MODULE_DESCRIPTION("Realtek RTL960x/LUNA GMAC driver");
MODULE_AUTHOR("Taiga Ogawa");
MODULE_LICENSE("GPL");
