/*
 * smsc91x.c
 *
 *  Created on: Nov 10, 2022
 *      Author: rami
 */


#include <assert.h>
#include <drivers/smsc/smsc91x.h>
#include <config.h>
#include <initcall.h>
#include <kernel/dt.h>
#include <kernel/dt_driver.h>
#include <libfdt.h>
#include <malloc.h>
#include <sys/queue.h>
#include <tee_api_defines_extensions.h>
#include <tee_api_types.h>
#include <drivers/smsc/errno.h>
#include <drivers/smsc/mii.h>
#include <io.h>
#include <kernel/delay.h>
#include <mm/core_memprot.h>


#ifndef SMC_NOWAIT
# define SMC_NOWAIT		0
#endif

/*
 * Wait time for memory to be free.  This probably shouldn't be
 * tuned that much, as waiting for this means nothing else happens
 * in the system
 */
#define MEMORY_WAIT_TIME	16

/*
 * The maximum number of processing loops allowed for each call to the
 * IRQ handler.
 */
#define MAX_IRQ_LOOPS		8

/* this enables an interrupt in the interrupt mask register */
#define SMC_ENABLE_INT(smsc, x) do {					\
	unsigned char mask;						\
	mask = SMC_GET_INT_MASK(smsc);					\
	mask |= (x);							\
	SMC_SET_INT_MASK(smsc, mask);					\
} while (0)

/* this disables an interrupt from the interrupt mask register */
#define SMC_DISABLE_INT(smsc, x) do {					\
	unsigned char mask;						\
	mask = SMC_GET_INT_MASK(smsc);					\
	mask &= ~(x);							\
	SMC_SET_INT_MASK(smsc, mask);					\
} while (0)


#if SMC_DEBUG > 3
static void PRINT_PKT(u8 *buf, int length)
{
	int i;
	int remainder;
	int lines;

	lines = length / 16;
	remainder = length % 16;

	for (i = 0; i < lines ; i ++) {
		int cur;
		for (cur = 0; cur < 8; cur++) {
			u8 a, b;
			a = *buf++;
			b = *buf++;
			DMSG("%02x%02x ", a, b);
		}
		DMSG("\n");
	}
	for (i = 0; i < remainder/2 ; i++) {
		u8 a, b;
		a = *buf++;
		b = *buf++;
		DMSG("%02x%02x ", a, b);
	}
	DMSG("\n");
}
#else
#define PRINT_PKT(x...)  do { } while(0)
#endif

struct platform_device_id {
    char name[20];
    unsigned long driver_data;
};

static struct smc_local * smsc = NULL;
static int nowait = SMC_NOWAIT;
static const char version[] =
	"smc91x.c: v1.1, sep 22 2004 by Nicolas Pitre <nico@fluxnic.net>\n";
static const char * chip_ids[ 16 ] =  {
	NULL, NULL, NULL,
	/* 3 */ "SMC91C90/91C92",
	/* 4 */ "SMC91C94",
	/* 5 */ "SMC91C95",
	/* 6 */ "SMC91C96",
	/* 7 */ "SMC91C100",
	/* 8 */ "SMC91C100FD",
	/* 9 */ "SMC91C11xFD",
	NULL, NULL, NULL,
	NULL, NULL, NULL};

static TEE_Result init_eth(void);
static TEE_Result smc_probe(paddr_t ioaddr);
static void smc_reset(void);
static void smc_phy_detect(void);
static void smc_phy_write(int phyaddr, int phyreg, int phydata);
static int smc_phy_read(int phyaddr, int phyreg);
static void smc_mii_out(unsigned int val, int bits);
static unsigned int smc_mii_in(int bits);
static int smc_open(void);
static void smc_enable(void);
static void smc_phy_configure(void);
static int smc_phy_reset(int phy);
static int smc_phy_fixed(void);
static void smc_phy_check_media(int init);
static void smc_10bt_check_media(int init);
static unsigned int mii_check_media (struct mii_if_info *mii, unsigned int ok_to_print, unsigned int init_media);
static int mii_link_ok (struct mii_if_info *mii);
static int smc_hard_start_xmit(struct sk_buff *skb);
static void smc_hardware_send_pkt(void);
static void smc_tx(void);
static void smc_rcv(void);

/**
 * is_multicast_ether_addr - Determine if the Ethernet address is a multicast.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is a multicast address.
 * By definition the broadcast address is also a multicast address.
 */
static inline int is_multicast_ether_addr(const u8 *addr)
{
	return 0x01 & addr[0];
}

/**
 * is_zero_ether_addr - Determine if give Ethernet address is all zeros.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is all zeroes.
 */
static inline int is_zero_ether_addr(const u8 *addr)
{
	return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}


/**
 * is_valid_ether_addr - Determine if the given Ethernet address is valid
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Check that the Ethernet address (MAC) is not 00:00:00:00:00:00, is not
 * a multicast address, and is not FF:FF:FF:FF:FF:FF.
 *
 * Return true if the address is valid.
 */
static inline int is_valid_ether_addr(const u8 *addr)
{
	/* FF:FF:FF:FF:FF:FF is a multicast address so we don't need to
	 * explicitly check for it here. */
	return !is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr);
}

/**
 * mii_nway_result
 * @negotiated: value of MII ANAR and'd with ANLPAR
 *
 * Given a set of MII abilities, check each bit and returns the
 * currently supported media, in the priority order defined by
 * IEEE 802.3u.  We use LPA_xxx constants but note this is not the
 * value of LPA solely, as described above.
 *
 * The one exception to IEEE 802.3u is that 100baseT4 is placed
 * between 100T-full and 100T-half.  If your phy does not support
 * 100T4 this is fine.  If your phy places 100T4 elsewhere in the
 * priority order, you will need to roll your own function.
 */
static inline unsigned int mii_nway_result (unsigned int negotiated)
{
	unsigned int ret;

	if (negotiated & LPA_100FULL)
		ret = LPA_100FULL;
	else if (negotiated & LPA_100BASE4)
		ret = LPA_100BASE4;
	else if (negotiated & LPA_100HALF)
		ret = LPA_100HALF;
	else if (negotiated & LPA_10FULL)
		ret = LPA_10FULL;
	else
		ret = LPA_10HALF;

	return ret;
}

static inline int netif_carrier_ok(const struct mii_if_info *mii)
{
	if (mii->link_state == NETDEV_LINK_STATE_PRESENT)
		return 1;
	else
		return 0;
}

static inline void netif_carrier_on(struct mii_if_info *mii)
{
	mii->link_state = NETDEV_LINK_STATE_PRESENT;
}

static inline void netif_carrier_off(struct mii_if_info *mii)
{
	mii->link_state = NETDEV_STATE_NOCARRIER;
}


static inline void io_read8s(volatile uintptr_t addr, void *buffer, int len)
{
	if (len) {
		u8 *buf = buffer;
		do {
			u8 x = io_read8((uintptr_t)addr);
			*buf++ = x;
		} while (--len);
	}
}

static inline void io_read16s(volatile uintptr_t addr, void *buffer, int len)
{
	if (len) {
		u16 *buf = buffer;
		do {
			u16 x = io_read16((uintptr_t)addr);
			*buf++ = x;
		} while (--len);
	}
}

static inline void io_read32s(volatile uintptr_t addr, void *buffer, int len)
{
	if (len) {
		u32 *buf = buffer;
		do {
			u32 x = io_read32((uintptr_t)addr);
			*buf++ = x;
		} while (--len);
	}
}

static inline void io_write8s(volatile uintptr_t addr, const void *buffer, int len)
{
	if (len) {
		const u8 *buf = buffer;
		do {
			io_write8((uintptr_t)addr, *buf++);
		} while (--len);
	}
}

static inline void io_write16s(volatile uintptr_t addr, const void *buffer, int len)
{
	if (len) {
		const u16 *buf = buffer;
		do {
			io_write16((uintptr_t)addr, *buf++);
		} while (--len);
	}
}

static inline void io_write32s(volatile uintptr_t addr, const void *buffer, int len)
{
	if (len) {
		const u32 *buf = buffer;
		do {
			io_write32((uintptr_t)addr, *buf++);
		} while (--len);
	}
}

static enum itr_return smsc91x_itr_cb(struct itr_handler *h)
{
	void __iomem *ioaddr = smsc->base;
	int status, mask, timeout, card_stats;
	int saved_pointer;
	enum itr_return ret = ITRR_NONE;

	saved_pointer = SMC_GET_PTR(smsc);
	mask = SMC_GET_INT_MASK(smsc);
	SMC_SET_INT_MASK(smsc, 0);

	/* set a timeout value, so I don't stay here forever */
	timeout = MAX_IRQ_LOOPS;

	do {
		status = SMC_GET_INT(smsc);

		DMSG("%s: INT 0x%02x MASK 0x%02x MEM 0x%04x FIFO 0x%04x\n",
			CARDNAME, status, mask,
			({ int meminfo; SMC_SELECT_BANK(smsc, 0);
			   meminfo = SMC_GET_MIR(smsc);
			   SMC_SELECT_BANK(smsc, 2); meminfo; }),
			SMC_GET_FIFO(smsc));

		status &= mask;
		if (!status)
			break;

		if (status & IM_TX_INT) {
			/* do this before RX as it will free memory quickly */
			DMSG("%s: TX int\n", CARDNAME);
			smc_tx();
			SMC_ACK_INT(smsc, IM_TX_INT);
		}
		else if (status & IM_RCV_INT) {
			DMSG("%s: RX irq\n", CARDNAME);
			smc_rcv();
		} else if (status & IM_ALLOC_INT) {
			DMSG("%s: Allocation irq\n", CARDNAME);
			mask &= ~IM_ALLOC_INT;
		} else if (status & IM_TX_EMPTY_INT) {
			DMSG("%s: TX empty\n", CARDNAME);
			mask &= ~IM_TX_EMPTY_INT;

			/* update stats */
#if 0
			SMC_SELECT_BANK(smsc, 0);
			card_stats = SMC_GET_COUNTER(smsc);
			SMC_SELECT_BANK(smsc, 2);

			/* single collisions */
			dev->stats.collisions += card_stats & 0xF;
			card_stats >>= 4;

			/* multiple collisions */
			dev->stats.collisions += card_stats & 0xF;
#endif
		}
//		else if (status & IM_RX_OVRN_INT) {
//			DBG(1, "%s: RX overrun (EPH_ST 0x%04x)\n", dev->name,
//			       ({ int eph_st; SMC_SELECT_BANK(lp, 0);
//				  eph_st = SMC_GET_EPH_STATUS(lp);
//				  SMC_SELECT_BANK(lp, 2); eph_st; }));
//			SMC_ACK_INT(lp, IM_RX_OVRN_INT);
//			dev->stats.rx_errors++;
//			dev->stats.rx_fifo_errors++;
//		} else if (status & IM_EPH_INT) {
//			smc_eph_interrupt(dev);
//		} else if (status & IM_MDINT) {
//			SMC_ACK_INT(lp, IM_MDINT);
//			smc_phy_interrupt(dev);
//		} else if (status & IM_ERCV_INT) {
//			SMC_ACK_INT(lp, IM_ERCV_INT);
//			PRINTK("%s: UNSUPPORTED: ERCV INTERRUPT \n", dev->name);
//		}
	} while (--timeout);

	/* restore register states */
	SMC_SET_PTR(smsc, saved_pointer);
	SMC_SET_INT_MASK(smsc, mask);
	//spin_unlock(&lp->lock);

#ifndef CONFIG_NET_POLL_CONTROLLER
	if (timeout == MAX_IRQ_LOOPS)
		EMSG("%s: spurious interrupt (mask = 0x%02x)\n",
		       CARDNAME, mask);
#endif
	DMSG("%s: Interrupt done (%d loops)\n",
			CARDNAME, MAX_IRQ_LOOPS - timeout);

	/*
	 * We return IRQ_HANDLED unconditionally here even if there was
	 * nothing to do.  There is a possibility that a packet might
	 * get enqueued into the chip right after TX_EMPTY_INT is raised
	 * but just before the CPU acknowledges the IRQ.
	 * Better take an unneeded IRQ in some occasions than complexifying
	 * the code for all cases.
	 */

	return ITRR_HANDLED;
}

static TEE_Result
smsc91x_probe(const void *fdt __unused, int dev, const void *data) {
	int ret = 0;
	size_t size = 0;
	struct dt_node_info dt_smsc = {};
	const struct platform_device_id *smsc_id_data;
	paddr_t pa;
	int irq = DT_INFO_INVALID_INTERRUPT;
	uint32_t irq_type = 0;
	uint32_t irq_prio = 0;
	struct itr_handler *it_hdlr;

	EMSG("SMSC probe()");

	ret = bstgw_ethpool_init();

	bstgw_ethpool_increase(NULL, 16);

	_fdt_fill_device_info(fdt, &dt_smsc, dev);

	smsc = malloc(sizeof(struct smc_local));
	if (!smsc)
		return -ENOMEM;

	memset(smsc, 0x00, sizeof(struct smc_local));
	smsc_id_data = data;

	EMSG("SMSC device name: %s", smsc_id_data->name);

	pa = _fdt_reg_base_address(fdt, 1);

	ret = dt_map_dev(fdt, dev, &pa, &size, DT_MAP_SECURE);

	EMSG("SMSC device addr: %lx, %i %lx", pa, ret, size);

	smsc->cfg.flags |= (SMC_CAN_USE_8BIT)  ? SMC91X_USE_8BIT  : 0;
	smsc->cfg.flags |= (SMC_CAN_USE_16BIT) ? SMC91X_USE_16BIT : 0;
	smsc->cfg.flags |= (SMC_CAN_USE_32BIT) ? SMC91X_USE_32BIT : 0;
	smsc->cfg.flags |= (nowait) ? SMC91X_NOWAIT : 0;

	smsc->io_shift = SMC91X_IO_SHIFT(smsc->cfg.flags);

	ret = smc_probe(pa);

	irq = dt_get_irq_type_prio(fdt, dev, &irq_type, &irq_prio);
	if (irq == DT_INFO_INVALID_INTERRUPT)
		EMSG("Failed to obtain IRQ");
	else
		EMSG("IRQ = %i type = %u prio = %u", irq, irq_type, irq_prio);


	it_hdlr = itr_alloc_add_type_prio(irq, &smsc91x_itr_cb, 0, smsc,
					  irq_type, irq_prio);

	if (!it_hdlr)
		EMSG("IRQ regiustration failed!");

	itr_enable(irq);

	init_eth();

	return ret;
}

static TEE_Result smc_probe(paddr_t ioaddr)
{
	static int version_printed = 0;
	int retval;
	unsigned int val, revision_register;
	const char *version_string;

	DMSG("%s: %s", CARDNAME, __func__);

	/* First, see if the high byte is 0x33 */
	val = SMC_CURRENT_BANK(smsc);
	DMSG("%s: bank signature probe returned 0x%04x", CARDNAME, val);
	if ((val & 0xFF00) != 0x3300) {
		if ((val & 0xFF) == 0x33) {
			DMSG(
				"%s: Detected possible byte-swapped interface"
				" at IOADDR %lx", CARDNAME, ioaddr);
		}
		retval = -ENODEV;
		goto err_out;
	}

	/*
	 * The above MIGHT indicate a device, but I need to write to
	 * further test this.
	 */
	SMC_SELECT_BANK(smsc, 0);
	val = SMC_CURRENT_BANK(smsc);
	if ((val & 0xFF00) != 0x3300) {
		retval = -ENODEV;
		goto err_out;
	}

	/*
	 * well, we've already written once, so hopefully another
	 * time won't hurt.  This time, I need to switch the bank
	 * register to bank 1, so I can access the base address
	 * register
	 */
	SMC_SELECT_BANK(smsc, 1);
	val = SMC_GET_BASE(smsc);
	val = ((val & 0x1F00) >> 3) << SMC_IO_SHIFT;
	if (((unsigned long)ioaddr & (0x3e0 << SMC_IO_SHIFT)) != val) {
		DMSG("%s: IOADDR %lx doesn't match configuration (%x).",
			CARDNAME, ioaddr, val);
	}

	/*
	 * check if the revision register is something that I
	 * recognize.  These might need to be added to later,
	 * as future revisions could be added.
	 */
	SMC_SELECT_BANK(smsc, 3);
	revision_register = SMC_GET_REV(smsc);
	DMSG("%s: revision = 0x%04x", CARDNAME, revision_register);
	version_string = chip_ids[ (revision_register >> 4) & 0xF];
	if (!version_string || (revision_register & 0xff00) != 0x3300) {
		/* I don't recognize this chip, so... */
		DMSG("%s: IO %lx: Unrecognized revision register 0x%04x"
			", Contact author.", CARDNAME,
			ioaddr, revision_register);

		retval = -ENODEV;
		goto err_out;
	}

	/* At this point I'll assume that the chip is an SMC91x. */
	if (version_printed++ == 0)
		DMSG("%s", version);

	smsc->base = ioaddr;
	smsc->version = revision_register & 0xff;
	smsc->lock = 0;

	/* Get the MAC address */
	SMC_SELECT_BANK(smsc, 1);
	SMC_GET_MAC_ADDR(smsc, smsc->mac_addr);

	DMSG("%s: dev addr:%02x:%02x:%02x:%02x:%02x:%02x", CARDNAME, smsc->mac_addr[5],smsc->mac_addr[4],smsc->mac_addr[3],smsc->mac_addr[2],smsc->mac_addr[1],smsc->mac_addr[0]);

	/* now, reset the chip, and put it into a known state */
	smc_reset();

	/*
	 * Locate the phy, if any.
	 */
	if (smsc->version >= (CHIP_91100 << 4))
		smc_phy_detect();

	/* Set default parameters */
	smsc->msg_enable = NETIF_MSG_LINK;
	smsc->ctl_rfduplx = 0;
	smsc->ctl_rspeed = 10;

	if (smsc->version >= (CHIP_91100 << 4)) {
		smsc->ctl_rfduplx = 1;
		smsc->ctl_rspeed = 100;
	}

	// Open the device
	smc_open();

	return TEE_SUCCESS;

	err_out:
		EMSG( "SMC probe failed");
		return retval;
}

/*
 * this does a soft reset on the device
 */
void smc_reset(void)
{
	paddr_t __iomem ioaddr = smsc->base;
	unsigned int ctl, cfg;
	struct sk_buff *pending_skb;

	/* Disable all interrupts, block TX tasklet */
	//spin_lock_irq(&smsc->lock);
	SMC_SELECT_BANK(smsc, 2);
	SMC_SET_INT_MASK(smsc, 0);
	pending_skb = smsc->pending_tx_skb;
	smsc->pending_tx_skb = NULL;
	//spin_unlock_irq(&smsc->lock);

	/* free any pending tx skb */
	if (pending_skb) {
		EMSG( "pending skb %p", pending_skb);
		dev_kfree_skb(pending_skb);
	}

	/*
	 * This resets the registers mostly to defaults, but doesn't
	 * affect EEPROM.  That seems unnecessary
	 */
	SMC_SELECT_BANK(smsc, 0);
	SMC_SET_RCR(smsc, RCR_SOFTRST);

	/*
	 * Setup the Configuration Register
	 * This is necessary because the CONFIG_REG is not affected
	 * by a soft reset
	 */
	SMC_SELECT_BANK(smsc, 1);

	cfg = CONFIG_DEFAULT;

	/*
	 * Setup for fast accesses if requested.  If the card/system
	 * can't handle it then there will be no recovery except for
	 * a hard reset or power cycle
	 */
	if (smsc->cfg.flags & SMC91X_NOWAIT)
		cfg |= CONFIG_NO_WAIT;

	/*
	 * Release from possible power-down state
	 * Configuration register is not affected by Soft Reset
	 */
	cfg |= CONFIG_EPH_POWER_EN;

	SMC_SET_CONFIG(smsc, cfg);

	/* this should pause enough for the chip to be happy */
	/*
	 * elaborate?  What does the chip _need_? --jgarzik
	 *
	 * This seems to be undocumented, but something the original
	 * driver(s) have always done.  Suspect undocumented timing
	 * info/determined empirically. --rmk
	 */
	udelay(1);

	/* Disable transmit and receive functionality */
	SMC_SELECT_BANK(smsc, 0);
	SMC_SET_RCR(smsc, RCR_CLEAR);
	SMC_SET_TCR(smsc, TCR_CLEAR);

	SMC_SELECT_BANK(smsc, 1);
	ctl = SMC_GET_CTL(smsc) | CTL_LE_ENABLE;

	/*
	 * Set the control register to automatically release successfully
	 * transmitted packets, to make the best use out of our limited
	 * memory
	 */
	if(!THROTTLE_TX_PKTS)
		ctl |= CTL_AUTO_RELEASE;
	else
		ctl &= ~CTL_AUTO_RELEASE;
	SMC_SET_CTL(smsc, ctl);

	/* Reset the MMU */
	SMC_SELECT_BANK(smsc, 2);
	SMC_SET_MMU_CMD(smsc, MC_RESET);
	//SMC_WAIT_MMU_BUSY(smsc); TODO
}


/*
 * Finds and reports the PHY address
 */
void smc_phy_detect(void)
{
	int phyaddr;

	smsc->phy_type = 0;

	/*
	 * Scan all 32 PHY addresses if necessary, starting at
	 * PHY#1 to PHY#31, and then PHY#0 last.
	 */
	for (phyaddr = 1; phyaddr < 33; ++phyaddr) {
		unsigned int id1, id2;

		/* Read the PHY identifiers */
		id1 = smc_phy_read(phyaddr & 31, MII_PHYSID1);
		id2 = smc_phy_read(phyaddr & 31, MII_PHYSID2);

		DMSG("phy_id1=0x%x, phy_id2=0x%x\n", id1, id2);

		/* Make sure it is a valid identifier */
		if (id1 != 0x0000 && id1 != 0xffff && id1 != 0x8000 &&
		    id2 != 0x0000 && id2 != 0xffff && id2 != 0x8000) {
			/* Save the PHY's address */
			smsc->mii.phy_id = phyaddr & 31;
			smsc->phy_type = id1 << 16 | id2;
			break;
		}
	}
}

/*
 * Reads a register from the MII Management serial interface
 */
int smc_phy_read(int phyaddr, int phyreg)
{
	paddr_t __iomem ioaddr = smsc->base;
	unsigned int phydata;

	SMC_SELECT_BANK(smsc, 3);

	/* Idle - 32 ones */
	smc_mii_out(0xffffffff, 32);

	/* Start code (01) + read (10) + phyaddr + phyreg */
	smc_mii_out(6 << 10 | phyaddr << 5 | phyreg, 14);

	/* Turnaround (2bits) + phydata */
	phydata = smc_mii_in(18);

	/* Return to idle state */
	SMC_SET_MII(smsc, SMC_GET_MII(smsc) & ~(MII_MCLK|MII_MDOE|MII_MDO));

	//EMSG("%s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n", __func__, phyaddr, phyreg, phydata);

	SMC_SELECT_BANK(smsc, 2);
	return phydata;
}

/*
 * Writes a register to the MII Management serial interface
 */
void smc_phy_write(int phyaddr, int phyreg,
			  int phydata)
{
	paddr_t __iomem ioaddr = smsc->base;

	SMC_SELECT_BANK(smsc, 3);

	/* Idle - 32 ones */
	smc_mii_out(0xffffffff, 32);

	/* Start code (01) + write (01) + phyaddr + phyreg + turnaround + phydata */
	smc_mii_out(5 << 28 | phyaddr << 23 | phyreg << 18 | 2 << 16 | phydata, 32);

	/* Return to idle state */
	SMC_SET_MII(smsc, SMC_GET_MII(smsc) & ~(MII_MCLK|MII_MDOE|MII_MDO));

	//EMSG("%s: phyaddr=0x%x, phyreg=0x%x, phydata=0x%x\n", __func__, phyaddr, phyreg, phydata);

	SMC_SELECT_BANK(smsc, 2);
}

/*---PHY CONTROL AND CONFIGURATION-----------------------------------------*/

void smc_mii_out(unsigned int val, int bits)
{
	paddr_t __iomem ioaddr = smsc->base;
	unsigned int mii_reg, mask;

	mii_reg = SMC_GET_MII(smsc) & ~(MII_MCLK | MII_MDOE | MII_MDO);
	mii_reg |= MII_MDOE;

	for (mask = 1 << (bits - 1); mask; mask >>= 1) {
		if (val & mask)
			mii_reg |= MII_MDO;
		else
			mii_reg &= ~MII_MDO;

		SMC_SET_MII(smsc, mii_reg);
		udelay(MII_DELAY);
		SMC_SET_MII(smsc, mii_reg | MII_MCLK);
		udelay(MII_DELAY);
	}
}

unsigned int smc_mii_in(int bits)
{
	paddr_t __iomem ioaddr = smsc->base;
	unsigned int mii_reg, mask, val;

	mii_reg = SMC_GET_MII(smsc) & ~(MII_MCLK | MII_MDOE | MII_MDO);
	SMC_SET_MII(smsc, mii_reg);

	for (mask = 1 << (bits - 1), val = 0; mask; mask >>= 1) {
		if (SMC_GET_MII(smsc) & MII_MDI)
			val |= mask;

		SMC_SET_MII(smsc, mii_reg);
		udelay(MII_DELAY);
		SMC_SET_MII(smsc, mii_reg | MII_MCLK);
		udelay(MII_DELAY);
	}

	return val;
}


/*
 * Open and Initialize the board
 *
 * Set up everything, reset the card, etc..
 */
int smc_open(void)
{
	/*
	 * Check that the address is valid.  If its not, refuse
	 * to bring the device up.  The user must specify an
	 * address using ifconfig eth0 hw ether xx:xx:xx:xx:xx:xx
	 */
	if (!is_valid_ether_addr(smsc->mac_addr)) {
		EMSG("%s: no valid ethernet hw addr\n", __func__);
		return -EINVAL;
	}

	/* Setup the default Register Modes */
	smsc->tcr_cur_mode = TCR_DEFAULT;
	smsc->rcr_cur_mode = RCR_DEFAULT;
	smsc->rpc_cur_mode = RPC_DEFAULT |
				smsc->cfg.leda << RPC_LSXA_SHFT |
				smsc->cfg.ledb << RPC_LSXB_SHFT;

	/*
	 * If we are not using a MII interface, we need to
	 * monitor our own carrier signal to detect faults.
	 */
	if (smsc->phy_type == 0)
		smsc->tcr_cur_mode |= TCR_MON_CSN;

	/* reset the hardware */
	smc_reset();
	smc_enable();

	/* Configure the PHY, initialize the link state */
	if (smsc->phy_type != 0)
		smc_phy_configure();
	else {
		smc_10bt_check_media(1);
	}

	return 0;
}


/*
 * Enable Interrupts, Receive, and Transmit
 */
void smc_enable(void)
{
	paddr_t __iomem ioaddr = smsc->base;
	int mask;

	/* see the header file for options in TCR/RCR DEFAULT */
	SMC_SELECT_BANK(smsc, 0);
	SMC_SET_TCR(lp, smsc->tcr_cur_mode);
	SMC_SET_RCR(lp, smsc->rcr_cur_mode);

	SMC_SELECT_BANK(smsc, 1);
	SMC_SET_MAC_ADDR(smsc, smsc->mac_addr);

	/* now, enable interrupts */
	mask = IM_EPH_INT|IM_RX_OVRN_INT|IM_RCV_INT;
	if (smsc->version >= (CHIP_91100 << 4))
		mask |= IM_MDINT;
	SMC_SELECT_BANK(smsc, 2);
	SMC_SET_INT_MASK(smsc, mask);

	/*
	 * From this point the register bank must _NOT_ be switched away
	 * to something else than bank 2 without proper locking against
	 * races with any tasklet or interrupt handlers until smc_shutdown()
	 * or smc_reset() is called.
	 */
}

/*
 * Configures the specified PHY through the MII management interface
 * using Autonegotiation.
 * Calls smc_phy_fixed() if the user has requested a certain config.
 * If RPC ANEG bit is set, the media selection is dependent purely on
 * the selection by the MII (either in the MII BMCR reg or the result
 * of autonegotiation.)  If the RPC ANEG bit is cleared, the selection
 * is controlled by the RPC SPEED and RPC DPLX bits.
 */
void smc_phy_configure(void)
{
	paddr_t __iomem ioaddr = smsc->base;
	int phyaddr = smsc->mii.phy_id;
	int my_phy_caps; /* My PHY capabilities */
	int my_ad_caps; /* My Advertised capabilities */
	int status;

	/*
	 * We should not be called if phy_type is zero.
	 */
	if (smsc->phy_type == 0)
		goto smc_phy_configure_exit;

	if (smc_phy_reset(phyaddr)) {
		EMSG("%s: PHY reset timed out\n", __func__);
		goto smc_phy_configure_exit;
	}

	/*
	 * Enable PHY Interrupts (for register 18)
	 * Interrupts listed here are disabled
	 */
	smc_phy_write(phyaddr, PHY_MASK_REG,
		PHY_INT_LOSSSYNC | PHY_INT_CWRD | PHY_INT_SSD |
		PHY_INT_ESD | PHY_INT_RPOL | PHY_INT_JAB |
		PHY_INT_SPDDET | PHY_INT_DPLXDET);

	/* Configure the Receive/Phy Control register */
	SMC_SELECT_BANK(smsc, 0);
	SMC_SET_RPC(smsc, smsc->rpc_cur_mode);

	/* If the user requested no auto neg, then go set his request */
	if (smsc->mii.force_media) {
		smc_phy_fixed();
		goto smc_phy_configure_exit;
	}

	/* Copy our capabilities from MII_BMSR to MII_ADVERTISE */
	my_phy_caps = smc_phy_read(phyaddr, MII_BMSR);

	if (!(my_phy_caps & BMSR_ANEGCAPABLE)) {
		DMSG("Auto negotiation NOT supported\n");
		smc_phy_fixed();
		goto smc_phy_configure_exit;
	}

	my_ad_caps = ADVERTISE_CSMA; /* I am CSMA capable */

	if (my_phy_caps & BMSR_100BASE4)
		my_ad_caps |= ADVERTISE_100BASE4;
	if (my_phy_caps & BMSR_100FULL)
		my_ad_caps |= ADVERTISE_100FULL;
	if (my_phy_caps & BMSR_100HALF)
		my_ad_caps |= ADVERTISE_100HALF;
	if (my_phy_caps & BMSR_10FULL)
		my_ad_caps |= ADVERTISE_10FULL;
	if (my_phy_caps & BMSR_10HALF)
		my_ad_caps |= ADVERTISE_10HALF;

	/* Disable capabilities not selected by our user */
	if (smsc->ctl_rspeed != 100)
		my_ad_caps &= ~(ADVERTISE_100BASE4|ADVERTISE_100FULL|ADVERTISE_100HALF);

	if (!smsc->ctl_rfduplx)
		my_ad_caps &= ~(ADVERTISE_100FULL|ADVERTISE_10FULL);

	/* Update our Auto-Neg Advertisement Register */
	smc_phy_write(phyaddr, MII_ADVERTISE, my_ad_caps);
	smsc->mii.advertising = my_ad_caps;

	/*
	 * Read the register back.  Without this, it appears that when
	 * auto-negotiation is restarted, sometimes it isn't ready and
	 * the link does not come up.
	 */
	status = smc_phy_read(phyaddr, MII_ADVERTISE);
	(void)status; /* suppress compile warning */

	DMSG("phy caps=%x\n", my_phy_caps);
	DMSG("phy advertised caps=%x\n", my_ad_caps);

	/* Restart auto-negotiation process in order to advertise my caps */
	smc_phy_write(phyaddr, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	smc_phy_check_media(1);

smc_phy_configure_exit:
	SMC_SELECT_BANK(smsc, 2);
}


/*
 * smc_phy_reset - reset the phy
 * @dev: net device
 * @phy: phy address
 *
 * Issue a software reset for the specified PHY and
 * wait up to 100ms for the reset to complete.  We should
 * not access the PHY for 50ms after issuing the reset.
 *
 * The time to wait appears to be dependent on the PHY.
 *
 * Must be called with lp->lock locked.
 */
int smc_phy_reset(int phy)
{
	unsigned int bmcr;
	int timeout;

	smc_phy_write(phy, MII_BMCR, BMCR_RESET);

	for (timeout = 2; timeout; timeout--) {
		mdelay(50);
		bmcr = smc_phy_read(phy, MII_BMCR);
		if (!(bmcr & BMCR_RESET))
			break;
	}

	return bmcr & BMCR_RESET;
}

/*
 * Sets the PHY to a configuration as determined by the user
 */
static int smc_phy_fixed(void)
{
	paddr_t __iomem ioaddr = smsc->base;
	int phyaddr = smsc->mii.phy_id;
	int bmcr, cfg1;

	/* Enter Link Disable state */
	cfg1 = smc_phy_read(phyaddr, PHY_CFG1_REG);
	cfg1 |= PHY_CFG1_LNKDIS;
	smc_phy_write(phyaddr, PHY_CFG1_REG, cfg1);

	/*
	 * Set our fixed capabilities
	 * Disable auto-negotiation
	 */
	bmcr = 0;

	if (smsc->ctl_rfduplx)
		bmcr |= BMCR_FULLDPLX;

	if (smsc->ctl_rspeed == 100)
		bmcr |= BMCR_SPEED100;

	/* Write our capabilities to the phy control register */
	smc_phy_write(phyaddr, MII_BMCR, bmcr);

	/* Re-Configure the Receive/Phy Control register */
	SMC_SELECT_BANK(smsc, 0);
	SMC_SET_RPC(smsc, smsc->rpc_cur_mode);
	SMC_SELECT_BANK(smsc, 2);

	return 1;
}


/*
 * smc_phy_check_media - check the media status and adjust TCR
 * @dev: net device
 * @init: set true for initialisation
 *
 * Select duplex mode depending on negotiation state.  This
 * also updates our carrier state.
 */
static void smc_phy_check_media(int init)
{
	paddr_t __iomem ioaddr = smsc->base;

	if (mii_check_media(&smsc->mii, netif_msg_link(smsc), init)) {
		/* duplex state has changed */
		if (smsc->mii.full_duplex) {
			smsc->tcr_cur_mode |= TCR_SWFDUP;
		} else {
			smsc->tcr_cur_mode &= ~TCR_SWFDUP;
		}

		SMC_SELECT_BANK(smsc, 0);
		SMC_SET_TCR(smsc, smsc->tcr_cur_mode);
	}
}


void smc_10bt_check_media(int init)
{
	paddr_t __iomem ioaddr = smsc->base;
	unsigned int new_carrier;

	SMC_SELECT_BANK(smsc, 0);
	new_carrier = (SMC_GET_EPH_STATUS(smsc) & ES_LINK_OK) ? 1 : 0;
	SMC_SELECT_BANK(smsc, 2);

	if (init) {
		DMSG("link %s\n", new_carrier ? "up" : "down");
	}
}


/**
 * mii_check_media - check the MII interface for a duplex change
 * @mii: the MII interface
 * @ok_to_print: OK to print link up/down messages
 * @init_media: OK to save duplex mode in @mii
 *
 * Returns 1 if the duplex mode changed, 0 if not.
 * If the media type is forced, always returns 0.
 */
unsigned int mii_check_media (struct mii_if_info *mii,
			      unsigned int ok_to_print,
			      unsigned int init_media)
{
	unsigned int old_carrier, new_carrier;
	int advertise, lpa, media, duplex;
	int lpa2 = 0;

	/* if forced media, go no further */
	if (mii->force_media)
		return 0; /* duplex did not change */

	/* check current and old link status */
	old_carrier = netif_carrier_ok(mii) ? 1 : 0;
	new_carrier = (unsigned int) mii_link_ok(mii);

	/* if carrier state did not change, this is a "bounce",
	 * just exit as everything is already set correctly
	 */
	if ((!init_media) && (old_carrier == new_carrier))
		return 0; /* duplex did not change */

	/* no carrier, nothing much to do */
	if (!new_carrier) {
		netif_carrier_off(mii);
		if (ok_to_print)
			EMSG("%s: link down\n", CARDNAME);
		return 0; /* duplex did not change */
	}

	/*
	 * we have carrier, see who's on the other end
	 */
	netif_carrier_on(mii);

	/* get MII advertise and LPA values */
	if ((!init_media) && (mii->advertising))
		advertise = mii->advertising;
	else {
		advertise = smc_phy_read(mii->phy_id, MII_ADVERTISE);
		mii->advertising = advertise;
	}
	lpa = smc_phy_read(mii->phy_id, MII_LPA);
	if (mii->supports_gmii)
		lpa2 = smc_phy_read(mii->phy_id, MII_STAT1000);

	/* figure out media and duplex from advertise and LPA values */
	media = mii_nway_result(lpa & advertise);
	duplex = (media & ADVERTISE_FULL) ? 1 : 0;
	if (lpa2 & LPA_1000FULL)
		duplex = 1;

	if (ok_to_print)
		EMSG("%s: link up, %sMbps, %s-duplex, lpa 0x%04X\n",
			   CARDNAME,
		       lpa2 & (LPA_1000FULL | LPA_1000HALF) ? "1000" :
		       media & (ADVERTISE_100FULL | ADVERTISE_100HALF) ? "100" : "10",
		       duplex ? "full" : "half",
		       lpa);


	if ((init_media) || (mii->full_duplex != duplex)) {
		mii->full_duplex = duplex;
		return 1; /* duplex changed */
	}

	return 0; /* duplex did not change */
}

/**
 * mii_link_ok - is link status up/ok
 * @mii: the MII interface
 *
 * Returns 1 if the MII reports link status up/ok, 0 otherwise.
 */
int mii_link_ok (struct mii_if_info *mii)
{
	/* first, a dummy read, needed to latch some MII phys */
	smc_phy_read(mii->phy_id, MII_BMSR);
	if (smc_phy_read(mii->phy_id, MII_BMSR) & BMSR_LSTATUS)
		return 1;
	return 0;
}


/*
 * Since I am not sure if I will have enough room in the chip's ram
 * to store the packet, I call this routine which either sends it
 * now, or set the card to generates an interrupt when ready
 * for the packet.
 */
int smc_hard_start_xmit(struct sk_buff *skb)
{
	paddr_t __iomem ioaddr = smsc->base;
	unsigned int numPages, poll_count, status;
	/*
	 * The MMU wants the number of pages to be the number of 256 bytes
	 * 'pages', minus 1 (since a packet can't ever have 0 pages :))
	 *
	 * The 91C111 ignores the size bits, but earlier models don't.
	 *
	 * Pkt size for allocating is data length +6 (for additional status
	 * words, length and ctl)
	 *
	 * If odd size then last byte is included in ctl word.
	 */
	numPages = ((skb->eth_buf.data_len & ~1) + (6 - 1)) >> 8;
	if (unlikely(numPages > 7)) {
		EMSG("%s: Far too big packet error.\n", CARDNAME);
		dev_kfree_skb(skb);
		return 0;
	}

	//smc_special_lock(&lp->lock, flags);

	/* now, try to allocate the memory */
	SMC_SET_MMU_CMD(smsc, MC_ALLOC | numPages);

	/*
	 * Poll the chip for a short amount of time in case the
	 * allocation succeeds quickly.
	 */
	poll_count = MEMORY_WAIT_TIME;
	do {
		status = SMC_GET_INT(smsc);
		if (status & IM_ALLOC_INT) {
			SMC_ACK_INT(smsc, IM_ALLOC_INT);
  			break;
		}
   	} while (--poll_count);

	//smc_special_unlock(&lp->lock, flags);

	smsc->pending_tx_skb = skb;
   	if (!poll_count) {
		/* oh well, wait until the chip finds memory later */
   		EMSG("%s: TX memory allocation deferred.\n", CARDNAME);
		SMC_ENABLE_INT(smsc, IM_ALLOC_INT);
   	} else {
		/*
		 * Allocation succeeded: push packet to the chip's own memory
		 * immediately.
		 */
		smc_hardware_send_pkt();
	}

	return 0;
}

/*
 * This is called to actually send a packet to the chip.
 */
void smc_hardware_send_pkt(void)
{
	void __iomem * ioaddr = smsc->base;
	struct sk_buff *skb;
	unsigned int packet_no, len;
	unsigned char *buf;

//	if (!smc_special_trylock(&lp->lock, flags)) {
//		netif_stop_queue(dev);
//		tasklet_schedule(&lp->tx_task);
//		return;
//	}

	skb = smsc->pending_tx_skb;
	if (unlikely(!skb)) {
		//smc_special_unlock(&lp->lock, flags);
		return;
	}
	smsc->pending_tx_skb = NULL;

	packet_no = SMC_GET_AR(smsc);
	if (unlikely(packet_no & AR_FAILED)) {
		EMSG("%s: Memory allocation failed.\n", CARDNAME);
//		smc_special_unlock(&lp->lock, flags);
		goto done;
	}

	/* point to the beginning of the packet */
	SMC_SET_PN(smsc, packet_no);
	SMC_SET_PTR(smsc, PTR_AUTOINC);

	buf = (unsigned char *)(&skb->eth_buf.buf);
	len = skb->eth_buf.data_len;
	DMSG("%s: TX PNR 0x%x LENGTH 0x%04x (%d) BUF 0x%p\n",
			CARDNAME, packet_no, len, len, buf);
	PRINT_PKT(buf, len);

	/*
	 * Send the packet length (+6 for status words, length, and ctl.
	 * The card will pad to 64 bytes with zeroes if packet is too small.
	 */
	SMC_PUT_PKT_HDR(smsc, 0, len + 6);

	/* send the actual data */
	SMC_PUSH_DATA(smsc, buf, (len & ~1));

	/* Send final ctl word with the last byte if there is one */
	SMC_outw(((len & 1) ? (0x2000 | buf[len-1]) : 0), ioaddr, DATA_REG(smsc));

	/* queue the packet for TX */
	SMC_SET_MMU_CMD(smsc, MC_ENQUEUE);
	//smc_special_unlock(&lp->lock, flags);

	SMC_ENABLE_INT(smsc, IM_TX_INT | IM_TX_EMPTY_INT);

done:
	dev_kfree_skb(skb);
}


void send_test_pkt(void)
{
	int rc = 0;
	int i = 0;

	bstgw_ethbuf_t * buff;
	struct sk_buff * skb;
	DMSG("Send test pkt");

	buff = bstgw_ethpool_buf_alloc(NULL);

	buff->data_len = 32;
	buff->data_off = sizeof(bstgw_ethbuf_t);
	buff->l2_len = 16;

	uint8_t* ptr = (uint8_t*)buff;

	for(; i < 1024; i+=4)
	{
		DMSG("%p: %x %x %x %x", ptr + i, ptr[i],ptr[i+1],ptr[i+2],ptr[i+3]);
	}


	skb = (struct sk_buff *)buff;

	rc = smc_hard_start_xmit(skb);

	DMSG("Send test pkt rc = %i", rc);
}

/*
 * This handles a TX interrupt, which is only called when:
 * - a TX error occurred, or
 * - CTL_AUTO_RELEASE is not set and TX of a packet completed.
 */
void smc_tx(void)
{
	void __iomem *ioaddr = smsc->base;
	unsigned int saved_packet, packet_no, tx_status, pkt_len;

	/* If the TX FIFO is empty then nothing to do */
	packet_no = SMC_GET_TXFIFO(smsc);
	if (unlikely(packet_no & TXFIFO_TEMPTY)) {
		EMSG("%s: smc_tx with nothing on FIFO.\n", CARDNAME);
		return;
	}

	/* select packet to read from */
	saved_packet = SMC_GET_PN(smsc);
	SMC_SET_PN(smsc, packet_no);

	/* read the first word (status word) from this packet */
	SMC_SET_PTR(smsc, PTR_AUTOINC | PTR_READ);
	SMC_GET_PKT_HDR(smsc, tx_status, pkt_len);
	(void)pkt_len; /* suppress compile warning */
	DMSG("%s: TX STATUS 0x%04x PNR 0x%02x\n",
		CARDNAME, tx_status, packet_no);

	//if (!(tx_status & ES_TX_SUC))
		//dev->stats.tx_errors++;

	//if (tx_status & ES_LOSTCARR)
		//dev->stats.tx_carrier_errors++;

	if (tx_status & (ES_LATCOL | ES_16COL)) {
		EMSG("%s: %s occurred on last xmit\n", CARDNAME,
		       (tx_status & ES_LATCOL) ?
			"late collision" : "too many collisions");
//		dev->stats.tx_window_errors++;
//		if (!(dev->stats.tx_window_errors & 63) && net_ratelimit()) {
//			printk(KERN_INFO "%s: unexpectedly large number of "
//			       "bad collisions. Please check duplex "
//			       "setting.\n", dev->name);
//		}
	}

	/* kill the packet */
	//SMC_WAIT_MMU_BUSY(smsc); TODO - check if wait is needed
	SMC_SET_MMU_CMD(smsc, MC_FREEPKT);

	/* Don't restore Packet Number Reg until busy bit is cleared */
	//SMC_WAIT_MMU_BUSY(smsc); TODO - check if wait is needed
	SMC_SET_PN(smsc, saved_packet);

	/* re-enable transmit */
	SMC_SELECT_BANK(smsc, 0);
	SMC_SET_TCR(smsc, smsc->tcr_cur_mode);
	SMC_SELECT_BANK(smsc, 2);
}

/*
 * This is the procedure to handle the receipt of a packet.
 */
void smc_rcv(void)
{
	void __iomem *ioaddr = smsc->base;
	unsigned int packet_number, status, packet_len;

	packet_number = SMC_GET_RXFIFO(smsc);
	if (unlikely(packet_number & RXFIFO_REMPTY)) {
		EMSG("%s: smc_rcv with nothing on FIFO.\n", CARDNAME);
		return;
	}

	/* read from start of packet */
	SMC_SET_PTR(smsc, PTR_READ | PTR_RCV | PTR_AUTOINC);

	/* First two words are status and packet length */
	SMC_GET_PKT_HDR(smsc, status, packet_len);
	packet_len &= 0x07ff;  /* mask off top bits */
	DMSG("%s: RX PNR 0x%x STATUS 0x%04x LENGTH 0x%04x (%d)\n",
		CARDNAME, packet_number, status,
		packet_len, packet_len);

	back:
	if (unlikely((packet_len < 6) || (status & RS_ERRORS))) {
		if ((status & RS_TOOLONG) && packet_len <= (1514 + 4 + 6)) {
			/* accept VLAN packets */
			status &= ~RS_TOOLONG;
			goto back;
		}
		if (packet_len < 6) {
			/* bloody hardware */
			EMSG("%s: fubar (rxlen %u status %x\n",
					CARDNAME, packet_len, status);
			status |= RS_TOOSHORT;
		}
		//SMC_WAIT_MMU_BUSY(smsc); TODO
		SMC_SET_MMU_CMD(lp, MC_RELEASE);
#if 0
		dev->stats.rx_errors++;
		if (status & RS_ALGNERR)
			dev->stats.rx_frame_errors++;
		if (status & (RS_TOOSHORT | RS_TOOLONG))
			dev->stats.rx_length_errors++;
		if (status & RS_BADCRC)
			dev->stats.rx_crc_errors++;
#endif
	} else {
		struct sk_buff *skb;
		unsigned char *data;
		unsigned int data_len;
#if 0
		/* set multicast stats */
		if (status & RS_MULTICAST)
			dev->stats.multicast++;
#endif
		/*
		 * Actual payload is packet_len - 6 (or 5 if odd byte).
		 * We want skb_reserve(2) and the final ctrl word
		 * (2 bytes, possibly containing the payload odd byte).
		 * Furthermore, we add 2 bytes to allow rounding up to
		 * multiple of 4 bytes on 32 bit buses.
		 * Hence packet_len - 6 + 2 + 2 + 2.
		 */
		skb = (struct sk_buff*) bstgw_ethpool_buf_alloc(NULL);
		if (unlikely(skb == NULL)) {
			EMSG("%s: Low memory, packet dropped.\n", CARDNAME);
			//SMC_WAIT_MMU_BUSY(smsc); TODO
			SMC_SET_MMU_CMD(smsc, MC_RELEASE);
#if 0
			dev->stats.rx_dropped++;
#endif
			return;
		}

		/* Align IP header to 32 bits */
		skb_reserve(skb, 2);

		/* BUG: the LAN91C111 rev A never sets this bit. Force it. */
		if (smsc->version == 0x90)
			status |= RS_ODDFRAME;

		/*
		 * If odd length: packet_len - 5,
		 * otherwise packet_len - 6.
		 * With the trailing ctrl byte it's packet_len - 4.
		 */
		data_len = packet_len - ((status & RS_ODDFRAME) ? 5 : 6);
		data = skb_put(skb, data_len);
		SMC_PULL_DATA(smsc, data, packet_len - 4);

		//SMC_WAIT_MMU_BUSY(smsc); //TODO
		SMC_SET_MMU_CMD(smsc, MC_RELEASE);

		PRINT_PKT(data, packet_len - 4);
#if 0
		skb->protocol = eth_type_trans(skb, dev);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += data_len;
#endif
	}
}

TEE_Result init_eth(void)
{
	return TEE_SUCCESS;
}

static const struct dt_device_match smsc91x_match_table[] = {
	{ .compatible = "smc91x", },
	{ }
};

DEFINE_DT_DRIVER(smc_dt_driver) = {
	.name = "smc91x",
	.type = DT_DRIVER_NOTYPE,
	.match_table = smsc91x_match_table,
	.probe = smsc91x_probe,
};

