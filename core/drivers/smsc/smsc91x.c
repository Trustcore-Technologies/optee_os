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
#include <io.h>

#ifndef SMC_NOWAIT
# define SMC_NOWAIT		0
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

static TEE_Result init_eth();
static TEE_Result smc_probe(void  *ioaddr);
static void smc_reset();

static TEE_Result
smsc91x_probe(const void *fdt __unused, int dev, const void *data) {
	EMSG("SMSC probe()");

	const struct platform_device_id *smsc_id_data;
	int ret = 0;

	ret = bstgw_ethpool_init();


	smsc = malloc(sizeof(struct smc_local));
	if (!smsc)
		return -ENOMEM;

	memset(smsc, 0x00, sizeof(struct smc_local));
	smsc_id_data = data;

	EMSG("SMSC device name: %s", smsc_id_data->name);

	paddr_t pa = _fdt_reg_base_address(fdt, 1);
	size_t size = 0;

	ret = dt_map_dev(fdt, dev, &pa, &size, DT_MAP_SECURE);

	EMSG("SMSC device addr: %lx, %i %x", pa, ret, size);

	smsc->cfg.flags |= (SMC_CAN_USE_8BIT)  ? SMC91X_USE_8BIT  : 0;
	smsc->cfg.flags |= (SMC_CAN_USE_16BIT) ? SMC91X_USE_16BIT : 0;
	smsc->cfg.flags |= (SMC_CAN_USE_32BIT) ? SMC91X_USE_32BIT : 0;
	smsc->cfg.flags |= (nowait) ? SMC91X_NOWAIT : 0;

	smsc->io_shift = SMC91X_IO_SHIFT(smsc->cfg.flags);

	ret = smc_probe(pa);


	init_eth();

	return ret;
}

static TEE_Result smc_probe(void  *ioaddr)
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
				" at IOADDR %p", CARDNAME, ioaddr);
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
		DMSG("%s: IOADDR %p doesn't match configuration (%x).",
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
		DMSG("%s: IO %p: Unrecognized revision register 0x%04x"
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

	return TEE_SUCCESS;

	err_out:
		EMSG( "SMC probe failed");
		return retval;
}

/*
 * this does a soft reset on the device
 */
void smc_reset()
{
	void __iomem * ioaddr = smsc->base;
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
	//udelay(1);

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
	//SMC_WAIT_MMU_BUSY(smsc);
}

TEE_Result init_eth()
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

