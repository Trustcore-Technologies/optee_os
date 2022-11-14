/*
 * smc91x.h
 *
 *  Created on: Nov 10, 2022
 *      Author: rami
 */

#ifndef SMC91X_H_
#define SMC91X_H_

#include <stdint.h>

#include <mm/core_mmu.h>

#include <drivers/smsc/napi.h>

#include <kernel/interrupt.h>

#include <kernel/dt.h>

#include "skbuff_stub.h"

#include "sw_fec_state.h"

#include <io.h>

#include <drivers/smsc/bstgw_pktq.h>


#define CARDNAME "smc91x"

#define SMC_REG(smsc, reg, bank)	(reg<<SMC_IO_SHIFT)

// Base Address Register
/* BANK 1 */
#define BASE_REG(lp)	SMC_REG(lp, 0x0002, 1)

// Revision Register
/* BANK 3 */
/* ( hi: chip id   low: rev # ) */
#define REV_REG(lp)		SMC_REG(lp, 0x000A, 3)

#define SMC_GET_REV(lp)		SMC_inw(ioaddr, REV_REG(lp))

#define SMC_CAN_USE_8BIT	1
#define SMC_CAN_USE_16BIT	1
#define SMC_CAN_USE_32BIT	1

#define SMC91X_USE_8BIT (1 << 0)
#define SMC91X_USE_16BIT (1 << 1)
#define SMC91X_USE_32BIT (1 << 2)

#define SMC91X_NOWAIT		(1 << 3)

/* two bits for IO_SHIFT, let's hope later designs will keep this sane */
#define SMC91X_IO_SHIFT_0	(0 << 4)
#define SMC91X_IO_SHIFT_1	(1 << 4)
#define SMC91X_IO_SHIFT_2	(2 << 4)
#define SMC91X_IO_SHIFT_3	(3 << 4)
#define SMC91X_IO_SHIFT(x)	(((x) >> 4) & 0x3)

#define SMC91X_USE_DMA		(1 << 6)

#define RPC_LED_100_10	(0x00)	/* LED = 100Mbps OR's with 10Mbps link detect */
#define RPC_LED_RES	(0x01)	/* LED = Reserved */
#define RPC_LED_10	(0x02)	/* LED = 10Mbps link detect */
#define RPC_LED_FD	(0x03)	/* LED = Full Duplex Mode */
#define RPC_LED_TX_RX	(0x04)	/* LED = TX or RX packet occurred */
#define RPC_LED_100	(0x05)	/* LED = 100Mbps link dectect */
#define RPC_LED_TX	(0x06)	/* LED = TX packet occurred */
#define RPC_LED_RX	(0x07)	/* LED = RX packet occurred */

#define SMC_inb(a, r)		io_read8((a) + (r))
#define SMC_inw(a, r)		io_read16((a) + (r))
#define SMC_inl(a, r)		io_read32((a) + (r))
#define SMC_outb(v, a, r)	io_write8((a) + (r), v)
#define SMC_outw(v, a, r)	io_write16((a) + (r), v)
#define SMC_outl(v, a, r)	io_write32((a) + (r), v)
//#define SMC_insw(a, r, p, l)	readsw((a) + (r), p, l)
//#define SMC_outsw(a, r, p, l)	writesw((a) + (r), p, l)
//#define SMC_insl(a, r, p, l)	readsl((a) + (r), p, l)
//#define SMC_outsl(a, r, p, l)	writesl((a) + (r), p, l)


#define SMC_IO_SHIFT		(smsc->io_shift)

#define SMC_8BIT(p)	((p)->cfg.flags & SMC91X_USE_8BIT)
#define SMC_16BIT(p)	((p)->cfg.flags & SMC91X_USE_16BIT)
#define SMC_32BIT(p)	((p)->cfg.flags & SMC91X_USE_32BIT)

/*
 * Hack Alert: Some setups just can't write 8 or 16 bits reliably when not
 * aligned to a 32 bit boundary.  I tell you that does exist!
 * Fortunately the affected register accesses can be easily worked around
 * since we can write zeroes to the preceeding 16 bits without adverse
 * effects and use a 32-bit access.
 *
 * Enforce it on any 32-bit capable setup for now.
 */
#define SMC_MUST_ALIGN_WRITE(lp)	SMC_32BIT(lp)

#define SMC_GET_BASE(lp)		SMC_inw(ioaddr, BASE_REG(lp))

/*
 . Bank Select Register:
 .
 .		yyyy yyyy 0000 00xx
 .		xx 		= bank number
 .		yyyy yyyy	= 0x33, for identification purposes.
*/
#define BANK_SELECT		(14 << SMC_IO_SHIFT)

#define SMC_CURRENT_BANK(lp)	SMC_inw(ioaddr, BANK_SELECT)

#define SMC_SELECT_BANK(lp, x)					\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(lp))				\
			SMC_outl((x)<<16, ioaddr, 12<<SMC_IO_SHIFT);	\
		else							\
			SMC_outw(x, ioaddr, BANK_SELECT);		\
	} while (0)

typedef unsigned int spinlock_t;

/* Maximum number of queues supported */
#define FEC_ENET_MAX_TX_QS	1
#define FEC_ENET_MAX_RX_QS	1

/* We assume 2 secure IRQs (plus 1 virtual IRQ for NW pass-on) */
#define FEC_IRQ_NUM		3

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef unsigned short ushort;
typedef unsigned int uint;

struct smc91x_platdata {
	unsigned long flags;
	unsigned char leda;
	unsigned char ledb;
};

struct smc_local {
//	/*
//	 * If I have to wait until memory is available to send a
//	 * packet, I will store the skbuff here, until I get the
//	 * desired memory.  Then, I'll send it out and free it.
//	 */
//	struct sk_buff *pending_tx_skb;
//#if 0
//	struct tasklet_struct tx_task;
//#endif
//
//	/* version/revision of the SMC91x chip */
//	int	version;
//
//	/* Contains the current active transmission mode */
//	int	tcr_cur_mode;
//
//	/* Contains the current active receive mode */
//	int	rcr_cur_mode;
//
//	/* Contains the current active receive/phy mode */
//	int	rpc_cur_mode;
//	int	ctl_rfduplx;
//	int	ctl_rspeed;
//
//	u32	msg_enable;
//	u32	phy_type;
//	struct mii_if_info mii;
//
//	/* work queue */
//	struct work_struct phy_configure;
//	struct net_device *dev;
//	int	work_pending;
//
//	spinlock_t lock;
//
//#ifdef CONFIG_ARCH_PXA
//	/* DMA needs the physical address of the chip */
//	u_long physaddr;
//	struct device *device;
//#endif
//	void __iomem *base;
//	void __iomem *datacs;
//

	/* the low address lines on some platforms aren't connected... */
	int	io_shift;

	struct smc91x_platdata cfg;
};



#endif /* SMC91X_H_ */
