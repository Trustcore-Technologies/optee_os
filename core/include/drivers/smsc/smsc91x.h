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

#define __iomem

#define CARDNAME "smc91x"

#define SMC_REG(smsc, reg, bank)	(reg<<SMC_IO_SHIFT)

// Base Address Register
/* BANK 1 */
#define BASE_REG(smsc)	SMC_REG(smsc, 0x0002, 1)


// Receive Control Register
/* BANK 0  */
#define RCR_REG(smsc)		SMC_REG(smsc, 0x0004, 0)
#define RCR_RX_ABORT	0x0001	// Set if a rx frame was aborted
#define RCR_PRMS	0x0002	// Enable promiscuous mode
#define RCR_ALMUL	0x0004	// When set accepts all multicast frames
#define RCR_RXEN	0x0100	// IFF this is set, we can receive packets
#define RCR_STRIP_CRC	0x0200	// When set strips CRC from rx packets
#define RCR_ABORT_ENB	0x0200	// When set will abort rx on collision
#define RCR_FILT_CAR	0x0400	// When set filters leading 12 bit s of carrier
#define RCR_SOFTRST	0x8000 	// resets the chip

/* the normal settings for the RCR register : */
#define RCR_DEFAULT	(RCR_STRIP_CRC | RCR_RXEN)
#define RCR_CLEAR	0x0	// set it to a base state


// Transmit Control Register
/* BANK 0  */
#define TCR_REG(smsc) 	SMC_REG(smsc, 0x0000, 0)
#define TCR_ENABLE	0x0001	// When 1 we can transmit
#define TCR_LOOP	0x0002	// Controls output pin LBK
#define TCR_FORCOL	0x0004	// When 1 will force a collision
#define TCR_PAD_EN	0x0080	// When 1 will pad tx frames < 64 bytes w/0
#define TCR_NOCRC	0x0100	// When 1 will not append CRC to tx frames
#define TCR_MON_CSN	0x0400	// When 1 tx monitors carrier
#define TCR_FDUPLX    	0x0800  // When 1 enables full duplex operation
#define TCR_STP_SQET	0x1000	// When 1 stops tx if Signal Quality Error
#define TCR_EPH_LOOP	0x2000	// When 1 enables EPH block loopback
#define TCR_SWFDUP	0x8000	// When 1 enables Switched Full Duplex mode

#define TCR_CLEAR	0	/* do NOTHING */
/* the default settings for the TCR register : */
#define TCR_DEFAULT	(TCR_ENABLE | TCR_PAD_EN)


// Control Register
/* BANK 1 */
#define CTL_REG(smsc)		SMC_REG(smsc, 0x000C, 1)
#define CTL_RCV_BAD	0x4000 // When 1 bad CRC packets are received
#define CTL_AUTO_RELEASE 0x0800 // When 1 tx pages are released automatically
#define CTL_LE_ENABLE	0x0080 // When 1 enables Link Error interrupt
#define CTL_CR_ENABLE	0x0040 // When 1 enables Counter Rollover interrupt
#define CTL_TE_ENABLE	0x0020 // When 1 enables Transmit Error interrupt
#define CTL_EEPROM_SELECT 0x0004 // Controls EEPROM reload & store
#define CTL_RELOAD	0x0002 // When set reads EEPROM into registers
#define CTL_STORE	0x0001 // When set stores registers into EEPROM


// Configuration Reg
/* BANK 1 */
#define CONFIG_REG(smsc)	SMC_REG(smsc, 0x0000,	1)
#define CONFIG_EXT_PHY	0x0200	// 1=external MII, 0=internal Phy
#define CONFIG_GPCNTRL	0x0400	// Inverse value drives pin nCNTRL
#define CONFIG_NO_WAIT	0x1000	// When 1 no extra wait states on ISA bus
#define CONFIG_EPH_POWER_EN 0x8000 // When 0 EPH is placed into low power mode.

// Default is powered-up, Internal Phy, Wait States, and pin nCNTRL=low
#define CONFIG_DEFAULT	(CONFIG_EPH_POWER_EN)


// MMU Command Register
/* BANK 2 */
#define MMU_CMD_REG(smsc)	SMC_REG(smsc, 0x0000, 2)
#define MC_BUSY		1	// When 1 the last release has not completed
#define MC_NOP		(0<<5)	// No Op
#define MC_ALLOC	(1<<5) 	// OR with number of 256 byte packets
#define MC_RESET	(2<<5)	// Reset MMU to initial state
#define MC_REMOVE	(3<<5) 	// Remove the current rx packet
#define MC_RELEASE  	(4<<5) 	// Remove and release the current rx packet
#define MC_FREEPKT  	(5<<5) 	// Release packet in PNR register
#define MC_ENQUEUE	(6<<5)	// Enqueue the packet for transmit
#define MC_RSTTXFIFO	(7<<5)	// Reset the TX FIFOs


/*
 * This selects whether TX packets are sent one by one to the SMC91x internal
 * memory and throttled until transmission completes.  This may prevent
 * RX overruns a litle by keeping much of the memory free for RX packets
 * but to the expense of reduced TX throughput and increased IRQ overhead.
 * Note this is not a cure for a too slow data bus or too high IRQ latency.
 */
#define THROTTLE_TX_PKTS	0

// Revision Register
/* BANK 3 */
/* ( hi: chip id   low: rev # ) */
#define REV_REG(smsc)		SMC_REG(smsc, 0x000A, 3)

#define SMC_GET_REV(smsc)		SMC_inw(ioaddr, REV_REG(smsc))

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

#define SMC_inb(a, r)		io_read8((uintptr_t)(a) + (uintptr_t)(r))
#define SMC_inw(a, r)		io_read16((uintptr_t)(a) + (uintptr_t)(r))
#define SMC_inl(a, r)		io_read32((uintptr_t)(a) + (uintptr_t)(r))
#define SMC_outb(v, a, r)	io_write8((uintptr_t)(a) + (uintptr_t)(r), v)
#define SMC_outw(v, a, r)	io_write16((uintptr_t)(a) + (uintptr_t)(r), v)
#define SMC_outl(v, a, r)	io_write32((uintptr_t)(a) + (uintptr_t)(r), v)
//#define SMC_insw(a, r, p, l)	readsw((a) + (r), p, l)
//#define SMC_outsw(a, r, p, l)	writesw((a) + (r), p, l)
//#define SMC_insl(a, r, p, l)	readsl((a) + (r), p, l)
//#define SMC_outsl(a, r, p, l)	writesl((a) + (r), p, l)


#define SMC_IO_SHIFT		(smsc->io_shift)

#define SMC_8BIT(smsc)	((smsc)->cfg.flags & SMC91X_USE_8BIT)
#define SMC_16BIT(smsc)	((smsc)->cfg.flags & SMC91X_USE_16BIT)
#define SMC_32BIT(smsc)	((smsc)->cfg.flags & SMC91X_USE_32BIT)

/*
 * Hack Alert: Some setups just can't write 8 or 16 bits reliably when not
 * aligned to a 32 bit boundary.  I tell you that does exist!
 * Fortunately the affected register accesses can be easily worked around
 * since we can write zeroes to the preceeding 16 bits without adverse
 * effects and use a 32-bit access.
 *
 * Enforce it on any 32-bit capable setup for now.
 */
#define SMC_MUST_ALIGN_WRITE(smsc)	SMC_32BIT(smsc)

#define SMC_GET_BASE(smsc)		SMC_inw(ioaddr, BASE_REG(smsc))

/*
 . Bank Select Register:
 .
 .		yyyy yyyy 0000 00xx
 .		xx 		= bank number
 .		yyyy yyyy	= 0x33, for identification purposes.
*/
#define BANK_SELECT		(14 << SMC_IO_SHIFT)

#define SMC_CURRENT_BANK(smsc)	SMC_inw(ioaddr, BANK_SELECT)

#define SMC_SELECT_BANK(smsc, x)					\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(smsc))				\
			SMC_outl((x)<<16, ioaddr, 12<<SMC_IO_SHIFT);	\
		else							\
			SMC_outw(x, ioaddr, BANK_SELECT);		\
	} while (0)


#define SMC_REG(smsc, reg, bank)	(reg<<SMC_IO_SHIFT)

#define SMC_GET_CTL(smsc)		SMC_inw(ioaddr, CTL_REG(smsc))

#define SMC_SET_CTL(smsc, x)		SMC_outw(x, ioaddr, CTL_REG(smsc))

#define SMC_SET_RCR(smsc, x)		SMC_outw(x, ioaddr, RCR_REG(smsc))

#define SMC_SET_TCR(smsc, x)		SMC_outw(x, ioaddr, TCR_REG(smsc))

#define SMC_SET_CONFIG(smsc, x)	SMC_outw(x, ioaddr, CONFIG_REG(smsc))

#define SMC_SET_MMU_CMD(smsc, x)	SMC_outw(x, ioaddr, MMU_CMD_REG(smsc))

// Interrupt Mask Register
/* BANK 2 */
#define IM_REG(smsc)		SMC_REG(smsc, 0x000D, 2)
#define IM_MDINT	0x80 // PHY MI Register 18 Interrupt
#define IM_ERCV_INT	0x40 // Early Receive Interrupt
#define IM_EPH_INT	0x20 // Set by Ethernet Protocol Handler section
#define IM_RX_OVRN_INT	0x10 // Set by Receiver Overruns
#define IM_ALLOC_INT	0x08 // Set when allocation request is completed
#define IM_TX_EMPTY_INT	0x04 // Set if the TX FIFO goes empty
#define IM_TX_INT	0x02 // Transmit Interrupt
#define IM_RCV_INT	0x01 // Receive Interrupt


// Interrupt Status/Acknowledge Register
/* BANK 2 */
#define INT_REG(smsc)		SMC_REG(smsc, 0x000C, 2)

#define SMC_SET_INT_MASK(smsc, x)					\
	do {								\
		if (SMC_8BIT(smsc))					\
			SMC_outb(x, ioaddr, IM_REG(smsc));		\
		else							\
			SMC_outw((x) << 8, ioaddr, INT_REG(smsc));	\
	} while (0)

// Individual Address Registers
/* BANK 1 */
#define ADDR0_REG(smsc)	SMC_REG(smsc, 0x0004, 1)
#define ADDR1_REG(smsc)	SMC_REG(smsc, 0x0006, 1)
#define ADDR2_REG(smsc)	SMC_REG(smsc, 0x0008, 1)

#ifndef SMC_GET_MAC_ADDR
#define SMC_GET_MAC_ADDR(smsc, addr)					\
	do {								\
		unsigned int __v;					\
		__v = SMC_inw(ioaddr, ADDR0_REG(smsc));			\
		addr[0] = __v; addr[1] = __v >> 8;			\
		__v = SMC_inw(ioaddr, ADDR1_REG(smsc));			\
		addr[2] = __v; addr[3] = __v >> 8;			\
		__v = SMC_inw(ioaddr, ADDR2_REG(smsc));			\
		addr[4] = __v; addr[5] = __v >> 8;			\
	} while (0)
#endif


typedef unsigned int spinlock_t;

#define IF_NAMESIZE 		(16)

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
	struct sk_buff *pending_tx_skb;

	/* version/revision of the SMC91x chip */
	int	version;
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
	spinlock_t lock;
	u8 mac_addr[6];
//
//#ifdef CONFIG_ARCH_PXA
//	/* DMA needs the physical address of the chip */
//	u_long physaddr;
//	struct device *device;
//#endif
	void __iomem *base;
//	void __iomem *datacs;
//

	/* the low address lines on some platforms aren't connected... */
	int	io_shift;

	char name[IF_NAMESIZE + 1];

	struct smc91x_platdata cfg;
};


#endif /* SMC91X_H_ */
