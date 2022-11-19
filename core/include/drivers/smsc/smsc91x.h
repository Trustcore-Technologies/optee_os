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

#define SMC_DEBUG 4

#define CHIP_9192	3
#define CHIP_9194	4
#define CHIP_9195	5
#define CHIP_9196	6
#define CHIP_91100	7
#define CHIP_91100FD	8
#define CHIP_91111FD	9

#define NETIF_MSG_LINK	0
#define	netif_msg_link(x)		0

#ifndef SMC_CAN_USE_DATACS
#define SMC_CAN_USE_DATACS	0
#endif

#define SMC_REG(smsc, reg, bank)	(reg<<SMC_IO_SHIFT)

// Base Address Register
/* BANK 1 */
#define BASE_REG(smsc)	SMC_REG(smsc, 0x0002, 1)

// Memory Information Register
/* BANK 0  */
#define MIR_REG(smsc)		SMC_REG(smsc, 0x0008, 0)

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


// Receive/Phy Control Register
/* BANK 0  */
#define RPC_REG(smsc)		SMC_REG(smsc, 0x000A, 0)
#define RPC_SPEED	0x2000	// When 1 PHY is in 100Mbps mode.
#define RPC_DPLX	0x1000	// When 1 PHY is in Full-Duplex Mode
#define RPC_ANEG	0x0800	// When 1 PHY is in Auto-Negotiate Mode
#define RPC_LSXA_SHFT	5	// Bits to shift LS2A,LS1A,LS0A to lsb
#define RPC_LSXB_SHFT	2	// Bits to get LS2B,LS1B,LS0B to lsb

/* the normal settings for the RCR register : */
#define RCR_DEFAULT	(RCR_STRIP_CRC | RCR_RXEN)
#define RCR_CLEAR	0x0	// set it to a base state

#define RPC_DEFAULT (RPC_ANEG | RPC_SPEED | RPC_DPLX)

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
 . Receive status bits
*/
#define RS_ALGNERR	0x8000
#define RS_BRODCAST	0x4000
#define RS_BADCRC	0x2000
#define RS_ODDFRAME	0x1000
#define RS_TOOLONG	0x0800
#define RS_TOOSHORT	0x0400
#define RS_MULTICAST	0x0001
#define RS_ERRORS	(RS_ALGNERR | RS_BADCRC | RS_TOOLONG | RS_TOOSHORT)

// RX FIFO Ports Register
/* BANK 2 */
#define RXFIFO_REG(smsc)	SMC_REG(smsc, 0x0005, 2)
#define RXFIFO_REMPTY	0x80	// RX FIFO Empty

#define FIFO_REG(smsc)	SMC_REG(smsc, 0x0004, 2)

#define SMC_GET_RXFIFO(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, RXFIFO_REG(smsc)))	\
				: (SMC_inw(ioaddr, TXFIFO_REG(smsc)) >> 8))

/*
 * This selects whether TX packets are sent one by one to the SMC91x internal
 * memory and throttled until transmission completes.  This may prevent
 * RX overruns a litle by keeping much of the memory free for RX packets
 * but to the expense of reduced TX throughput and increased IRQ overhead.
 * Note this is not a cure for a too slow data bus or too high IRQ latency.
 */
#define THROTTLE_TX_PKTS	0

/*
 * The MII clock high/low times.  2x this number gives the MII clock period
 * in microseconds. (was 50, but this gives 6.4ms for each MII transaction!)
 */
#define MII_DELAY		1

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

#define SMC_insb(a, r, p, l)	io_read8s((uintptr_t)(a) + (uintptr_t)(r), p, l)
#define SMC_outsb(a, r, p, l)	io_write8s((uintptr_t)(a) + (uintptr_t)(r), p, l)
#define SMC_insw(a, r, p, l)	io_read16s((uintptr_t)(a) + (uintptr_t)(r), p, l)
#define SMC_outsw(a, r, p, l)	io_write16s((uintptr_t)(a) + (uintptr_t)(r), p, l)
#define SMC_insl(a, r, p, l)	io_read32s((uintptr_t)(a) + (uintptr_t)(r), p, l)
#define SMC_outsl(a, r, p, l)	io_write32s((uintptr_t)(a) + (uintptr_t)(r), p, l)

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

#define SMC_SET_MII(smsc, x)		SMC_outw(x, ioaddr, MII_REG(smsc))

#define SMC_GET_MII(smsc)		SMC_inw(ioaddr, MII_REG(smsc))

#define SMC_GET_MIR(smsc)		SMC_inw(ioaddr, MIR_REG(smsc))

#define SMC_SET_MIR(smsc, x)		SMC_outw(x, ioaddr, MIR_REG(smsc))

#define SMC_GET_FIFO(smsc)		SMC_inw(ioaddr, FIFO_REG(smsc))

#define SMC_SET_RPC(smsc, x)						\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(smsc))				\
			SMC_outl((x)<<16, ioaddr, SMC_REG(smsc, 8, 0));	\
		else							\
			SMC_outw(x, ioaddr, RPC_REG(smsc));		\
	} while (0)

#define SMC_GET_EPH_STATUS(smsc)	SMC_inw(ioaddr, EPH_STATUS_REG(smsc))

// EPH Status Register
/* BANK 0  */
#define EPH_STATUS_REG(smsc)	SMC_REG(smsc, 0x0002, 0)
#define ES_TX_SUC	0x0001	// Last TX was successful
#define ES_SNGL_COL	0x0002	// Single collision detected for last tx
#define ES_MUL_COL	0x0004	// Multiple collisions detected for last tx
#define ES_LTX_MULT	0x0008	// Last tx was a multicast
#define ES_16COL	0x0010	// 16 Collisions Reached
#define ES_SQET		0x0020	// Signal Quality Error Test
#define ES_LTXBRD	0x0040	// Last tx was a broadcast
#define ES_TXDEFR	0x0080	// Transmit Deferred
#define ES_LATCOL	0x0200	// Late collision detected on last tx
#define ES_LOSTCARR	0x0400	// Lost Carrier Sense
#define ES_EXC_DEF	0x0800	// Excessive Deferral
#define ES_CTR_ROL	0x1000	// Counter Roll Over indication
#define ES_LINK_OK	0x4000	// Driven by inverted value of nLNK pin
#define ES_TXUNRN	0x8000	// Tx Underrun


// PHY Configuration Register 1
#define PHY_CFG1_REG		0x10
#define PHY_CFG1_LNKDIS		0x8000	// 1=Rx Link Detect Function disabled
#define PHY_CFG1_XMTDIS		0x4000	// 1=TP Transmitter Disabled
#define PHY_CFG1_XMTPDN		0x2000	// 1=TP Transmitter Powered Down
#define PHY_CFG1_BYPSCR		0x0400	// 1=Bypass scrambler/descrambler
#define PHY_CFG1_UNSCDS		0x0200	// 1=Unscramble Idle Reception Disable
#define PHY_CFG1_EQLZR		0x0100	// 1=Rx Equalizer Disabled
#define PHY_CFG1_CABLE		0x0080	// 1=STP(150ohm), 0=UTP(100ohm)
#define PHY_CFG1_RLVL0		0x0040	// 1=Rx Squelch level reduced by 4.5db
#define PHY_CFG1_TLVL_SHIFT	2	// Transmit Output Level Adjust
#define PHY_CFG1_TLVL_MASK	0x003C
#define PHY_CFG1_TRF_MASK	0x0003	// Transmitter Rise/Fall time


// PHY Configuration Register 2
#define PHY_CFG2_REG		0x11
#define PHY_CFG2_APOLDIS	0x0020	// 1=Auto Polarity Correction disabled
#define PHY_CFG2_JABDIS		0x0010	// 1=Jabber disabled
#define PHY_CFG2_MREG		0x0008	// 1=Multiple register access (MII mgt)
#define PHY_CFG2_INTMDIO	0x0004	// 1=Interrupt signaled with MDIO pulseo

// PHY Status Output (and Interrupt status) Register
#define PHY_INT_REG		0x12	// Status Output (Interrupt Status)
#define PHY_INT_INT		0x8000	// 1=bits have changed since last read
#define PHY_INT_LNKFAIL		0x4000	// 1=Link Not detected
#define PHY_INT_LOSSSYNC	0x2000	// 1=Descrambler has lost sync
#define PHY_INT_CWRD		0x1000	// 1=Invalid 4B5B code detected on rx
#define PHY_INT_SSD		0x0800	// 1=No Start Of Stream detected on rx
#define PHY_INT_ESD		0x0400	// 1=No End Of Stream detected on rx
#define PHY_INT_RPOL		0x0200	// 1=Reverse Polarity detected
#define PHY_INT_JAB		0x0100	// 1=Jabber detected
#define PHY_INT_SPDDET		0x0080	// 1=100Base-TX mode, 0=10Base-T mode
#define PHY_INT_DPLXDET		0x0040	// 1=Device in Full Duplex

// PHY Interrupt/Status Mask Register
#define PHY_MASK_REG		0x13	// Interrupt Mask
// Uses the same bit definitions as PHY_INT_REG

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

// Management Interface Register (MII)
/* BANK 3 */
#define MII_REG(smsc)		SMC_REG(smsc, 0x0008, 3)
#define MII_MSK_CRS100	0x4000 // Disables CRS100 detection during tx half dup
#define MII_MDOE	0x0008 // MII Output Enable
#define MII_MCLK	0x0004 // MII Clock, pin MDCLK
#define MII_MDI		0x0002 // MII Input, pin MDI
#define MII_MDO		0x0001 // MII Output, pin MDO

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

#define SMC_GET_INT(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, INT_REG(smsc)))	\
				: (SMC_inw(ioaddr, INT_REG(smsc)) & 0xFF))

#define SMC_ACK_INT(smsc, x)						\
	do {								\
		if (SMC_8BIT(smsc))					\
			SMC_outb(x, ioaddr, INT_REG(smsc));		\
		else {							\
			int __mask;					\
			__mask = SMC_inw(ioaddr, INT_REG(smsc)) & ~0xff; \
			SMC_outw(__mask | (x), ioaddr, INT_REG(smsc));	\
		}							\
	} while (0)

#define SMC_GET_INT_MASK(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, IM_REG(smsc)))	\
				: (SMC_inw(ioaddr, INT_REG(smsc)) >> 8))

#define SMC_SET_INT_MASK(smsc, x)					\
	do {								\
		if (SMC_8BIT(smsc))					\
			SMC_outb(x, ioaddr, IM_REG(smsc));		\
		else							\
			SMC_outw((x) << 8, ioaddr, INT_REG(smsc));	\
	} while (0)

#define SMC_GET_AR(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, AR_REG(smsc)))	\
				: (SMC_inw(ioaddr, PN_REG(smsc)) >> 8))

// Allocation Result Register
/* BANK 2 */
#define AR_REG(smsc)		SMC_REG(smsc, 0x0003, 2)
#define AR_FAILED	0x80	// Alocation Failed

// Data Register
/* BANK 2 */
#define DATA_REG(smsc)	SMC_REG(smsc, 0x0008, 2)

// Packet Number Register
/* BANK 2 */
#define PN_REG(smsc)		SMC_REG(smsc, 0x0002, 2)

#define SMC_GET_PN(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, PN_REG(smsc)))	\
				: (SMC_inw(ioaddr, PN_REG(smsc)) & 0xFF))

#define SMC_SET_PN(smsc, x)						\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(smsc))				\
			SMC_outl((x)<<16, ioaddr, SMC_REG(smsc, 0, 2));	\
		else if (SMC_8BIT(smsc))				\
			SMC_outb(x, ioaddr, PN_REG(smsc));		\
		else							\
			SMC_outw(x, ioaddr, PN_REG(smsc));		\
	} while (0)

// TX FIFO Ports Register
/* BANK 2 */
#define TXFIFO_REG(smsc)	SMC_REG(smsc, 0x0004, 2)
#define TXFIFO_TEMPTY	0x80	// TX FIFO Empty

#define SMC_GET_TXFIFO(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, TXFIFO_REG(smsc)))	\
				: (SMC_inw(ioaddr, TXFIFO_REG(smsc)) & 0xFF))

// Pointer Register
/* BANK 2 */
#define PTR_REG(smsc)		SMC_REG(smsc, 0x0006, 2)
#define PTR_RCV		0x8000 // 1=Receive area, 0=Transmit area
#define PTR_AUTOINC 	0x4000 // Auto increment the pointer on each access
#define PTR_READ	0x2000 // When 1 the operation is a read

#define SMC_GET_PTR(smsc)		SMC_inw(ioaddr, PTR_REG(smsc))


#define SMC_GET_INT_MASK(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, IM_REG(smsc)))	\
				: (SMC_inw(ioaddr, INT_REG(smsc)) >> 8))

#define SMC_SET_INT_MASK(smsc, x)					\
	do {								\
		if (SMC_8BIT(smsc))					\
			SMC_outb(x, ioaddr, IM_REG(smsc));		\
		else							\
			SMC_outw((x) << 8, ioaddr, INT_REG(smsc));	\
	} while (0)

#define SMC_GET_INT(smsc)						\
	(SMC_8BIT(smsc)	? (SMC_inb(ioaddr, INT_REG(smsc)))	\
				: (SMC_inw(ioaddr, INT_REG(smsc)) & 0xFF))

#define SMC_ACK_INT(smsc, x)						\
	do {								\
		if (SMC_8BIT(smsc))					\
			SMC_outb(x, ioaddr, INT_REG(smsc));		\
		else {							\
			unsigned long __flags;				\
			int __mask;					\
			__mask = SMC_inw(ioaddr, INT_REG(smsc)) & ~0xff; \
			SMC_outw(__mask | (x), ioaddr, INT_REG(smsc));	\
		}							\
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

#define SMC_SET_MAC_ADDR(smsc, addr)					\
	do {								\
		SMC_outw(addr[0]|(addr[1] << 8), ioaddr, ADDR0_REG(smsc)); \
		SMC_outw(addr[2]|(addr[3] << 8), ioaddr, ADDR1_REG(smsc)); \
		SMC_outw(addr[4]|(addr[5] << 8), ioaddr, ADDR2_REG(smsc)); \
	} while (0)

#define SMC_PUT_PKT_HDR(smsc, status, length)				\
	do {								\
		if (SMC_32BIT(smsc))					\
			SMC_outl((status) | (length)<<16, ioaddr,	\
				 DATA_REG(smsc));			\
		else {							\
			SMC_outw(status, ioaddr, DATA_REG(smsc));	\
			SMC_outw(length, ioaddr, DATA_REG(smsc));	\
		}							\
	} while (0)

#define SMC_GET_PKT_HDR(smsc, status, length)				\
	do {								\
		if (SMC_32BIT(smsc)) {				\
			unsigned int __val = SMC_inl(ioaddr, DATA_REG(smsc)); \
			(status) = __val & 0xffff;			\
			(length) = __val >> 16;				\
		} else {						\
			(status) = SMC_inw(ioaddr, DATA_REG(smsc));	\
			(length) = SMC_inw(ioaddr, DATA_REG(smsc));	\
		}							\
	} while (0)

#define SMC_PUSH_DATA(smsc, p, l)					\
	do {								\
		if (SMC_32BIT(smsc)) {				\
			void *__ptr = (p);				\
			int __len = (l);				\
			void __iomem *__ioaddr = ioaddr;		\
			if (__len >= 2 && (unsigned long)__ptr & 2) {	\
				__len -= 2;				\
				SMC_outw(*(u16 *)__ptr, ioaddr,		\
					DATA_REG(smsc));		\
				__ptr += 2;				\
			}						\
			if (SMC_CAN_USE_DATACS && smsc->datacs)		\
				__ioaddr = smsc->datacs;			\
			SMC_outsl(__ioaddr, DATA_REG(smsc), __ptr, __len>>2); \
			if (__len & 2) {				\
				__ptr += (__len & ~3);			\
				SMC_outw(*((u16 *)__ptr), ioaddr,	\
					 DATA_REG(smsc));		\
			}						\
		} else if (SMC_16BIT(smsc))				\
			SMC_outsw(ioaddr, DATA_REG(smsc), p, (l) >> 1);	\
		else if (SMC_8BIT(smsc))				\
			SMC_outsb(ioaddr, DATA_REG(smsc), p, l);	\
	} while (0)

#define SMC_PULL_DATA(smsc, p, l)					\
	do {								\
		if (SMC_32BIT(smsc)) {				\
			void *__ptr = (p);				\
			int __len = (l);				\
			void __iomem *__ioaddr = ioaddr;		\
			if ((unsigned long)__ptr & 2) {			\
				/*					\
				 * We want 32bit alignment here.	\
				 * Since some buses perform a full	\
				 * 32bit fetch even for 16bit data	\
				 * we can't use SMC_inw() here.		\
				 * Back both source (on-chip) and	\
				 * destination pointers of 2 bytes.	\
				 * This is possible since the call to	\
				 * SMC_GET_PKT_HDR() already advanced	\
				 * the source pointer of 4 bytes, and	\
				 * the skb_reserve(skb, 2) advanced	\
				 * the destination pointer of 2 bytes.	\
				 */					\
				__ptr -= 2;				\
				__len += 2;				\
				SMC_SET_PTR(smsc,			\
					2|PTR_READ|PTR_RCV|PTR_AUTOINC); \
			}						\
			if (SMC_CAN_USE_DATACS && smsc->datacs)		\
				__ioaddr = smsc->datacs;			\
			__len += 2;					\
			SMC_insl(__ioaddr, DATA_REG(smsc), __ptr, __len>>2); \
		} else if (SMC_16BIT(smsc))				\
			SMC_insw(ioaddr, DATA_REG(smsc), p, (l) >> 1);	\
		else if (SMC_8BIT(smsc))				\
			SMC_insb(ioaddr, DATA_REG(smsc), p, l);		\
	} while (0)

#define SMC_SET_PTR(smsc, x)						\
	do {								\
		if (SMC_MUST_ALIGN_WRITE(smsc))				\
			SMC_outl((x)<<16, ioaddr, SMC_REG(smsc, 4, 2));	\
		else							\
			SMC_outw(x, ioaddr, PTR_REG(smsc));		\
	} while (0)

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

enum netdev_link_state {
	NETDEV_STATE_NOCARRIER = 0,
	NETDEV_LINK_STATE_PRESENT,
};


struct mii_if_info {
	int phy_id;
	int advertising;
	int phy_id_mask;
	int reg_num_mask;

	unsigned int full_duplex : 1;	/* is full duplex? */
	unsigned int force_media : 1;	/* is autoneg. disabled? */
	unsigned int supports_gmii : 1; /* are GMII registers supported? */

	enum netdev_link_state link_state;
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

	/* Contains the current active transmission mode */
	int	tcr_cur_mode;

	/* Contains the current active receive mode */
	int	rcr_cur_mode;

	/* Contains the current active receive/phy mode */
	int	rpc_cur_mode;
	int	ctl_rfduplx;
	int	ctl_rspeed;

	u32	msg_enable;
	u32	phy_type;
	struct mii_if_info mii;

	spinlock_t lock;
	u8 mac_addr[6];
//
//#ifdef CONFIG_ARCH_PXA
//	/* DMA needs the physical address of the chip */
//	u_long physaddr;
//	struct device *device;
//#endif
	paddr_t __iomem base;
	paddr_t __iomem datacs;
//

	/* the low address lines on some platforms aren't connected... */
	int	io_shift;

	char name[IF_NAMESIZE + 1];

	struct smc91x_platdata cfg;
};

void send_test_pkt(void);

#endif /* SMC91X_H_ */
