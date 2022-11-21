/*
 * smsc91x_defs.h
 *
 *  Created on: Nov 14, 2022
 *      Author: rami
 */

#ifndef SMSC91X_DEFS_H_
#define SMSC91X_DEFS_H_

#define SMC_inb(a, r)		io_read8((uintptr_t)(a) + (uintptr_t)(r))
#define SMC_inw(a, r)		io_read16((uintptr_t)(a) + (uintptr_t)(r))
#define SMC_inl(a, r)		io_read32((uintptr_t)(a) + (uintptr_t)(r))
#define SMC_outb(v, a, r)	io_write8((uintptr_t)(a) + (uintptr_t)(r), v)
#define SMC_outw(v, a, r)	io_write16((uintptr_t)(a) + (uintptr_t)(r), v)
#define SMC_outl(v, a, r)	io_write32((uintptr_t)(a) + (uintptr_t)(r), v)




#endif /* SMSC91X_DEFS_H_ */
