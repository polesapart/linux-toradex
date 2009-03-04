/* linux/arch/arm/mach-s3c24xx/sleep-s3c2443.S
 *
 * $Id: sleep-s3c2443.S,v 1.3 2006/11/08 01:07:53 eyryu Exp $
 *
 * Copyright (c) 2006 Samsung Electronics
 *	Ryu Euiyoul <steven.ryu@samsung.com>
 *
 * S3C Power Manager (Suspend-To-RAM) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

@ #include <linux/config.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>


#if 0
#define ENABLE_GPIO_TOGGLE
#endif
	
	.text

/* s3c_cpu_suspend
 *
 * put the cpu into sleep mode
 *
 * entry:
 *	r0 = sleep save block
*/

ENTRY(s3c2443_cpu_suspend)
	stmfd	sp!, { r4 - r12, lr }	@ save registers on stack

	@@ get co-processor registers
	mrc	p15, 0, r4, c3, c0, 0	@ Domain ID
	mrc	p15, 0, r5, c2, c0, 0	@ translation table base address
	mrc	p15, 0, r6, c13, c0, 0	@ PID
	mrc	p15, 0, r7, c1, c0, 0	@ control register

	@ store them plus current virtual stact ptr on stack
	mov	r8, sp
	stmfd	sp!, {r4 - r8}


	mov	r1, #0	
	mcr	p15, 0, r1, c8, c7, 0		@ flush I+D TLBs
	mcr	p15, 0, r1, c7, c7, 0		@ flush I & D caches
	mcr     p15, 0, r1, c9, c0, 0           @ invalidate RB
	
	mov	r0, sp
  	bl	sleep_phys_sp
	ldr	r1, =sleep_save_sp
	str	r0, [r1]

        @@ flush the caches to ensure everything is back out to
        @@ SDRAM before the core powers down

        bl      arm920_flush_kern_cache_all
	
	b	s3c_goto_sleep

	.align	5


 	/* 
	 * Before entering the sleep mode we must store the jump address in the
 	 * configuration register INFORM1
	 */	
s3c_goto_sleep:
        ldr     r0, =0xf4100040 @ =S3C_PWRMODE
        ldr     r1, =0x2BED
        str     r1, [r0]

@ _sleeping:
@ 	nop
@ 	nop
@ 	nop
@ 	b	_sleeping
	.ltorg

	/* s3c_cpu_resume
	 *
	 * resume code entry for bootloader to call
	 *
	 * we must put this code here in the data segment as we have no
	 * other way of restoring the stack pointer after sleep, and we
	 * must not write to the code segment (code is read-only)
	*/
	.data
	.align 5 
ENTRY(s3c2443_cpu_resume)

        @@@@@@@@@ The U-Boot set the registers: r10 -> GPLDAT
#if defined(ENABLE_GPIO_TOGGLE)
        adr     r11, _gplcon_virtual_address
        ldr     r10, [r11]
        mov     r5, #50
_toggle_gpio:
        ldr     r6, [r10]
        eor     r6, r6, #0x800
        str     r6, [r10]
        mov     r4, #0x800000
_dec_timer:
        sub     r4, r4, #1
        cmp     r4, #0
        bne     _dec_timer

        sub     r5, r5, #1
        cmp     r5, #0
        bne     _toggle_gpio
#endif /* ENABLE_GPIO_TOGGLE */
        @@@@@@@@@ The U-Boot set the registers: r5 -> GPLDAT

	
	mov	r0, #PSR_I_BIT | PSR_F_BIT | SVC_MODE
	msr	cpsr_c, r0

	ldr	r0, sleep_save_sp	@ address of restore block
	ldr	r2, =resume_after_mmu
	ldmfd	r0, {r4 - r7, sp}
	
	mov	r1, #0	
	mcr	p15, 0, r1, c8, c7, 0		@ flush I+D TLBs
	mcr	p15, 0, r1, c7, c7, 0		@ flush I & D caches
	mcr     p15, 0, r1, c9, c0, 0           @ invalidate RB
	mcr     p15, 0, r1, c9, c0, 5           @ allow user space to use RB

	mcr	p15, 0, r4, c3, c0, 0		@ Domain ID
	mcr	p15, 0, r5, c2, c0, 0		@ translation table base
	mcr	p15, 0, r6, c13, c0, 0		@ PID
	b       resume_turn_on_mmu              @ cache align execution

	.align 5 
resume_turn_on_mmu:
        mcr     p15, 0, r7, c1, c0, 0           @ turn on MMU, caches, etc.
	nop 
	mov	pc, r2
	nop
	nop
	nop

	.ltorg
sleep_save_sp:
	.word	0

        @@ DEBUG: Virtual address of the register GPLDAT
_gplcon_virtual_address:
	@ .word   0xfb0000f4
        .word	0x560000f4
	
	.text
resume_after_mmu:
        ldmfd   sp!, { r4 - r12, pc }

