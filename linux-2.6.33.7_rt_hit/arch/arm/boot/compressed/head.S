/*
 *  linux/arch/arm/boot/compressed/head.S
 *
 *  Copyright (C) 1996-2002 Russell King
 *  Copyright (C) 2004 Hyok S. Choi (MPU support)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>

		.macro	kputc, val
		mov	r0, \val
1:		ldr r2, =0xC0016000
		ldrh r3, [r2, #12]
		tst r3, #512
		bne 1b
		uxth r3, r0
		strh r3, [r2, #16]
2:		ldr r2, =0xC0016000
		ldrh r3, [r2, #12]
		tst r3, #512
		bne 2b
		.endm

		.macro	kputh, val
		mov	r0, \val
		mov r1, #7
3:		ldr r2, =0xC0016000
		lsl r3, r1, #2
		ldrh r2, [r2, #12]
		asr r3, r0, r3
		tst r2, #512
		and r3, r3, #15
		bne 3b
		cmp r3, #9
		add r2, r3, #55
		addle r2, r3, #48
		ldr r3, =0xC0016000
		subs r1, r1, #1
		strh r2, [r3, #16]
		bpl 3b
4:		ldr r3, =0xC0016000
		ldrh r3, [r3, #12]
		tst r3, #512
		bne 4b
		.endm

		.section ".start", #alloc, #execinstr
/*
 * sort out different calling conventions
 */
		.align
start:
		.type	start,#function
		b	1f
		.word	0x016f2818		@ Magic numbers to help the loader
		.word	start			@ absolute load/run zImage address
		.word	_edata			@ zImage end address
1:		mov	r7, r1			@ save architecture ID
		mov	r8, r2			@ save atags pointer

		mrs	r2, cpsr		@ turn off interrupts to
		orr	r2, r2, #0xc0		@ prevent angel from running
		msr	cpsr_c, r2

		.text
		adr	r0, LC0
 ARM(		ldmia	r0, {r1, r2, r3, r4, r5, r6, r11, ip, sp})
 THUMB(		ldmia	r0, {r1, r2, r3, r4, r5, r6, r11, ip}	)
 THUMB(		ldr	sp, [r0, #32]				)
		subs	r0, r0, r1		@ calculate the delta offset

		/*
		 * We're running at a different address.  We need to fix
		 * up various pointers:
		 *   r5 - zImage base address (_start)
		 *   r6 - size of decompressed image
		 *   r11 - GOT start
		 *   ip - GOT end
		 */
		add	r5, r5, r0
		add	r11, r11, r0
		add	ip, ip, r0

		/*
		 *   r2 - BSS start
		 *   r3 - BSS end
		 *   sp - stack pointer
		 */
		add	r2, r2, r0
		add	r3, r3, r0
		add	sp, sp, r0

		/*
		 * Relocate all entries in the GOT table.
		 */
1:		ldr	r1, [r11, #0]		@ relocate entries in the GOT
		add	r1, r1, r0		@ table.  This fixes up the
		str	r1, [r11], #4		@ C references.
		cmp	r11, ip
		blo	1b

not_relocated:	mov	r0, #0
1:		str	r0, [r2], #4		@ clear bss
		str	r0, [r2], #4
		str	r0, [r2], #4
		str	r0, [r2], #4
		cmp	r2, r3
		blo	1b

		/*
		 * The C runtime environment should now be setup
		 * sufficiently.  Turn the cache on, set up some
		 * pointers, and start decompressing.
		 */

		bl	cache_on

		mov	r1, sp			@ malloc space above stack
		add	r2, sp, #0x10000	@ 64k max

/*
 * Check to see if we will overwrite ourselves.
 *   r4 = final kernel address
 *   r5 = start of this image
 *   r6 = size of decompressed image
 *   r2 = end of malloc space (and therefore this image)
 * We basically want:
 *   r4 >= r2 -> OK
 *   r4 + image length <= r5 -> OK
 */
		mov	r5, r2			@ decompress after malloc space
		mov	r0, r5
		mov	r3, r7

		bl	decompress_kernel

		add	r0, r0, #127 + 128	@ alignment + stack
		bic	r0, r0, #127		@ align the kernel length
/*
 * r0     = decompressed kernel length
 * r1-r3  = unused
 * r4     = kernel execution address
 * r5     = decompressed kernel start
 * r7     = architecture ID
 * r8     = atags pointer
 * r9-r12,r14 = corrupted
 */
		add	r1, r5, r0		@ end of decompressed kernel
		adr	r2, reloc_start
		ldr	r3, LC1
		add	r3, r2, r3
1:		ldmia	r2!, {r9 - r12, r14}	@ copy relocation code
		stmia	r1!, {r9 - r12, r14}
		ldmia	r2!, {r9 - r12, r14}
		stmia	r1!, {r9 - r12, r14}
		cmp	r2, r3
		blo	1b
		mov	sp, r1
		add	sp, sp, #128		@ relocate the stack

		bl	cache_clean_flush
 ARM(		add	pc, r5, r0		) @ call relocation code
 THUMB(		add	r12, r5, r0		)
 THUMB(		mov	pc, r12			) @ call relocation code

		.align	2
		.type	LC0, #object
LC0:		.word	LC0			@ r1
		.word	__bss_start		@ r2
		.word	_end			@ r3
		.word	zreladdr		@ r4
		.word	_start			@ r5
		.word	_image_size		@ r6
		.word	_got_start		@ r11
		.word	_got_end		@ ip
		.word	user_stack+4096		@ sp
LC1:		.word	reloc_end - reloc_start
		.size	LC0, . - LC0

/*
 * Turn on the cache.  We need to setup some page tables so that we
 * can have both the I and D caches on.
 *
 * We place the page tables 16k down from the kernel execution address,
 * and we hope that nothing else is using it.  If we're using it, we
 * will go pop!
 *
 * On entry,
 *  r4 = kernel execution address
 *  r7 = architecture number
 *  r8 = atags pointer
 *  r9 = run-time address of "start"  (???)
 * On exit,
 *  r1, r2, r3, r9, r10, r12 corrupted
 * This routine must preserve:
 *  r4, r5, r6, r7, r8
 */
		.align	5
cache_on:	mov	r3, #8			@ cache_on function
		b	call_cache_fn

/*
 * Initialize the highest priority protection region, PR7
 * to cover all 32bit address and cacheable and bufferable.
 */
__setup_mmu:	sub	r3, r4, #16384		@ Page directory size
		bic	r3, r3, #0xff		@ Align the pointer
		bic	r3, r3, #0x3f00
/*
 * Initialise the page tables, turning on the cacheable and bufferable
 * bits for the RAM area only.
 */
		mov	r0, r3
		mov	r9, r0, lsr #18
		mov	r9, r9, lsl #18		@ start of RAM
		add	r10, r9, #0x10000000	@ a reasonable RAM size
		mov	r1, #0x12
		orr	r1, r1, #3 << 10
		add	r2, r3, #16384
1:		cmp	r1, r9			@ if virt > start of RAM
		orrhs	r1, r1, #0x0c		@ set cacheable, bufferable
		cmp	r1, r10			@ if virt > end of RAM
		bichs	r1, r1, #0x0c		@ clear cacheable, bufferable
		str	r1, [r0], #4		@ 1:1 mapping
		add	r1, r1, #1048576
		teq	r0, r2
		bne	1b
/*
 * If ever we are running from Flash, then we surely want the cache
 * to be enabled also for our execution instance...  We map 2MB of it
 * so there is no map overlap problem for up to 1 MB compressed kernel.
 * If the execution is in RAM then we would only be duplicating the above.
 */
		mov	r1, #0x1e
		orr	r1, r1, #3 << 10
		mov	r2, pc, lsr #20
		orr	r1, r1, r2, lsl #20
		add	r0, r3, r2, lsl #2
		str	r1, [r0], #4
		add	r1, r1, #1048576
		str	r1, [r0]
		mov	pc, lr
ENDPROC(__setup_mmu)

__armv4_mmu_cache_on:
		mov	r12, lr
		bl	__setup_mmu
		mov	r0, #0
		mcr	p15, 0, r0, c7, c10, 4	@ drain write buffer
		mcr	p15, 0, r0, c8, c7, 0	@ flush I,D TLBs
		mrc	p15, 0, r0, c1, c0, 0	@ read control reg
		orr	r0, r0, #0x5000		@ I-cache enable, RR cache replacement
		orr	r0, r0, #0x0030
		bl	__common_mmu_cache_on
		mov	r0, #0
		mcr	p15, 0, r0, c8, c7, 0	@ flush I,D TLBs
		mov	pc, r12

__common_mmu_cache_on:
		orr	r0, r0, #0x000d		@ Write buffer, mmu
		mov	r1, #-1
		mcr	p15, 0, r3, c2, c0, 0	@ load page table pointer
		mcr	p15, 0, r1, c3, c0, 0	@ load domain access control
		b	1f
		.align	5			@ cache line aligned
1:		mcr	p15, 0, r0, c1, c0, 0	@ load control register
		mrc	p15, 0, r0, c1, c0, 0	@ and read it back to
		sub	pc, lr, r0, lsr #32	@ properly flush pipeline

/*
 * All code following this line is relocatable.  It is relocated by
 * the above code to the end of the decompressed kernel image and
 * executed there.  During this time, we have no stacks.
 *
 * r0     = decompressed kernel length
 * r1-r3  = unused
 * r4     = kernel execution address
 * r5     = decompressed kernel start
 * r7     = architecture ID
 * r8     = atags pointer
 * r9-r12,r14 = corrupted
 */
		.align	5
reloc_start:
		add	r9, r5, r0
		sub	r9, r9, #128		@ do not copy the stack
		mov	r1, r4
1:
		.rept	4
		ldmia	r5!, {r0, r2, r3, r10 - r12, r14}	@ relocate kernel
		stmia	r1!, {r0, r2, r3, r10 - r12, r14}
		.endr

		cmp	r5, r9
		blo	1b
		mov	sp, r1
		add	sp, sp, #128		@ relocate the stack

call_kernel:	bl	cache_clean_flush
		bl	cache_off
		mov	r0, #0			@ must be zero
		mov	r1, r7			@ restore architecture number
		mov	r2, r8			@ restore atags pointer
		mov	pc, r4			@ call kernel

/*
 * Here follow the relocatable cache support functions for the
 * various processors.  This is a generic hook for locating an
 * entry and jumping to an instruction at the specified offset
 * from the start of the block.  Please note this is all position
 * independent code.
 *
 *  r1  = corrupted
 *  r2  = corrupted
 *  r3  = block offset
 *  r9  = corrupted
 *  r12 = corrupted
 */

call_cache_fn:	adr	r12, proc_types
#ifdef CONFIG_CPU_CP15
		mrc	p15, 0, r9, c0, c0	@ get processor ID
#endif
1:		ldr	r1, [r12, #0]		@ get value
		ldr	r2, [r12, #4]		@ get mask
		eor	r1, r1, r9		@ (real ^ match)
		tst	r1, r2			@       & mask
 ARM(		addeq	pc, r12, r3		) @ call cache function
 THUMB(		addeq	r12, r3			)
 THUMB(		moveq	pc, r12			) @ call cache function
		add	r12, r12, #4*5
		b	1b

/*
 * Table for cache operations.  This is basically:
 *   - CPU ID match
 *   - CPU ID mask
 *   - 'cache on' method instruction
 *   - 'cache off' method instruction
 *   - 'cache flush' method instruction
 *
 * We match an entry using: ((real_id ^ match) & mask) == 0
 *
 * Writethrough caches generally only need 'on' and 'off'
 * methods.  Writeback caches _must_ have the flush method
 * defined.
 */
		.align	2
		.type	proc_types,#object
proc_types:
		.word	0x000f0000		@ new CPU Id. but equals to "nxp2120(arm1176, ARMv6)"
		.word	0x000f0000
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv6_mmu_cache_flush

		.size	proc_types, . - proc_types

/*
 * Turn off the Cache and MMU.  ARMv3 does not support
 * reading the control register, but ARMv4 does.
 *
 * On exit, r0, r1, r2, r3, r9, r12 corrupted
 * This routine must preserve: r4, r6, r7
 */
		.align	5
cache_off:	mov	r3, #12			@ cache_off function
		b	call_cache_fn

__armv4_mmu_cache_off:
		mrc	p15, 0, r0, c1, c0
		bic	r0, r0, #0x000d
		mcr	p15, 0, r0, c1, c0	@ turn MMU and cache off
		mov	r0, #0
		mcr	p15, 0, r0, c7, c7	@ invalidate whole cache v4
		mcr	p15, 0, r0, c8, c7	@ invalidate whole TLB v4
		mov	pc, lr

/*
 * Clean and flush the cache to maintain consistency.
 *
 * On exit,
 *  r1, r2, r3, r9, r11, r12 corrupted
 * This routine must preserve:
 *  r0, r4, r5, r6, r7
 */
		.align	5
cache_clean_flush:
		mov	r3, #16
		b	call_cache_fn

__armv6_mmu_cache_flush:
		mov	r1, #0
		mcr	p15, 0, r1, c7, c14, 0	@ clean+invalidate D
		mcr	p15, 0, r1, c7, c5, 0	@ invalidate I+BTB
		mcr	p15, 0, r1, c7, c15, 0	@ clean+invalidate unified
		mcr	p15, 0, r1, c7, c10, 4	@ drain WB
		mov	pc, lr

/*
 * Various debugging routines for printing hex characters and
 * memory, which again must be relocatable.
 */
		.ltorg
reloc_end:

		.align
		.section ".stack", "w"
user_stack:	.space	4096
