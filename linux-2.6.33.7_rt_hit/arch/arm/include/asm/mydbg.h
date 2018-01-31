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

		.macro mmuoff
		// flush
		mov	r1, #0
		mcr	p15, 0, r1, c7, c14, 0	@ clean+invalidate D
		mcr	p15, 0, r1, c7, c5, 0	@ invalidate I+BTB
		mcr	p15, 0, r1, c7, c15, 0	@ clean+invalidate unified
		mcr	p15, 0, r1, c7, c10, 4	@ drain WB

		// mmu off
		mrc	p15, 0, r0, c1, c0
		bic	r0, r0, #0x000d
		mcr	p15, 0, r0, c1, c0	@ turn MMU and cache off

		// TLB,cache off
		mov	r0, #0
		mcr	p15, 0, r0, c7, c7	@ invalidate whole cache v4
		mcr	p15, 0, r0, c8, c7	@ invalidate whole TLB v4
		.endm
