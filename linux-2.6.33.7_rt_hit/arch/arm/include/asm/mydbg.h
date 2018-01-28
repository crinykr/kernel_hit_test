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
