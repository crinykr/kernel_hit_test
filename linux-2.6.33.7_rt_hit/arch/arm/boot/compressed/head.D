
head.o:     file format elf32-littlearm


Disassembly of section .text:

00000000 <not_relocated-0x38>:
   0:	e28f00a8 	add	r0, pc, #168	; 0xa8
   4:	e890387e 	ldm	r0, {r1, r2, r3, r4, r5, r6, fp, ip, sp}
   8:	e0500001 	subs	r0, r0, r1
   c:	e0855000 	add	r5, r5, r0
  10:	e08bb000 	add	fp, fp, r0
  14:	e08cc000 	add	ip, ip, r0
  18:	e0822000 	add	r2, r2, r0
  1c:	e0833000 	add	r3, r3, r0
  20:	e08dd000 	add	sp, sp, r0
  24:	e59b1000 	ldr	r1, [fp]
  28:	e0811000 	add	r1, r1, r0
  2c:	e48b1004 	str	r1, [fp], #4
  30:	e15b000c 	cmp	fp, ip
  34:	3afffffa 	bcc	24 <not_relocated-0x14>

00000038 <not_relocated>:
  38:	e3a00000 	mov	r0, #0
  3c:	e4820004 	str	r0, [r2], #4
  40:	e4820004 	str	r0, [r2], #4
  44:	e4820004 	str	r0, [r2], #4
  48:	e4820004 	str	r0, [r2], #4
  4c:	e1520003 	cmp	r2, r3
  50:	3afffff9 	bcc	3c <not_relocated+0x4>
  54:	eb000021 	bl	e0 <cache_on>
  58:	e1a0100d 	mov	r1, sp
  5c:	e28d2801 	add	r2, sp, #65536	; 0x10000
  60:	e1a05002 	mov	r5, r2
  64:	e1a00005 	mov	r0, r5
  68:	e1a03007 	mov	r3, r7
  6c:	ebfffffe 	bl	0 <decompress_kernel>
  70:	e28000ff 	add	r0, r0, #255	; 0xff
  74:	e3c0007f 	bic	r0, r0, #127	; 0x7f
  78:	e0851000 	add	r1, r5, r0
  7c:	e28f2f4f 	add	r2, pc, #316	; 0x13c
  80:	e59f304c 	ldr	r3, [pc, #76]	; d4 <LC1>
  84:	e0823003 	add	r3, r2, r3
  88:	e8b25e00 	ldm	r2!, {r9, sl, fp, ip, lr}
  8c:	e8a15e00 	stmia	r1!, {r9, sl, fp, ip, lr}
  90:	e8b25e00 	ldm	r2!, {r9, sl, fp, ip, lr}
  94:	e8a15e00 	stmia	r1!, {r9, sl, fp, ip, lr}
  98:	e1520003 	cmp	r2, r3
  9c:	3afffff9 	bcc	88 <not_relocated+0x50>
  a0:	e1a0d001 	mov	sp, r1
  a4:	e28dd080 	add	sp, sp, #128	; 0x80
  a8:	eb00007c 	bl	2a0 <cache_clean_flush>
  ac:	e085f000 	add	pc, r5, r0

000000b0 <LC0>:
  b0:	000000b0 	strheq	r0, [r0], -r0	; <UNPREDICTABLE>
	...
  d0:	00001000 	andeq	r1, r0, r0

000000d4 <LC1>:
  d4:	000001ac 	andeq	r0, r0, ip, lsr #3
  d8:	e1a00000 	nop			; (mov r0, r0)
  dc:	e1a00000 	nop			; (mov r0, r0)

000000e0 <cache_on>:
  e0:	e3a03008 	mov	r3, #8
  e4:	ea00004a 	b	214 <call_cache_fn>

000000e8 <__setup_mmu>:
  e8:	e2443901 	sub	r3, r4, #16384	; 0x4000
  ec:	e3c330ff 	bic	r3, r3, #255	; 0xff
  f0:	e3c33c3f 	bic	r3, r3, #16128	; 0x3f00
  f4:	e1a00003 	mov	r0, r3
  f8:	e1a09920 	lsr	r9, r0, #18
  fc:	e1a09909 	lsl	r9, r9, #18
 100:	e289a201 	add	sl, r9, #268435456	; 0x10000000
 104:	e3a01012 	mov	r1, #18
 108:	e3811b03 	orr	r1, r1, #3072	; 0xc00
 10c:	e2832901 	add	r2, r3, #16384	; 0x4000
 110:	e1510009 	cmp	r1, r9
 114:	2381100c 	orrcs	r1, r1, #12
 118:	e151000a 	cmp	r1, sl
 11c:	23c1100c 	biccs	r1, r1, #12
 120:	e4801004 	str	r1, [r0], #4
 124:	e2811601 	add	r1, r1, #1048576	; 0x100000
 128:	e1300002 	teq	r0, r2
 12c:	1afffff7 	bne	110 <__setup_mmu+0x28>
 130:	e3a0101e 	mov	r1, #30
 134:	e3811b03 	orr	r1, r1, #3072	; 0xc00
 138:	e1a02a2f 	lsr	r2, pc, #20
 13c:	e1811a02 	orr	r1, r1, r2, lsl #20
 140:	e0830102 	add	r0, r3, r2, lsl #2
 144:	e4801004 	str	r1, [r0], #4
 148:	e2811601 	add	r1, r1, #1048576	; 0x100000
 14c:	e5801000 	str	r1, [r0]
 150:	e1a0f00e 	mov	pc, lr

00000154 <__armv7_mmu_cache_on>:
 154:	e1a0c00e 	mov	ip, lr
 158:	ee10bf91 	mrc	15, 0, fp, cr0, cr1, {4}
 15c:	e31b000f 	tst	fp, #15
 160:	1bffffe0 	blne	e8 <__setup_mmu>
 164:	e3a00000 	mov	r0, #0
 168:	ee070f9a 	mcr	15, 0, r0, cr7, cr10, {4}
 16c:	e31b000f 	tst	fp, #15
 170:	1e080f17 	mcrne	15, 0, r0, cr8, cr7, {0}
 174:	ee110f10 	mrc	15, 0, r0, cr1, cr0, {0}
 178:	e3800a05 	orr	r0, r0, #20480	; 0x5000
 17c:	e380003c 	orr	r0, r0, #60	; 0x3c
 180:	13800001 	orrne	r0, r0, #1
 184:	13e01000 	mvnne	r1, #0
 188:	1e023f10 	mcrne	15, 0, r3, cr2, cr0, {0}
 18c:	1e031f10 	mcrne	15, 0, r1, cr3, cr0, {0}
 190:	ee010f10 	mcr	15, 0, r0, cr1, cr0, {0}
 194:	ee110f10 	mrc	15, 0, r0, cr1, cr0, {0}
 198:	e3a00000 	mov	r0, #0
 19c:	ee070f95 	mcr	15, 0, r0, cr7, cr5, {4}
 1a0:	e1a0f00c 	mov	pc, ip
 1a4:	e1a00000 	nop			; (mov r0, r0)
 1a8:	e1a00000 	nop			; (mov r0, r0)
 1ac:	e1a00000 	nop			; (mov r0, r0)
 1b0:	e1a00000 	nop			; (mov r0, r0)
 1b4:	e1a00000 	nop			; (mov r0, r0)
 1b8:	e1a00000 	nop			; (mov r0, r0)
 1bc:	e1a00000 	nop			; (mov r0, r0)

000001c0 <reloc_start>:
 1c0:	e0859000 	add	r9, r5, r0
 1c4:	e2499080 	sub	r9, r9, #128	; 0x80
 1c8:	e1a01004 	mov	r1, r4
 1cc:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 1d0:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 1d4:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 1d8:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 1dc:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 1e0:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 1e4:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 1e8:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 1ec:	e1550009 	cmp	r5, r9
 1f0:	3afffff5 	bcc	1cc <reloc_start+0xc>
 1f4:	e1a0d001 	mov	sp, r1
 1f8:	e28dd080 	add	sp, sp, #128	; 0x80

000001fc <call_kernel>:
 1fc:	eb000027 	bl	2a0 <cache_clean_flush>
 200:	eb000016 	bl	260 <cache_off>
 204:	e3a00000 	mov	r0, #0
 208:	e1a01007 	mov	r1, r7
 20c:	e1a02008 	mov	r2, r8
 210:	e1a0f004 	mov	pc, r4

00000214 <call_cache_fn>:
 214:	e28fc01c 	add	ip, pc, #28
 218:	ee109f10 	mrc	15, 0, r9, cr0, cr0, {0}
 21c:	e59c1000 	ldr	r1, [ip]
 220:	e59c2004 	ldr	r2, [ip, #4]
 224:	e0211009 	eor	r1, r1, r9
 228:	e1110002 	tst	r1, r2
 22c:	008cf003 	addeq	pc, ip, r3
 230:	e28cc014 	add	ip, ip, #20
 234:	eafffff8 	b	21c <call_cache_fn+0x8>

00000238 <proc_types>:
 238:	000f0000 	andeq	r0, pc, r0
 23c:	000f0000 	andeq	r0, pc, r0
 240:	eaffffc3 	b	154 <__armv7_mmu_cache_on>
 244:	ea000007 	b	268 <__armv7_mmu_cache_off>
 248:	ea000016 	b	2a8 <__armv7_mmu_cache_flush>
 24c:	e1a00000 	nop			; (mov r0, r0)
 250:	e1a00000 	nop			; (mov r0, r0)
 254:	e1a00000 	nop			; (mov r0, r0)
 258:	e1a00000 	nop			; (mov r0, r0)
 25c:	e1a00000 	nop			; (mov r0, r0)

00000260 <cache_off>:
 260:	e3a0300c 	mov	r3, #12
 264:	eaffffea 	b	214 <call_cache_fn>

00000268 <__armv7_mmu_cache_off>:
 268:	ee110f10 	mrc	15, 0, r0, cr1, cr0, {0}
 26c:	e3c0000d 	bic	r0, r0, #13
 270:	ee010f10 	mcr	15, 0, r0, cr1, cr0, {0}
 274:	e1a0c00e 	mov	ip, lr
 278:	eb00000a 	bl	2a8 <__armv7_mmu_cache_flush>
 27c:	e3a00000 	mov	r0, #0
 280:	ee080f17 	mcr	15, 0, r0, cr8, cr7, {0}
 284:	ee070fd5 	mcr	15, 0, r0, cr7, cr5, {6}
 288:	ee070f9a 	mcr	15, 0, r0, cr7, cr10, {4}
 28c:	ee070f95 	mcr	15, 0, r0, cr7, cr5, {4}
 290:	e1a0f00c 	mov	pc, ip
 294:	e1a00000 	nop			; (mov r0, r0)
 298:	e1a00000 	nop			; (mov r0, r0)
 29c:	e1a00000 	nop			; (mov r0, r0)

000002a0 <cache_clean_flush>:
 2a0:	e3a03010 	mov	r3, #16
 2a4:	eaffffda 	b	214 <call_cache_fn>

000002a8 <__armv7_mmu_cache_flush>:
 2a8:	ee10afb1 	mrc	15, 0, sl, cr0, cr1, {5}
 2ac:	e31a080f 	tst	sl, #983040	; 0xf0000
 2b0:	e3a0a000 	mov	sl, #0
 2b4:	0a000001 	beq	2c0 <hierarchical>
 2b8:	ee07af1e 	mcr	15, 0, sl, cr7, cr14, {0}
 2bc:	ea000023 	b	350 <iflush>

000002c0 <hierarchical>:
 2c0:	ee07afba 	mcr	15, 0, sl, cr7, cr10, {5}
 2c4:	e92d0eff 	push	{r0, r1, r2, r3, r4, r5, r6, r7, r9, sl, fp}
 2c8:	ee300f30 	mrc	15, 1, r0, cr0, cr0, {1}
 2cc:	e2103407 	ands	r3, r0, #117440512	; 0x7000000
 2d0:	e1a03ba3 	lsr	r3, r3, #23
 2d4:	0a00001a 	beq	344 <finished>
 2d8:	e3a0a000 	mov	sl, #0

000002dc <loop1>:
 2dc:	e08a20aa 	add	r2, sl, sl, lsr #1
 2e0:	e1a01230 	lsr	r1, r0, r2
 2e4:	e2011007 	and	r1, r1, #7
 2e8:	e3510002 	cmp	r1, #2
 2ec:	ba000011 	blt	338 <skip>
 2f0:	ee40af10 	mcr	15, 2, sl, cr0, cr0, {0}
 2f4:	ee07af95 	mcr	15, 0, sl, cr7, cr5, {4}
 2f8:	ee301f10 	mrc	15, 1, r1, cr0, cr0, {0}
 2fc:	e2012007 	and	r2, r1, #7
 300:	e2822004 	add	r2, r2, #4
 304:	e59f4058 	ldr	r4, [pc, #88]	; 364 <iflush+0x14>
 308:	e01441a1 	ands	r4, r4, r1, lsr #3
 30c:	e16f5f14 	clz	r5, r4
 310:	e59f7050 	ldr	r7, [pc, #80]	; 368 <iflush+0x18>
 314:	e01776a1 	ands	r7, r7, r1, lsr #13

00000318 <loop2>:
 318:	e1a09004 	mov	r9, r4

0000031c <loop3>:
 31c:	e18ab519 	orr	fp, sl, r9, lsl r5
 320:	e18bb217 	orr	fp, fp, r7, lsl r2
 324:	ee07bf5e 	mcr	15, 0, fp, cr7, cr14, {2}
 328:	e2599001 	subs	r9, r9, #1
 32c:	aafffffa 	bge	31c <loop3>
 330:	e2577001 	subs	r7, r7, #1
 334:	aafffff7 	bge	318 <loop2>

00000338 <skip>:
 338:	e28aa002 	add	sl, sl, #2
 33c:	e153000a 	cmp	r3, sl
 340:	caffffe5 	bgt	2dc <loop1>

00000344 <finished>:
 344:	e8bd0eff 	pop	{r0, r1, r2, r3, r4, r5, r6, r7, r9, sl, fp}
 348:	e3a0a000 	mov	sl, #0
 34c:	ee40af10 	mcr	15, 2, sl, cr0, cr0, {0}

00000350 <iflush>:
 350:	ee07af9a 	mcr	15, 0, sl, cr7, cr10, {4}
 354:	ee07af15 	mcr	15, 0, sl, cr7, cr5, {0}
 358:	ee07af9a 	mcr	15, 0, sl, cr7, cr10, {4}
 35c:	ee07af95 	mcr	15, 0, sl, cr7, cr5, {4}
 360:	e1a0f00e 	mov	pc, lr
 364:	000003ff 	strdeq	r0, [r0], -pc	; <UNPREDICTABLE>
 368:	00007fff 	strdeq	r7, [r0], -pc	; <UNPREDICTABLE>

0000036c <reloc_end>:
 36c:	e1a00000 	nop			; (mov r0, r0)
 370:	e1a00000 	nop			; (mov r0, r0)
 374:	e1a00000 	nop			; (mov r0, r0)
 378:	e1a00000 	nop			; (mov r0, r0)
 37c:	e1a00000 	nop			; (mov r0, r0)

Disassembly of section .start:

00000000 <start>:
   0:	e3a00053 	mov	r0, #83	; 0x53
   4:	e59f211c 	ldr	r2, [pc, #284]	; 128 <start+0x128>
   8:	e1d230bc 	ldrh	r3, [r2, #12]
   c:	e3130c02 	tst	r3, #512	; 0x200
  10:	1afffffb 	bne	4 <start+0x4>
  14:	e6ff3070 	uxth	r3, r0
  18:	e1c231b0 	strh	r3, [r2, #16]
  1c:	e59f2104 	ldr	r2, [pc, #260]	; 128 <start+0x128>
  20:	e1d230bc 	ldrh	r3, [r2, #12]
  24:	e3130c02 	tst	r3, #512	; 0x200
  28:	1afffffb 	bne	1c <start+0x1c>
  2c:	e3a00030 	mov	r0, #48	; 0x30
  30:	e59f20f0 	ldr	r2, [pc, #240]	; 128 <start+0x128>
  34:	e1d230bc 	ldrh	r3, [r2, #12]
  38:	e3130c02 	tst	r3, #512	; 0x200
  3c:	1afffffb 	bne	30 <start+0x30>
  40:	e6ff3070 	uxth	r3, r0
  44:	e1c231b0 	strh	r3, [r2, #16]
  48:	e59f20d8 	ldr	r2, [pc, #216]	; 128 <start+0x128>
  4c:	e1d230bc 	ldrh	r3, [r2, #12]
  50:	e3130c02 	tst	r3, #512	; 0x200
  54:	1afffffb 	bne	48 <start+0x48>
  58:	e3a0003a 	mov	r0, #58	; 0x3a
  5c:	e59f20c4 	ldr	r2, [pc, #196]	; 128 <start+0x128>
  60:	e1d230bc 	ldrh	r3, [r2, #12]
  64:	e3130c02 	tst	r3, #512	; 0x200
  68:	1afffffb 	bne	5c <start+0x5c>
  6c:	e6ff3070 	uxth	r3, r0
  70:	e1c231b0 	strh	r3, [r2, #16]
  74:	e59f20ac 	ldr	r2, [pc, #172]	; 128 <start+0x128>
  78:	e1d230bc 	ldrh	r3, [r2, #12]
  7c:	e3130c02 	tst	r3, #512	; 0x200
  80:	1afffffb 	bne	74 <start+0x74>
  84:	e1a0000f 	mov	r0, pc
  88:	e3a01007 	mov	r1, #7
  8c:	e59f2094 	ldr	r2, [pc, #148]	; 128 <start+0x128>
  90:	e1a03101 	lsl	r3, r1, #2
  94:	e1d220bc 	ldrh	r2, [r2, #12]
  98:	e1a03350 	asr	r3, r0, r3
  9c:	e3120c02 	tst	r2, #512	; 0x200
  a0:	e203300f 	and	r3, r3, #15
  a4:	1affffec 	bne	5c <start+0x5c>
  a8:	e3530009 	cmp	r3, #9
  ac:	e2832037 	add	r2, r3, #55	; 0x37
  b0:	d2832030 	addle	r2, r3, #48	; 0x30
  b4:	e59f306c 	ldr	r3, [pc, #108]	; 128 <start+0x128>
  b8:	e2511001 	subs	r1, r1, #1
  bc:	e1c321b0 	strh	r2, [r3, #16]
  c0:	5afffff1 	bpl	8c <start+0x8c>
  c4:	e59f305c 	ldr	r3, [pc, #92]	; 128 <start+0x128>
  c8:	e1d330bc 	ldrh	r3, [r3, #12]
  cc:	e3130c02 	tst	r3, #512	; 0x200
  d0:	1afffffb 	bne	c4 <start+0xc4>
  d4:	e3a0000a 	mov	r0, #10
  d8:	e59f2048 	ldr	r2, [pc, #72]	; 128 <start+0x128>
  dc:	e1d230bc 	ldrh	r3, [r2, #12]
  e0:	e3130c02 	tst	r3, #512	; 0x200
  e4:	1afffffb 	bne	d8 <start+0xd8>
  e8:	e6ff3070 	uxth	r3, r0
  ec:	e1c231b0 	strh	r3, [r2, #16]
  f0:	e59f2030 	ldr	r2, [pc, #48]	; 128 <start+0x128>
  f4:	e1d230bc 	ldrh	r3, [r2, #12]
  f8:	e3130c02 	tst	r3, #512	; 0x200
  fc:	1afffffb 	bne	f0 <start+0xf0>
 100:	eafffffe 	b	100 <start+0x100>
 104:	ea000002 	b	114 <start+0x114>
 108:	016f2818 	cmneq	pc, r8, lsl r8	; <UNPREDICTABLE>
	...
 114:	e1a07001 	mov	r7, r1
 118:	e1a08002 	mov	r8, r2
 11c:	e10f2000 	mrs	r2, CPSR
 120:	e38220c0 	orr	r2, r2, #192	; 0xc0
 124:	e121f002 	msr	CPSR_c, r2
 128:	c0016000 	andgt	r6, r1, r0

Disassembly of section .stack:

00000000 <user_stack>:
	...

Disassembly of section .ARM.attributes:

00000000 <.ARM.attributes>:
   0:	00000f41 	andeq	r0, r0, r1, asr #30
   4:	61656100 	cmnvs	r5, r0, lsl #2
   8:	01006962 	tsteq	r0, r2, ror #18
   c:	00000005 	andeq	r0, r0, r5
