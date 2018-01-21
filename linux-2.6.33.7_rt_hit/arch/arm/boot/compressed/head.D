
head.o:     file format elf32-littlearm


Disassembly of section .text:

00000000 <not_relocated-0x38>:
   0:	e28f0f55 	add	r0, pc, #340	; 0x154
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
  54:	eb000051 	bl	1a0 <cache_on>
  58:	e1a0100d 	mov	r1, sp
  5c:	e28d2801 	add	r2, sp, #65536	; 0x10000
  60:	e1a05002 	mov	r5, r2
  64:	e1a00005 	mov	r0, r5
  68:	e1a03007 	mov	r3, r7
  6c:	e3a00058 	mov	r0, #88	; 0x58
  70:	e59f23ac 	ldr	r2, [pc, #940]	; 424 <iflush+0x14>
  74:	e1d230bc 	ldrh	r3, [r2, #12]
  78:	e3130c02 	tst	r3, #512	; 0x200
  7c:	1afffffb 	bne	70 <not_relocated+0x38>
  80:	e6ff3070 	uxth	r3, r0
  84:	e1c231b0 	strh	r3, [r2, #16]
  88:	e59f2394 	ldr	r2, [pc, #916]	; 424 <iflush+0x14>
  8c:	e1d230bc 	ldrh	r3, [r2, #12]
  90:	e3130c02 	tst	r3, #512	; 0x200
  94:	1afffffb 	bne	88 <not_relocated+0x50>
  98:	e3a0003b 	mov	r0, #59	; 0x3b
  9c:	e59f2380 	ldr	r2, [pc, #896]	; 424 <iflush+0x14>
  a0:	e1d230bc 	ldrh	r3, [r2, #12]
  a4:	e3130c02 	tst	r3, #512	; 0x200
  a8:	1afffffb 	bne	9c <not_relocated+0x64>
  ac:	e6ff3070 	uxth	r3, r0
  b0:	e1c231b0 	strh	r3, [r2, #16]
  b4:	e59f2368 	ldr	r2, [pc, #872]	; 424 <iflush+0x14>
  b8:	e1d230bc 	ldrh	r3, [r2, #12]
  bc:	e3130c02 	tst	r3, #512	; 0x200
  c0:	1afffffb 	bne	b4 <not_relocated+0x7c>
  c4:	e1a0000f 	mov	r0, pc
  c8:	e3a01007 	mov	r1, #7
  cc:	e59f2350 	ldr	r2, [pc, #848]	; 424 <iflush+0x14>
  d0:	e1a03101 	lsl	r3, r1, #2
  d4:	e1d220bc 	ldrh	r2, [r2, #12]
  d8:	e1a03350 	asr	r3, r0, r3
  dc:	e3120c02 	tst	r2, #512	; 0x200
  e0:	e203300f 	and	r3, r3, #15
  e4:	1afffff8 	bne	cc <not_relocated+0x94>
  e8:	e3530009 	cmp	r3, #9
  ec:	e2832037 	add	r2, r3, #55	; 0x37
  f0:	d2832030 	addle	r2, r3, #48	; 0x30
  f4:	e59f3328 	ldr	r3, [pc, #808]	; 424 <iflush+0x14>
  f8:	e2511001 	subs	r1, r1, #1
  fc:	e1c321b0 	strh	r2, [r3, #16]
 100:	5afffff1 	bpl	cc <not_relocated+0x94>
 104:	e59f3318 	ldr	r3, [pc, #792]	; 424 <iflush+0x14>
 108:	e1d330bc 	ldrh	r3, [r3, #12]
 10c:	e3130c02 	tst	r3, #512	; 0x200
 110:	1afffffb 	bne	104 <not_relocated+0xcc>
 114:	eafffffe 	b	114 <not_relocated+0xdc>
 118:	ebfffffe 	bl	0 <decompress_kernel>
 11c:	e28000ff 	add	r0, r0, #255	; 0xff
 120:	e3c0007f 	bic	r0, r0, #127	; 0x7f
 124:	e0851000 	add	r1, r5, r0
 128:	e28f2e15 	add	r2, pc, #336	; 0x150
 12c:	e59f304c 	ldr	r3, [pc, #76]	; 180 <LC1>
 130:	e0823003 	add	r3, r2, r3
 134:	e8b25e00 	ldm	r2!, {r9, sl, fp, ip, lr}
 138:	e8a15e00 	stmia	r1!, {r9, sl, fp, ip, lr}
 13c:	e8b25e00 	ldm	r2!, {r9, sl, fp, ip, lr}
 140:	e8a15e00 	stmia	r1!, {r9, sl, fp, ip, lr}
 144:	e1520003 	cmp	r2, r3
 148:	3afffff9 	bcc	134 <not_relocated+0xfc>
 14c:	e1a0d001 	mov	sp, r1
 150:	e28dd080 	add	sp, sp, #128	; 0x80
 154:	eb000081 	bl	360 <cache_clean_flush>
 158:	e085f000 	add	pc, r5, r0

0000015c <LC0>:
 15c:	0000015c 	andeq	r0, r0, ip, asr r1
	...
 17c:	00001000 	andeq	r1, r0, r0

00000180 <LC1>:
 180:	000001b0 			; <UNDEFINED> instruction: 0x000001b0
 184:	e1a00000 	nop			; (mov r0, r0)
 188:	e1a00000 	nop			; (mov r0, r0)
 18c:	e1a00000 	nop			; (mov r0, r0)
 190:	e1a00000 	nop			; (mov r0, r0)
 194:	e1a00000 	nop			; (mov r0, r0)
 198:	e1a00000 	nop			; (mov r0, r0)
 19c:	e1a00000 	nop			; (mov r0, r0)

000001a0 <cache_on>:
 1a0:	e3a03008 	mov	r3, #8
 1a4:	ea00004a 	b	2d4 <call_cache_fn>

000001a8 <__setup_mmu>:
 1a8:	e2443901 	sub	r3, r4, #16384	; 0x4000
 1ac:	e3c330ff 	bic	r3, r3, #255	; 0xff
 1b0:	e3c33c3f 	bic	r3, r3, #16128	; 0x3f00
 1b4:	e1a00003 	mov	r0, r3
 1b8:	e1a09920 	lsr	r9, r0, #18
 1bc:	e1a09909 	lsl	r9, r9, #18
 1c0:	e289a201 	add	sl, r9, #268435456	; 0x10000000
 1c4:	e3a01012 	mov	r1, #18
 1c8:	e3811b03 	orr	r1, r1, #3072	; 0xc00
 1cc:	e2832901 	add	r2, r3, #16384	; 0x4000
 1d0:	e1510009 	cmp	r1, r9
 1d4:	2381100c 	orrcs	r1, r1, #12
 1d8:	e151000a 	cmp	r1, sl
 1dc:	23c1100c 	biccs	r1, r1, #12
 1e0:	e4801004 	str	r1, [r0], #4
 1e4:	e2811601 	add	r1, r1, #1048576	; 0x100000
 1e8:	e1300002 	teq	r0, r2
 1ec:	1afffff7 	bne	1d0 <__setup_mmu+0x28>
 1f0:	e3a0101e 	mov	r1, #30
 1f4:	e3811b03 	orr	r1, r1, #3072	; 0xc00
 1f8:	e1a02a2f 	lsr	r2, pc, #20
 1fc:	e1811a02 	orr	r1, r1, r2, lsl #20
 200:	e0830102 	add	r0, r3, r2, lsl #2
 204:	e4801004 	str	r1, [r0], #4
 208:	e2811601 	add	r1, r1, #1048576	; 0x100000
 20c:	e5801000 	str	r1, [r0]
 210:	e1a0f00e 	mov	pc, lr

00000214 <__armv7_mmu_cache_on>:
 214:	e1a0c00e 	mov	ip, lr
 218:	ee10bf91 	mrc	15, 0, fp, cr0, cr1, {4}
 21c:	e31b000f 	tst	fp, #15
 220:	1bffffe0 	blne	1a8 <__setup_mmu>
 224:	e3a00000 	mov	r0, #0
 228:	ee070f9a 	mcr	15, 0, r0, cr7, cr10, {4}
 22c:	e31b000f 	tst	fp, #15
 230:	1e080f17 	mcrne	15, 0, r0, cr8, cr7, {0}
 234:	ee110f10 	mrc	15, 0, r0, cr1, cr0, {0}
 238:	e3800a05 	orr	r0, r0, #20480	; 0x5000
 23c:	e380003c 	orr	r0, r0, #60	; 0x3c
 240:	13800001 	orrne	r0, r0, #1
 244:	13e01000 	mvnne	r1, #0
 248:	1e023f10 	mcrne	15, 0, r3, cr2, cr0, {0}
 24c:	1e031f10 	mcrne	15, 0, r1, cr3, cr0, {0}
 250:	ee010f10 	mcr	15, 0, r0, cr1, cr0, {0}
 254:	ee110f10 	mrc	15, 0, r0, cr1, cr0, {0}
 258:	e3a00000 	mov	r0, #0
 25c:	ee070f95 	mcr	15, 0, r0, cr7, cr5, {4}
 260:	e1a0f00c 	mov	pc, ip
 264:	e1a00000 	nop			; (mov r0, r0)
 268:	e1a00000 	nop			; (mov r0, r0)
 26c:	e1a00000 	nop			; (mov r0, r0)
 270:	e1a00000 	nop			; (mov r0, r0)
 274:	e1a00000 	nop			; (mov r0, r0)
 278:	e1a00000 	nop			; (mov r0, r0)
 27c:	e1a00000 	nop			; (mov r0, r0)

00000280 <reloc_start>:
 280:	e0859000 	add	r9, r5, r0
 284:	e2499080 	sub	r9, r9, #128	; 0x80
 288:	e1a01004 	mov	r1, r4
 28c:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 290:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 294:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 298:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 29c:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 2a0:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 2a4:	e8b55c0d 	ldm	r5!, {r0, r2, r3, sl, fp, ip, lr}
 2a8:	e8a15c0d 	stmia	r1!, {r0, r2, r3, sl, fp, ip, lr}
 2ac:	e1550009 	cmp	r5, r9
 2b0:	3afffff5 	bcc	28c <reloc_start+0xc>
 2b4:	e1a0d001 	mov	sp, r1
 2b8:	e28dd080 	add	sp, sp, #128	; 0x80

000002bc <call_kernel>:
 2bc:	eb000027 	bl	360 <cache_clean_flush>
 2c0:	eb000016 	bl	320 <cache_off>
 2c4:	e3a00000 	mov	r0, #0
 2c8:	e1a01007 	mov	r1, r7
 2cc:	e1a02008 	mov	r2, r8
 2d0:	e1a0f004 	mov	pc, r4

000002d4 <call_cache_fn>:
 2d4:	e28fc01c 	add	ip, pc, #28
 2d8:	ee109f10 	mrc	15, 0, r9, cr0, cr0, {0}
 2dc:	e59c1000 	ldr	r1, [ip]
 2e0:	e59c2004 	ldr	r2, [ip, #4]
 2e4:	e0211009 	eor	r1, r1, r9
 2e8:	e1110002 	tst	r1, r2
 2ec:	008cf003 	addeq	pc, ip, r3
 2f0:	e28cc014 	add	ip, ip, #20
 2f4:	eafffff8 	b	2dc <call_cache_fn+0x8>

000002f8 <proc_types>:
 2f8:	000f0000 	andeq	r0, pc, r0
 2fc:	000f0000 	andeq	r0, pc, r0
 300:	eaffffc3 	b	214 <__armv7_mmu_cache_on>
 304:	ea000007 	b	328 <__armv7_mmu_cache_off>
 308:	ea000016 	b	368 <__armv7_mmu_cache_flush>
 30c:	e1a00000 	nop			; (mov r0, r0)
 310:	e1a00000 	nop			; (mov r0, r0)
 314:	e1a00000 	nop			; (mov r0, r0)
 318:	e1a00000 	nop			; (mov r0, r0)
 31c:	e1a00000 	nop			; (mov r0, r0)

00000320 <cache_off>:
 320:	e3a0300c 	mov	r3, #12
 324:	eaffffea 	b	2d4 <call_cache_fn>

00000328 <__armv7_mmu_cache_off>:
 328:	ee110f10 	mrc	15, 0, r0, cr1, cr0, {0}
 32c:	e3c0000d 	bic	r0, r0, #13
 330:	ee010f10 	mcr	15, 0, r0, cr1, cr0, {0}
 334:	e1a0c00e 	mov	ip, lr
 338:	eb00000a 	bl	368 <__armv7_mmu_cache_flush>
 33c:	e3a00000 	mov	r0, #0
 340:	ee080f17 	mcr	15, 0, r0, cr8, cr7, {0}
 344:	ee070fd5 	mcr	15, 0, r0, cr7, cr5, {6}
 348:	ee070f9a 	mcr	15, 0, r0, cr7, cr10, {4}
 34c:	ee070f95 	mcr	15, 0, r0, cr7, cr5, {4}
 350:	e1a0f00c 	mov	pc, ip
 354:	e1a00000 	nop			; (mov r0, r0)
 358:	e1a00000 	nop			; (mov r0, r0)
 35c:	e1a00000 	nop			; (mov r0, r0)

00000360 <cache_clean_flush>:
 360:	e3a03010 	mov	r3, #16
 364:	eaffffda 	b	2d4 <call_cache_fn>

00000368 <__armv7_mmu_cache_flush>:
 368:	ee10afb1 	mrc	15, 0, sl, cr0, cr1, {5}
 36c:	e31a080f 	tst	sl, #983040	; 0xf0000
 370:	e3a0a000 	mov	sl, #0
 374:	0a000001 	beq	380 <hierarchical>
 378:	ee07af1e 	mcr	15, 0, sl, cr7, cr14, {0}
 37c:	ea000023 	b	410 <iflush>

00000380 <hierarchical>:
 380:	ee07afba 	mcr	15, 0, sl, cr7, cr10, {5}
 384:	e92d0eff 	push	{r0, r1, r2, r3, r4, r5, r6, r7, r9, sl, fp}
 388:	ee300f30 	mrc	15, 1, r0, cr0, cr0, {1}
 38c:	e2103407 	ands	r3, r0, #117440512	; 0x7000000
 390:	e1a03ba3 	lsr	r3, r3, #23
 394:	0a00001a 	beq	404 <finished>
 398:	e3a0a000 	mov	sl, #0

0000039c <loop1>:
 39c:	e08a20aa 	add	r2, sl, sl, lsr #1
 3a0:	e1a01230 	lsr	r1, r0, r2
 3a4:	e2011007 	and	r1, r1, #7
 3a8:	e3510002 	cmp	r1, #2
 3ac:	ba000011 	blt	3f8 <skip>
 3b0:	ee40af10 	mcr	15, 2, sl, cr0, cr0, {0}
 3b4:	ee07af95 	mcr	15, 0, sl, cr7, cr5, {4}
 3b8:	ee301f10 	mrc	15, 1, r1, cr0, cr0, {0}
 3bc:	e2012007 	and	r2, r1, #7
 3c0:	e2822004 	add	r2, r2, #4
 3c4:	e59f405c 	ldr	r4, [pc, #92]	; 428 <iflush+0x18>
 3c8:	e01441a1 	ands	r4, r4, r1, lsr #3
 3cc:	e16f5f14 	clz	r5, r4
 3d0:	e59f7054 	ldr	r7, [pc, #84]	; 42c <iflush+0x1c>
 3d4:	e01776a1 	ands	r7, r7, r1, lsr #13

000003d8 <loop2>:
 3d8:	e1a09004 	mov	r9, r4

000003dc <loop3>:
 3dc:	e18ab519 	orr	fp, sl, r9, lsl r5
 3e0:	e18bb217 	orr	fp, fp, r7, lsl r2
 3e4:	ee07bf5e 	mcr	15, 0, fp, cr7, cr14, {2}
 3e8:	e2599001 	subs	r9, r9, #1
 3ec:	aafffffa 	bge	3dc <loop3>
 3f0:	e2577001 	subs	r7, r7, #1
 3f4:	aafffff7 	bge	3d8 <loop2>

000003f8 <skip>:
 3f8:	e28aa002 	add	sl, sl, #2
 3fc:	e153000a 	cmp	r3, sl
 400:	caffffe5 	bgt	39c <loop1>

00000404 <finished>:
 404:	e8bd0eff 	pop	{r0, r1, r2, r3, r4, r5, r6, r7, r9, sl, fp}
 408:	e3a0a000 	mov	sl, #0
 40c:	ee40af10 	mcr	15, 2, sl, cr0, cr0, {0}

00000410 <iflush>:
 410:	ee07af9a 	mcr	15, 0, sl, cr7, cr10, {4}
 414:	ee07af15 	mcr	15, 0, sl, cr7, cr5, {0}
 418:	ee07af9a 	mcr	15, 0, sl, cr7, cr10, {4}
 41c:	ee07af95 	mcr	15, 0, sl, cr7, cr5, {4}
 420:	e1a0f00e 	mov	pc, lr
 424:	c0016000 	andgt	r6, r1, r0
 428:	000003ff 	strdeq	r0, [r0], -pc	; <UNPREDICTABLE>
 42c:	00007fff 	strdeq	r7, [r0], -pc	; <UNPREDICTABLE>

00000430 <reloc_end>:
 430:	e1a00000 	nop			; (mov r0, r0)
 434:	e1a00000 	nop			; (mov r0, r0)
 438:	e1a00000 	nop			; (mov r0, r0)
 43c:	e1a00000 	nop			; (mov r0, r0)

Disassembly of section .start:

00000000 <start>:
   0:	ea000002 	b	10 <start+0x10>
   4:	016f2818 	cmneq	pc, r8, lsl r8	; <UNPREDICTABLE>
	...
  10:	e1a07001 	mov	r7, r1
  14:	e1a08002 	mov	r8, r2
  18:	e10f2000 	mrs	r2, CPSR
  1c:	e38220c0 	orr	r2, r2, #192	; 0xc0
  20:	e121f002 	msr	CPSR_c, r2

Disassembly of section .stack:

00000000 <user_stack>:
	...

Disassembly of section .ARM.attributes:

00000000 <.ARM.attributes>:
   0:	00000f41 	andeq	r0, r0, r1, asr #30
   4:	61656100 	cmnvs	r5, r0, lsl #2
   8:	01006962 	tsteq	r0, r2, ror #18
   c:	00000005 	andeq	r0, r0, r5
