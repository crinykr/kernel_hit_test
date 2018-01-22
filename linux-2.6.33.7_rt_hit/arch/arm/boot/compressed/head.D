
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
  38:	e3a00000 	mov	r0, #0	; 0x0
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
  b0:	000000b0 	.word	0x000000b0
	...
  d0:	00001000 	.word	0x00001000

000000d4 <LC1>:
  d4:	00000100 	.word	0x00000100
  d8:	e1a00000 	.word	0xe1a00000
  dc:	e1a00000 	.word	0xe1a00000

000000e0 <cache_on>:
  e0:	e3a03008 	mov	r3, #8	; 0x8
  e4:	ea00004a 	b	214 <call_cache_fn>

000000e8 <__setup_mmu>:
  e8:	e2443901 	sub	r3, r4, #16384	; 0x4000
  ec:	e3c330ff 	bic	r3, r3, #255	; 0xff
  f0:	e3c33c3f 	bic	r3, r3, #16128	; 0x3f00
  f4:	e1a00003 	mov	r0, r3
  f8:	e1a09920 	lsr	r9, r0, #18
  fc:	e1a09909 	lsl	r9, r9, #18
 100:	e289a201 	add	sl, r9, #268435456	; 0x10000000
 104:	e3a01012 	mov	r1, #18	; 0x12
 108:	e3811b03 	orr	r1, r1, #3072	; 0xc00
 10c:	e2832901 	add	r2, r3, #16384	; 0x4000
 110:	e1510009 	cmp	r1, r9
 114:	2381100c 	orrcs	r1, r1, #12	; 0xc
 118:	e151000a 	cmp	r1, sl
 11c:	23c1100c 	biccs	r1, r1, #12	; 0xc
 120:	e4801004 	str	r1, [r0], #4
 124:	e2811601 	add	r1, r1, #1048576	; 0x100000
 128:	e1300002 	teq	r0, r2
 12c:	1afffff7 	bne	110 <__setup_mmu+0x28>
 130:	e3a0101e 	mov	r1, #30	; 0x1e
 134:	e3811b03 	orr	r1, r1, #3072	; 0xc00
 138:	e1a02a2f 	lsr	r2, pc, #20
 13c:	e1811a02 	orr	r1, r1, r2, lsl #20
 140:	e0830102 	add	r0, r3, r2, lsl #2
 144:	e4801004 	str	r1, [r0], #4
 148:	e2811601 	add	r1, r1, #1048576	; 0x100000
 14c:	e5801000 	str	r1, [r0]
 150:	e1a0f00e 	mov	pc, lr

00000154 <__armv4_mmu_cache_on>:
 154:	e1a0c00e 	.word	0xe1a0c00e
 158:	ebffffe2 	.word	0xebffffe2
 15c:	e3a00000 	.word	0xe3a00000
 160:	ee070f9a 	.word	0xee070f9a
 164:	ee080f17 	.word	0xee080f17
 168:	ee110f10 	.word	0xee110f10
 16c:	e3800a05 	.word	0xe3800a05
 170:	e3800030 	.word	0xe3800030
 174:	eb000002 	.word	0xeb000002
 178:	e3a00000 	.word	0xe3a00000
 17c:	ee080f17 	.word	0xee080f17
 180:	e1a0f00c 	.word	0xe1a0f00c

00000184 <__common_mmu_cache_on>:
 184:	e380000d 	.word	0xe380000d
 188:	e3e01000 	.word	0xe3e01000
 18c:	ee023f10 	.word	0xee023f10
 190:	ee031f10 	.word	0xee031f10
 194:	ea000001 	.word	0xea000001
 198:	e1a00000 	.word	0xe1a00000
 19c:	e1a00000 	.word	0xe1a00000
 1a0:	ee010f10 	.word	0xee010f10
 1a4:	ee110f10 	.word	0xee110f10
 1a8:	e04ef020 	.word	0xe04ef020
 1ac:	e1a00000 	.word	0xe1a00000
 1b0:	e1a00000 	.word	0xe1a00000
 1b4:	e1a00000 	.word	0xe1a00000
 1b8:	e1a00000 	.word	0xe1a00000
 1bc:	e1a00000 	.word	0xe1a00000

000001c0 <reloc_start>:
 1c0:	e0859000 	.word	0xe0859000
 1c4:	e2499080 	.word	0xe2499080
 1c8:	e1a01004 	.word	0xe1a01004
 1cc:	e8b55c0d 	.word	0xe8b55c0d
 1d0:	e8a15c0d 	.word	0xe8a15c0d
 1d4:	e8b55c0d 	.word	0xe8b55c0d
 1d8:	e8a15c0d 	.word	0xe8a15c0d
 1dc:	e8b55c0d 	.word	0xe8b55c0d
 1e0:	e8a15c0d 	.word	0xe8a15c0d
 1e4:	e8b55c0d 	.word	0xe8b55c0d
 1e8:	e8a15c0d 	.word	0xe8a15c0d
 1ec:	e1550009 	.word	0xe1550009
 1f0:	3afffff5 	.word	0x3afffff5
 1f4:	e1a0d001 	.word	0xe1a0d001
 1f8:	e28dd080 	.word	0xe28dd080

000001fc <call_kernel>:
 1fc:	eb000027 	.word	0xeb000027
 200:	eb000016 	.word	0xeb000016
 204:	e3a00000 	.word	0xe3a00000
 208:	e1a01007 	.word	0xe1a01007
 20c:	e1a02008 	.word	0xe1a02008
 210:	e1a0f004 	.word	0xe1a0f004

00000214 <call_cache_fn>:
 214:	e28fc01c 	.word	0xe28fc01c
 218:	ee109f10 	.word	0xee109f10
 21c:	e59c1000 	.word	0xe59c1000
 220:	e59c2004 	.word	0xe59c2004
 224:	e0211009 	.word	0xe0211009
 228:	e1110002 	.word	0xe1110002
 22c:	008cf003 	.word	0x008cf003
 230:	e28cc014 	.word	0xe28cc014
 234:	eafffff8 	.word	0xeafffff8

00000238 <proc_types>:
 238:	000f0000 	.word	0x000f0000
 23c:	000f0000 	.word	0x000f0000
 240:	eaffffc3 	b	154 <__armv4_mmu_cache_on>
 244:	ea000007 	b	268 <__armv4_mmu_cache_off>
 248:	ea000016 	b	2a8 <__armv6_mmu_cache_flush>
 24c:	e1a00000 	nop			(mov r0,r0)
 250:	e1a00000 	nop			(mov r0,r0)
 254:	e1a00000 	nop			(mov r0,r0)
 258:	e1a00000 	nop			(mov r0,r0)
 25c:	e1a00000 	nop			(mov r0,r0)

00000260 <cache_off>:
 260:	e3a0300c 	mov	r3, #12	; 0xc
 264:	eaffffea 	b	214 <call_cache_fn>

00000268 <__armv4_mmu_cache_off>:
 268:	ee110f10 	mrc	15, 0, r0, cr1, cr0, {0}
 26c:	e3c0000d 	bic	r0, r0, #13	; 0xd
 270:	ee010f10 	mcr	15, 0, r0, cr1, cr0, {0}
 274:	e3a00000 	mov	r0, #0	; 0x0
 278:	ee070f17 	mcr	15, 0, r0, cr7, cr7, {0}
 27c:	ee080f17 	mcr	15, 0, r0, cr8, cr7, {0}
 280:	e1a0f00e 	mov	pc, lr
 284:	e1a00000 	nop			(mov r0,r0)
 288:	e1a00000 	nop			(mov r0,r0)
 28c:	e1a00000 	nop			(mov r0,r0)
 290:	e1a00000 	nop			(mov r0,r0)
 294:	e1a00000 	nop			(mov r0,r0)
 298:	e1a00000 	nop			(mov r0,r0)
 29c:	e1a00000 	nop			(mov r0,r0)

000002a0 <cache_clean_flush>:
 2a0:	e3a03010 	mov	r3, #16	; 0x10
 2a4:	eaffffda 	b	214 <call_cache_fn>

000002a8 <__armv6_mmu_cache_flush>:
 2a8:	e3a01000 	mov	r1, #0	; 0x0
 2ac:	ee071f1e 	mcr	15, 0, r1, cr7, cr14, {0}
 2b0:	ee071f15 	mcr	15, 0, r1, cr7, cr5, {0}
 2b4:	ee071f1f 	mcr	15, 0, r1, cr7, cr15, {0}
 2b8:	ee071f9a 	mcr	15, 0, r1, cr7, cr10, {4}
 2bc:	e1a0f00e 	mov	pc, lr

Disassembly of section .start:

00000000 <start>:
   0:	ea000005 	b	1c <start+0x1c>
   4:	016f2818 	.word	0x016f2818
	...
  10:	12345678 	.word	0x12345678
  14:	12345678 	.word	0x12345678
  18:	12345678 	.word	0x12345678
  1c:	e1a07001 	mov	r7, r1
  20:	e1a08002 	mov	r8, r2
  24:	e3a00041 	mov	r0, #65	; 0x41
  28:	e59f20e4 	ldr	r2, [pc, #228]	; 114 <start+0x114>
  2c:	e1d230bc 	ldrh	r3, [r2, #12]
  30:	e3130c02 	tst	r3, #512	; 0x200
  34:	1afffffb 	bne	28 <start+0x28>
  38:	e6ff3070 	uxth	r3, r0
  3c:	e1c231b0 	strh	r3, [r2, #16]
  40:	e59f20cc 	ldr	r2, [pc, #204]	; 114 <start+0x114>
  44:	e1d230bc 	ldrh	r3, [r2, #12]
  48:	e3130c02 	tst	r3, #512	; 0x200
  4c:	1afffffb 	bne	40 <start+0x40>
  50:	e3a0003a 	mov	r0, #58	; 0x3a
  54:	e59f20b8 	ldr	r2, [pc, #184]	; 114 <start+0x114>
  58:	e1d230bc 	ldrh	r3, [r2, #12]
  5c:	e3130c02 	tst	r3, #512	; 0x200
  60:	1afffffb 	bne	54 <start+0x54>
  64:	e6ff3070 	uxth	r3, r0
  68:	e1c231b0 	strh	r3, [r2, #16]
  6c:	e59f20a0 	ldr	r2, [pc, #160]	; 114 <start+0x114>
  70:	e1d230bc 	ldrh	r3, [r2, #12]
  74:	e3130c02 	tst	r3, #512	; 0x200
  78:	1afffffb 	bne	6c <start+0x6c>
  7c:	e1a0000f 	mov	r0, pc
  80:	e2400028 	sub	r0, r0, #40	; 0x28
  84:	e5900000 	ldr	r0, [r0]
  88:	e1a00000 	nop			(mov r0,r0)
  8c:	e3a01007 	mov	r1, #7	; 0x7
  90:	e59f207c 	ldr	r2, [pc, #124]	; 114 <start+0x114>
  94:	e1a03101 	lsl	r3, r1, #2
  98:	e1d220bc 	ldrh	r2, [r2, #12]
  9c:	e1a03350 	asr	r3, r0, r3
  a0:	e3120c02 	tst	r2, #512	; 0x200
  a4:	e203300f 	and	r3, r3, #15	; 0xf
  a8:	1afffff8 	bne	90 <start+0x90>
  ac:	e3530009 	cmp	r3, #9	; 0x9
  b0:	e2832037 	add	r2, r3, #55	; 0x37
  b4:	d2832030 	addle	r2, r3, #48	; 0x30
  b8:	e59f3054 	ldr	r3, [pc, #84]	; 114 <start+0x114>
  bc:	e2511001 	subs	r1, r1, #1	; 0x1
  c0:	e1c321b0 	strh	r2, [r3, #16]
  c4:	5afffff1 	bpl	90 <start+0x90>
  c8:	e59f3044 	ldr	r3, [pc, #68]	; 114 <start+0x114>
  cc:	e1d330bc 	ldrh	r3, [r3, #12]
  d0:	e3130c02 	tst	r3, #512	; 0x200
  d4:	1afffffb 	bne	c8 <start+0xc8>
  d8:	e3a0000a 	mov	r0, #10	; 0xa
  dc:	e59f2030 	ldr	r2, [pc, #48]	; 114 <start+0x114>
  e0:	e1d230bc 	ldrh	r3, [r2, #12]
  e4:	e3130c02 	tst	r3, #512	; 0x200
  e8:	1afffffb 	bne	dc <start+0xdc>
  ec:	e6ff3070 	uxth	r3, r0
  f0:	e1c231b0 	strh	r3, [r2, #16]
  f4:	e59f2018 	ldr	r2, [pc, #24]	; 114 <start+0x114>
  f8:	e1d230bc 	ldrh	r3, [r2, #12]
  fc:	e3130c02 	tst	r3, #512	; 0x200
 100:	1afffffb 	bne	f4 <start+0xf4>
 104:	eafffffe 	b	104 <start+0x104>
 108:	e10f2000 	mrs	r2, CPSR
 10c:	e38220c0 	orr	r2, r2, #192	; 0xc0
 110:	e121f002 	msr	CPSR_c, r2
 114:	c0016000 	.word	0xc0016000

Disassembly of section .stack:

00000000 <user_stack>:
	...

Disassembly of section .ARM.attributes:

00000000 <.ARM.attributes>:
   0:	00000f41 	andeq	r0, r0, r1, asr #30
   4:	61656100 	cmnvs	r5, r0, lsl #2
   8:	01006962 	tsteq	r0, r2, ror #18
   c:	00000005 	andeq	r0, r0, r5
