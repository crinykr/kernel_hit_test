
misc.o:     file format elf32-littlearm


Disassembly of section .text:

00000000 <putstr>:
   0:	e1a0c00d 	mov	ip, sp
   4:	e92dd800 	push	{fp, ip, lr, pc}
   8:	e24cb004 	sub	fp, ip, #4	; 0x4
   c:	ea00000d 	b	48 <putstr+0x48>
  10:	e352000a 	cmp	r2, #10	; 0xa
  14:	1a000005 	bne	30 <putstr+0x30>
  18:	e59f1038 	ldr	r1, [pc, #56]	; 58 <putstr+0x58>
  1c:	e1d130bc 	ldrh	r3, [r1, #12]
  20:	e3130c02 	tst	r3, #512	; 0x200
  24:	1afffffb 	bne	18 <putstr+0x18>
  28:	e3a0300d 	mov	r3, #13	; 0xd
  2c:	e1c131b0 	strh	r3, [r1, #16]
  30:	e59f1020 	ldr	r1, [pc, #32]	; 58 <putstr+0x58>
  34:	e1d130bc 	ldrh	r3, [r1, #12]
  38:	e3130c02 	tst	r3, #512	; 0x200
  3c:	1afffffb 	bne	30 <putstr+0x30>
  40:	e6ff3072 	uxth	r3, r2
  44:	e1c131b0 	strh	r3, [r1, #16]
  48:	e4d02001 	ldrb	r2, [r0], #1
  4c:	e3520000 	cmp	r2, #0	; 0x0
  50:	1affffee 	bne	10 <putstr+0x10>
  54:	e89da800 	ldm	sp, {fp, sp, pc}
  58:	c0016000 	.word	0xc0016000

0000005c <memcpy>:
  5c:	e1a0c00d 	mov	ip, sp
  60:	e92dd870 	push	{r4, r5, r6, fp, ip, lr, pc}
  64:	e24cb004 	sub	fp, ip, #4	; 0x4
  68:	e1a05002 	mov	r5, r2
  6c:	e1a06000 	mov	r6, r0
  70:	e1a0e001 	mov	lr, r1
  74:	e1a04000 	mov	r4, r0
  78:	e1a021a2 	lsr	r2, r2, #3
  7c:	ea000011 	b	c8 <memcpy+0x6c>
  80:	e5de3000 	ldrb	r3, [lr]
  84:	e5c43000 	strb	r3, [r4]
  88:	e5de3001 	ldrb	r3, [lr, #1]
  8c:	e5c43001 	strb	r3, [r4, #1]
  90:	e5de3002 	ldrb	r3, [lr, #2]
  94:	e5c43002 	strb	r3, [r4, #2]
  98:	e5de3003 	ldrb	r3, [lr, #3]
  9c:	e5c43003 	strb	r3, [r4, #3]
  a0:	e5de3004 	ldrb	r3, [lr, #4]
  a4:	e5c43004 	strb	r3, [r4, #4]
  a8:	e5de3005 	ldrb	r3, [lr, #5]
  ac:	e5c43005 	strb	r3, [r4, #5]
  b0:	e5de3006 	ldrb	r3, [lr, #6]
  b4:	e5c43006 	strb	r3, [r4, #6]
  b8:	e5de3007 	ldrb	r3, [lr, #7]
  bc:	e28ee008 	add	lr, lr, #8	; 0x8
  c0:	e5c43007 	strb	r3, [r4, #7]
  c4:	e2844008 	add	r4, r4, #8	; 0x8
  c8:	e3520000 	cmp	r2, #0	; 0x0
  cc:	e2422001 	sub	r2, r2, #1	; 0x1
  d0:	caffffea 	bgt	80 <memcpy+0x24>
  d4:	e3150004 	tst	r5, #4	; 0x4
  d8:	0a00000d 	beq	114 <memcpy+0xb8>
  dc:	e1a0200e 	mov	r2, lr
  e0:	e1a01004 	mov	r1, r4
  e4:	e4d23001 	ldrb	r3, [r2], #1
  e8:	e4c13001 	strb	r3, [r1], #1
  ec:	e5de3001 	ldrb	r3, [lr, #1]
  f0:	e2820001 	add	r0, r2, #1	; 0x1
  f4:	e281c001 	add	ip, r1, #1	; 0x1
  f8:	e5c43001 	strb	r3, [r4, #1]
  fc:	e5d23001 	ldrb	r3, [r2, #1]
 100:	e28c4002 	add	r4, ip, #2	; 0x2
 104:	e280e002 	add	lr, r0, #2	; 0x2
 108:	e5c13001 	strb	r3, [r1, #1]
 10c:	e5d03001 	ldrb	r3, [r0, #1]
 110:	e5cc3001 	strb	r3, [ip, #1]
 114:	e3150002 	tst	r5, #2	; 0x2
 118:	0a000007 	beq	13c <memcpy+0xe0>
 11c:	e1a0200e 	mov	r2, lr
 120:	e1a01004 	mov	r1, r4
 124:	e4d23001 	ldrb	r3, [r2], #1
 128:	e4c13001 	strb	r3, [r1], #1
 12c:	e5de3001 	ldrb	r3, [lr, #1]
 130:	e282e001 	add	lr, r2, #1	; 0x1
 134:	e5c43001 	strb	r3, [r4, #1]
 138:	e2814001 	add	r4, r1, #1	; 0x1
 13c:	e3150001 	tst	r5, #1	; 0x1
 140:	e1a00006 	mov	r0, r6
 144:	15de3000 	ldrbne	r3, [lr]
 148:	15c43000 	strbne	r3, [r4]
 14c:	e89da870 	ldm	sp, {r4, r5, r6, fp, sp, pc}

00000150 <error>:
 150:	e1a0c00d 	mov	ip, sp
 154:	e92ddc10 	push	{r4, sl, fp, ip, lr, pc}
 158:	e24cb004 	sub	fp, ip, #4	; 0x4
 15c:	e59fa028 	ldr	sl, [pc, #40]	; 18c <error+0x3c>
 160:	e08fa00a 	add	sl, pc, sl
 164:	e1a04000 	mov	r4, r0
 168:	e59f0020 	ldr	r0, [pc, #32]	; 190 <error+0x40>
 16c:	e08a0000 	add	r0, sl, r0
 170:	ebffffa2 	bl	0 <putstr>
 174:	e1a00004 	mov	r0, r4
 178:	ebffffa0 	bl	0 <putstr>
 17c:	e59f0010 	ldr	r0, [pc, #16]	; 194 <error+0x44>
 180:	e08a0000 	add	r0, sl, r0
 184:	ebffff9d 	bl	0 <putstr>
 188:	eafffffe 	b	188 <error+0x38>
 18c:	00000024 	.word	0x00000024
	...

00000198 <__div0>:
 198:	e1a0c00d 	mov	ip, sp
 19c:	e92ddc00 	push	{sl, fp, ip, lr, pc}
 1a0:	e24cb004 	sub	fp, ip, #4	; 0x4
 1a4:	e59fa010 	ldr	sl, [pc, #16]	; 1bc <__div0+0x24>
 1a8:	e08fa00a 	add	sl, pc, sl
 1ac:	e59f000c 	ldr	r0, [pc, #12]	; 1c0 <__div0+0x28>
 1b0:	e08a0000 	add	r0, sl, r0
 1b4:	ebfffffe 	bl	150 <error>
 1b8:	e89dac00 	ldm	sp, {sl, fp, sp, pc}
 1bc:	0000000c 	.word	0x0000000c
 1c0:	00000000 	.word	0x00000000

000001c4 <decompress_kernel>:
 1c4:	e1a0c00d 	mov	ip, sp
 1c8:	e92ddff0 	push	{r4, r5, r6, r7, r8, r9, sl, fp, ip, lr, pc}
 1cc:	e24cb004 	sub	fp, ip, #4	; 0x4
 1d0:	e59fa0fc 	ldr	sl, [pc, #252]	; 2d4 <decompress_kernel+0x110>
 1d4:	e24dd008 	sub	sp, sp, #8	; 0x8
 1d8:	e08fa00a 	add	sl, pc, sl
 1dc:	e59fc0f4 	ldr	ip, [pc, #244]	; 2d8 <decompress_kernel+0x114>
 1e0:	e59f50f4 	ldr	r5, [pc, #244]	; 2dc <decompress_kernel+0x118>
 1e4:	e79a700c 	ldr	r7, [sl, ip]
 1e8:	e3a0c003 	mov	ip, #3	; 0x3
 1ec:	e1c5c0b0 	strh	ip, [r5]
 1f0:	e3a0c005 	mov	ip, #5	; 0x5
 1f4:	e1c5c0b2 	strh	ip, [r5, #2]
 1f8:	e59fc0e0 	ldr	ip, [pc, #224]	; 2e0 <decompress_kernel+0x11c>
 1fc:	e5574002 	ldrb	r4, [r7, #-2]
 200:	e1d5e0b4 	ldrh	lr, [r5, #4]
 204:	e79ac00c 	ldr	ip, [sl, ip]
 208:	e5578003 	ldrb	r8, [r7, #-3]
 20c:	e5579001 	ldrb	r9, [r7, #-1]
 210:	e5576004 	ldrb	r6, [r7, #-4]
 214:	e50bc02c 	str	ip, [fp, #-44]
 218:	e59fc0c4 	ldr	ip, [pc, #196]	; 2e4 <decompress_kernel+0x120>
 21c:	e38ee007 	orr	lr, lr, #7	; 0x7
 220:	e1c5e0b4 	strh	lr, [r5, #4]
 224:	e79ac00c 	ldr	ip, [sl, ip]
 228:	e1a04804 	lsl	r4, r4, #16
 22c:	e50bc030 	str	ip, [fp, #-48]
 230:	e51bc02c 	ldr	ip, [fp, #-44]
 234:	e595e040 	ldr	lr, [r5, #64]
 238:	e1866408 	orr	r6, r6, r8, lsl #8
 23c:	e58c0000 	str	r0, [ip]
 240:	e59fc0a0 	ldr	ip, [pc, #160]	; 2e8 <decompress_kernel+0x124>
 244:	e38ee00c 	orr	lr, lr, #12	; 0xc
 248:	e1844c09 	orr	r4, r4, r9, lsl #24
 24c:	e79ac00c 	ldr	ip, [sl, ip]
 250:	e1866004 	orr	r6, r6, r4
 254:	e59f0090 	ldr	r0, [pc, #144]	; 2ec <decompress_kernel+0x128>
 258:	e58c1000 	str	r1, [ip]
 25c:	e59f108c 	ldr	r1, [pc, #140]	; 2f0 <decompress_kernel+0x12c>
 260:	e3a0c003 	mov	ip, #3	; 0x3
 264:	e08a0000 	add	r0, sl, r0
 268:	e79a1001 	ldr	r1, [sl, r1]
 26c:	e5812000 	str	r2, [r1]
 270:	e59f207c 	ldr	r2, [pc, #124]	; 2f4 <decompress_kernel+0x130>
 274:	e79a2002 	ldr	r2, [sl, r2]
 278:	e5823000 	str	r3, [r2]
 27c:	e585e040 	str	lr, [r5, #64]
 280:	e51b3030 	ldr	r3, [fp, #-48]
 284:	e5836000 	str	r6, [r3]
 288:	e3a03fc9 	mov	r3, #804	; 0x324
 28c:	e5853044 	str	r3, [r5, #68]
 290:	e1c5c1b4 	strh	ip, [r5, #20]
 294:	ebffff59 	bl	0 <putstr>
 298:	e51b302c 	ldr	r3, [fp, #-44]
 29c:	e59f1054 	ldr	r1, [pc, #84]	; 2f8 <decompress_kernel+0x134>
 2a0:	e5932000 	ldr	r2, [r3]
 2a4:	e59f3050 	ldr	r3, [pc, #80]	; 2fc <decompress_kernel+0x138>
 2a8:	e79a0001 	ldr	r0, [sl, r1]
 2ac:	e79a3003 	ldr	r3, [sl, r3]
 2b0:	e0601007 	rsb	r1, r0, r7
 2b4:	ebfffffe 	bl	0 <do_decompress>
 2b8:	e59f0040 	ldr	r0, [pc, #64]	; 300 <decompress_kernel+0x13c>
 2bc:	e08a0000 	add	r0, sl, r0
 2c0:	ebffff4e 	bl	0 <putstr>
 2c4:	e51bc030 	ldr	ip, [fp, #-48]
 2c8:	e59c0000 	ldr	r0, [ip]
 2cc:	e24bd028 	sub	sp, fp, #40	; 0x28
 2d0:	e89daff0 	ldm	sp, {r4, r5, r6, r7, r8, r9, sl, fp, sp, pc}
 2d4:	000000f4 	.word	0x000000f4
 2d8:	00000000 	.word	0x00000000
 2dc:	c0016000 	.word	0xc0016000
	...

Disassembly of section .bss:

00000000 <__machine_arch_type>:
   0:	00000000 	andeq	r0, r0, r0

00000004 <output_data>:
   4:	00000000 	andeq	r0, r0, r0

00000008 <output_ptr>:
   8:	00000000 	andeq	r0, r0, r0

0000000c <free_mem_ptr>:
   c:	00000000 	andeq	r0, r0, r0

00000010 <free_mem_end_ptr>:
  10:	00000000 	andeq	r0, r0, r0

Disassembly of section .rodata.str1.4:

00000000 <.LC0>:
   0:	00000a0a 	andeq	r0, r0, sl, lsl #20

00000004 <.LC1>:
   4:	2d200a0a 	fstmdbscs	r0!, {s0-s9}
   8:	7953202d 	ldmdbvc	r3, {r0, r2, r3, r5, sp}^
   c:	6d657473 	cfstrdvs	mvd7, [r5, #-460]!
  10:	6c616820 	stclvs	8, cr6, [r1], #-128
  14:	00646574 	rsbeq	r6, r4, r4, ror r5

00000018 <.LC2>:
  18:	65747441 	ldrbvs	r7, [r4, #-1089]!
  1c:	6974706d 	ldmdbvs	r4!, {r0, r2, r3, r5, r6, ip, sp, lr}^
  20:	6420676e 	strtvs	r6, [r0], #-1902
  24:	73697669 	cmnvc	r9, #110100480	; 0x6900000
  28:	206e6f69 	rsbcs	r6, lr, r9, ror #30
  2c:	30207962 	eorcc	r7, r0, r2, ror #18
  30:	00000021 	andeq	r0, r0, r1, lsr #32

00000034 <.LC3>:
  34:	6f636e55 	svcvs	0x00636e55
  38:	6572706d 	ldrbvs	r7, [r2, #-109]!
  3c:	6e697373 	mcrvs	3, 3, r7, cr9, cr3, {3}
  40:	694c2067 	stmdbvs	ip, {r0, r1, r2, r5, r6, sp}^
  44:	2e78756e 	cdpcs	5, 7, cr7, cr8, cr14, {3}
  48:	00002e2e 	andeq	r2, r0, lr, lsr #28

0000004c <.LC4>:
  4c:	6e6f6420 	cdpvs	4, 6, cr6, cr15, cr0, {1}
  50:	62202c65 	eorvs	r2, r0, #25856	; 0x6500
  54:	69746f6f 	ldmdbvs	r4!, {r0, r1, r2, r3, r5, r6, r8, r9, sl, fp, sp, lr}^
  58:	7420676e 	strtvc	r6, [r0], #-1902
  5c:	6b206568 	blvs	819604 <decompress_kernel+0x819440>
  60:	656e7265 	strbvs	r7, [lr, #-613]!
  64:	000a2e6c 	andeq	r2, sl, ip, ror #28

Disassembly of section .comment:

00000000 <.comment>:
   0:	43434700 	movtmi	r4, #14080	; 0x3700
   4:	4728203a 	undefined
   8:	2029554e 	eorcs	r5, r9, lr, asr #10
   c:	2e312e34 	mrccs	14, 1, r2, cr1, cr4, {1}
  10:	Address 0x00000010 is out of bounds.


Disassembly of section .ARM.attributes:

00000000 <.ARM.attributes>:
   0:	00000f41 	andeq	r0, r0, r1, asr #30
   4:	61656100 	cmnvs	r5, r0, lsl #2
   8:	01006962 	tsteq	r0, r2, ror #18
   c:	00000005 	andeq	r0, r0, r5
