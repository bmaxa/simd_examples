format elf64
;default rel
extrn posix_memalign
extrn malloc
extrn free
public testSSEasm
public testAVX2asm
public avx2sort
public bitonic_compare_avx2
public bitonic_sort_avx2
public radix_sort_avx2
public radix_bernard

section '.text' executable align 16
align 16
testSSEasm:
	xor ecx,ecx
	vpxor xmm4,xmm4,xmm4
.L0:
	vmovdqu xmm0,[rdi+rcx*4]
	vmovdqu xmm1,[rsi+rcx*4]
	vpaddd xmm3,xmm0,xmm1
	vpmaxsd xmm4,xmm4,xmm3
	add rcx,4
	cmp rcx,rdx
	jl .L0
	
	vpshufd xmm0,xmm4,0xe
	vpmaxsd xmm4,xmm4,xmm0
	vpshufd xmm0,xmm4,1
	vpmaxsd xmm4,xmm4,xmm0
	vpextrd eax,xmm4,0
	ret
align 16
testAVX2asm:
	xor ecx,ecx
	vpxor ymm4,ymm4,ymm4
.L0:
	vmovdqu ymm0,[rdi+rcx*4]
	vmovdqu ymm1,[rsi+rcx*4]
	vpaddd ymm3,ymm0,ymm1
	vpmaxsd ymm4,ymm4,ymm3
	add rcx,8
	cmp rcx,rdx
	jl .L0
	
	vperm2i128 ymm0,ymm4,ymm4,1
	vpmaxsd ymm4,ymm4,ymm0
	vpermq ymm0,ymm4,1
	vpmaxsd ymm4,ymm4,ymm0
	vpshufd ymm0,ymm4,1
	vpmaxsd ymm4,ymm4,ymm0
	vmovd eax,xmm4
	ret	
align 16
bitonic_sort_avx2:
	cmp rdx,1
	jle .exit
	push rdi
	push rsi
	push rdx
	shr rdx,1
	mov rdi,1
	call bitonic_sort_avx2
	mov rdx,[rsp]
	mov rsi,[rsp+8]
	mov rdi,[rsp+16]
	shr rdx,1
	lea rsi,[rsi+rdx*4]
	mov rdi,0
	call bitonic_sort_avx2
	pop rdx
	pop rsi
	pop rdi
	jmp bitonic_merge_avx2
.exit:
	ret
align 16
bitonic_merge_avx2:
	cmp rdx,1
	jle .exit
	push rdi
	push rsi
	push rdx
	call bitonic_compare_avx2
	mov rdx,[rsp]
	mov rsi,[rsp+8]
	mov rdi,[rsp+16]
	shr rdx,1
	call bitonic_merge_avx2
	pop rdx
	pop rsi
	pop rdi
	shr rdx,1
	lea rsi,[rsi+rdx*4] 
	jmp bitonic_merge_avx2
.exit:
	ret
align 16
bitonic_compare_avx2:
	shr rdx,1
	mov rcx,rdx
	shr rcx,3
	mov r8,rcx
	shl r8,3
	mov r9,rdx
	sub r9,r8
	test rdi,rdi
	jz .down
.up:
	test r9,r9
	jz .eightup
	bt r9,2
	jc .fourup
	bt r9,1
	jc .twoup
.oneup:
	vmovq xmm0,[rsi]
	vpshufd xmm1,xmm0,1b
	vpminsd xmm2,xmm0,xmm1
	vpmaxsd xmm3,xmm0,xmm1
	vmovd [rsi],xmm2
	vmovd [rsi+4],xmm3
	ret
align 16
.twoup:
	vmovdqu xmm0,[rsi]
	vpshufd xmm1,xmm0,1110b
	vpminsd xmm2,xmm0,xmm1
	vpmaxsd xmm3,xmm0,xmm1
	vmovq [rsi],xmm2
	vmovq [rsi+8],xmm3
	ret
align 16
.fourup:
	vmovdqu xmm0,[rsi]
	vmovdqu xmm1,[rsi+16]
	vpminsd xmm2,xmm0,xmm1
	vpmaxsd xmm3,xmm0,xmm1
	vmovdqu [rsi],xmm2
	vmovdqu [rsi+16],xmm3
	ret
align 16
.eightup:
	vmovdqu ymm0,[rsi]
	vmovdqu ymm1,[rsi+rdx*4]
	vpminsd ymm2,ymm0,ymm1
	vpmaxsd ymm3,ymm0,ymm1
	vmovdqu [rsi],ymm2
	vmovdqu [rsi+rdx*4],ymm3
	add rsi,32
	dec rcx
	jnz .eightup
	ret
.down:
	test r9,r9
	jz .eightdown
	bt r9,2
	jc .fourdown
	bt r9,1
	jc .twodown

.onedown:
	vmovq xmm0,[rsi]
	vpshufd xmm1,xmm0,1b
	vpmaxsd xmm2,xmm0,xmm1
	vpminsd xmm3,xmm0,xmm1
	vmovd [rsi],xmm2
	vmovd [rsi+4],xmm3
	ret
align 16
.twodown:
	vmovdqu xmm0,[rsi]
	vpshufd xmm1,xmm0,1110b
	vpmaxsd xmm2,xmm0,xmm1
	vpminsd xmm3,xmm0,xmm1
	vmovq [rsi],xmm2
	vmovq [rsi+8],xmm3
	ret
align 16
.fourdown:
	vmovdqu xmm0,[rsi]
	vmovdqu xmm1,[rsi+16]
	vpmaxsd xmm2,xmm0,xmm1
	vpminsd xmm3,xmm0,xmm1
	vmovdqu [rsi],xmm2
	vmovdqu [rsi+16],xmm3
	ret
align 16
.eightdown:
	vmovdqu ymm0,[rsi]
	vmovdqu ymm1,[rsi+rdx*4]
	vpmaxsd ymm2,ymm0,ymm1
	vpminsd ymm3,ymm0,ymm1
	vmovdqu [rsi],ymm2
	vmovdqu [rsi+rdx*4],ymm3
	add rsi,32
	dec rcx
	jnz .eightdown
	ret
align 16
avx2sort:
	ret
align 16
; rdi array,rsi size
radix_sort_avx2:
	push rbx
	push rbp
	mov rbp,rsp
	sub rsp,16+16*8+16*8
	mov [rbp-8],rdi
	mov [rbp-16],rsi
	xor rbx,rbx
	vpxor ymm0,ymm0,ymm0
	vmovdqu [rbp-(16+16*8)],ymm0
	vmovdqu [rbp-(16+12*8)],ymm0
	vmovdqu [rbp-(16+8*8)],ymm0
	vmovdqu [rbp-(16+4*8)],ymm0
.L00: ; allocate 16 buckets
	lea rdi,[rbp-(16+16*8)+rbx*8] ; destination
	mov rdx,[rbp-16]
	mov rsi,16
	mov rax,16*16384
	cmp rdx,rax ; if array is bigger then L2 cache
	cmovg rsi,rax ; alignment, L2 cache size
	lea rdx,[rdx*4] ; size
	call [_posix_memalign] ; huge malloc is actually 
			      ; never allocated
                              ; untill page is touched
	test rax,rax
	jnz .exit
	inc rbx
	cmp rbx,16
	jnz .L00
	vpxor ymm0,ymm0,ymm0
	vmovdqu [rbp-(16+16*8+16*8)],ymm0
	vmovdqu [rbp-(16+16*8+12*8)],ymm0
	vmovdqu [rbp-(16+16*8+8*8)],ymm0
	vmovdqu [rbp-(16+16*8+4*8)],ymm0
	xor rcx,rcx
.L0:
	xor rdx,rdx
	vmovd xmm2,ecx
;	vpbroadcastd ymm2,xmm1
	mov rdi,[rbp-8]
	vpslld xmm2,xmm2,2
.L1:; calculate index with SIMD, based on [(num[rdx]>> (ecx << 2))&0xf]
	vmovdqu ymm1,[rdi+rdx*4]
	;vpsrlvd ymm3,ymm1,ymm2 ; heh, finally got why vector shift
				; is usefull
	vpsrld ymm3,ymm1,xmm2
	vpand ymm3,ymm3,yword[mask]
	mov r10,8
	vmovdqa ymm15,ymm3
	vmovdqa ymm14,ymm1
.L22:	; this is scatter to buckets, according to dword index
	; this can be unrolled, but
	; I don;t know by how much performance would be increased;
	vmovd esi, xmm3
	vmovd r11d,xmm1
	mov r8,[rbp-(16+16*8)+rsi*8] ; get bucket
	vpsrldq xmm3,xmm3,4
	mov r9,[rbp-(16+16*8+16*8)+rsi*8] ; get index into backet
	inc qword[rbp-(16+16*8+16*8)+rsi*8] ; update index
	vpsrldq xmm1,xmm1,4
	mov [r8+r9*4],r11d
	dec r10
	cmp r10,4 ; unfortunatelly vpsrldq will not shift 
                  ; upper 128 bits into lower
	je .adjust
.L44:
	inc rdx
	cmp rdx,[rbp-16]
	jge .L33
	test r10,r10
	jnz .L22
	jmp .L1
.adjust:
	vextracti128 xmm3,ymm15,1
	vextracti128 xmm1,ymm14,1
	jmp .L44
.L33: ; gather buckets (one after another) into input array
	xor rax,rax
	lea r8,[rbp-(16+16*8+16*8)]
	lea r9,[rbp-(16+16*8)]
.L2:
	mov rsi,[r9]
	mov r10,[r8]
	test r10,r10
	jz .L5
.L3:
.L4:
	cmp r10,4
	jl .one
	vmovdqa xmm15,[rsi]
	vmovdqu [rdi],xmm15
	add rsi,16
	add rdi,16
	sub r10,4
	jnz .L4
	jmp .L5
.one:
	mov r11d,[rsi]
	mov [rdi],r11d
	add rsi,4
	add rdi,4
	dec r10
	jnz .one

.L5:
	add r8,8
	add r9,8
	inc rax
	cmp rax,16
	jnz .L2
	vmovdqu [rbp-(16+16*8+16*8)],ymm0
	vmovdqu [rbp-(16+16*8+12*8)],ymm0
	vmovdqu [rbp-(16+16*8+8*8)],ymm0
	vmovdqu [rbp-(16+16*8+4*8)],ymm0
	inc rcx
	cmp rcx,8 ; 2*(sizeof(int))
	jl .L0
.exit:
	xor rbx,rbx
.L11:
	mov rdi,[rbp-(16+16*8)+rbx*8]
	call [_free]
.next:
	inc rbx
	cmp rbx,16
	jnz .L11
	mov rsp,rbp
	pop rbp
	pop rbx
	ret
radix_bernard:
	push rbp
	mov rbp,rsp
	sub rsp,24
	mov [rbp-8],rdi
	mov [rbp-16],rsi
	imul rsi,rsi,33 ; 33 buckets
	lea rdi,[rsi*4]
	call [_malloc]
	test rax,rax
	jz .exit
	mov [rbp-24],rax
	mov rdi,rax
	mov rcx,[rbp-16]
	imul rcx,32
	xor eax,eax
	rep stosd ; initialize buckets to zero
	mov rcx,[rbp-16]
	mov rsi,[rbp-8]
	mov rdi,[rbp-24]
	imul r9,rcx,4
	mov r8,32
	dec rcx
.L0:
	mov eax,[rsi]
	lzcnt edx,eax
	imul rdx,r9
	add rdx,rdi
	mov [rdx+rcx*4],eax
;	add rdi,4
	add rsi,4
	dec rcx
	jnz .L0

;	jmp .exit
	mov rdx,32
	mov rsi,[rbp-24]
	mov rdi,[rbp-8]
	mov rcx,[rbp-16]
	imul rdx,rcx
.L1:
	mov eax,[rsi]
	test eax,eax
	jz .skip
	mov [rdi],eax
	add rdi,4
	dec rcx
.skip:
	add rsi,4
	dec rdx
	jnz .L1
	xor eax,eax
	rep stosd ; populate rest with zeros
	
	mov rdi,[rbp-24]
	call [_free]
.exit:
	mov rsp,rbp
	pop rbp
	ret
section '.data' writeable align 32
mask dd 8 dup(0xf)
shf db 0,4,8,12
    db -128,-128,-128,-128
    db -128,-128,-128,-128
    db -128,-128,-128,-128
    db 0,4,8,12
    db -128,-128,-128,-128
    db -128,-128,-128,-128
    db -128,-128,-128,-128
perm dd 00,4,00,00,00,00,00,00
_posix_memalign dq posix_memalign
_free dq free
_malloc dq malloc
