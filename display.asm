; short display command delay
delay_disp:  
		ldi dcount2, 8
ddm2: 	ldi dcount, 255
ddm:	dec dcount 
		brne ddm
		dec dcount2
		brne ddm2
		ret

; send command to display OVER SHIFT REGISTER
SEND_CMD:
		cbi Displ_Port,Displ_Pin_En

		cbi Displ_Port,Displ_Pin_Rs
		MOV SH_DATA, CHAR_BUF
		RCALL SH_OUT
		sbi Displ_Port,Displ_Pin_En
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		cbi Displ_Port,Displ_Pin_En
		RCALL delay_disp
		ret

; send data to display OVER SHIFT REGISTER
SEND_DAT:
		cbi Displ_Port,Displ_Pin_En
		sbi Displ_Port,Displ_Pin_Rs
		MOV SH_DATA, CHAR_BUF
		RCALL SH_OUT
		sbi Displ_Port,Displ_Pin_En
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		cbi Displ_Port,Displ_Pin_En
		RCALL delay_disp
		ret

; initialize display
Init_Display:
		LDI CHAR_BUF, D_Init_ON
		RCALL SEND_CMD
		LDI CHAR_BUF, D_2Line
		RCALL SEND_CMD
		LDI CHAR_BUF, D_SetAddrDRAMZero
		RCALL SEND_CMD
		LDI CHAR_BUF, D_SetEntryMode 
		RCALL SEND_CMD

		LDI CHAR_BUF,D_SetAddrCGRAMZero
		RCALL SEND_CMD

; load custom chars - switch icons and a fucking stick
		ldi r25,8
		ldi ZL,low(chan_char_disbl*2)
		ldi ZH,high(chan_char_disbl*2)
load_char:
		LPM CHAR_BUF,Z+
		rcall SEND_DAT
		dec r25
		brne load_char

		LDI CHAR_BUF,D_SetAddrCGRAMZero+0x08
		RCALL SEND_CMD

		ldi r25,8
		ldi ZL,low(chan_char_enabl*2)
		ldi ZH,high(chan_char_enabl*2)
load_char2:
		LPM CHAR_BUF,Z+
		rcall SEND_DAT
		dec r25
		brne load_char2

		LDI CHAR_BUF,D_SetAddrCGRAMZero+0x10
		RCALL SEND_CMD

		ldi r25,8
		ldi ZL,low(char_stick*2)
		ldi ZH,high(char_stick*2)
load_char3:
		LPM CHAR_BUF,Z+
		rcall SEND_DAT
		dec r25
		brne load_char3


		LDI CHAR_BUF, D_SetAddrDRAMZero
		RCALL SEND_CMD
		ret

; Put formatted string: print with predefined control characters
; accepts string address from Z
putsf:	
putm:	LPM R16, Z+
		CPI R16, C_STREND
		BREQ put_eof
		st X+,r16
		rjmp putm
put_eof:ret



; Puts string from a table by index from R16
; ZL:ZH should already have index table loaded
; YL:YH should already have cgbuffer address loaded
; usage:
;	ldi r16,2 ; load third string
;	ldi ZL,low(strings_table*2)
;	ldi ZH,high(strings_table*2)
;	rcall puts_t


puts_t:
	lsl r16
	clr r17
	
	
	ADD ZL,R16
	ADC ZH,R17
	LPM R16,Z+
	LPM R17,Z+

	lsl r17
	bst r16,7
	bld r17,0
	lsl r16

	movw Z,r16:r17

	; lpm to Y is unnecessary, here i can call regular putsf
	rcall putsf	
	ret

; converts byte from itoa_in into ascii value to itoa_ch
itoa:	clr r16
		clr r17
		
		ldi r16, '0'
		sts itoa_ch,r16
		sts itoa_ch+1,r16
		sts itoa_ch+2,r16
		
		lds r17,itoa_pad
		cpi r17,Itoa_ZeroPadding
		breq itoa_main
		

		ldi r16, ' '
		lds r17,itoa_in
		cpi r17,100
		brlo pc+2
		rjmp itoa_main

		sts itoa_ch,r16
		cpi r17,10
		brlo pc+2
		rjmp itoa_main
		sts itoa_ch+1,r16
	
itoa_main:
		lds r17, itoa_in

i_hund:	cpi r17, 100
		brlo i_decs
		lds r16,itoa_ch
		inc r16
		sts itoa_ch,r16
		subi r17,100
		cpi r17,0
		breq enditoa
		rjmp i_hund	
		
i_decs: cpi r17, 10
		brlo i_sing
		lds r16,itoa_ch+1
		inc r16
		sts itoa_ch+1,r16
		subi r17,10
		cpi r17,0
		breq enditoa
		rjmp i_decs	

i_sing:	cpi r17,0
		breq enditoa
		lds r16,itoa_ch+2
		inc r16
		sts itoa_ch+2,r16
		dec r17
		rjmp i_sing
		
enditoa:lds r16,itoa_pad
		cpi r16,Itoa_LeftSpacePadding
		brne ex_itoa

		lds r16,itoa_in
		cpi r16,100
		brlo pc+2
		rjmp ex_itoa

		ldi r17,' '
		lds r16,itoa_ch+1
		sts itoa_ch,r16
		lds r16,itoa_ch+2
		sts itoa_ch+1,r16
		sts itoa_ch+2,r17
	

		lds r16,itoa_in
		cpi r16,10
		brlo pc+2
		rjmp ex_itoa
		
		ldi r17,' '
		lds r16,itoa_ch+1
		sts itoa_ch,r16
		sts itoa_ch+1,r17
		
		
ex_itoa:
		ret

; draws stuff on screen
present:
		ldi CHAR_BUF,D_SetAddr
		RCALL SEND_CMD
		ldi XL,low(cgbuf_l)
		ldi XH,high(cgbuf_l)
		ldi r21,D_LineLength
pres_l_t:	
		ld CHAR_BUF,X+
		rcall SEND_DAT
		dec r21
		brne pres_l_t
		
		ldi CHAR_BUF,D_SetAddr+0x40
		RCALL SEND_CMD
		
		ldi r21,D_LineLength
pres_l_b:	
		ld CHAR_BUF,X+
		rcall SEND_DAT
		dec r21
		brne pres_l_b


;-draw right part of screen
		ldi CHAR_BUF,D_SetAddr+D_LineLength
		RCALL SEND_CMD
		ldi XL,low(cgbuf_r)
		ldi XH,high(cgbuf_r)
		ldi r21,D_LineLength
pres_r_t:	
		ld CHAR_BUF,X+
		rcall SEND_DAT
		dec r21
		brne pres_r_t
		
		ldi CHAR_BUF,D_SetAddr+0x40+D_LineLength
		RCALL SEND_CMD
		
		ldi r21,D_LineLength
pres_r_b:	
		ld CHAR_BUF,X+
		rcall SEND_DAT
		dec r21
		brne pres_r_b
		ret

clr_bufrs:
		ldi XL,low(cgbuf_l)
		ldi XH,high(cgbuf_l)
		ldi r21,D_LineLength*2
		ldi r16,' '
		st X+,r16
		dec r21
		brne pc-2

		ldi XL,low(cgbuf_r)
		ldi XH,high(cgbuf_r)
		ldi r21,D_LineLength*2
		ldi r16,' '
		st X+,r16
		dec r21
		brne pc-2

		ret

; prints integer to X display position from itoa_ch
print_int:
		lds r16,itoa_ch
		st X+,r16
		lds r16,itoa_ch+1
		st X+,r16
		lds r16,itoa_ch+2
		st X+,r16
		ret

; convert bcd to decimals
; dec =  ((bcd >> 4)*10+(bcd & 0x0F));

btoi:
	lds r16,btoi_in
	mov r17,r16

	lsr r17
	lsr r17
	lsr r17
	lsr r17

	clr r18
	ldi r18,10
	MUL r17,r18
	mov r17,r0
	ANDI r16,0x0F
	ADD r17,r16
	sts btoi_out,r17
	ret

; convert decimal to bcd
; bcd = (((dec/10)<<4)(dec%10))
itob:
	lds r16,itob_in
	cpi r16,10
	brlo itob_end

	mov r17,r16

; first part
	clr r18
itob_div_m:
	inc r18
	SUBI r16,10
	cpi r16,10
	brlo pc+2
	rjmp itob_div_m
	NOP
	
	lsl r18
	lsl r18
	lsl r18
	lsl r18

	mov r16,r18

; second part
	clr r18
itob_div_m2:
	inc r18
	SUBI r17,10
	cpi r17,10
	brlo pc+2
	rjmp itob_div_m2

	OR r16,r17

itob_end:
	sts itob_out,r16
	ret
