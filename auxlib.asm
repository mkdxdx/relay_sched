; AUXILLIARY FUNCTIONS, DEVICE UNRELATED

; timer 1 init: 8 bit CTC OC toggle mode 1/8 presc, timer disable

timer1_init:
	push r16
	ldi r16,(1<<WGM10)
	OUT TCCR1A,r16
	ldi r16,(1<<WGM12|1<<CS11)
	out TCCR1B,r16
	pop r16
	ret

timer1_disable:
	push r16
	ldi r16,(0<<CS11)
	out TCCR1B,r16
	pop r16
	ret

; eeprom write/read

ewrite: 
	SBIC	EECR,EEWE		
	RJMP	ewrite 		
	OUT 	EEARL,R16
	OUT		EEARH,R17	
	OUT 	EEDR,R18		
	SBI 	EECR,EEMWE	
	SBI 	EECR,EEWE		
	RET

eread:	
	SBIC 	EECR,EEWE	
	RJMP	eread			
	OUT 	EEARL, R16
	OUT		EEARH, R17	
	SBI 	EECR,EERE 		
	IN 		R18, EEDR 	
	RET

; i2c_sequental read
i2c_seq_rw:	
	ldi r16,I2C_Delay
	sts t_delay_val,r16


	;=== set START condition
	rcall i2c_start
	;===

	; write address+WRITE
	ldi r16,RTC_Addr_WR
	sts i2c_buf,r16
	rcall i2c_WRCycle
	
	; wait for ACK
	rcall i2c_wait_ack
	;===
	
	; write 0x00 register word
	ldi r16,0x00
	sts i2c_buf,r16
	rcall i2c_WRCycle	
	
	; wait for ACK
	rcall i2c_wait_ack
	;===
	rcall i2c_stop
	


	;=== set START repeat condition
	rcall i2c_start
	;===

	; ask chip to be read
	ldi r16,RTC_Addr_RD
	sts i2c_buf,r16
	rcall i2c_WRCycle

	; wait for ACK
	rcall i2c_wait_ack
	;===

	ldi r18,7
	ldi XL,low(rtc_data)
	ldi XH,high(rtc_data)

rd_m:

	rcall i2c_RDCycle
	lds r16,i2c_buf
	st X+,r16
	; generate ACK
	cpi r18,1
	breq skip_ack
	rcall sda_lo
	rcall scl_hi
	rcall scl_lo
skip_ack:
	dec r18
	brne rd_m
	

	; send NACK

	rcall scl_hi
	rcall scl_hi
	rcall scl_lo

	; === STOP
	rcall i2c_stop

	ret
; 


; getting keyboard value, check if pressed continuously
get_kbd:lds r16, kb_cur
		sts kb_prev,r16

gkbd_m:	LDI COUNT, 8
		CLR r17
		CLT
		cbi KBIN, Key_CLK
		sbi KBIN, Key_CLK
		cbi KBIN, SHL
		cbi KBIN, Key_CLK
		sbi KBIN, Key_CLK
		sbi KBIN, SHL
shmark:	IN R16,KBINP
		BST R16,DAT
		lsl r17
		BLD r17,0
		CBI KBIN, Key_CLK
		SBI KBIN, Key_CLK
		dec COUNT
		brne shmark
		com r17
		sts kb_cur, r17
		; check if same button is hold

		CLR r16
		sts kb_cont,r16

		lds r16,kb_cur
		lds r17,kb_prev
		cp r16,r17
		brne gkbd_e

		ldi r16,255
		sts kb_cont,r16
		
gkbd_e:	ret

; turn on the timer
init_timer:
		outi TCCR1A, (1<<COM1A1|1<<COM1B1|1<<WGM10)
		outi TCCR1B, (1<<WGM12|1<<CS11)

		outi TCCR2, (1<<WGM21|1<<WGM20|1<<COM21|0<<COM20|1<<CS21)
		ret
; turn off the timer
off_timer:
		outi TCCR1A, (0<<COM1A1|0<<COM1B1|1<<WGM10)
		outi TCCR1B, (1<<WGM12|0<<CS11)
		outi TCCR2, (1<<WGM21|1<<WGM20|0<<COM21|0<<COM20|0<<CS21)
		ret

; put data out into shift register
sh_out:
		mov SH_Roll, SH_Data		; move data into roller
		ldi SH_Count,Res	; load resolution into count

		sbi SH_CLK_DIR,SH_CLK
		sbi SH_DS_DIR,DS
		
		cbi SH_CLK_PORT,SH_CLK
		cbi SH_DS_PORT,DS

shsm:				; start shifting cycle
		SBRC SH_ROLL,7	
		SBI SH_DS_PORT, DS
		SBRS SH_ROLL,7
		CBI SH_DS_PORT, DS

		CBI SH_CLK_PORT, SH_CLK
		SBI SH_CLK_PORT, SH_CLK
	
		ROL SH_Roll
		dec SH_count
		brne shsm

		CBI SH_CLK_PORT, SH_CLK
		SBI SH_CLK_PORT, SH_CLK

		ret


; turn on the usart
init_usart:	
		LDI 	R16, low(bauddivider)
		OUT 	UBRRL,R16
		LDI 	R16, high(bauddivider)
		OUT 	UBRRH,R16
		OUTI UCSRA, 0
		OUTI UCSRB, (1<<RXCIE|1<<RXEN|1<<TXEN)
		OUTI UCSRC, (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1)
		RET

; turn off the usart
off_usart:
		CBI UCSRB,RXEN
		CBI UCSRB,TXEN
		ret


; send stuff over usart
uart_send:
		SBIS 	UCSRA,UDRE	
		RJMP	uart_send
		OUT	UDR, UDATA	
		RET



; init ADC
init_adc:
		OUTI ADMUX, (1<<ADLAR|1<<MUX2|1<<MUX1|1<<REFS0) ; mux is at ADC6 from start
		OUTI ADCSRA, (1<<ADEN|1<<ADSC|0<<ADFR|1<<ADPS2|1<<ADPS0) ; adc enable, adc single conversion mode, 	
		
		ret

; ADC single conversion, result into ADC_res in RAM

get_adc:
		
		cbi ADMUX,MUX3
		sbi ADMUX,MUX2
		sbi ADMUX,MUX1
		cbi ADMUX,MUX0
		SBI ADCSRA, ADSC
	
wait_adc1:
		sbic ADCSRA, ADSC
		rjmp wait_adc1		
		in r16, ADCH
		sts adc_result,r16


		cbi ADMUX,MUX3
		sbi ADMUX,MUX2
		sbi ADMUX,MUX1
		sbi ADMUX,MUX0
		SBI ADCSRA, ADSC

wait_adc2:
		sbic ADCSRA, ADSC
		rjmp wait_adc2
		in r16, ADCH
		sts adc_result+1,r16
				
		ret


; shows main menu





clr_ram:
		ldi r16,128
		ldi XL,0x60
		ldi XH,0
		ldi r17,0
cl_m:	st X+,r17
		dec r16
		brne cl_m
		ret


; I2C related

scl_hi:
	cbi I2C_Dir,I2C_SCL
	rcall t_delay_us
	ret

scl_lo:
	sbi I2C_Dir,I2C_SCL
	rcall t_delay_us
	ret

sda_hi:
	cbi I2C_Dir,I2C_SDA
	rcall t_delay_us
	ret

sda_lo:
	sbi I2C_Dir,I2C_SDA
	rcall t_delay_us
	ret

i2c_wait_ack:
	rcall scl_hi
	SBIC I2C_Pin,I2C_SDA
	rjmp pc-1
	rcall scl_lo

	ret

i2c_start:
	rcall sda_hi
	rcall scl_hi
	rcall sda_lo
	rcall scl_lo
	ret

i2c_stop:	

	rcall scl_lo
	rcall sda_lo
	rcall scl_hi
	rcall sda_hi
	ret


i2c_WRCycle:
	lds r17,i2c_buf
	ldi r16,I2C_ByteLength
i2c_wrm:
	
	
	SBRC r17,7	 ; if 7 bit is 1, let pullup set line to 1
	rcall sda_hi
	SBRS r17,7
	rcall sda_lo

	rcall scl_hi
	rcall scl_lo

	ROL r17
	dec r16
	brne i2c_wrm
	SBRC r17,7
	rcall sda_hi
	ret


i2c_RDCycle:
	clr r17
	ldi r16,I2C_ByteLength
	rcall sda_hi
i2c_rdm:
	ROL R17

	rcall scl_hi

	SBIC I2C_Pin,I2C_SDA
	SBR R17,1
	SBIS I2C_Pin,I2C_SDA
	CBR R17,1
	
	
	rcall scl_lo
	
	dec r16
	brne i2c_rdm
	sts i2c_buf,r17
	ret

; 


