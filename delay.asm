; long delay
delay:  ldi dcount2, 255
dm2: 	ldi dcount, 255
dm:		dec dcount 
		brne dm
		dec dcount2
		brne dm2
		ret

; timer-based delay, in milliseconds

t_delay_ms:
		lds r16,t_delay_val
		
		ldi r17,(1<<CS01|1<<CS00)
		OUT TCCR0,r17

tdm_m2:	ldi r17,0
		out TCNT0,r17
tdm_m:	in r17,TCNT0
		cpi r17,125
		brne tdm_m
		dec r16
		brne tdm_m2
		ret

; timer-based delay, in microseconds
t_delay_us:
		push R16
		lds r16,t_delay_val
		
tdu_m:	NOP
		NOP
		NOP
		NOP
		NOP

		dec r16
		brne tdu_m
		pop r16
		ret
