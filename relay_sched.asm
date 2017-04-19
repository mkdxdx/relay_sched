; KB_CUR: function keys: 7-3 bits, d-pad: 3-0 bits 
; HC165 readout:
; 1. CLK fall, CLK rise
; 2. SH fall, CLK fall, CLK rise
; 3. SH rise
; 4. Read DATA (QH), CLK fall, CLK rise
; 5. repeat 7 times

; interfacing with st7066 though 595 as follows:
; 0. set LATCH low
; 1. place D0-D7 in SH_Data
; 2. call sh_out
; 3. place RS, R/W, E in SH_Data (7,6,5 bits accordingly)
; 4. call sh_out
; 5. set LATCH high


.include "m8def.inc"   ; 
;= Start macro.inc ========================================
 
	.MACRO OUTI
	LDI R16, @1
	OUT @0, R16
	.ENDM

	.MACRO LDI_SR
	LDI R16, @1
	STS @0, R16
	.ENDM


	; PUTSTR static_address, screen+shift, 5 operands for branching
	.MACRO PUTSTR
	ldi ZL,low(@0*2)
	ldi ZH,high(@0*2)
	ldi XL,low(@1)
	ldi XH,high(@1)
	rcall putsf
	.ENDM

	.MACRO PUTSTR_t
	mov r16,@0 ; move string number from register!
	ldi ZL,low(@1*2)
	ldi ZH,high(@1*2)
	rcall puts_t
	.endm


		
;= End macro.inc  ========================================

; RAM =====================================================
		.DSEG			; ram


; DEFINE HC165 STUFF
.EQU	SHL = 1	; define shift register pins
.EQU	Key_CLK = 0
.EQU	DAT = 2
.EQU	KBIN = PORTC; define keyboard port
.EQU	KBIND= DDRC ; define keyboard port direction register
.EQU	KBINP= PINC ; define actual pins for inputting

.def	COUNT = R18	; define count register


; DEFINE HC595 STUFF

.EQU SH_CLK  = 5		; these are hc595 pins
;.EQU LATCH= 4	; no latch, linked with CLK
;.EQU OE	  = 1	; no oe today
;.EQU RST  = 3
.EQU DS   = 0
.EQU RES  = 8		; resolution, how many cycles to push data
.EQU SH_CLK_PORT = PortC	;
.EQU SH_CLK_DIR  = DDRC
.EQU SH_DS_PORT = PortD
.EQU SH_DS_DIR = DDRD
.def SH_Data = R23		; move shiftout data here
.def SH_Roll = R18		; temp roller
.def SH_Count= R19		; cycle register

; DEFINE DISPLAY STUFF

.EQU Displ_Port = PORTC
.EQU Displ_Dir = DDRC
.EQU Displ_Pin_En = 4
.EQU Displ_Pin_Rs = 3

.EQU D_RS = 0b10000000
.EQU D_RW = 0b01000000
.EQU D_En = 0b00100000

.EQU D_Init_ON = 0b00001100 ; display ON, cursor ON, blink ON
.EQU D_2Line = 0b00111000 ; 8 bit mode, 2 line mode, 5x8 font
.EQU D_SetAddrDRAMZero = 0b00000010 ; Sets AC to 0, cursor to top-left
.EQU D_SetEntryMode = 0b00000110 ; increment DRAM, shift cursor, not display
.EQU D_SetAddr = 0b10000000 ; D_SetAddr + address = shift cursor to address
.EQU D_Clear = 0b00000001 ; clear screen, goto zero
.EQU D_ShiftCursorRight = 0b00010100 ; shift cursor right
.EQU D_ShiftScreenLeft = 0b00011100
.EQU D_ShiftScreenRight = 0b00011000
.EQU D_SetAddrCGRAMZero = 0b01000000

.EQU D_LineLength = 16 ; 16 character display

; DEFINE OTHER STUFF

.EQU Itoa_ZeroPadding = 0
.EQU Itoa_RightSpacePadding= 1
.EQU Itoa_LeftSpacePadding = 2

.def 	DCOUNT= R19 ; define delay count1
.def	DCOUNT2=R20 ; define delay count2
.def	UDATA = R22

.def CHAR_BUF = R24

.EQU C_STREND = 255
.EQU C_Char = 254
.EQU C_MoveCursor = 253
.EQU C_ClearScreen = 252


.EQU Key_Left = 0b00000001
.EQU Key_Down = 0b00000010
.EQU Key_Right= 0b00000100
.EQU Key_Up =   0b00001000


.EQU Key_F1	=	0b00010000
.EQU Key_OK =	Key_F1
.EQU Key_F2 = 	0b00100000
.EQU Key_F3 = 	0b01000000
.EQU Key_F4 =	0b10000000
.EQU Key_Esc=	Key_F4


.EQU Relay1_2_Port = PortD
.EQU Relay1_2_Dir = DDRD
.EQU Relay1_2_Pin = PinD

.EQU Relay1 = 6
.EQU Relay2 = 7

.EQU Relay3_Port = PortB
.EQU Relay3_Dir = DDRB
.EQU Relay3_Pin = PinB

.EQU Relay3 = 0


.EQU PWM_Port = PortB
.EQU PWM_Dir = DDRB

.EQU PWM_1 = 1
.EQU PWM_2 = 2


.EQU Resvd_Port = PortB
.EQU Resvd_Dir = DDRB

.EQU Resvd_1 = 3
.EQU Resvd_2 = 4
.EQU Resvd_3 = 5

; RTC STUFF DEFINITIONS

.EQU I2C_CTRL_ACK = 0b00000000 ; i2c end of read bit
.EQU I2C_CTRL_NACK= 0b00000001 ; if set ACK, at each packet
							   ; mcu will generate ACK
							   ; otherwise it will be NACK
							   ; to end read
.EQU I2C_ByteLength = 8
.EQU I2C_Delay = 10

.EQU I2C_Port = PortD
.EQU I2C_Dir = DDRD
.EQU I2C_Pin = PinD
.EQU I2C_SCL = 5
.EQU I2C_SDA = 4


.EQU RTC_Addr_WR = 0b11010000
.EQU RTC_Addr_RD = 0b11010001

.EQU RTC_Reg_Seconds = 0x00
.EQU RTC_Reg_Minutes = 0x01
.EQU RTC_Reg_Hours = 0x02
.EQU RTC_Reg_DOW = 0x03
.EQU RTC_Reg_Day = 0x04
.EQU RTC_Reg_Month=0x05
.EQU RTC_Reg_Year = 0x06
.EQU RTC_Reg_Control = 0x07
.EQU RTC_Reg_RAMStart = 0x08
.EQU RTC_Reg_RAMEnd = 0x3F


.EQU RTC_Init_Seconds = 0
.EQU RTC_Init_Minutes = 48
.EQU RTC_Init_Hours = 18
.EQU RTC_Init_Day = 4 ; doesnt affect anything
.EQU RTC_Init_Date = 15
.EQU RTC_Init_Month = 4
.EQU RTC_Init_Year = 15

.EQU RTC_ClockHalt = 0b10000000
.EQU RTC_ClockEnable = 0x00000000
.EQU RTC_Hours_12 = 0b00000000
.EQU RTC_Hours_24 = 0b01000000
; Below it what RTC will be initialized with: OUT is 0, SQWE is 0, RS1/RS0 are 1
.EQU RTC_Control_Init = 0b00000011

; SCHEDULER CONSTANTS

.EQU S_TaskDisplayIndexTest = 5


;Task list table:
;1. Task flag byte (Disabled/Queued,X,X,X,4 bit Condition [1-Time, 2-Ext int,2-ADC, 3-Sensor])
;2. Condition byte 1 (Minutes for Time, Channel number for EI/ADC, Sensor ID for sensor)
;3. Condition byte 2 (Hours for Time, Channel value for EI/ADC, Sensor data for sensor)
;4. Condition byte 3 (Reserved)
;5. Channel byte (Relay 1-3, Pwm/Sink 1-2, Reserved 1-3)
;6. Channel value (On,Off for Relay/Sink, 8 bit timer value for PWM)

.EQU S_TaskArrayMaxLength = 32
.EQU S_TaskItemLength = 6

; Describe each bit and byte for table
.EQU Task_Flags = 0
.EQU Task_Flags_Queued = 7
.EQU Task_Flags_Type_Time = 0
.EQU Task_Flags_Type_Ext = 1
.EQU Task_Flags_Type_ADC = 2
.EQU Task_Flags_Type_Sensor = 3

.EQU Task_FirstData = 1
.EQU Task_SecondData= 2
.EQU Task_Hours = 1
.EQU Task_Minutes = 2
.EQU Task_Int_Channel = 1
.EQU Task_Int_Value = 2
.EQU Task_ADC_Channel = 1
.EQU Task_ADC_Value = 2
.EQU Task_Sensor_ID = 1
.EQU Task_Sensor_Value = 2

.EQU Task_Condition_Reserved = 3

.EQU Task_Exec_Channel = 4
.EQU Task_Exec_Value = 5
; Channel/Action definition

.EQU Task_EC_Blank = 0
.EQU Task_EC_Relay1 = 1
.EQU Task_EC_Relay2= 2
.EQU Task_EC_Relay3 = 3
.EQU Task_EC_PWM1 = 4
.EQU Task_EC_PWM2 = 6
.EQU Task_EC_Res1 = 7
.EQU Task_EC_Res2 = 8
.EQU Task_EC_Res3 = 9
.EQU Task_EC_AllRel = 10
.EQU Task_EC_AllPWM = 11
.EQU Task_EC_AllChan= 12

.EQU Task_EC_RelayOff = 0
.EQU Task_EC_RelayOn = 1 


; Taskmgr misc
.EQU Taskmgr_MD_View = 0
.EQU Taskmgr_MD_Modify = 1

.EQU Mode_Menu = 0
.EQU Mode_Clock = 1
.EQU Mode_Taskmgr = 2
.EQU Mode_SetClock = 3
.EQU Mode_Override = 4
.EQU Mode_EEPROMSave = 5
.EQU Mode_ClearTasks = 6
.EQU Mode_About = 7


.EQU DT_Seconds = 0
.EQU DT_Minutes = 1
.EQU DT_Hours = 2
.EQU DT_Day = 3
.EQU DT_Date = 4
.EQU DT_Month= 5
.EQU DT_Year = 6


; VARIABLES

.ORG SRAM_START+0x0010

cgbuf_l: .byte D_LineLength*2
cgbuf_r: .byte D_LineLength*2

scr_scroll: .byte 1

mode: .byte 1
menu_index: .byte 1

kb_cur: .byte 1
kb_prev:.byte 1
kb_cont:.byte 1

itoa_ch: .byte 3
itoa_in: .byte 1
itoa_pad:.byte 1

btoi_in: .byte 1
btoi_out:.byte 1

itob_in: .byte 1
itob_out:.byte 1

adc_result: .byte 2

t_delay_val: .byte 1

io_state: .byte 1

; SCHEDULER VARIABLES


task_display_index: .byte 1
taskmgr_mode: .byte 1
task_modify_selector: .byte 1
clock_modify_selector: .byte 1

i2c_buf: .byte 1
rtc_data: .byte 8
task_array: .byte S_TaskArrayMaxLength*S_TaskItemLength


; FLASH ===================================================
		.CSEG			; code

	.ORG $000        		; (RESET) 
     RJMP   Init	 		; 
	.ORG $00B				; (Handle USART received data)
	 RJMP USART_RX_HNDL			

	.ORG INT_VECTORS_SIZE	; end of int vector table, start from 


USART_RX_HNDL:	
	reti 

.equ 	XTAL = 8000000 	
.equ 	baudrate = 9600  
.equ 	bauddivider = XTAL/(16*baudrate)-1

init:	OUTI SPL, Low(RAMEND)
		OUTI SPH, High(RAMEND)


		

		rcall initialize
		
				

main:	rcall clr_bufrs
		rcall get_rtc
		rcall poll_tasks
		rcall get_kbd
		rcall process
		rcall present
		rjmp main

process:
		
		; indexed call over fucking here, maggots
		clr r17
		lds r16,mode
		lsl r16
		ldi ZL,low(mode_table*2)
		ldi ZH,high(mode_table*2)
		ADD ZL,R16
		ADC ZH,R17
		LPM R16,Z+
		LPM R17,Z+
		MOVW Z,r16:r17
		icall
		ret




; ====================================
; AUXILIARY PROCEDURES ARE PLACED HERE
; ====================================


; this is magic function

poll_tasks:
	LDI XL,low(task_array)
	LDI XH,high(task_array)
	ldi r16,S_TaskArrayMaxLength

ld_task:
	push r16
	PUSH XL
	PUSH XH

	ld r16,X+
	SBRS r16,Task_Flags_Queued
	rjmp next_task

	CLR r17
	CBR r16,(1<<Task_Flags_Queued)
	lsl r16
	ldi ZL,low(compare_table*2)
	ldi ZH,high(compare_table*2)
	ADD ZL,R16
	ADC ZH,R17
	LPM R16,Z+
	LPM R17,Z+
	MOVW Z,r16:r17
	icall
	

next_task:
	POP XH
	POP XL
	ADIW X,S_TaskItemLength
	pop r16
	dec r16
	brne ld_task


	ret

compare_time:
	; here i'm gonna be using WORD to WORD comparison through CP/CPC
	; as shown on Atmel's manual CPC - compare with carry
	; r0:r1 - low:high task's time 
	; r16:r17 - low:high current time

	ld r1,X+ ; load hours/high byte
	ld r0,X+ ; load minutes/low byte

	lds r17,rtc_data+DT_Hours
	lds r16,rtc_data+DT_Minutes
	cp r16,r0 ; magic goes on here
	cpc r17,r1
	brne end_c_t

	ADIW X,1  ; increase to skip over RESERVED byte
	ld r16,X+ ; load Task Exec Channel to r16
	ld r18,X ; ; load Task_Exec_Value to r18!
	rcall task_execute

end_c_t:
	ret


compare_timH:
	; here i'm gonna be using WORD to WORD comparison through CP/CPC
	; as shown on Atmel's manual CPC - compare with carry
	; r0:r1 - low:high task's time 
	; r16:r17 - low:high current time

	ld r1,X+ ; load hours/high byte
	ld r0,X+ ; load minutes/low byte

	lds r17,rtc_data+DT_Hours
	lds r16,rtc_data+DT_Minutes
	cp r16,r0 ; magic goes on here
	cpc r17,r1
	brlo end_c_th
	
	ADIW X,1  ; increase to skip over RESERVED byte
	ld r16,X+ ; load Task Exec Channel to r16
	ld r18,X ; ; load Task_Exec_Value to r18!
	rcall task_execute

end_c_th:
	ret


compare_adc:ret
compare_interrupt:ret
compare_sensor:	ret
	
	
task_execute:
	CLR r17
	CBR r16,(1<<Task_Flags_Queued)
	lsl r16
	ldi ZL,low(execute_table*2)
	ldi ZH,high(execute_table*2)
	ADD ZL,R16
	ADC ZH,R17
	LPM R16,Z+
	LPM R17,Z+
	MOVW Z,r16:r17
	icall

	sts io_state,r19
	
	SBRC r19,Relay1
	SBI Relay1_2_Port,Relay1
	SBRS r19,Relay1
	CBI Relay1_2_Port,Relay1

	SBRC r19,Relay2
	SBI Relay1_2_Port,Relay2
	SBRS r19,Relay2
	CBI Relay1_2_Port,Relay2

	SBRC r19,Relay3
	SBI Relay3_Port,Relay3
	SBRS r19,Relay3
	CBI Relay3_Port,Relay3
	NOP

	ret

exec_na:ret

exec_rel1:
	lds r19,io_state

	cpi r18,Task_EC_RelayOff
	brne pc+3
	CBR r19,(1<<Relay1)
	ret
	SBR r19,(1<<Relay1)
	ret

exec_rel2:
	cpi r18,Task_EC_RelayOff
	brne pc+3
	CBR r19,(1<<Relay2)
	ret
	SBR r19,(1<<Relay2)
	ret

exec_rel3:
	cpi r18,Task_EC_RelayOff
	brne pc+3
	CBR r19,(1<<Relay3)
	ret
	SBR r19,(1<<Relay3)
	ret


exec_pwm1:
	
	cpi r18,1
	brlo p1_dis
	out OCR1AL,r18
	rcall timer1_init

	in r16, TCCR1A
	SBR r16,(1<<COM1A1)
	OUT TCCR1A,r16

	ret
p1_dis:
	in r16, TCCR1A
	CBR r16,(1<<COM1A1)
	OUT TCCR1A,r16
	ret


exec_pwm2:
	
	cpi r18,1
	brlo p2_dis
	out OCR1BL,r18
	rcall timer1_init

	in r16, TCCR1A
	SBR r16,(1<<COM1B1)
	OUT TCCR1A,r16
	
	ret
p2_dis:
	in r16, TCCR1A
	CBR r16,(1<<COM1B1)
	OUT TCCR1A,r16
	ret

exec_res1:ret
exec_res2:ret
exec_res3:ret
exec_allrelay:
	lds r19,io_state
	cpi r18,Task_EC_RelayOff
	brne pc+3
	CBR r19,(1<<Relay1|1<<Relay2|1<<Relay3)
	ret
	SBR r19,(1<<Relay1|1<<Relay2|1<<Relay3)
	ret

exec_allpwm: 
	cpi r18,1
	brlo ap_dis
	out OCR1AL,r18
	out OCR1BL,r18
	rcall timer1_init

	in r16, TCCR1A
	SBR r16,(1<<COM1B1|1<<COM1A1)
	OUT TCCR1A,r16

	ret
ap_dis:
	in r16, TCCR1A
	CBR r16,(1<<COM1B1|1<<COM1A1)
	OUT TCCR1A,r16
	ret


exec_allchan:
	lds r19,io_state
	cpi r18,Task_EC_RelayOff
	brne pc+3
	CBR r19,(1<<Relay1|1<<Relay2|1<<Relay3)
	rjmp pc+2
	SBR r19,(1<<Relay1|1<<Relay2|1<<Relay3)
	NOP

	cpi r18,1
	brlo ac_dis
	out OCR1AL,r18
	out OCR1BL,r18
	rcall timer1_init

	in r16, TCCR1A
	SBR r16,(1<<COM1B1|1<<COM1A1)
	OUT TCCR1A,r16
	ret

ac_dis:
	in r16, TCCR1A
	CBR r16,(1<<COM1B1|1<<COM1A1)
	OUT TCCR1A,r16
	rcall timer1_disable
	; CHANGE THIS HOLY SHIT
	ret




get_rtc:
	rcall i2c_seq_rw
	rcall convert_bcd2dec
	ret




set_rtc:
	ldi r16,I2C_Delay
	sts t_delay_val,r16

	;=== set START condition
	rcall i2c_start
	;===

	; write address+WRITE
	ldi r16,RTC_Addr_WR
	sts i2c_buf,r16
	rcall i2c_WRCycle
	
	rcall i2c_wait_ack
	;===
	; write 0x00 register word
	ldi r16,0x00
	sts i2c_buf,r16
	rcall i2c_WRCycle	
	
	rcall i2c_wait_ack

	; write 0x00 register word
	lds r16,rtc_data+DT_Seconds
	sts i2c_buf,r16
	rcall i2c_WRCycle	
	rcall i2c_wait_ack


	lds r16,rtc_data+DT_Minutes
	sts itob_in,r16
	rcall itob
	lds r16,itob_out
	sts i2c_buf,r16
	rcall i2c_WRCycle
	rcall i2c_wait_ack

	lds r16,rtc_data+DT_Hours
	sts itob_in,r16
	rcall itob
	sts i2c_buf,r16
	rcall i2c_WRCycle
	rcall i2c_wait_ack

	ldi r16,RTC_Init_Day
	sts itob_in,r16
	rcall itob
	sts i2c_buf,r16
	rcall i2c_WRCycle
	rcall i2c_wait_ack

	lds r16,rtc_data+DT_Date
	sts itob_in,r16
	rcall itob
	sts i2c_buf,r16
	rcall i2c_WRCycle
	rcall i2c_wait_ack

	lds r16,rtc_data+DT_Month
	sts itob_in,r16
	rcall itob
	sts i2c_buf,r16
	rcall i2c_WRCycle
	rcall i2c_wait_ack

	lds r16,rtc_data+DT_Year
	sts itob_in,r16
	rcall itob
	sts i2c_buf,r16
	rcall i2c_WRCycle
	rcall i2c_wait_ack



	rcall i2c_stop
	ret










showmenu:
	lds r16,kb_cont
	cpi r16,255
	breq sm_skip_kb

	lds r16,kb_cur
	cpi r16,Key_Esc
	brne sm_k_dn
	ldi r16,Mode_Clock
	sts mode,r16
sm_k_dn:
	cpi r16,Key_Down
	brne sm_k_up
	lds r17,menu_index
	inc r17
	cpi r17,Mode_About+1
	breq pc+2
	rjmp pc+2
	ldi r17,Mode_About
	sts menu_index,r17
sm_k_up:
	cpi r16,Key_Up
	brne sm_k_ok
	lds r17,menu_index
	dec r17
	cpi r17,0
	breq pc+2
	rjmp pc+2
	ldi r17,1
	sts menu_index,r17
sm_k_ok:
	cpi r16,Key_OK
	brne sm_skip_kb
	lds r17,menu_index
	sts mode,r17

sm_skip_kb:
	lds r17,menu_index
	ldi XL,low(cgbuf_l+1)
	ldi XH,high(cgbuf_l+1)
	PUTSTR_t r17,table_menu_strings
	ldi r16,'>'
	sts cgbuf_l,r16
	ret



showclock:
	ldi r16,Itoa_ZeroPadding
	sts itoa_pad,r16

	lds r16,rtc_data+DT_Seconds
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+11)
	ldi XH,high(cgbuf_l+11)
	rcall print_int

	lds r16,rtc_data+DT_Minutes
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+8)
	ldi XH,high(cgbuf_l+8)
	rcall print_int

	lds r16,rtc_data+DT_Hours
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+5)
	ldi XH,high(cgbuf_l+5)
	rcall print_int


	lds r16,rtc_data+DT_Year
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+27)
	ldi XH,high(cgbuf_l+27)
	rcall print_int

	lds r16,rtc_data+DT_Month
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+24)
	ldi XH,high(cgbuf_l+24)
	rcall print_int

	lds r16,rtc_data+DT_Date
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+21)
	ldi XH,high(cgbuf_l+21)
	rcall print_int

	ldi r16,'['
	sts cgbuf_l+5,r16
	sts cgbuf_l+21,r16

	ldi r16,']'
	sts cgbuf_l+14,r16
	sts cgbuf_l+30,r16

	ldi r16,':'
	sts cgbuf_l+11,r16
	sts cgbuf_l+8,r16

	ldi r16,'/'
	sts cgbuf_l+24,r16
	sts cgbuf_l+27,r16

	; display relay status
	ldi r17,1
	ldi r16,0
	sts cgbuf_l+D_LineLength+1,r16
	sts cgbuf_l+D_LineLength+2,r16
	sts cgbuf_l+D_LineLength+3,r16
	
	SBIC Relay1_2_Pin,Relay1
	sts cgbuf_l+D_LineLength+1,r17
	SBIC Relay1_2_Pin,Relay2
	sts cgbuf_l+D_LineLength+2,r17

	SBIC Relay3_Pin,Relay3
	sts cgbuf_l+D_LineLength+3,r17

	ldi r16,'1'
	sts cgbuf_l+1,r16
	ldi r16,'2'
	sts cgbuf_l+2,r16
	ldi r16,'3'
	sts cgbuf_l+3,r16
	ldi r16,2
	sts cgbuf_l+4,r16
	sts cgbuf_l+D_LineLength+4,r16


	lds r16,kb_cont
	cpi r16,255
	breq sc_end
	lds r16,kb_cur
	cpi r16,Key_F1
	brne pc+3
	ldi r16,Mode_Menu
	sts mode,r16

	
sc_end:
	ret



showtaskmgr:
	lds r16,kb_cur
	cpi r16,Key_OK
	brne stmg_endkb
	lds r16,kb_cont
	cpi r16,255
	breq stmg_endkb
	
	ldi r16,Taskmgr_MD_Modify
	sts taskmgr_mode,r16	

stmg_endkb:
	lds r16,taskmgr_mode
	cpi r16,Taskmgr_MD_View
	brne pc+2
	rcall taskmgr_view
	cpi r16,Taskmgr_MD_Modify
	brne pc+2
	rcall taskmgr_modify
	rcall print_task
	ret
	



; TASK MODIFICATION
taskmgr_modify:
	lds r16,kb_cur
	cpi r16,Key_Esc
	brne tm_mod_right
	lds r16,kb_cont
	cpi r16,255
	breq tm_mod_check

	ldi r16,Taskmgr_MD_View
	sts taskmgr_mode,r16

tm_mod_right:
	cpi r16,Key_Right
	brne tm_mod_left
	lds r16,kb_cont
	cpi r16,255
	breq tm_mod_check

	lds r16,task_modify_selector
	inc r16
	cpi r16,4
	brge pc+3
	sts task_modify_selector,r16
	rjmp tm_mod_check
	ldi r16,2
	sts task_modify_selector,r16
	rjmp tm_mod_check

tm_mod_left:
	cpi r16,Key_Left
	brne tm_mod_check
	lds r16,kb_cont
	cpi r16,255
	breq tm_mod_check

	lds r16,task_modify_selector
	dec r16
	cpi r16,255
	brne pc+2
	ldi r16,0
	sts task_modify_selector,r16	


tm_mod_check:

	ldi XL,low(task_array)
	ldi XH,high(task_array)
	lds r16,task_display_index
	cpi r16,0
	breq tm_mod

tm_mod_shift:
	ADIW X,S_TaskItemLength	; add length til we get to needed task
	dec r16
	brne tm_mod_shift
	
	
tm_mod:
	ADIW X,Task_Flags
	ld r16,X

	lds r17,kb_cur
	cpi r17,Key_F2
	brne tm_mod_f3
	lds r17,kb_cont
	cpi r17,255
	breq tm_mod_2
	ldi r17,0
	ldi r17,(1<<Task_Flags_Queued)
	EOR r16,r17
	st X,r16
	rjmp tm_mod_2

tm_mod_f3:
	cpi r17,Key_F3
	brne tm_mod_2
	lds r17,kb_cont
	cpi r17,255
	breq tm_mod_2

	cbr r16,0b10000000
	inc r16
	ld r17,X

	cpi r16,Task_Flags_Type_Sensor+1
	brlo pc+2
	ldi r16,Task_Flags_Type_Time

	SBRC r17,Task_Flags_Queued
	sbr r16,(1<<Task_Flags_Queued)
	st X,r16
	

tm_mod_2:
	ld r16,X
	CBR r16,0b10000000
	clr r17
	lsl r16
	ldi ZL,low(modify_table*2)
	ldi ZH,high(modify_table*2)
	ADD ZL,R16
	ADC ZH,R17
	LPM R16,Z+
	LPM R17,Z+
	MOVW Z,r16:r17
	icall

	ADIW X,Task_Exec_Channel
	ld r16,X
	cpi r16,Task_EC_ALLChan
	brlo pc+2
	ldi r16,0
	st X,r16
	ret



modify_time:
	PUSH XL
	PUSH XH
	
	CLR r17
	lds r16,task_modify_selector
	lsl r16
	ldi ZL,low(selector_table*2)
	ldi ZH,high(selector_table*2)
	ADD ZL,r16
	ADC ZH,r17

	lpm r16,Z+
	
	ADD XL,r16
	ADC XH,r17

;check keyboard
	
	lds r16,kb_cur
	cpi r16,Key_Up
	brne md_tm_kd

	lds r16,taskmgr_mode
	cpi r16,Taskmgr_MD_Modify
	brne md_tm_kend

	ld r17,X
	inc r17
	st X,r17
md_tm_kd:
	lds r16,kb_cur
	cpi r16,Key_Down
	brne md_tm_kend
	
	lds r16,taskmgr_mode
	cpi r16,Taskmgr_MD_Modify
	brne md_tm_kend
	
	ld r17,X
	dec r17
	st X,r17

md_tm_kend:
	

; check constraints for Time type task
	POP XH
	POP XL

	PUSH XL
	PUSH XH
	
	
	
	ADIW X,Task_Hours
	ld r16,X
	cpi r16,24
	brlo pc+3
	ldi r16,0
	st X,r16

	POP XH
	POP XL

	PUSH XL
	PUSH XH

	ADIW X,Task_Minutes
	ld r16,X
	cpi r16,60
	brlo pc+3
	ldi r16,0
	st X,r16

	POP XH
	POP XL

	ret


modify_adc:
	ret
modify_interrupt:
	ret
modify_sensor:
	ret


; TASK VIEWING
taskmgr_view:
	lds r16,kb_cur
	cpi r16,Key_Down
	brne tm_vk_up
	lds r17,task_display_index
	inc r17
	sts task_display_index,r17
tm_vk_up:
	cpi r16,Key_Up
	brne tm_vk_esc
	lds r17,task_display_index
	dec r17
	sts task_display_index,r17
tm_vk_esc:
	cpi r16,Key_Esc
	brne tm_vk_end
	lds r16,kb_cont
	cpi r16,255
	breq tm_vk_end
	ldi r17,Mode_Menu
	sts mode,r17
tm_vk_end:

	lds r16,task_display_index
	cpi r16,S_TaskArrayMaxLength
	brlo pc+2
	clr r16
	cpi r16,0
	brge pc+2
	ldi r16,S_TaskArrayMaxLength
	sts task_display_index,r16



	ret



print_task:
		ldi YL,low(task_array)
		ldi YH,high(task_array)
		lds r16,task_display_index

		cpi r16,0
		breq displ_task

task_idx:
		ADIW Y,S_TaskItemLength	; add length til we get to needed task
		dec r16
		brne task_idx


displ_task:
		ldi r16,Itoa_LeftSpacePadding
		sts itoa_pad,r16

		
		lds r16,task_display_index
		inc r16
		sts itoa_in,r16
		rcall itoa
		ldi XL,low(cgbuf_l)
		ldi XH,high(cgbuf_l)
		rcall print_int

		push YL
		push YH
		ADIW Y,Task_Flags
		ld r16,Y+			; display task type; MAKE IT INDEXED
		CBR r16,0b10000000
		ldi XL,low(cgbuf_l+3)
		ldi XH,high(cgbuf_l+3)
		PUTSTR_t r16,str_tasktype_table	
		pop YH
		pop YL

		push YL
		push YH
		ldi r17,0
		ADIW Y,Task_Flags
		ld r16,Y		; display flag
		SBRC r16,Task_Flags_Queued
		ldi r17,1
		sts cgbuf_l+D_LineLength+1,r17
		pop YH
		pop YL

		push YL
		push YH
		ADIW Y,Task_FirstData
		ld r16,Y			; display first data byte
		sts itoa_in,r16
		rcall itoa
		ldi XL,low(cgbuf_l+8)
		ldi XH,high(cgbuf_l+8)
		rcall print_int
		pop YH
		pop YL

		push YL
		push YH
		ADIW Y,Task_SecondData
		ld r16,Y			; display second data byte
		sts itoa_in,r16
		rcall itoa
		ldi XL,low(cgbuf_l+12)
		ldi XH,high(cgbuf_l+12)
		rcall print_int
		pop YH
		pop YL

		push YL
		push YH
		ADIW Y,Task_Exec_Channel
		ld r16,Y+			; display channel and action; MAKE IT INDEXED
		ldi XL,low(cgbuf_l+D_LineLength+3)
		ldi XH,high(cgbuf_l+D_LineLength+3)
		PUTSTR_t r16,str_channels_table
		pop YH
		pop YL

		push YL
		push YH
		ADIW Y,Task_Exec_Value
		ld r16,Y+			; display channel and action; MAKE IT INDEXED
		sts itoa_in,r16
		rcall itoa
		ldi XL,low(cgbuf_l+D_LineLength+12)
		ldi XH,high(cgbuf_l+D_LineLength+12)
		rcall print_int	
		pop YH
		pop YL

		push YL
		push YH
		ADIW Y,Task_Exec_Value
		ld r16,Y+			; display channel and action; MAKE IT INDEXED
		cpi r16,2
		brlo pc+2
		ldi r16,1
		ldi XL,low(cgbuf_l+D_LineLength+8)
		ldi XH,high(cgbuf_l+D_LineLength+8)	
		PUTSTR_t r16,str_table_onoff
		pop YH
		pop YL

	
		
		
	
			
		lds r16,taskmgr_mode
		cpi r16,Taskmgr_MD_Modify
		brne pc+3
		ldi r16,'M'
		sts cgbuf_l+D_LineLength,r16


		ldi r16,':'
		sts cgbuf_l+11,r16

		ldi r16,2
		sts cgbuf_l+2,r16
		sts cgbuf_l+D_LineLength+2,r16
		sts cgbuf_l+7,r16
		sts cgbuf_l+D_LineLength+7,r16

		lds r16,taskmgr_mode
		cpi r16,Taskmgr_MD_View
		brne pc+2
		ret
		; put cursor on which parameter i currently modify
		ldi r18,'>'

		CLR r17
		lds r16,task_modify_selector
		lsl r16
		ldi ZL,low(selector_table*2)
		ldi ZH,high(selector_table*2)
		ADD ZL,r16
		ADC ZH,r17

		lpm r16,Z+
		lpm r16,Z+

		ldi XL,low(cgbuf_l)
		ldi XH,high(cgbuf_l)

		add XL,r16
		adc XH,r17

		st X,r18

	
		ret






setclock:
	rcall showclock

	ldi r18,'>'

	CLR r17
	lds r16,clock_modify_selector
	lsl r16
	ldi ZL,low(clock_selector_table*2)
	ldi ZH,high(clock_selector_table*2)
	ADD ZL,r16
	ADC ZH,r17

	lpm r16,Z+
	mov r19,r16 ; catch first column of clock selector table
	lpm r16,Z+
	
	ldi XL,low(cgbuf_l)
	ldi XH,high(cgbuf_l)
	add XL,r16
	adc XH,r17
	st X,r18

	; using same load cycle, move selector index to r16 and do index load
	mov r16,r19
	clr r17
	ldi XL,low(rtc_data)
	ldi XH,high(rtc_data)
	ADD XL,r16
	ADC XH,r17

	
; do some crazy shit
	lds r16,kb_cur
	cpi r16,Key_Up
	brne sc_k_dn
	
	ld r18,X
	inc r18
	st X,r18
	rjmp sc_endkey

sc_k_dn:
	cpi r16,Key_Down
	brne sc_k_rt

	ld r18,X
	dec r18
	st X,r18
	rjmp sc_endkey

sc_k_rt:
	cpi r16,Key_Right
	brne sc_k_lt
	lds r16,kb_cont
	cpi r16,255
	breq sc_endkey

	lds r18,clock_modify_selector
	inc r18
	sts clock_modify_selector,r18
	rjmp sc_endkey

sc_k_lt:
	cpi r16,Key_Left
	brne sc_k_esc
	lds r16,kb_cont
	cpi r16,255
	breq sc_endkey

	lds r18,clock_modify_selector
	dec r18
	sts clock_modify_selector,r18
	rjmp sc_endkey

sc_k_esc:
	cpi r16,Key_Esc
	brne sc_endkey
	lds r16,kb_cont
	cpi r16,255
	breq sc_endkey

	ldi r18,Mode_Clock
	sts mode,r18

sc_endkey:
; compare with limits
	lds r16,rtc_data+DT_Seconds
	cpi r16,60
	brlo pc+3
	ldi r16,0
	sts rtc_data+DT_Seconds,r16

	CBR r16,1<<RTC_ClockHalt
	sts rtc_data+DT_Seconds,r16

	lds r16,rtc_data+DT_Minutes
	cpi r16,60
	brlo pc+3
	ldi r16,0
	sts rtc_data+DT_Minutes,r16

	lds r16,rtc_data+DT_Hours
	cpi r16,24
	brlo pc+3
	ldi r16,0
	sts rtc_data+DT_Hours,r16


	lds r16,rtc_data+DT_Date
	cpi r16,32
	brlo pc+3
	ldi r16,1
	sts rtc_data+DT_Date,r16

	lds r16,rtc_data+DT_Date
	cpi r16,0
	brne pc+3
	ldi r16,1
	sts rtc_data+DT_Date,r16


	lds r16,rtc_data+DT_Month
	cpi r16,13
	brlo pc+3
	ldi r16,1
	sts rtc_data+DT_Month,r16

	lds r16,rtc_data+DT_Month
	cpi r16,0
	brne pc+3
	ldi r16,1
	sts rtc_data+DT_Month,r16

	lds r16,rtc_data+DT_Year
	cpi r16,100
	brlo pc+3
	ldi r16,0
	sts rtc_data+DT_Year,r16

	lds r16,clock_modify_selector
	cpi r16,DT_Year+1
	brlo pc+3
	ldi r16,0
	sts clock_modify_selector,r16

	rcall set_rtc

	

endsetclock:
	ret


override:
	lds r16,kb_cur
	cpi r16,Key_Esc
	brne pc+3
	ldi r16,Mode_Menu
	sts mode,r16
	NOP

	putstr str_override,cgbuf_l
	ret



eepromsave:
	; put WAIT before drawing loop

	LDI CHAR_BUF, D_SetAddr+0x40
	RCALL SEND_CMD
	LDI CHAR_BUF,'S'
	rcall SEND_DAT
	LDI CHAR_BUF,'a'
	rcall SEND_DAT
	LDI CHAR_BUF,'v'
	rcall SEND_DAT
	LDI CHAR_BUF,'i'
	rcall SEND_DAT
	LDI CHAR_BUF,'n'
	rcall SEND_DAT
	LDI CHAR_BUF,'g'
	rcall SEND_DAT

 	ldi r16,1
	ldi r17,0
	clr r18
	ldi r19,S_TaskArrayMaxLength*S_TaskItemLength

	ldi xl,low(task_array)
	ldi xh,high(task_array)

eesav_m:
	ld r18,X+
	rcall ewrite	
	inc r16
	dec r19
	brne eesav_m
	ldi r16,Mode_Menu
	sts mode,r16
	ret

 

eepromload:
	ldi r16,1
	ldi r17,0
	clr r18
	ldi r19,S_TaskArrayMaxLength*S_TaskItemLength

	ldi xl,low(task_array)
	ldi xh,high(task_array)

eeld_m:
	rcall eread	
	st X+,r18
	inc r16
	dec r19
	brne eeld_m
	ldi r16,Mode_Menu
	sts mode,r16
	ret

clrtasks:
	PUTSTR str_confirm,cgbuf_l

	lds r16,kb_cur
	cpi r16,Key_F2
	brne clr_kb_esc
	lds r16,kb_cont
	cpi r16,255
	breq clr_kb_end
	

	ldi r16,S_TaskArrayMaxLength*S_TaskItemLength
	clr r17
	ldi xl,low(task_array)
	ldi xh,high(task_array)
clr_m:
	st X+,r17
	dec r16
	brne clr_m

clr_kb_esc:
	cpi r16,Key_Esc
	brne clr_kb_end
	ldi r16,Mode_Menu
	sts mode,r16


clr_kb_end:

	ret


showabout:
	lds r16,kb_cur
	cpi r16,Key_Esc
	brne pc+3
	ldi r16,Mode_Menu
	sts mode,r16
	NOP

	PUTSTR str_about_menu,cgbuf_l
	ret



initialize:
	sbi Displ_Dir,Displ_Pin_En
	sbi Displ_Dir,Displ_Pin_Rs
	cbi Displ_Port,Displ_Pin_En
	cbi Displ_Port,Displ_Pin_Rs

	sbi Relay1_2_Dir,Relay1
	sbi Relay1_2_Dir,Relay2
	sbi Relay3_Dir,Relay3
	sbi PWM_Dir,PWM_1
	sbi PWM_Dir,PWM_2
	sbi Resvd_Dir,Resvd_1
	sbi Resvd_Dir,Resvd_2
	sbi Resvd_Dir,Resvd_3

	cbi Relay1_2_Port,Relay1
	cbi Relay1_2_Port,Relay2
	cbi Relay3_Port,Relay3
	cbi PWM_Port,PWM_1
	cbi PWM_Port,PWM_2
	cbi Resvd_Port,Resvd_1
	cbi Resvd_Port,Resvd_2
	cbi Resvd_Port,Resvd_3

	sbi I2C_Dir,I2C_SCL ; OPEN COLLECTOR NETWORK
	sbi I2C_Dir,I2C_SDA ; OPEN COLLECTOR NETWORK
	cbi I2C_Port,I2C_SCL
	cbi I2C_Port,I2C_SDA

	

	sbi KBIND,Key_CLK			
	sbi KBIND,SHL
	cbi KBIND,DAT 

	rcall clr_ram
	rcall delay
	rcall init_display
	
	rcall clr_tasks
	rcall eepromload

	ldi r16,0
	sts task_display_index,r16
	sts taskmgr_mode,r16
	sts task_modify_selector,r16
	sts io_state,r16
	sts clock_modify_selector,r16

	ldi r16,Mode_Clock
	sts mode,r16
	inc r16
	sts menu_index,r16

	;rcall rtc_setup

	ret



; shows current stack pointer
show_stack:
	ldi r16,Itoa_ZeroPadding
	sts itoa_pad,r16

	in r16,SPL
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+17)
	ldi XH,high(cgbuf_l+17)
	rcall print_int

	in r16,SPH
	sts itoa_in,r16
	rcall itoa
	ldi XL,low(cgbuf_l+20)
	ldi XH,high(cgbuf_l+20)
	rcall print_int
	ret



clr_tasks:
	ldi YL,low(task_array)
	ldi YH,high(task_array)
	clr r16


	ldi r17,S_TaskArrayMaxLength
clr_task_m:
	ldi r18,S_TaskItemLength
clr_task_m2:
	st Y+,r16
	dec r18
	brne clr_task_m2
	
	dec r17
	brne clr_task_m


	sts task_display_index,r16
	ret



convert_bcd2dec:
	ldi XL,low(rtc_data)
	ldi XH,high(rtc_data)
	ldi r19,7
cnv_m:
	ld r16,X
	sts btoi_in,r16
	rcall btoi
	lds r16,btoi_out	
	st X+,r16
	dec r19
	brne cnv_m
	ret








; PUT STRINGS OUT OF DWORD TABLE, CORRECT THIS!



.include "display.asm"
.include "delay.asm"
.include "auxlib.asm"
.include "static.asm"

.eseg

.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0

.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0

.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0

.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
.db 0,0,0,0,0,0
