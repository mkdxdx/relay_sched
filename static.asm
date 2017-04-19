str_about_menu:
		.db "Relay scheduler By mkdxdx",C_STREND

str_confirm:
		.db "  Erase tasks?  F2:YES    ESC:NO"


; Channel names strings and table
str_channels_table:
		.dw str_no_action
		.dw str_ch1
		.dw str_ch2
		.dw str_ch3
		.dw str_chpwm1
		.dw str_chpwm2
		.dw str_res1
		.dw str_res2
		.dw str_res3
		.dw str_allrel
		.dw str_allpwm
		.dw str_allchan

str_no_action:
			.db "N/A",C_STREND
str_ch1:	.db "CH1",C_STREND
str_ch2:	.db "CH2",C_STREND
str_ch3:	.db "CH3",C_STREND
str_chpwm1:	.db "PWM1",C_STREND
str_chpwm2:	.db "PWM2",C_STREND
str_res1:	.db "R1",C_STREND
str_res2:	.db "R2",C_STREND
str_res3:	.db "R3",C_STREND
str_allrel:	.db "C1-3",C_STREND
str_allpwm:	.db "P1-2",C_STREND
str_allchan:.db "ALL",C_STREND


; channel state strings
str_table_onoff:
		.dw str_off
		.dw str_on

str_on:		.db " ON",C_STREND
str_off:	.db "OFF",C_STREND


; task type strings and table

str_tasktype_table:
		.dw str_time
		.dw str_timh
		.dw str_adc
		.dw str_interrupt
		.dw str_sensor

str_time:	.db "T ==",C_STREND
str_timH:	.db "T >=",C_STREND
str_adc:	.db "Ext",C_STREND
str_interrupt:	.db "ADC",C_STREND
str_sensor:	.db "Sens",C_STREND

; main menu strings and table

table_menu_strings:
		.dw str_menu
		.dw str_clock
		.dw str_taskmgr
		.dw str_setclock
		.dw str_override
		.dw str_eepromsave
		.dw str_cleartasks
		.dw str_about


str_menu:		.db " ",C_STREND
str_clock:		.db "Show clock",C_STREND
str_taskmgr:	.db "Task manager",C_STREND
str_setclock:	.db "Set clock",C_STREND
str_override:	.db "Relay override",C_STREND
str_eepromsave:	.db "Save tasks", C_STREND
str_cleartasks:	.db "Clear tasks",C_STREND
str_about:		.db "About",C_STREND
; mode table

mode_table:
		.dw showmenu
		.dw	showclock
		.dw	showtaskmgr
		.dw	setclock
		.dw	override
		.dw eepromsave
		.dw clrtasks
		.dw	showabout

; modify subroutines

modify_table:
		.dw modify_time
		.dw modify_time ; since timh doesnt differ in modifying than vanilla time
		.dw modify_adc
		.dw modify_interrupt
		.dw modify_sensor

; modify parameters selector display: 
; 1. parameter index
; 2. Task table byte
; 3. Screen shift for marker
selector_table:
	.db Task_FirstData,7
	.db Task_SecondData,11
	.db Task_Exec_Channel,D_LineLength+2
	.db Task_Exec_Value,D_LineLength+7

; compare suborutine table

compare_table:
	.dw compare_time
	.dw compare_timh
	.dw compare_adc
	.dw compare_interrupt
	.dw compare_sensor

; execute subroutine table

execute_table:
	.dw exec_na
	.dw exec_rel1
	.dw exec_rel2
	.dw exec_rel3
	.dw exec_pwm1
	.dw exec_pwm2
	.dw exec_res1
	.dw exec_res2
	.dw exec_res3
	.dw exec_allrelay
	.dw exec_allpwm
	.dw exec_allchan

; CGRAM custom symbols: disabled/enabled channels

chan_char_disbl:
	.db 0b00011111,0b00010001
	.db 0b00010001,0b00010001
	.db 0b00011111,0b00011111
	.db 0b00011111,0b00011111
chan_char_enabl:
	.db 0b00011111,0b00011111
	.db 0b00011111,0b00011111
	.db 0b00010001,0b00010001
	.db 0b00010001,0b00011111

char_stick:
	.db 0b00000100,0b00000100
	.db 0b00000100,0b00000100
	.db 0b00000100,0b00000100
	.db 0b00000100,0b00000100

; SetClock selector table: parameter index, screen position

clock_selector_table:
	.db DT_Hours,5
	.db DT_Minutes,8
	.db DT_Seconds,11
	.db DT_Date,D_LineLength+5
	.db DT_Month,D_LineLength+8
	.db DT_Year,D_LineLength+11
	
	
	
	


	
	
	
