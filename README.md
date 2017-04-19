# relay_sched
AVR ATmega8 relay scheduler project

This unit is used to program around 32 timers to control up to 8 relays (or less if PWM channels are used).
Was made as AVR assembler language practice, uses DS1307 RTC, HC595/HC165 registers and hd44780 based character display. 
PCB in SprintLayout format inside. Project was written in AVRStudio 4.

To-do:
 - port it to C language?
 - add USART control, more conditions, EEPROM memory etc.
