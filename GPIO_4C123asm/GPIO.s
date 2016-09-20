; GPIO.s
; Runs on LM4F120/TM4C123
; Initialize four GPIO pins as outputs.  Continually generate output to
; drive simulated stepper motor.
; Daniel Valvano
; May 3, 2015

;  This example accompanies the book
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2015
;  Volume 1 Program 4.5

;"Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
;   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
;   Volume 2 Example 2.2, Program 2.8, Figure 2.32
;
;Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
;   You may use, edit, run or distribute this file
;   as long as the above copyright notice remains
;THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
;OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
;MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
;VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
;OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
;For more information about my classes, my research, and my books, see
;http://users.ece.utexas.edu/~valvano/

; PC4 is an output to LED3, negative logic
; PC5 is an output to LED2, negative logic

LEDS                EQU 0x400060C0  ; access PC4-5
GPIO_PORTC_DATA_R   EQU 0x400063FC
GPIO_PORTC_DIR_R    EQU 0x40006400
GPIO_PORTC_AFSEL_R  EQU 0x40006420
GPIO_PORTC_DEN_R    EQU 0x4000651C
GPIO_PORTC_AMSEL_R  EQU 0x40006528
GPIO_PORTC_PCTL_R   EQU 0x4000652C
SYSCTL_RCGCGPIO_R   EQU 0x400FE608
SYSCTL_RCGC2_GPIOC  EQU 0x00000004  ; port C Clock Gating Control
PINSHIGH            EQU 0x00000030
PINSLOW             EQU 0x00000000
NUMLEDS             EQU 0x00000028
DELAYLENGTH         EQU 0x000009C4

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start
GPIO_Init
    ; 1) activate clock for Port C
    LDR R1, =SYSCTL_RCGCGPIO_R
    LDR R0, [R1]
    ORR R0, R0, #SYSCTL_RCGC2_GPIOC ; clock
    STR R0, [R1]
    NOP
    NOP                             ; allow time to finish activating
    ; 2) no need to unlock PC4-5
    ; 3) disable analog functionality
    LDR R1, =GPIO_PORTC_AMSEL_R
    LDR R0, [R1]
    BIC R0, R0, #0x30               ; disable analog functionality on PC4-5
    STR R0, [R1]
    ; 4) configure as GPIO
    LDR R1, =GPIO_PORTC_PCTL_R
    LDR R0, [R1]
    BIC R0, R0, #0x00FF0000                  ; clear port control field for PC4-5
    STR R0, [R1]
    ; 5) set direction register
    LDR R1, =GPIO_PORTC_DIR_R
    LDR R0, [R1]
    ORR R0, R0, #0x30               ; make PC4-5 output
    STR R0, [R1]
    ; 6) regular port function
    LDR R1, =GPIO_PORTC_AFSEL_R
    LDR R0, [R1]
    BIC R0, R0, #0x30               ; disable alt funct on PC4-5
    STR R0, [R1]
    ; 7) enable digital port
    LDR R1, =GPIO_PORTC_DEN_R
    LDR R0, [R1]
    ORR R0, R0, #0x30               ; enable digital I/O on PC4-5
    STR R0, [R1]
    BX  LR

zeroPulse   ; 0 pulse (0.5us high, 2.0us low)
    STR R1, [R0]                    ; set PC4-5 high
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
    STR R2, [R0]                    ; set PC4-5 low
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
    BX  LR

onePulse    ; 1 pulse (1.2us high, 1.3us low)
    STR R1, [R0]                    ; set PC4-5 high
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
    STR R2, [R0]                    ; set PC4-5 low
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
	NOP								; no operation
    BX  LR

ledDelay    ; delay between frames (~315us)
	LDR     R3, =DELAYLENGTH     ; R3 = 3, number of loops between pauses
spinCycle     ; write single LED orange
    SUB     R3, R3, #1  ; decrement counter (R3)
	CMP     R3, #0
    IT      NE          ; has counter decreased to zero?
	BNE     spinCycle ; write another light
    BX  LR

orangeLED
	; inter-frame delay (>=50us)
	PUSH {LR}
	; green (0x00)
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  onePulse    ; produce one pulse
    BL  onePulse    ; produce one pulse
    BL  onePulse    ; produce one pulse
    BL  onePulse    ; produce one pulse
	; red (0x1F)
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  onePulse    ; produce one pulse
    BL  onePulse    ; produce one pulse
    BL  onePulse    ; produce one pulse
    BL  onePulse    ; produce one pulse
    BL  onePulse    ; produce one pulse
	; blue (0x00)
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
    BL  zeroPulse   ; produce zero pulse
	POP {LR}
    BX  LR

orangeLights    ; write all LEDs orange
    PUSH    {LR}        
    LDR     R0, =LEDS   ; R0 = LEDS
    LDR     R1, =PINSHIGH   ; R1 = 0x30 = 0b00110000
    LDR     R2, =PINSLOW    ; R2 = 0x00 = 0b00000000
	LDR     R3, =NUMLEDS     ; R3 = 3, number of loops between pauses
singleLight     ; write single LED orange
	BL      orangeLED   ; execute orangeLED function
    SUB     R3, R3, #1  ; decrement counter (R3)
	CMP     R3, #0
    IT      NE          ; has counter decreased to zero?
	BNE     singleLight ; write another light
	POP     {LR}
    BX      LR          ; exit light update function

Start
    BL      GPIO_Init
loop
    BL      orangeLights
	BL      ledDelay
    B       loop        ; unconditional branch to 'loop'

    ALIGN               ; make sure the end of this section is aligned
    END                 ; end of file
