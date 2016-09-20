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

; PD3 is an output to LED3, negative logic
; PD2 is an output to LED2, negative logic
; PD1 is an output to LED1, negative logic
; PD0 is an output to LED0, negative logic

LEDS                EQU     0x40006030,32   ; access PC4-5
GPIO_PORTC_DATA_R   EQU     0x400063FC,32
GPIO_PORTC_DIR_R    EQU     0x40006400,32
GPIO_PORTC_AFSEL_R  EQU     0x40006420,32
GPIO_PORTC_DR8R_R   EQU     0x40006508,32
GPIO_PORTC_DEN_R    EQU     0x4000651C,32
GPIO_PORTC_AMSEL_R  EQU     0x40006528,32
GPIO_PORTC_PCTL_R   EQU     0x4000652C,32
SYSCTL_RCGCGPIO_R   EQU     0x400FE608,32
SYSCTL_RCGC2_GPIOC  EQU     0x00000004      ; port D Clock Gating Control



      .global main
GPIO_Init:  .asmfunc
    ; 1) activate clock for Port D
    LDR R1, SYSCTL_RCGCGPIO_R
    LDR R0, [R1]
    ORR R0, R0, #SYSCTL_RCGC2_GPIOC ; clock
    STR R0, [R1]
    NOP
    NOP                             ; allow time to finish activating
    ; 2) no need to unlock PD3-0
    ; 3) disable analog functionality
    LDR R1, GPIO_PORTC_AMSEL_R
    LDR R0, [R1]
    BIC R0, R0, #0x30               ; disable analog functionality on PC4-5
    STR R0, [R1]
    ; 4) configure as GPIO
    LDR R1, GPIO_PORTC_PCTL_R
    LDR R0, [R1]
    MOV R2, #0x00FF0000
    BIC R0, R0, R2                  ; clear port control field for PC4-5
    STR R0, [R1]

    ; 5) set direction register
    LDR R1, GPIO_PORTC_DIR_R
    LDR R0, [R1]
    ORR R0, R0, #0x30               ; make PC4-5 output
    STR R0, [R1]
    ; 6) regular port function
    LDR R1, GPIO_PORTC_AFSEL_R
    LDR R0, [R1]
    BIC R0, R0, #0x30               ; disable alt funct on PC4-5
    STR R0, [R1]
    ; enable 8mA drive (only necessary for bright LEDs)
    LDR R1, GPIO_PORTC_DR8R_R
    LDR R0, [R1]
    ORR R0, R0, #0x30               ; enable 8mA drive on PC4-5
    STR R0, [R1]
    ; 7) enable digital port
    LDR R1, GPIO_PORTC_DEN_R
    LDR R0, [R1]
    ORR R0, R0, #0x30               ; enable digital I/O on PC4-5
    STR R0, [R1]
    BX  LR
    .endasmfunc

main:  .asmfunc
    BL  GPIO_Init
    LDR R0, LEDS                    ; R0 = LEDS
    MOV R1, #16                     ; R1 = 16 = 0x00010000
    MOV R2, #48                     ; R2 = 48 = 0x00110000
    MOV R3, #32                     ; R3 = 32 = 0x00100000
    MOV R4, #0                      ; R4 = 0  = 0x00000000
loop
    STR R1, [R0]                    ; [R0] = R1 = 10
    STR R2, [R0]                    ; [R0] = R2 = 9
    STR R3, [R0]                    ; [R0] = R3 = 5
    STR R4, [R0]                    ; [R0] = R4 = 6
    B loop                          ; unconditional branch to 'loop'
    .endasmfunc

    .end                             ; end of file
