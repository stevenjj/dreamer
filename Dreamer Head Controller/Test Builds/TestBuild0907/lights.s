
; lights.s
; Contains code that generates control signals for WS2812 LEDs.

; This file is part of Dreamer Head TestBuild0906
; Travis Llado, travis@travisllado.com
; Last modified 2016.10.12

PTC_DIR_R           EQU 0x40006400  ; GPIO direction
PTC_AFSEL_R         EQU 0x40006420  ; alternate functions
PTC_DEN_R           EQU 0x4000651C  ; GPIO select
PTC_AMSEL_R         EQU 0x40006528  ; analog functions
PTC_PCTL_R          EQU 0x4000652C  ; port control
RCGCGPIO_R          EQU 0x400FE608  ; port C clock
RCGC2_GPIOC         EQU 0x00000004  ; port C Clock Gating Control
LEDS                EQU 0x400060C0  ; access PC4-5
PINS_HIGH           EQU 0x00000030  ; write PC4-5 high
PINS_LOW            EQU 0x00000000  ; write PC4-5 low
NUM_LEDS            EQU 0x00000028  ; number of WS2812 LEDs in series (40)
BIT_MASK            EQU 0x00800000  ; masks bit 24, first bit of LED signal
ZERO_HIGH           EQU 0x00000004  ; number of loops pins stay high for zero
ONE_HIGH            EQU 0x0000000A  ; number of loops pins stay high for one
BOTH_LOW            EQU 0x00000008  ; number of loops pins stay low for either

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  lightsInit
        EXPORT  lightsUpdate

lightsInit
    PUSH    {R4-R11, LR}            ; push current LR to stack
    ; 1) activate clock for Port C
    LDR     R1, =RCGCGPIO_R
    LDR     R0, [R1]
    ORR     R0, R0, #RCGC2_GPIOC    ; clock
    STR     R0, [R1]
    NOP                             ; allow time to finish activating
    NOP
    ; 3) disable analog functionality
    LDR     R1, =PTC_AMSEL_R
    LDR     R0, [R1]
    BIC     R0, R0, #0x30           ; disable analog functionality on PC4/5
    STR     R0, [R1]
    ; 4) configure as GPIO
    LDR     R1, =PTC_PCTL_R
    LDR     R0, [R1]
    BIC     R0, R0, #0x00FF0000     ; clear port control field for PC4/5
    STR     R0, [R1]
    ; 5) set direction register
    LDR     R1, =PTC_DIR_R
    LDR     R0, [R1]
    ORR     R0, R0, #0x30           ; make PC4/5 output
    STR     R0, [R1]
    ; 6) regular port function
    LDR     R1, =PTC_AFSEL_R
    LDR     R0, [R1]
    BIC     R0, R0, #0x30           ; disable alt funct on PC4/5
    STR     R0, [R1]
    ; 7) enable digital port
    LDR     R1, =PTC_DEN_R
    LDR     R0, [R1]
    ORR     R0, R0, #0x30           ; enable digital I/O on PC4/5
    STR     R0, [R1]
    ; Return
    POP     {R4-R11, LR}            ; pull previous LR from stack
    BX      LR                      ; return

lightsUpdate                ; write arbitrary color value to all ear lights
    PUSH    {R4-R11, LR}    ; push previous R4-R11, LR values to stack
    ; received argument     ; R0 = color value to be written
    LDR     R1, =LEDS       ; R1 = register that controls Port C GPIO Pins
    LDR     R2, =PINS_HIGH  ; R2 = value that sets PC4-5 high
    LDR     R3, =PINS_LOW   ; R3 = value that sets PC4-5 low
    LDR     R4, =NUM_LEDS   ; R4 = number of LEDs, times light loop runs
    MOV     R5, #2          ; R5 = 2, used to shift values left by one bit
nextLight                   ; write new color to single LED
    LDR     R6, =BIT_MASK   ; load BIT_MASK into R6
nextBit                     ; write single bit to current LED
    AND     R7, R0, R6      ; extract next bit to be written
    UDIV    R7, R6          ; shift next bit to 1's column
    UDIV    R6, R5          ; shift BIT_MASK one place down
    ; Determine next bit to be written
    CMP     R7, #0          ; compare next bit value to zero
    ITE     NE              ; are they NOT equal?
    LDRNE   R8, =ONE_HIGH   ; if yes, then set timing for one high
    LDREQ   R8, =ZERO_HIGH  ; if no, then set timing for zero high
    STR     R2, [R1]        ; set PC4-5 high
waitHigh                    ; Wait required time with GPIO pins high
    SUB     R8, #1          ; decrement loop counter
    CMP     R8, #0          ; compare loop counter value to 0
    IT      NE              ; are they not equal?
    BNE     waitHigh        ; if yes, then wait again
    LDR     R8, =BOTH_LOW   ; set timing for low
    STR     R3, [R1]        ; set PC4-5 low
waitLow                     ; Wait required time with GPIO pins low
    SUB     R8, #1          ; decrement loop counter
    CMP     R8, #0          ; compare loop counter value to exit value
    IT      NE              ; are they not equal?
    BNE     waitLow         ; if yes, then wait again
    ; Check whether or not current light is complete
    CMP     R6, #0          ; compare BIT_MASKvalue to zero
    IT      NE              ; are they not equal?
    BNE     nextBit         ; if yes, then write another bit
    ; Check whether current string of lights is complete
    SUB     R4, #1          ; decrement LED counter
    CMP     R4, #0          ; compare LED counter value to zero
    IT      NE              ; are they not equal?
    BNE     nextLight       ; if yes, then write another light
    ; Return
    POP     {R4-R11, LR}    ; pull previous R4-R11, LR values from stack
    BX      LR              ; return

    ALIGN                   ; make sure the end of this section is aligned
    END                     ; end of file
