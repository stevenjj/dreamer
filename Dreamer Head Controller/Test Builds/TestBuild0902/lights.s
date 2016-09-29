; lightsASM.s
; Contains code that generates control signals for WS2812 LEDs.

; This file is part of Dreamer Head TestBuild0902.
; Travis Llado, travis@travisllado.com
; Last modified 2016.09.28

PTC_DATA_R          EQU 0x400063FC
PTC_DIR_R           EQU 0x40006400
PTC_AFSEL_R         EQU 0x40006420
PTC_DEN_R           EQU 0x4000651C
PTC_AMSEL_R         EQU 0x40006528
PTC_PCTL_R          EQU 0x4000652C
RCGCGPIO_R          EQU 0x400FE608
RCGC2_GPIOC         EQU 0x00000004  ; port C Clock Gating Control
LEDS                EQU 0x400060C0  ; access PC4-5
PINSHIGH            EQU 0x00000030  ; write PC4-5 high
PINSLOW             EQU 0x00000000  ; write PC4-5 low
NUMLEDS             EQU 0x00000028  ; number of WS2812 LEDs in series
BITMASK             EQU 0x00800000  ; masks bit 24, first bit of LED signal
CLRPURPLE           EQU 0x00000F0F  ; purple color code, used for initialization

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  lightsInit
        EXPORT  lightsUpdate

lightsInit
    ; 1) activate clock for Port C
    LDR     R1, =RCGCGPIO_R
    LDR     R0, [R1]
    ORR     R0, R0, #RCGC2_GPIOC ; clock
    STR     R0, [R1]
    NOP                     ; allow time to finish activating
    NOP
    ; 3) disable analog functionality
    LDR     R1, =PTC_AMSEL_R
    LDR     R0, [R1]
    BIC     R0, R0, #0x30   ; disable analog functionality on PC4/5
    STR     R0, [R1]
    ; 4) configure as GPIO
    LDR     R1, =PTC_PCTL_R
    LDR     R0, [R1]
    BIC     R0, R0, #0x00FF0000 ; clear port control field for PC4/5
    STR     R0, [R1]

    ; 5) set direction register
    LDR     R1, =PTC_DIR_R
    LDR     R0, [R1]
    ORR     R0, R0, #0x30   ; make PC4/5 output
    STR     R0, [R1]
    ; 6) regular port function
    LDR     R1, =PTC_AFSEL_R
    LDR     R0, [R1]
    BIC     R0, R0, #0x30   ; disable alt funct on PC4/5
    STR     R0, [R1]
    ; 7) enable digital port
    LDR     R1, =PTC_DEN_R
    LDR     R0, [R1]
    ORR     R0, R0, #0x30   ; enable digital I/O on PC4/5
    STR     R0, [R1]
    PUSH    {LR}            ; push current LR to stack
    LDR     R1, =CLRPURPLE  ; load CLRPURPLE into R1
    PUSH    {R1}            ; push CLRPURPLE onto stack
    BL      lightsUpdate    ; update lights to CLRPURPLE
    POP     {LR}            ; restore LR from stack
    BX      LR              ; return

zeroPulse                   ; generate 0 pulse (0.5us high, 2.0us low)
    STR     R2, [R1]        ; set PC4-5 high
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    STR     R3, [R1]        ; set PC4-5 low
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    BX      LR              ; return

onePulse                    ; generate 1 pulse (1.2us high, 1.3us low)
    STR     R2, [R1]        ; set PC4-5 high
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    STR     R3, [R1]        ; set PC4-5 low
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    NOP                     ; no operation
    BX      LR              ; return

lightsUpdate                ; write arbitrary color value to all ear lights
    POP     {R0}            ; pull new color value from stack to R0
    PUSH    {LR, R4-R7}     ; push previous LR, R4-R7 valuesto stack
    LDR     R1, =LEDS       ; R1 = register that controls Port C GPIO Pins
    LDR     R2, =PINSHIGH   ; R2 = value that sets PC4-5 high
    LDR     R3, =PINSLOW    ; R3 = value that sets PC4-5 low
    LDR     R4, =NUMLEDS    ; R4 = number of LEDs, times light loop runs
    MOV     R7, #2          ; store value '2' in R7 for division later on
nextLight                   ; write new color to single LED
    LDR     R5, =BITMASK    ; load BITMASK into R5
nextBit                     ; write single bit to current LED
    AND     R6, R0, R5      ; extract next bit to be written
    UDIV    R6, R5          ; shift next bit to 1's column
    UDIV    R5, R7          ; shift BITMASK one place down
    CMP     R6, #0          ; compare next bit value to zero
    IT      NE              ; are they not equal?
    BLNE    onePulse        ; if yes, then write a one
    IT      EQ              ; are they equal?
    BLEQ    zeroPulse       ; if yes, then write a zero
    CMP     R5, #0          ; compare BITMASK value to zero
    IT      NE              ; are they not equal?
    BNE     nextBit         ; if yes, then write another bit
    SUB     R4, #1          ; decrement LED counter
    CMP     R4, #0          ; compare LED counter value to zero
    IT      NE              ; are they not equal?
    BNE     nextLight       ; if yes, then write another light
    POP     {LR, R4-R7}     ; pull previous LR, R4-R7 values from stack
    BX      LR              ; return

    ALIGN                   ; make sure the end of this section is aligned
    END                     ; end of file
