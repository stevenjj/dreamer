; lightsASM.s
; Contains code that generates control signals for WS2812 LEDs.

; This file is part of Dreamer Head TestBuild0902.
; Travis Llado, travis@travisllado.com
; Last modified 2016.10.11

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

CLRBLUE             EQU 0x0000001F  ; blue
CLRBLUEGREEN        EQU 0x00070017  ; blue-green
CLRGREEN            EQU 0x001F0000  ; green
CLRMAGENTA          EQU 0x00000F0F  ; magenta
CLRORANGE           EQU 0x00071700  ; orange
CLRPINK             EQU 0x00051505  ; pink
CLRPURPLE           EQU 0x00000B14  ; purple
CLRRED              EQU 0x00001F00  ; red
CLRWHITE            EQU 0x000F0F0F  ; white
CLRYELLOW           EQU 0x00171700  ; yellow

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  lightsInit
        EXPORT  lightsBlue
        EXPORT  lightsBlueGreen
        EXPORT  lightsGreen
        EXPORT  lightsMagenta
        EXPORT  lightsOrange
        EXPORT  lightsPink
        EXPORT  lightsPurple
        EXPORT  lightsRed
        EXPORT  lightsWhite
        EXPORT  lightsYellow

lightsInit
    PUSH    {LR}                    ; push current LR to stack
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
    ; Update lights to init color
    LDR     R0, =CLRPURPLE          ; load CLRPURPLE into R0
    PUSH    {R0}                    ; push CLRPURPLE to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return

lightsBlue
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRBLUE            ; load CLRBLUE into R0
    PUSH    {R0}                    ; push CLRBLUE to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsBlueGreen
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRBLUEGREEN       ; load CLRBLUEGREEN into R0
    PUSH    {R0}                    ; push CLRBLUEGREEN to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsGreen
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRGREEN           ; load CLRGREEN into R0
    PUSH    {R0}                    ; push CLRGREEN to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsMagenta
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRMAGENTA         ; load CLRMAGENTA into R0
    PUSH    {R0}                    ; push CLRMAGENTA to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsOrange
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRORANGE          ; load CLRORANGE into R0
    PUSH    {R0}                    ; push CLRORANGE to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsPink
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRPINK            ; load CLRPINK into R0
    PUSH    {R0}                    ; push CLRPINK to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsPurple
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRPURPLE          ; load CLRPURPLE into R0
    PUSH    {R0}                    ; push CLRPURPLE to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsRed
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRRED             ; load CLRRED into R0
    PUSH    {R0}                    ; push CLRRED to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsWhite
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRWHITE           ; load CLRWHITE into R0
    PUSH    {R0}                    ; push CLRWHITE to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return


lightsYellow
    PUSH    {LR}                    ; push current LR to stack
    ; Update lights to init color
    LDR     R0, =CLRYELLOW          ; load CLRYELLOW into R0
    PUSH    {R0}                    ; push CLRYELLOW to stack
    BL      lightsUpdate            ; branch to lightsUpdate, then return here
    ; Return
    POP     {LR}                    ; pull previous LR from stack
    BX      LR                      ; return

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
    ; Retrive arguments, store LR
    POP     {R0}            ; pull new color value from stack to R0
    PUSH    {R4-R7, LR}     ; push previous R4-R7, LR values to stack
    ; Arrange necessary values
    LDR     R1, =LEDS       ; R1 = register that controls Port C GPIO Pins
    LDR     R2, =PINSHIGH   ; R2 = value that sets PC4-5 high
    LDR     R3, =PINSLOW    ; R3 = value that sets PC4-5 low
    LDR     R4, =NUMLEDS    ; R4 = number of LEDs, times light loop runs
    MOV     R7, #2          ; store value '2' in R7 for division later on
nextLight                   ; write new color to single LED
    ; Begin writing to another LED
    LDR     R5, =BITMASK    ; load BITMASK into R5s
nextBit                     ; write single bit to current LED
    ; Begin writing next bit to an LED
    AND     R6, R0, R5      ; extract next bit to be written
    UDIV    R6, R5          ; shift next bit to 1's column
    UDIV    R5, R7          ; shift BITMASK one place down
    ; Determine next bit to be written
    CMP     R6, #0          ; compare next bit value to zero
    IT      NE              ; are they not equal?
    BLNE    onePulse        ; if yes, then write a one
    IT      EQ              ; are they equal?
    BLEQ    zeroPulse       ; if yes, then write a zero
    ; Check whether or not current light is complete
    CMP     R5, #0          ; compare BITMASK value to zero
    IT      NE              ; are they not equal?
    BNE     nextBit         ; if yes, then write another bit
    ; Check whether current string of lights is complete
    SUB     R4, #1          ; decrement LED counter
    CMP     R4, #0          ; compare LED counter value to zero
    IT      NE              ; are they not equal?
    BNE     nextLight       ; if yes, then write another light
    ; Return
    POP     {R4-R7, LR}     ; pull previous R4-R7, LR values from stack
    BX      LR              ; return

    ALIGN                   ; make sure the end of this section is aligned
    END                     ; end of file
