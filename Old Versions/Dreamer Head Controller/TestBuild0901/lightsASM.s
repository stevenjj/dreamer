; lightsASM.s
; Contains code that generates control signals for WS2812 LEDs.

; This file is part of Dreamer Head TestBuild0010.
; Travis Llado, travis@travisllado.com
; Last modified 2016.09.19

LEDS                EQU 0x400060C0  ; access PC4-5
PINSHIGH            EQU 0x00000030  ; write PC4-5 high
PINSLOW             EQU 0x00000000  ; write PC4-5 low
NUMLEDS             EQU 0x00000028  ; number of WS2812 LEDs in series

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  orangeLights
        EXPORT  blueLights

zeroPulse           		; generate 0 pulse (0.5us high, 2.0us low)
    STR R1, [R0]    		; set PC4-5 high
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    STR R2, [R0]    		; set PC4-5 low
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    BX  LR          		; return

onePulse            		; generate 1 pulse (1.2us high, 1.3us low)
    STR R1, [R0]    		; set PC4-5 high
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    STR R2, [R0]    		; set PC4-5 low
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    NOP             		; no operation
    BX  LR          		; return

orangeLED           		; generate signal for single orange LED
    PUSH {LR}       		; store initial link register
    ; green (0x0F)
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    ; red (0x1F)
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    ; blue (0x00)
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    POP {LR}        		; restore initial link register
    BX  LR          		; return

blueLED             		; generate signal for single blue LED
    PUSH {LR}       		; store initial link register
    ; green (0x00)
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    ; red (0x1F)
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    ; blue (0x00)
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  zeroPulse   		; produce zero pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    BL  onePulse    		; produce one pulse
    POP {LR}        		; restore initial link register
    BX  LR          		; return

orangeLights                ; write all LEDs orange
    PUSH    {LR}            ; store initial link register
    LDR     R0, =LEDS       ; R0 = register that controls Port C GPIO Pins
    LDR     R1, =PINSHIGH   ; R1 = value that sets PC4-5 high
    LDR     R2, =PINSLOW    ; R2 = value that sets PC4-5 low
    LDR     R3, =NUMLEDS    ; R3 = number of LEDs, times loop is repeated
singleOrange                ; write single LED orange
    BL      orangeLED       ; produce signal for single LED
    SUB     R3, R3, #1      ; decrement counter (R3)
    CMP     R3, #0          ; compare counter value
    IT      NE              ; has counter reached zero?
    BNE     singleOrange    ; if not, write another light
    POP     {LR}            ; restore initial link register
    BX      LR              ; return

blueLights                  ; write all LEDs blue
    PUSH    {LR}            ; store initial link register
    LDR     R0, =LEDS       ; R0 = register that controls Port C GPIO Pins
    LDR     R1, =PINSHIGH   ; R1 = value that sets PC4-5 high
    LDR     R2, =PINSLOW    ; R2 = value that sets PC4-5 low
    LDR     R3, =NUMLEDS    ; R3 = number of LEDs, times loop is repeated
singleBlue                  ; write single LED blue
    BL      blueLED         ; produce signal for single LED
    SUB     R3, R3, #1      ; decrement counter (R3)
    CMP     R3, #0          ; compare counter value
    IT      NE              ; has counter reached zero?
    BNE     singleBlue      ; if not, write another light
    POP     {LR}            ; restore initial link register
    BX      LR              ; return

    ALIGN                   ; make sure the end of this section is aligned
    END                     ; end of file
