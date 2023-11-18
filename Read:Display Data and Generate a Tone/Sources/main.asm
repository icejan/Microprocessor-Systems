;*****************************************************************
;* This program reads the switches SW1 and immediately displays  *
;* their states on the LED1.                                     *
;*                                                               *
;* Course: COE 538                                               *
;* Section: 2                                                    *
;* Lab 2: Assignment 1: Read/Display Data and Generate a Tone    *
;* Name: Janice Zhu                                              *
;* Student ID: 501040242                                         *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup  ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		        INCLUDE 'derivative.inc' 

;*****************************************************************
;* Code section                                                  *
;*****************************************************************


;*****************************************************************
;* The actual program starts here                                *
;*****************************************************************
            ORG   $4000

Entry:
_Startup:

; Exercise 1
            LDAA  #$FF            ; ACCA = $FF
            STAA  DDRH            ; Config. Port H for output
            STAA  PERT            ; Enab. pull-up res. of Port T
            
Loop1:      LDAA  PTT             ; Read Port T
            STAA  PTH             ; Display SWI on LED1 connected 
                                  ; to port H
            BRA   Loop1            ; Loop
            
; Exercise 2
            BSET  DDRP,%11111111  ; Configure Port P for output 
                                  ; (LED2 cntrl)
            BSET  DDRE,%00010000  ; Configure pin PE4 for output
                                  ; (enable bit)
            BCLR  PORTE,%00010000 ; Enable keypad
            
Loop2:      LDAA  PTS             ; Read a key code into AccA
            LSRA                  ; Shift right AccA
            LSRA                  ; _"_
            LSRA                  ; _"_
            LSRA                  ; _"_
            STAA  PTP             ; Output AccA content to LED2
            STAA  PTH             ; Display on LED1
            BRA   Loop2            ; Loop
                          
; Exercise 3
            BSET  DDRP,%11111111  ; Configure Port P for output 
            LDAA  #%10000000      ; Prepare to drive PP7 high
       
MainLoop    STAA  PTP             ; Drive PP7
            LDX   #$1FFF          ; Initialize the loop counter
Delay       DEX                   ; Decrement the loop counter
            BNE   Delay           ; If not done, continue to loop
            EORA  #%10000000      ; Toggle the MSB of AccA
            BRA   MainLoop        ; Go to MainLoop

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            FDB   Entry           ; Reset Vector
