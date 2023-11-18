;*****************************************************************
;* Timer Alarms                                                  *
;*****************************************************************

; export symbols              
            XDEF Entry, _Startup  ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point

; Include derivative-specific definitions 
	        	INCLUDE 'derivative.inc' 
; Definitions
OneSec      EQU    23              ; 1 second delay (at 23Hz)
TwoSec      EQU    46              ; 2 second delay (at 23Hz)

LCD_DAT     EQU   PORTB		    		; LCD data port, bits - PB7,...,PB0 
LCD_CNTR    EQU   PTJ 	  		  	; LCD control port, bits - PE7(RS),PE4(E)
LCD_E       EQU   $80		     		  ; LCD E-signal pin
LCD_RS      EQU   $40      				; LCD RS-signal pin



;TFLG2       EQU    $004F           ; MSB is the timer overflow flag
;TSCR1       EQU    $0046           ; Contains the TEN (timer enable bit)
;TSCR2       EQU    $004D           ; Contains the TOI (timer overflow interrupt enable bit)

; Variable/data section

            ORG    $3850           ; Where our TOF counter register lives
            
TOF_COUNTER RMB    1               ; The timer, incremented at 23Hz
AT_DEMO     RMB    1               ; The alarm time for this demo


; Code section
            ORG   $4000
Entry:
_Startup:


; Motor Control                                            

			    	BSET    DDRA,%00000011	;PA0, PA1
		    		BSET    DDRT,%00110000	;PT4, PT5
				
	    			JSR     STARFWD
	    			JSR     PORTFWD
	    		  JSR     STARON
    				JSR     PORTON
	    			JSR     STARREV
	    			JSR     PORTREV
	    			JSR     STAROFF
	    			JSR     PORTOFF

            LDS    #$4000          ; initialize the stack pointer
            JSR    initLCD         ; initialize the LCD
            JSR    clrLCD          ; clear LCD & home cursor
            JSR    ENABLE_TOF      ; Jump to TOF initialization
            CLI                    ; Enable global interrupt

            LDAA   #'A'             ; Display A (for 1 sec)
            JSR    putcLCD          ;  --"--

            LDAA   TOF_COUNTER      ; Initialize the alarm time
            ADDA   #OneSec          ; by adding on the 1 sec delay
            STAA   AT_DEMO          ; and save it in the alarm

CHK_DELAY_1 LDAA   TOF_COUNTER      ; If the current time
            CMPA   AT_DEMO          ; equals the alarm time
            BEQ    A1               ; then display B
            BRA    CHK_DELAY_1      ; and check the alarm again

A1          LDAA   #'B'
            JSR    putcLCD

            LDAA   AT_DEMO
            ADDA   #TwoSec          ; ... add on the 2 sec delay
            STAA   AT_DEMO          ; ... and save it in the alarm

CHK_DELAY_2 LDAA   TOF_COUNTER      ; If the current time
            CMPA   AT_DEMO          ; ... equals the alarm time
            BEQ    A2               ; then display C
            BRA    CHK_DELAY_2      ; and check the alarm again

A2          LDAA   #'C'
            JSR    putcLCD

            SWI

; Subroutine section
;*********************************************************************
;* Initialization of the LCD: 4=bit data width, 2-line display,      *
;* turn on display, cursor and blinking off. Shift cursor right.     *                      
;*********************************************************************
initLCD     BSET  DDRB,%11111111 ; configure pins bits - PB7,...,PB0 
            BSET  DDRJ,%11000000      ; configure pins PE7(RS),PE4(E)
            LDY   #2000               ; wait for LCD to be ready
            JSR   del_50us            ; -""-
            LDAA  #$28                ; set 4-bit data, 2-line display
            JSR   cmd2LCD             ; -""-
            LDAA  #$0C                ;display on, cursor off, blinking off
            JSR   cmd2LCD             ; -""-
            LDAA  #$06                ; move cursor right after entering a character
            JSR   cmd2LCD             ; -""-
            RTS
;*********************************************************************
;* Clear display and home cursor                                     *
;*********************************************************************
clrLCD      LDAA  #$01                ; clear cursor and return to home position
            JSR   cmd2LCD             ; -""-
            LDY   #40                 ; wait until "clear cursor" command is complete
            JSR   del_50us            ; -""-
            RTS
;*********************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns.                     *
;*********************************************************************
del_50us:   PSHX                      ;2 E-clk
eloop:      LDX   #30                 ;2 E-clk ---------
iloop:      PSHA                      ;2 E-clk - 		|
            PULA                      ;3 E-clk  |		|
            PSHA                      ;2 E-clk  |		|
            PULA                      ;3 E-clk  |		|
            PSHA                      ;2 E-clk  |		|
            PULA                      ;3 E-clk  |		|
            PSHA                      ;2 E-clk  |		|
            PULA                      ;3 E-clk  | 40 	| 40 x 42ns = 1680ns
            PSHA                      ;2 E-clk  | E-clk	| 30 x 1680ns = 50us
            PULA                      ;3 E-clk  |		|
            PSHA                      ;2 E-clk  |		|
            PULA                      ;3 E-clk  |		|
            PSHA                      ;2 E-clk  |		| 
            PULA                      ;3 E-clk  |		|
            NOP                       ;1 E-clk  |		|
            NOP                       ;1 E-clk  |		|
            DBNE  X,iloop             ;3 E-clik -		|
            DBNE  Y,eloop             ;3 E-clik --------
            PULX                      ;3 E-clik
            RTS                       ;5 E-clik

;*********************************************************************
;* This function sends a command in accumulator A to the LCD         *
;*********************************************************************
cmd2LCD     BCLR  LCD_CNTR,LCD_RS     ; select the LCD Instruction Register (IR)
            JSR   dataMov             ; send data to IR
            RTS

;*********************************************************************
;* This function outputs the character in accumulator A to LCD       *
;*********************************************************************
putcLCD     BSET  LCD_CNTR,LCD_RS     ; select the LCD Data register (DR)
            JSR   dataMov             ; send data to DR
            RTS

;*********************************************************************
;* This function sends data to the LCD IR or DR depending on RS      *
;*********************************************************************
dataMov     BSET  LCD_CNTR,LCD_E      ; pull the LCD E-signal high
            STAA  LCD_DAT             ; send the upper 4 bits of data to LCD
            BCLR  LCD_CNTR,LCD_E      ; pull the LCD E-signal low to complete the write oper.
            
            LSLA                      ; match the lower 4 bits with the LCD data pins
            LSLA                      ;  -""-
            LSLA                      ;  -""-
            LSLA                      ;  -""-
                  
            BSET  LCD_CNTR,LCD_E      ; pull the LCD E signal high
            STAA  LCD_DAT             ; send the lower 4 bits of data to LCD
            BCLR  LCD_CNTR,LCD_E      ; pull the LCD E-signal low to complete the write oper.
            
            LDY   #1               	  ; adding this delay will complete the internal
            JSR   del_50us            ; operation for most instructions
            RTS

ENABLE_TOF  LDAA    #%10000000
            STAA    TSCR1          ; Enable TCNT
            STAA    TFLG2          ; Clear TOF
            LDAA    #%10000100     ; Enable TOI and select prescale factor equal to 16
            STAA    TSCR2
            RTS

TOF_ISR     INC     TOF_COUNTER     ; increment the overflow counter
            LDAA    #%10000000      ; ... Clear the TOF flag
            STAA    TFLG2           ; by setting(!) bit 7
            RTI

;*****************************************************************
;* Motor subroutines                                             *
;*****************************************************************
STARON  		LDAA    PTT
	    			ORAA    #%00100000	;PT5=1 (ON)
	    			STAA    PTT
	    			RTS
				
STAROFF	  	LDAA    PTT
	    			ANDA    #%11011111	;PT5=0 (OFF)
	    			STAA    PTT
	    			RTS
				
STARFWD	  	LDAA    PORTA
		    		ANDA    #%11111101	;PA1=0 (fwd)
    				STAA    PORTA
	    			RTS
   
STARREV 		LDAA    PORTA
	    			ORAA    #%00000010	;PA1=1 (rev)
    				STAA    PORTA
	    			RTS

PORTON  		LDAA    PTT
		    		ORAA    #%00010000	;PT4=1 (ON)
	     			STAA    PTT
	    			RTS
				
PORTOFF	  	LDAA    PTT
		    		ANDA    #%11101111	;PT4=0 (OFF)
		    		STAA    PTT
		    		RTS			
				
PORTFWD	  	LDAA    PORTA
		    		ANDA    #%11111110	;PA0=0 (fwd)
	    			STAA    PORTA
	    			RTS
				
PORTREV 		LDAA    PORTA
	    			ORAA    #%00000001	;PA0=1 (rev)
	    			STAA    PORTA			;PTH
		    		RTS
		    						
;*********************************************************************
;*                 Interrupt Vectors                                 *
;*********************************************************************
            ORG   $FFFE
            DC.W  Entry               ; Reset Vector
            
            ORG   $FFDE
            DC.W  TOF_ISR
