;*****************************************************************
;* This stationery serves as the framework for a              *
;* user application (single file, absolute assembly application) *
;* For a more comprehensive program that                      *
;* demonstrates the more advanced functionality of this          *
;* processor, please see the demonstration applications          *
;* located in the examples subdirectory of the                *
;* Freescale CodeWarrior for the HC12 Program directory          *
;*****************************************************************

; export symbols
                XDEF Entry, _Startup            ; export 'Entry' symbol
                ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
                INCLUDE 'derivative.inc' 

; LCD DEFINITIONS
;     ADDRESSES
LCD_CNTR        EQU PTJ  ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT         EQU PORTB ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E           EQU $80  ; LCD E-signal pin
LCD_RS          EQU $40  ; LCD RS-signal pin

;     COMMANDS
CLEAR_HOME      EQU $01  ; Clear the display and home the cursor
INTERFACE       EQU $28  ; 8 bit interface, two line display
CURSOR_OFF      EQU $0C  ; Display on, cursor off
SHIFT_OFF       EQU $06  ; Address increments, no character shift
LCD_SEC_LINE    EQU 64  ; Starting addr. of 2nd line of LCD (note decimal value!)

;     CHARACTERS
NULL            EQU 00  ; The string 'null terminator'
CR              EQU $0D  ; 'Carriage Return' character
SPACE           EQU ' '  ; The 'space' character


; STATE MACHINE DEFINITIONS
;     SYSTEM STATES
START           EQU     0
FWD             EQU     1
REV             EQU     2
ALL_STP         EQU     3
L_TURN          EQU     4
R_TURN          EQU     5

;     DIRECTIONS
NORTH           EQU     0
EAST            EQU     1
WEST            EQU     2
SOUTH           EQU     3

;     MOTOR INITIALIZATIONS
FWD_INT         EQU     69
REV_INT         EQU     69
FWD_TRN_INT     EQU     46
REV_TRN_INT     EQU     46


; SENSOR THRESHOLDS
TH_LINE_LEFT        EQU     $90
TH_LINE_RIGHT       EQU     $70
TH_MIDDLE       EQU     $B0



; variable/data section
             ORG $3800
             
TOF_COUNTER     dc.b    0
CRNT_STATE      dc.b    3
CRNT_DIR        dc.b    1
T_FWD           ds.b    1
T_REV           ds.b    1
T_FWD_TRN       ds.b    1
T_REV_TRN       ds.b    1


;---------------------------------------------------------------------------
; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)
SENSOR_LINE     FCB $01  ; Storage for guider sensor readings
SENSOR_BOW      FCB $23  ; Initialized to test values
SENSOR_PORT     FCB $45
SENSOR_MID      FCB $67
SENSOR_STBD     FCB $89

SENSOR_NUM      RMB 1   ; The currently selected sensor

TOP_LINE        RMB 20  ; Top line of display
                FCB NULL  ;  terminated by null
        
BOT_LINE        RMB 20  ; Bottom line of display
                FCB NULL  ;  terminated by null
        
CLEAR_LINE      FCC '                    '
                FCB NULL  ;  terminated by null

TEMP            RMB 1   ; Temporary location

BUMPER_BOW      RMB 1
BUMPER_STERN    RMB 1


; code section
                ORG $4000 ; Start of program text (FLASH memory)

Entry:
_Startup:
                
                LDS #$4000  ; Initialize the stack pointer
                CLI    ; Enable interrupts

                JSR     INIT   ; Initialize ports
                JSR     openADC  ; Initialize the ATD
                JSR     openLCD  ; Initialize the LCD
                JSR     CLR_LCD_BUF ; Write 'space' characters to the LCD buffer
                JSR     ENABLE_TOF

;---------------------------------------------------------------------------
;               Display Sensors

MAIN            LDAA  PTT
                EORA  #$40
                STAA  PTT
                JSR   G_LEDS_ON   ; Enable the guider LEDs
                JSR   READ_SENSORS ; Read the 5 guider sensors
                JSR   G_LEDS_OFF  ; Disable the guider LEDs
                JSR   READ_BUMPERS
                JSR   DISPLAY_SENSORS ; and write them to the LCD
                
                LDAA    CRNT_STATE
                JSR     DISPATCHER
                
                LDY #3000     ; 300 ms delay to avoid
                JSR del_50us    ;  display artifacts
                BRA MAIN            ; Loop forever


msg1          dc.b    "Battery volt ",0
msg2          dc.b    "State ",0
tab           dc.b    "START  ",0
              dc.b    "FWD    ",0                                          
              dc.b    "REV    ",0
              dc.b    "ALL_STP",0
              dc.b    "FWD_TRN",0
              dc.b    "REV_TRN",0
              
READ_BUMPERS    BRCLR   PORTAD0,$04,bowON
                LDAA    #$31 ;1
                BRA     bowOFF
bowON           LDAA    #$30  ;0
bowOFF          STAA     BUMPER_BOW

                BRCLR   PORTAD0,$08,sternON
                LDAA    #$31
                BRA     sternOFF
sternON         LDAA    #$30
sternOFF        STAA     BUMPER_STERN
                RTS
; subrotine section
;---------------------------------------------------------------------------
DISPATCHER    CMPA    #START
              BNE     NOT_START
              JSR     START_ST
              BRA     DISP_EXIT
              
NOT_START     CMPA    #ALL_STP
              BNE     NOT_STP
              JSR     ALL_STP_ST
              BRA     DISP_EXIT

NOT_STP       CMPA    #FWD
              BNE     NOT_FWD
              JSR     FWD_ST
              BRA     DISP_EXIT
              
NOT_FWD       CMPA    #REV
              BNE     NOT_REV
              NOP;JSR     REV_ST
              BRA     DISP_EXIT 

NOT_REV       CMPA    #L_TURN
              BNE     NOT_L_TURN
              JSR     L_TURN_ST
              BRA     DISP_EXIT           

NOT_L_TURN    CMPA    #R_TURN
              BNE     NOT_R_TRN
              JSR     R_TURN_ST
              BRA     DISP_EXIT
              
NOT_R_TRN   SWI
DISP_EXIT     RTS



***********************************************************
START_ST      BRCLR   PORTAD0,$04,NO_FWD
              JSR     INIT_FWD
              MOVB    #FWD,CRNT_STATE
              BRA     START_EXIT

NO_FWD        NOP
START_EXIT    RTS
***********************************************************
FWD_ST        BSET    PTT,%00110000
              BRSET   PORTAD0,$08,NO_ALL_STP  ; check rear bump
              JSR     INIT_ALL_STP
              MOVB    #ALL_STP,CRNT_STATE
              BRA     FWD_EXIT


NO_ALL_STP    BRSET   PORTAD0,$04,NO_FWD_TRN  ; check front bump
              JSR     INIT_REV
              MOVB    #REV,CRNT_STATE
              BRA     FWD_EXIT

NO_FWD_TRN    LDAA    TH_MIDDLE
              CMPA    SENSOR_MID
              BGT     FWD_EXIT

              LDAA    TH_LINE_LEFT
              CMPA    SENSOR_LINE
              BGT     INIT_R_TURN       ; check if above threshold
              
              
              LDAA    TH_LINE_RIGHT
              CMPA    SENSOR_LINE
              BLO     INIT_L_TURN       ; check if less than threshold
              RTS

INIT_L_TURN   MOVB    #L_TURN,CRNT_STATE
              RTS
              
INIT_R_TURN   MOVB    #R_TURN,CRNT_STATE
              RTS


R_TURN_ST     BCLR    PTT,%00100000
              MOVB    #FWD,CRNT_STATE
    
              RTS
L_TURN_ST     BCLR    PTT,%00010000
              MOVB    #FWD,CRNT_STATE
              RTS


  
NO_FWD_TMR    NOP              
FWD_EXIT      RTS

LINE_TH_CHECK 

***********************************************************
ALL_STP_ST    BRSET   PORTAD0,$04,NO_START
              JSR     INIT_ALL_STP
              MOVB    #START,CRNT_STATE
              BRA     ALL_STP_EXIT
              
NO_START      NOP
ALL_STP_EXIT  RTS
***********************************************************                                                        

***********************************************************
INIT_FWD      BCLR    PORTA,%00000011
              BSET    PTT,%00110000
              LDAA    TOF_COUNTER
              ADDA    #FWD_INT
              STAA    T_FWD
              RTS
***********************************************************
INIT_REV      BSET    PORTA,%00000011
              BSET    PTT,%00110000
              LDAA    TOF_COUNTER
              ADDA    #REV_INT
              STAA    T_REV
              RTS
***********************************************************
INIT_ALL_STP  BCLR    PTT,%00110000
              RTS
***********************************************************              
INIT_FWD_TRN  BSET    PORTA,%00000010
              LDAA    TOF_COUNTER 
              ADDA    #FWD_TRN_INT
              STAA    T_FWD_TRN
              RTS
***********************************************************              
INIT_REV_TRN  BCLR    PORTA,%00000010
              LDAA    TOF_COUNTER 
              ADDA    #REV_TRN_INT
              STAA    T_REV_TRN
              RTS              
***********************************************************              
ENABLE_TOF    LDAA    #%10000000
              STAA    TSCR1
              STAA    TFLG2
              LDAA    #%10000100
              STAA    TSCR2
              RTS
              
TOF_ISR       INC     TOF_COUNTER
              LDAA    #%10000000
              STAA    TFLG2
              RTI
              
DISABLE_TOF   LDAA    #%00000100
              STAA    TSCR2
              RTS







;               Initialize ports

INIT            BCLR DDRAD,$FF; Make PORTAD an input (DDRAD @ $0272)
                BSET DDRA,$FF                                    ; Make PORTA an output (DDRA @ $0002)
                
                BSET DDRB,%11110000                                   ; Make PORTB an output (DDRB @ $0003)
                BSET DDRJ,%11000000                                    ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                BSET DDRT,%01110000
                BSET ATDDIEN,$0C
                
                RTS

;---------------------------------------------------------------------------
;               Initialize the ADC

openADC         MOVB #$80,ATDCTL2; Turn on ADC (ATDCTL2 @ $0082)
                LDY #1; Waitfor50usforADCtobeready
                JSR  del_50us          ; -"-
                MOVB #$20,ATDCTL3      ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                MOVB #$97,ATDCTL4      ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                RTS

;---------------------------------------------------------------------------
;               Clear LCD Buffer
; This routine writes 'space' characters (ascii 20) into the LCD display
;  buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
;  display routine should maintain the buffer properly.

CLR_LCD_BUF     LDX #CLEAR_LINE
                LDY #TOP_LINE
                JSR STRCPY

CLB_SECOND      LDX #CLEAR_LINE
                LDY #BOT_LINE
                JSR STRCPY

CLB_EXIT        RTS

;---------------------------------------------------------------------------
;               String Copy
; Copies a null-terminated string (including the null) from one location to
; another
; Passed: X contains starting address of null-terminated string
;   Y contains first address of destination

STRCPY          PSHX      ; Protect the registers used
                PSHY      
                PSHA      

STRCPY_LOOP     LDAA 0,X           ; Get a source character
                STAA 0,Y         ; Copy it to the destination
                BEQ  STRCPY_EXIT  ; If it was the null, then exit
                INX              ; Else increment the pointers
                INY 
                BRA  STRCPY_LOOP  ; and do it again

STRCPY_EXIT     PULA              ; Restore the registers
                PULY
                PULX
                RTS

;---------------------------------------------------------------------------
;               Guider LEDs ON
; This routine enables the guider LEDs so that readings of the sensor
;  correspond to the 'illuminated' situation.
; Passed:  Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed

G_LEDS_ON       BSET PORTA,%00100000 ; Set bit 5
                RTS
; 
; Guider LEDs OFF

; This routine disables the guider LEDs. Readings of the sensor
;  correspond to the 'ambient lighting' situation.

; Passed:  Nothing
; Returns: Nothing
; Side:    PORTA bit 5 is changed

G_LEDS_OFF      BCLR PORTA,%00100000 ; Clear bit 5
                RTS

;---------------------------------------------------------------------------
;               Read Sensors
;
; This routine reads the eebot guider sensors and puts the results in RAM
; registers.
; Note: Do not confuse the analog multiplexer on the Guider board with the
;  multiplexer in the HCS12. The guider board mux must be set to the
;  appropriate channel using the SELECT_SENSOR routine. The HCS12 always
;  reads the selected sensor on the HCS12 A/D channel AN1.
; The A/D conversion mode used in this routine is to read the A/D channel
;  AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
;  used in this routine is the value from AN1, read from ATDDR0. However,
;  other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.
; Passed: None
; Returns: Sensor readings in:
;          SENSOR_LINE (0) (Sensor E/F)
;         SENSOR_BOW  (1) (Sensor A)
;         SENSOR_PORT (2) (Sensor B)
;         SENSOR_MID  (3) (Sensor C)
;         SENSOR_STBD (4) (Sensor D)
; Note:
;   The sensor number is shown in brackets
;
; Algorithm:
;        Initialize the sensor number to 0


;        Initialize a pointer into the RAM at the start of the Sensor Array storage
; Loop Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
;        Repeat
;          Read ATDSTAT0
;       Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
;       Store the contents of ATDDR0L at the pointer
;       If the pointer is at the last entry in Sensor Array, then
;          Exit
;       Else
;          Increment the sensor number
;          Increment the pointer
;       Loop again.
 
READ_SENSORS    CLR  SENSOR_NUM      ; Select sensor number 0
                LDX  #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP    LDAA SENSOR_NUM      ; Select the correct sensor input
                JSR  SELECT_SENSOR   ;  on the hardware
                LDY  #400            ; 20 ms delay to allow the
                JSR  del_50us         ;  sensor to stabilize

                LDAA #%10000001      ; Start A/D conversion on AN1
                STAA ATDCTL5
                BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done

                LDAA ATDDR0L         ; A/D conversion is complete in ATDDR0L
                STAA 0,X            ;  so copy it to the sensor register
                CPX  #SENSOR_STBD   ; If this is the last reading
                BEQ  RS_EXIT         ; Then exit
 
                INC  SENSOR_NUM      ; Else, increment the sensor number
                INX               ;  and the pointer into the sensor array
                BRA  RS_MAIN_LOOP    ;  and do it again
 
RS_EXIT         RTS
 

;---------------------------------------------------------------------------
;               Select Sensor
; This routine selects the sensor number passed in ACCA. The motor direction
;  bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
;  same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
;  which selects the guider sensor to be connected to AN1.
; Passed: Sensor Number in ACCA
; Returns: Nothing
; Side Effects: ACCA is changed
; Algorithm:
; First, copy the contents of PORTA into a temporary location TEMP and clear
;        the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
;        11100011. The zeros in the mask clear the corresponding bits in the
;        TEMP. The 1's have no effect.
; Next, move the sensor selection number left two positions to align it
;        with the correct bit positions for sensor selection.
; Clear all the bits around the (shifted) sensor number by ANDing it with
;  the mask 00011100. The zeros in the mask clear everything except
;        the sensor number.
; Now we can combine the sensor number with the TEMP using logical OR.
;  The effect is that only bits 2,3,4 are changed in the TEMP, and these
;  bits now correspond to the sensor number.
; Finally, save the TEMP to the hardware.

SELECT_SENSOR   PSHA             ; Save the sensor number for the moment

                LDAA PORTA       ; Clear the sensor selection bits to zeros
                ANDA #%11100011   ;
                STAA TEMP       ; and save it into TEMP
 
                PULA             ; Get the sensor number
                ASLA             ; Shift the selection number left, twice
                ASLA             ;
                ANDA #%00011100   ; Clear irrelevant bit positions
 
                ORAA TEMP        ; OR it into the sensor bit positions
                STAA PORTA        ; Update the hardware
                RTS

;---------------------------------------------------------------------------
;               Display Sensor Readings
; Passed: Sensor values in RAM locations SENSOR_LINE through SENSOR_STBD.
; Returns: Nothing
; Side: Everything
; This routine writes the sensor values to the LCD. It uses the 'shadow buffer' approach.
;  The display buffer is built by the display controller routine and then copied in its
;  entirety to the actual LCD display. Although simpler approaches will work in this
;  application, we take that approach to make the code more re-useable.
; It's important that the display controller not write over other information on the
;  LCD, so writing the LCD has to be centralized with a controller routine like this one.
; In a more complex program with additional things to display on the LCD, this routine
;  would be extended to read other variables and place them on the LCD. It might even
;  read some 'display select' variable to determine what should be on the LCD.
; For the purposes of this routine, we'll put the sensor values on the LCD
;  in such a way that they (sort of) mimic the position of the sensors, so
;  the display looks like this:
;   01234567890123456789
;   ___FF_______________
;   PP_MM_SS_LL_________
; Where FF is the front sensor, PP is port, MM is mid, SS is starboard and
;  LL is the line sensor.
; The corresponding addresses in the LCD buffer are defined in the following
;  equates (In all cases, the display position is the MSDigit).

DP_FRONT_SENSOR EQU TOP_LINE+0
DP_MID_SENSOR   EQU TOP_LINE+3
DP_LINE_SENSOR  EQU TOP_LINE+6
DP_DIRECTION    EQU TOP_LINE+11
DP_STATE        EQU TOP_LINE+13


DP_PORT_SENSOR  EQU BOT_LINE+0
DP_STBD_SENSOR  EQU BOT_LINE+3
DP_BUMPERS       EQU BOT_LINE+9





DISPLAY_SENSORS LDAA SENSOR_BOW      ; Get the FRONT sensor value
                JSR  BIN2ASC
                LDX  #DP_FRONT_SENSOR ; Point to the LCD buffer position
                STD  0,X
 
                LDAA SENSOR_PORT
                JSR  BIN2ASC
                LDX  #DP_PORT_SENSOR
                STD  0,X
 
                LDAA SENSOR_MID
                JSR  BIN2ASC
                LDX  #DP_MID_SENSOR
                STD  0,X
 
                LDAA SENSOR_STBD
                JSR  BIN2ASC
                LDX  #DP_STBD_SENSOR
                STD  0,X
 
                LDAA SENSOR_LINE
                JSR  BIN2ASC
                LDX  #DP_LINE_SENSOR
                STD  0,X
  
                LDAA  BUMPER_BOW
                LDAB  BUMPER_STERN
                LDX   #DP_BUMPERS
                STD   0,X

                LDAA  CRNT_DIR
                JSR  BIN2ASC
                LDX   #DP_DIRECTION
                STD  0,X
                  
                LDAA  CRNT_STATE
                JSR  BIN2ASC
                LDX   #DP_STATE
                STD  0,X
   
                LDAA #CLEAR_HOME
                JSR  cmd2LCD
 
                LDY  #40
                JSR  del_50us
 
                LDX  #TOP_LINE
                JSR  putsLCD
 
                LDAA #LCD_SEC_LINE
                JSR  LCD_POS_CRSR
 
                LDX  #BOT_LINE
                JSR  putsLCD
                RTS
  
  ;---------------------------------------------------------------------------
;               Binary to ASCII

HEX_TABLE       FCC '0123456789ABCDEF'
                 
BIN2ASC         PSHA
                TAB
                ANDB #%00001111
                CLRA
                ADDD #HEX_TABLE
                XGDX
                LDAA 0,X
 
                PULB
                PSHA
                RORB
                RORB
                RORB
                RORB
                ANDB #%00001111
                CLRA
                ADDD #HEX_TABLE
                XGDX
                LDAA 0,X
                PULB
                RTS
  ;---------------------------------------------------------------------------
;     Routines to control the Liquid Crystal Display
;---------------------------------------------------------------------------
;               Initialize the LCD

*******************************************************************
* Initialization of the LCD: 4-bit data width, 2-line display, *
* turn on display, cursor and blinking off. Shift cursor right. *
*******************************************************************
openLCD       BSET  DDRB,%11110000 ; configure pins PS7,PS6,PS5,PS4 for output
              BSET  DDRJ,%11000000 ; configure pins PJ7,PJ6 for output
              LDY   #2000 ; wait for LCD to be ready
              JSR   del_50us ; -"-
              LDAA  #$28 ; set 4-bit data, 2-line display
              JSR   cmd2LCD ; -"-
              LDAA  #$0C ; display on, cursor off, blinking off
              JSR   cmd2LCD ; -"-
              LDAA  #$06 ; move cursor right after entering a character
              JSR   cmd2LCD ; -"-
              RTS
*******************************************************************
* Clear display and home cursor *
*******************************************************************
clrLCD        LDAA  #$01 ; clear cursor and return to home position
              JSR   cmd2LCD ; -"-
              LDY   #40 ; wait until "clear cursor" command is complete
              JSR   del_50us ; -"-
              RTS
;---------------------------------------------------------------------------
;               Send a command in accumulator A to the LCD
;
cmd2LCD         BCLR LCD_CNTR,LCD_RS   ; Select the LCD Instruction register
                JSR  dataMov           ; Send data to IR or DR of the LCD
                RTS
;---------------------------------------------------------------------------
;               Send a character in accumulator in A to LCD
putcLCD         BSET LCD_CNTR,LCD_RS   ; select the LCD Data register
                JSR  dataMov           ; send data to IR or DR of the LCD
                RTS
;---------------------------------------------------------------------------
; Send a NULL-terminated string pointed to by 

putsLCD         LDAA 1,X+
                BEQ  donePS
                JSR  putcLCD
                BRA  putsLCD
donePS          RTS
;---------------------------------------------------------------------------
;               Send data to the LCD IR or DR depending on the RS signal
dataMov       BSET  LCD_CNTR,LCD_E ; pull the LCD E-sigal high
              STAA  LCD_DAT ; send the upper 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
              LSLA ; match the lower 4 bits with the LCD data pins
              LSLA ; -"-
              LSLA ; -"-
              LSLA ; -"-
              BSET  LCD_CNTR,LCD_E ; pull the LCD E signal high
              STAA  LCD_DAT ; send the lower 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
              LDY   #1 ; adding this delay will complete the internal
              JSR   del_50us ; operation for most instructions
              RTS
;---------------------------------------------------------------------------
;               Position the Cursor

LCD_POS_CRSR    ORAA #%10000000        ; Set the high bit of the control word
                JSR  cmd2LCD           ;  and set the cursor address
                RTS
;---------------------------------------------------------------------------
; 50 Microsecond Delay
;

del_50us        PSHX
eloop           LDX #300
iloop           NOP
                DBNE  X,iloop
                DBNE  Y,eloop
                PULX
                RTS
 
;---------------------------------------------------------------------------
;               Interrupt Vectors
             ORG   $FFFE
             DC.W  Entry            ; Reset Vector


            ORG   $FFDE
            DC.W  TOF_ISR