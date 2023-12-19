;***********************************************************
; COE 538 eeBot Project - F2023                            *
; Members: Janice Zhu, Dalton Crowe                        *
;***********************************************************
; export symbols
                    XDEF Entry, _Startup                    ; export 'Entry' symbol
                    ABSENTRY Entry                          ; for absolute assembly: mark this as application entry point

; Include derivative-specific definitions
                    INCLUDE 'derivative.inc'

;***********************************************************
; DEFINITIONS                                              *
;***********************************************************
; LCD ADDRESSES
LCD_CNTR            EQU         PTJ                         ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT             EQU         PORTB                       ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E               EQU         $80                         ; LCD E-signal pin
LCD_RS              EQU         $40                         ; LCD RS-signal pin

; LCD COMMANDS
CLEAR_HOME          EQU         $01                         ; Clear the display and home the cursor
INTERFACE           EQU         $28                         ; 8 bit interface, two line display
CURSOR_OFF          EQU         $0C                         ; Display on, cursor off
SHIFT_OFF           EQU         $06                         ; Address increments, no character shift
LCD_SEC_LINE        EQU         64                          ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD TIMING
LCD_REFRESH         EQU         20

; LCD CHARACTERS
NULL                EQU         00                          ; The string 'null terminator'

; DELAY VALUES
DLY_FWD             EQU         2000
DLY_LEFT            EQU         2000
DLY_RIGHT           EQU         3250
DLY_REV             EQU         3500
DLY_180             EQU         18500
DLY_MAIN            EQU         250

; STATE MACHINE STATES
SS_START            EQU         0
SS_FWD              EQU         1
SS_REV              EQU         2
SS_STOP             EQU         3
SS_LEFT             EQU         4
SS_RIGHT            EQU         5
SS_180              EQU         6
SS_THINK            EQU         7
SS_INTER            EQU         8

; SENSOR THRESHOLDS
TH_LINE_LEFT        EQU         $CE                         ;$F0
TH_LINE_RIGHT       EQU         $A8                         ;$A0
TH_MIDDLE           EQU         $A0                         ;W$36 B$9C
TH_BOW              EQU         $A0                         ;W$54 B$CA
TH_PORT             EQU         $B0
TH_STAR             EQU         $60

;***********************************************************
; VARIABLES/DATA                                           *
;***********************************************************
                    ORG         $3800

STATE_CRNT          dc.b        3
DISP_REFRESH        dc.b        0
DEBUG_1             dc.b        $30
DEBUG_2             dc.b        $30

; FLAGS
PATH_PORT           dc.b        0
PATH_STAR           dc.b        0
INTERSEC_LAST       dc.b        0
COMPLETED_180       dc.b        0

; SENSORS AND BUMPERS
SENSOR_LINE         dc.b        $01                         ; Storage for guider sensor readings
SENSOR_BOW          dc.b        $23                         ; Initialized to test values
SENSOR_PORT         dc.b        $45
SENSOR_MID          dc.b        $67
SENSOR_STAR         dc.b        $89
SENSOR_NUM          ds.b        1                           ; The currently selected sensor
BUMPER_BOW          ds.b        1
BUMPER_STERN        ds.b        1

; LCD BUFFER
TOP_LINE            RMB         20                          ; Top line of display
                    FCB         NULL                        ;  terminated by null
BOT_LINE            RMB         20                          ; Bottom line of display
                    FCB         NULL                        ;  terminated by null
CLEAR_LINE          FCC         '                                           '
                    FCB         NULL                        ;  terminated by null
TEMP                RMB         1                           ; Temporary location

;***********************************************************
; MAIN CODE                                                *
;***********************************************************
                    ORG         $4000                       ; Start of program text (FLASH memory)

Entry:
_Startup:
                    LDS         #$4000                      ; Initialize the stack pointer
                    CLI                                     ; Enable interrupts

                    JSR         INIT                        ; Initialize ports
                    JSR         openADC                     ; Initialize the ATD
                    JSR         openLCD                     ; Initialize the LCD
                    JSR         CLR_LCD_BUF                 ; Write 'space' characters to the LCD buffer

MAIN                LDAA        PTT
                    EORA        #$40
                    STAA        PTT
                    JSR         G_LEDS_ON                   ; Enable the guider LEDs
                    JSR         READ_SENSORS                ; Read the 5 guider sensors
                    JSR         G_LEDS_OFF                  ; Disable the guider LEDs
                    JSR         READ_BUMPERS
                    LDAB        DISP_REFRESH
                    CMPB        #LCD_REFRESH
                    BEQ         MAIN_CONT
                    JSR         DISPLAY_SENSORS             ; and write them to the LCD
                    LDAB        #0
                    STAB        DISP_REFRESH

MAIN_CONT           LDAA        STATE_CRNT
                    JSR         DISPATCHER
                    LDAB        DISP_REFRESH
                    INCB
                    STAB        DISP_REFRESH
                    LDY         #DLY_MAIN                   ; 150 ms delay to avoid 6000 = 300ms
                    JSR         del_50us                    ;  display artifacts
                    BRA         MAIN                        ; Loop forever

READ_BUMPERS        BRCLR       PORTAD0,$04,bowON
                    LDAA        #$31
                    BRA         bowOFF
bowON               LDAA        #$30
bowOFF              STAA        BUMPER_BOW

                    BRCLR       PORTAD0,$08,sternON
                    LDAA        #$31
                    BRA         sternOFF
sternON             LDAA        #$30
sternOFF            STAA        BUMPER_STERN
                    RTS
;***********************************************************
; SUBROUTINES - STATE MACHINE                              *
;***********************************************************
; Dispatcher
DISPATCHER          CMPA        #SS_START                   ; Start state
                    BNE         NOT_SS_START
                    JSR         STATE_START
                    BRA         DISPATCHER_EXIT

NOT_SS_START        CMPA        #SS_STOP                    ; Stop state
                    BNE         NOT_SS_STOP
                    JSR         STATE_STOP
                    BRA         DISPATCHER_EXIT

NOT_SS_STOP         CMPA        #SS_FWD                     ; Forward state
                    BNE         NOT_SS_FWD
                    JSR         STATE_FWD
                    BRA         DISPATCHER_EXIT

NOT_SS_FWD          CMPA        #SS_REV                     ; Reverse state
                    BNE         NOT_SS_REV
                    JSR         STATE_REV
                    BRA         DISPATCHER_EXIT

NOT_SS_REV          CMPA        #SS_LEFT                    ; Left turn state
                    BNE         NOT_SS_LEFT
                    JSR         STATE_LEFT
                    BRA         DISPATCHER_EXIT

NOT_SS_LEFT         CMPA        #SS_RIGHT                   ; Right turn state
                    BNE         NOT_SS_RIGHT
                    JSR         STATE_RIGHT
                    BRA         DISPATCHER_EXIT

NOT_SS_RIGHT        CMPA        #SS_180                     ; Right turn state
                    BNE         NOT_SS_180
                    JSR         STATE_180
                    BRA         DISPATCHER_EXIT

NOT_SS_180          CMPA        #SS_THINK                   ; Right turn state
                    BNE         NOT_SS_THINK
                    JSR         STATE_THINK
                    BRA         DISPATCHER_EXIT

NOT_SS_THINK        CMPA        #SS_INTER                   ; intersection state
                    BNE         NOT_SS_INTERSECTION
                    JSR         STATE_INTERSECTION
                    BRA         DISPATCHER_EXIT

NOT_SS_INTERSECTION SWI
DISPATCHER_EXIT     RTS

; Stop State
STATE_STOP          BCLR        PTT,%00110000
                    BRSET       PORTAD0,$04,NOT_START       ; Check if front bumper is pushed
                    JSR         INIT_SS_STOP                ; if so, initialize stop
                    MOVB        #SS_START,STATE_CRNT        ; set current tate to start
                    BRA         STOP_EXIT                   ; exit

NOT_START           NOP
STOP_EXIT           RTS

INIT_SS_STOP        BCLR        PTT,%00110000
                    RTS

; Think State
STATE_THINK         BCLR        PTT,%00110000
                    LDAA        PATH_PORT                   ; no left path found
                    CMPA        #$01
                    BNE         CHK_STAR_INT
                    MOVB        #SS_INTER,STATE_CRNT
                    RTS

CHK_STAR_INT        LDAA        PATH_STAR                   ; no right path found
                    CMPA        #$01
                    BNE         GO_FWD
                    MOVB        #SS_INTER,STATE_CRNT
                    RTS

GO_FWD              MOVB        #SS_FWD,STATE_CRNT
                    RTS

INIT_SS_THINK       BCLR        PTT,%00110000
                    RTS

; Start State
STATE_START         BCLR        PTT,%00110000
                    BRCLR       PORTAD0,$08,NO_FWD          ; Check if rear bumper is released
                    MOVB        #SS_FWD,STATE_CRNT          ; Set current state to forward
                    BRA         START_EXIT

NO_FWD              NOP
START_EXIT          RTS

; Forward State
STATE_FWD           BCLR        PTT,%00110000
                    BRSET       PORTAD0,$08,NO_STOP         ; Check if the rear bumper is triggered
                    JSR         INIT_SS_STOP                ; Initialize the all stop state
                    MOVB        #SS_STOP,STATE_CRNT         ; Set the current state to all stop
                    BRA         FWD_EXIT                    ; Return

NO_STOP             BRSET       PORTAD0,$04,NO_REV          ; Check if the front bumper is triggered
                    MOVB        #SS_REV,STATE_CRNT          ; Set the current state to reverse
                    BRA         FWD_EXIT                    ; Return

NO_REV              LDAA        #TH_PORT                    ; Check if the left sensor has found a line
                    CMPA        SENSOR_PORT
                    BHI         PORT_NOT_BLK                ; If not, then branch
                    BSET        PATH_PORT,#$01              ; Otherwise mark it on the map
                    MOVB        #$31,DEBUG_1
                    MOVB        #SS_THINK,STATE_CRNT
                    BRA         FWD_EXIT

PORT_NOT_BLK        LDAA        #TH_STAR                    ; Check if the right sensor has found a line
                    CMPA        SENSOR_STAR                 
                    BHI         STAR_NOT_BLK                ; If not, then branch
                    BSET        PATH_STAR,#$01              ; Otherwise mark it on the map
                    MOVB        #$31,DEBUG_2
                    MOVB        #SS_THINK,STATE_CRNT
                    BRA         FWD_EXIT

STAR_NOT_BLK        LDAA        #TH_BOW                     ; Check if the front is on a line
                    CMPA        SENSOR_BOW
                    BLO         BOW_IS_BLK                  ; If it is, branch

                    LDAA        #TH_LINE_LEFT               ; Check if the line follower is left of the line
                    CMPA        SENSOR_LINE
                    BLO         INIT_SS_RIGHT               ; if threshold is greater than sensor, start a right turn

                    LDAA        #TH_LINE_RIGHT              ; Check if the line follower is right of the line
                    CMPA        SENSOR_LINE
                    BHI         INIT_SS_LEFT                ; if threshold is less than sensor, start a left turn

BOW_IS_BLK          MOVB        #SS_THINK,STATE_CRNT
                    BRA         INIT_SS_FWD                 ; On the line

FWD_EXIT            RTS

INIT_SS_FWD         BCLR        PORTA,%00000011             ; Set both motor directions to forward
                    BSET        PTT,%00110000               ; Turn on the drive motors
                    LDY         #DLY_FWD
                    JSR         del_50us
                    BCLR        PTT,%00110000               ; Turn off drive motors
                    RTS

; Left State
STATE_LEFT          BCLR        PORTA,%00000011
                    BSET        PTT,%00100000
                    BCLR        PTT,%00010000
                    LDY         #DLY_LEFT
                    JSR         del_50us
                    BCLR        PTT,%00110000
                    MOVB        #SS_THINK,STATE_CRNT
                    RTS

INIT_SS_LEFT        MOVB        #SS_LEFT,STATE_CRNT
                    RTS
					
; Right State
STATE_RIGHT         BCLR        PORTA,%00000011
                    BSET        PTT,%00010000
                    BCLR        PTT,%00100000
                    LDY         #DLY_RIGHT
                    JSR         del_50us
                    BCLR        PTT,%00110000
                    MOVB        #SS_THINK,STATE_CRNT
                    RTS

INIT_SS_RIGHT       MOVB        #SS_RIGHT,STATE_CRNT
                    RTS

; Intersection State
STATE_INTERSECTION  BCLR        PTT,%00110000
                    LDAA        COMPLETED_180
                    CMPA        #$00
                    BEQ         CHK_PORT
                    LDAA        INTERSEC_LAST
                    CMPA        #$00
                    BEQ         RMV_STAR

RMV_PORT            BCLR        PATH_PORT,#$01
                    BRA         CHK_PORT
RMV_STAR            BCLR        PATH_STAR,#$01

CHK_PORT            LDAA        PATH_PORT
                    CMPA        #$01
                    BNE         CHK_IF_STAR
                    LDAA        #TH_PORT  ;a0
                    CMPA        SENSOR_PORT
                    BHI         CHK_BOW                     ; if not on line
                    MOVB        #SS_LEFT,STATE_CRNT
                    BCLR        INTERSEC_LAST,#$01
                    MOVB        #$30,DEBUG_2
                    RTS
CHK_BOW             LDAA        #TH_BOW ;a0
                    CMPA        SENSOR_BOW
                    BLO         INTERSECT_DONE              ; if not on line
                    MOVB        #SS_LEFT,STATE_CRNT
                    BCLR        INTERSEC_LAST,#$01
                    RTS
CHK_IF_STAR         LDAA        PATH_STAR
                    CMPA        #$01
                    BNE         INTER_EXIT
                    LDAA        #TH_STAR
                    CMPA        SENSOR_STAR
                    BHI         CHK_BOW2
                    MOVB        #SS_RIGHT,STATE_CRNT
                    BSET        INTERSEC_LAST,#$01
                    MOVB        #$31,DEBUG_2
                    RTS

CHK_BOW2            LDAA        #TH_BOW
                    CMPA        SENSOR_BOW
                    BLO         INTERSECT_DONE              ; if not on line
                    MOVB        #SS_RIGHT,STATE_CRNT
                    BSET        INTERSEC_LAST,#$01
                    RTS
INTERSECT_DONE      BCLR        PATH_PORT,#$01
                    BCLR        PATH_STAR,#$01
                    MOVB        #$30,DEBUG_1
                    MOVB        #$30,DEBUG_2
                    MOVB        #SS_THINK,STATE_CRNT
                    BCLR        COMPLETED_180,#$01
                    RTS
INTER_EXIT          MOVB        #SS_THINK,STATE_CRNT
                    RTS

; Reverse State
STATE_REV           BSET        PORTA,%00000011             ; Set both motor directions to reverse
                    BSET        PTT,%00110000               ; Turn on the drive motors
                    LDY         #DLY_REV
                    JSR         del_50us
                    BCLR        PTT,%00110000               ; Turn off the drive motors
                    MOVB        #SS_180,STATE_CRNT
                    BRA         REV_EXIT                    ; Return

REV_EXIT            RTS

INIT_SS_REV         BSET        PORTA,%00000011             ; Set both motor directions to reverse
                    BSET        PTT,%00110000               ; Turn on the drive motors
                    RTS

; 180 Degree Turn State
STATE_180           BCLR        PORTA,%00000001             ; Set both motor directions to reverse
                    BSET        PTT,%00110000               ; Turn on the drive motors
                    LDY         #DLY_180                    
                    JSR         del_50us
                    BCLR        PTT,%00110000               ; Turn off the drive motors
                    MOVB        #SS_THINK,STATE_CRNT
                    BRA         SPIN_EXIT                   ; Return

SPIN_EXIT           BSET        COMPLETED_180,#$01
                    MOVB        #$31,DEBUG_1
                    RTS

INIT_SS_180         BCLR        PORTA,%00000010             ; Set the right motor direction to forward
                    RTS

;***********************************************************
; SUBROUTINES - SYSTEM                                     *
;***********************************************************
; Initialization
INIT                BCLR        DDRAD,$FF                   ; Make PORTAD an input (DDRAD @ $0272)
                    BSET        DDRA,$FF                    ; Make PORTA an output (DDRA @ $0002)
                    BSET        DDRB,%11110000              ; Make PORTB an output (DDRB @ $0003)
                    BSET        DDRJ,%11000000              ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                    BSET        DDRT,%01110000
                    BSET        ATDDIEN,$0C
                    RTS

; Software Delay
del_50us            PSHX
eloop               LDX         #300
iloop               NOP
                    DBNE        X,iloop
                    DBNE        Y,eloop
                    PULX
                    RTS

;***********************************************************
; SUBROUTINES - ADC                                        *
;***********************************************************
; Open ADC
openADC             MOVB        #$80,ATDCTL2                ; Turn on ADC (ATDCTL2 @ $0082)
                    LDY         #1                          ; Waitfor50usforADCtobeready
                    JSR         del_50us                    ; -"-
                    MOVB        #$20,ATDCTL3                ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                    MOVB        #$97,ATDCTL4                ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                    RTS

;***********************************************************
; SUBROUTINES - LCD                                        *
;***********************************************************
; Clear Buffer
CLR_LCD_BUF         LDX         #CLEAR_LINE
                    LDY         #TOP_LINE
                    JSR         STRCPY

CLB_SECOND          LDX         #CLEAR_LINE
                    LDY         #BOT_LINE
                    JSR         STRCPY

CLB_EXIT            RTS

; Copy String
STRCPY              PSHX                                    ; Protect the registers used
                    PSHY
                    PSHA

STRCPY_LOOP         LDAA        0,X                         ; Get a source character
                    STAA        0,Y                         ; Copy it to the destination
                    BEQ         STRCPY_EXIT                 ; If it was the null, then exit
                    INX                                     ; Else increment the pointers
                    INY
                    BRA         STRCPY_LOOP                 ; and do it again

STRCPY_EXIT         PULA                                    ; Restore the registers
                    PULY
                    PULX
                    RTS

; LCD Position Definitions
DP_FRONT_SENSOR     EQU         TOP_LINE+0
DP_MID_SENSOR       EQU         TOP_LINE+3
DP_LINE_SENSOR      EQU         TOP_LINE+6
DP_STATE            EQU         TOP_LINE+13

DP_PORT_SENSOR      EQU         BOT_LINE+0
DP_STBD_SENSOR      EQU         BOT_LINE+3
DP_BUMPERS          EQU         BOT_LINE+9
DP_DEBUG            EQU         BOT_LINE+13

; Display Sensors
DISPLAY_SENSORS     LDAA        SENSOR_BOW                  ; Get the FRONT sensor value
                    JSR         BIN2ASC
                    LDX         #DP_FRONT_SENSOR            ; Point to the LCD buffer position
                    STD         0,X

                    LDAA        SENSOR_PORT
                    JSR         BIN2ASC
                    LDX         #DP_PORT_SENSOR
                    STD         0,X

                    LDAA        SENSOR_MID
                    JSR         BIN2ASC
                    LDX         #DP_MID_SENSOR
                    STD         0,X

                    LDAA        SENSOR_STAR
                    JSR         BIN2ASC
                    LDX         #DP_STBD_SENSOR
                    STD         0,X

                    LDAA        SENSOR_LINE
                    JSR         BIN2ASC
                    LDX         #DP_LINE_SENSOR
                    STD         0,X

                    LDAA        BUMPER_BOW
                    LDAB        BUMPER_STERN
                    LDX         #DP_BUMPERS
                    STD         0,X

                    LDAA        DEBUG_1
                    LDAB        DEBUG_2
                    LDX         #DP_DEBUG
                    STD         0,X

                    LDAA        STATE_CRNT
                    JSR         BIN2ASC
                    LDX         #DP_STATE
                    STD         0,X

                    LDAA        #CLEAR_HOME
                    JSR         cmd2LCD

                    LDY         #40
                    JSR         del_50us

                    LDX         #TOP_LINE
                    JSR         putsLCD

                    LDAA        #LCD_SEC_LINE
                    JSR         LCD_POS_CRSR

                    LDX         #BOT_LINE
                    JSR         putsLCD
                    RTS

; Initialize
openLCD             BSET        DDRB,%11110000              ; configure pins PS7,PS6,PS5,PS4 for output
                    BSET        DDRJ,%11000000              ; configure pins PJ7,PJ6 for output
                    LDY         #2000                       ; wait for LCD to be ready
                    JSR         del_50us                    ; -"-
                    LDAA        #INTERFACE                  ; set 4-bit data, 2-line display
                    JSR         cmd2LCD                     ; -"-
                    LDAA        #CURSOR_OFF                 ; display on, cursor off, blinking off
                    JSR         cmd2LCD                     ; -"-
                    LDAA        #SHIFT_OFF                  ; move cursor right after entering a character
                    JSR         cmd2LCD                     ; -"-
                    RTS

; Clear LCD
clrLCD              LDAA        #$01                        ; clear cursor and return to home position
                    JSR         cmd2LCD                     ; -"-
                    LDY         #40                         ; wait until "clear cursor" command is complete
                    JSR         del_50us                    ; -"-
                    RTS

; Send a command
cmd2LCD             BCLR        LCD_CNTR,LCD_RS             ; Select the LCD Instruction register
                    JSR         dataMov                     ; Send data to IR or DR of the LCD
                    RTS

; Print a character
putcLCD             BSET        LCD_CNTR,LCD_RS             ; select the LCD Data register
                    JSR         dataMov                     ; send data to IR or DR of the LCD
                    RTS

; Print a string
putsLCD             LDAA        1,X+
                    BEQ         donePS
                    JSR         putcLCD
                    BRA         putsLCD
donePS              RTS

; Send Data
dataMov             BSET        LCD_CNTR,LCD_E              ; pull the LCD E-sigal high
                    STAA        LCD_DAT                     ; send the upper 4 bits of data to LCD
                    BCLR        LCD_CNTR,LCD_E              ; pull the LCD E-signal low to complete the write oper.
                    LSLA                                    ; match the lower 4 bits with the LCD data pins
                    LSLA                                    ; -"-
                    LSLA                                    ; -"-
                    LSLA                                    ; -"-
                    BSET        LCD_CNTR,LCD_E              ; pull the LCD E signal high
                    STAA        LCD_DAT                     ; send the lower 4 bits of data to LCD
                    BCLR        LCD_CNTR,LCD_E              ; pull the LCD E-signal low to complete the write oper.
                    LDY         #1                          ; adding this delay will complete the internal
                    JSR         del_50us                    ; operation for most instructions
                    RTS

; Set Cursor Position
LCD_POS_CRSR        ORAA        #%10000000                  ; Set the high bit of the control word
                    JSR         cmd2LCD                     ;  and set the cursor address
                    RTS

;***********************************************************
; SUBROUTINES - GUIDER                                     *
;***********************************************************
; LED's On
G_LEDS_ON           BSET        PORTA,%00100000             ; Set bit 5
                    RTS

; LED's Off
G_LEDS_OFF          BCLR        PORTA,%00100000             ; Clear bit 5
                    RTS

; Select Sensor
SELECT_SENSOR       PSHA                                    ; Save the sensor number for the moment

                    LDAA        PORTA                       ; Clear the sensor selection bits to zeros
                    ANDA        #%11100011
                    STAA        TEMP                        ; and save it into TEMP

                    PULA                                    ; Get the sensor number
                    ASLA                                    ; Shift the selection number left, twice
                    ASLA
                    ANDA        #%00011100                  ; Clear irrelevant bit positions

                    ORAA        TEMP                        ; OR it into the sensor bit positions
                    STAA        PORTA                       ; Update the hardware
                    RTS
					
; Read Sensors
READ_SENSORS        CLR         SENSOR_NUM                  ; Select sensor number 0
                    LDX         #SENSOR_LINE                ; Point at the start of the sensor array

RS_MAIN_LOOP        LDAA        SENSOR_NUM                  ; Select the correct sensor input
                    JSR         SELECT_SENSOR               ;  on the hardware
                    LDY         #400                        ; 20 ms delay to allow the
                    JSR         del_50us                    ;  sensor to stabilize

                    LDAA        #%10000001                  ; Start A/D conversion on AN1
                    STAA        ATDCTL5
                    BRCLR        ATDSTAT0,$80,*             ; Repeat until A/D signals done

                    LDAA        ATDDR0L                     ; A/D conversion is complete in ATDDR0L
                    STAA        0,X                         ;  so copy it to the sensor register
                    CPX         #SENSOR_STAR                ; If this is the last reading
                    BEQ         RS_EXIT                     ; Then exit

                    INC         SENSOR_NUM                  ; Else, increment the sensor number
                    INX                                     ;  and the pointer into the sensor array
                    BRA         RS_MAIN_LOOP                ;  and do it again

RS_EXIT             RTS

;***********************************************************
; SUBROUTINES - Converters                                 *
;***********************************************************
; Binary to ASCII
HEX_TABLE           FCC         '0123456789ABCDEF'

BIN2ASC             PSHA
                    TAB
                    ANDB        #%00001111
                    CLRA
                    ADDD        #HEX_TABLE
                    XGDX
                    LDAA        0,X

                    PULB
                    PSHA
                    RORB
                    RORB
                    RORB
                    RORB
                    ANDB        #%00001111
                    CLRA
                    ADDD        #HEX_TABLE
                    XGDX
                    LDAA        0,X
                    PULB
                    RTS

;***********************************************************
; SYSTEM INTERRUPTS                                        *
;***********************************************************
; Reset Vector
                    ORG         $FFFE
                    DC.W        Entry