;*****************************************************************
;* This program multiplies (unsigned) two 8 bit numbers and      *
;* leaves the result in the 'PRODUCT' location.                  *
;*                                                               *
;* Course    : COE538                                            *
;* Term      : F2023                                             *
;* Assignment: Lab 1                                             *
;*                                                               *
;* Name      : Janice Zhu                                        *
;* StudentID : 501040242                                         *
;* Section   : 2                                                 *
;*****************************************************************
; export symbols

                XDEF Entry, _Startup  ; export 'Entry' symbol
                ABSENTRY Entry        ; for absolute assembly: mark 
                                      ; this as applicat. entry point

; Include derivative-specific definitions 
	            	INCLUDE 'derivative.inc' 

;*****************************************************************
;* Code section                                                  *
;*****************************************************************
                ORG $3000
               
MULTIPLICAND    FCB  05               ; First student number
MULTIPLIER      FCB  02               ; Second student number
PRODUCT         RMB  1                ; Result of unsigned multiply 

;*****************************************************************
;* The actual program starts here                                *
;*****************************************************************
                ORG   $4000
Entry:
_Startup:
                LDAA   MULTIPLICAND   ; Get the multiplicand
                LDAB   MULTIPLIER     ; Get the multiplier
                MUL                   ; 8 by 8 multiply (unsigned)
                STD    PRODUCT        ; Store the result
                SWI                   ; Break to the monitor

;*****************************************************************
;* Interrupt Vectors                                             *
;*****************************************************************
                ORG   $FFFE
                FDB   Entry           ; Reset Vector
