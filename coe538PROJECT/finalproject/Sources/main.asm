;****************************************************************************
;* lab 7 *
;****************************************************************************
;; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

ROMStart    EQU  $4000  ; absolute address to place my code/constant data

; variable/data section

            ORG RAMStart
 ; Insert here your data definition.
LCD_DAT     EQU PORTB ; LCD data port
LCD_CNTR    EQU PTJ
LCD_E       EQU $80   ;LCD E-signal pin
LCD_RS      EQU $40   ; LCD RS-signal pin

;*******************************************************************
; black line detecting thresholds

                       ;    
THRESH_A    EQU $99    ;    
THRESH_B    EQU $94    ;             
THRESH_C    EQU $59    ;              
THRESH_D    EQU $D0    ;            

                                             
THRESH_E    EQU $50    ;adjust left when sensor value is less than E threshold
THRESH_F    EQU $60    ;adjust right when sensor value is greater than F threshold 

;*******************************************************************
; TURNING TIMES

INC_DIS     EQU 100      
FWD_DIS     EQU 1000    
REV_DIS     EQU 1000    
STR_DIS     EQU 250    
TRN_DIS     EQU 13000   ; 90 degree turn
UTRN_DIS    EQU 12000   ; 180 degree turn

;*******************************************************************
; STATES

LFT_PTH    EQU 0              
RGT_PTH   EQU 1                    

START       EQU 0
FWD         EQU 1
REV         EQU 2

RGT_TRN      EQU 3
LFT_TRN      EQU 4

COMPLETED   EQU 5
ALL_STP     EQU 6

;*******************************************************************
; NON-CONSTANT VARIABLES

            ORG $3850 ; Where our TOF counter register lives
            
CRNT_STATE  dc.b 6 ; Current state register

COUNT1    dc.w 1 ;
COUNT2    dc.w 1 ;

SENS_A    DC.B 0 ; 0 state white, 1 state black (line)
SENS_B 	  DC.B 0
SENS_C    DC.B 0
SENS_D    DC.B 0
SENS_E    DC.B 0
SENS_F    DC.B 0

RETURN      dc.b 0
NEXT_D      dc.b 1 ; New direction 

TEN_THOUS   ds.b 1 ; 10,000 digit
THOUSANDS   ds.b 1 ; 1,000 digit
HUNDREDS    ds.b 1 ; 100 digit
TENS        ds.b 1 ; 10 digit
UNITS       ds.b 1 ; 1 digit
NO_BLANK    ds.b 1 ; Used in ?leading zero? blanking by BCD2ASC
BCD_SPARE   RMB 10   ; Extra space for decimal point and string terminator

;*******************************************************************
; SENSOR READINGS

; Returns: Sensor readings in:
; SENSOR_LINE (0) (Sensor E/F)
; SENSOR_BOW (1) (Sensor A)
; SENSOR_PORT (2) (Sensor B)
; SENSOR_MID (3) (Sensor C)
; SENSOR_STBD (4) (Sensor D)
; Note:
; The sensor number is shown in brackets
SENSOR_LINE     FCB $00 ; Storage for guider sensor readings
SENSOR_BOW      FCB $00 ; Initialized to test values
SENSOR_PORT     FCB $00
SENSOR_MID      FCB $00
SENSOR_STBD     FCB $00

SENSOR_NUM      RMB 1 ; The currently selected sensor
TEMP            RMB 10                                                                  


;*******************************************************************
;*******************************************************************
; CODE STARTS
;*******************************************************************
;*******************************************************************

            ORG ROMStart ; Where the code starts --------------------
Entry:
_Startup:
            LDS #$4000                  ; Initialize the stack pointer
                                        ;                                     I
           ;BSET DDRA,%00000011         ; STAR_DIR, PORT_DIR                  N
           ;BSET DDRT,%00110000         ; STAR_SPEED, PORT_SPEED    ON/OFF    I
                                        ;                                     T
            JSR initAD                  ; Initialize ATD converter            I
            JSR initPORTS               ;                                     A
            JSR initLCD                 ; Initialize the LCD                  L
            JSR clrLCD                  ; Clear LCD & home cursor             I
            JSR initTCNT              ; Jump to TOF initialization     
                     
            
            CLI                         ; Enable interrupts                   |         
                                        ;                                     Z
            LDX   #msg1                   ; Display msg1                        A
            JSR   putsLCD                 ; "                                   T
                      
            LDAA  #$8A                  ; move to end of msg1
            JSR   cmd2LCD                           
            
            LDX   #msg2                 ;print msg2
            JSR   putsLCD
                                       ;                                     I
            LDAA  #$C0                   ; Move LCD cursor to the 2nd row      O
            JSR   cmd2LCD                 ;                                     N
                 
            LDX   #msg3                   ; Display msg3                        |
            JSR   putsLCD                 ; "                                   |
            
            LDAA  #$C7                   ; MOVE TO END OF MSG3
            JSR   cmd2LCD
            
            LDX   #msg4
            JSR   putsLCD
                                        ;                                     |
            
MAIN        JSR   UPDT_READING              ; ----------------------------------------- M
            JSR   UPDT_DISPL
            LDAA  CRNT_STATE             ;                                           A
            JSR   DISPATCHER              ;                                           I
            BRA   MAIN                    ; ----------------------------------------- N

; data section
;*******************************************************************
msg1        dc.b "S:",0      ;   curr state
msg2        dc.b "",0       ;  sensors
msg3        dc.b "V:",0       ;        bat
msg4        dc.b "",0        ;      bumper

                            ;state names, max 7 characters
tab         dc.b "START  ",0
            dc.b "FWD    ",0
            dc.b "REV    ",0
            dc.b "RGT_TRN ",0
            dc.b "LFT_TRN ",0
            dc.b "CMPLTE ",0
            dc.b "STOP   ",0

;*******************************************************************
; MOTOR ROUTINES

STARON      BSET  PTT,  %00100000
            RTS
STAROFF     BCLR  PTT,  %00100000
            RTS
STARFWD     BCLR  PORTA,%00000010
            RTS
STARREV     BSET  PORTA,%00000010
            RTS
PORTON      BSET  PTT,  %00010000
            RTS
PORTOFF     BCLR  PTT,  %00010000
            RTS
PORTFWD     BCLR  PORTA,%00000001
            RTS
PORTREV     BSET  PORTA,%00000001
            RTS            

; subroutine section
;*******************************************************************
DISPATCHER  CMPA  #START       ; If it?s the START state -----------------
            BNE   NOT_START                                              ; |
            JSR   START_ST      ; then call START_ST routine D
            BRA   DISP_EXIT      ; and exit                  I
                                                         ; S
NOT_START   CMPA  #FWD
            BNE   NOT_FWD  ;
            JSR   FWD_ST       ;
            JMP   DISP_EXIT                                 ; P

NOT_FWD     CMPA  #RGT_TRN
            BNE   NOT_RGT_TRN
            JSR   RGT_TRN_ST
            JMP   DISP_EXIT

NOT_RGT_TRN  CMPA  #LFT_TRN
            BNE   NOT_LFT_TRN
            JSR   LFT_TRN_ST
            JMP   DISP_EXIT

NOT_LFT_TRN  CMPA  #REV
            BNE   NOT_REVERSE
            JSR   REV_ST
            JMP   DISP_EXIT
            
NOT_REVERSE CMPA  #COMPLETED
            BNE   NT_CMPLTE
            JMP   BK_TRK_ST
            
NT_CMPLTE   CMPA  #ALL_STP
            BNE   NOT_STP
            JSR   ALL_STP_ST    
            JMP   DISP_EXIT                                       ; A
                                                          ; T
NOT_STP     NOP

DISP_EXIT   RTS ; Exit from the state dispatcher ----------

;*******************************************************************
START_ST    BRCLR PORTAD0,$04,NO_FWD ; If /fwd_bmp  
            JSR   INIT_FWD
            MOVB  #FWD, CRNT_STATE
            BRA   START_EXIT    

NO_FWD      NOP ; Else
START_EXIT  RTS ; return to the MAIN routine
;*******************************************************************
FWD_ST      PULD
            BRSET PORTAD0,$04,NO_FWD_BUMP      ;IF THERE IS FWD BUMP THEN CHANGE NEXT_D VAL TO 1 
                                               ;ELSE GO TO NEXT LABEL TO CHECK FOR REAR BMP
            LDAA  RGT_PTH
            STAA  NEXT_D
            JSR   INIT_REV                     ;AND GOTO REVERSE STATE
            MOVB  #REV, CRNT_STATE ;
            JMP   FWD_EXIT; and return

NO_FWD_BUMP BRSET PORTAD0,$08,NO_REAR_BUMP ;IF REAR BUMP THEN COMPLETED AND IT WILL GO THROUGH MAZE IN REV
                                           ;ELSE CHECK SENSORS
            JSR   INIT_COMPLETE
            MOVB  #COMPLETED,CRNT_STATE
            JMP   FWD_EXIT 

NO_REAR_BUMP 
            LDAA  SENS_D           ;CHECK THE D SENSOR
            BEQ   NO_RGT_INTXN        ;IF IT IS WHITE THEN NO RIGHT TURN 
            LDAA  NEXT_D             ;ELSE IT IS BLACK
            PSHA                     ;THEN PUSH LAST INTERSECTION DIR TO STACK
            LDAA  LFT_PTH           ;STORE NEW DIRECTION TO NEXT_D
            STAA  NEXT_D
            JSR   INIT_RGT_TRN  ;     ;TRY MAKING A RIGHT TURN
            MOVB  #RGT_TRN, CRNT_STATE
            JMP   FWD_EXIT

NO_RGT_INTXN LDAA  SENS_B            ;CHECKING B SENSOR
            BEQ   NO_LFT_INTXN         ;IF 
            LDAA  SENS_A
            BEQ   LFT_TURN
            LDAA  NEXT_D
            PSHA
            LDAA  LFT_PTH
            STAA  NEXT_D
            BRA   NO_SHFT_LT
            
LFT_TURN     LDAA  NEXT_D
            PSHA
            LDAA  RGT_PTH
            STAA  NEXT_D
            JSR   INIT_LFT_TRN
            MOVB  #LFT_TRN, CRNT_STATE
            JMP   FWD_EXIT
            
NO_LFT_INTXN LDAA  SENS_F
            BEQ   NO_SHFT_RT
            JSR   PORTON

RGT_FWD_DIS  LDD   COUNT2
            CPD   #INC_DIS
            BLO   RGT_FWD_DIS    ;IF Dc > Dfwd THEN
            JSR   INIT_FWD      ;TURN MOTORS OFF
            JMP   FWD_EXIT
            
NO_SHFT_RT  LDAA  SENS_E
            BEQ   NO_SHFT_LT
            JSR   STARON
            
LFT_FWD_DIS  LDD   COUNT1
            CPD   #INC_DIS
            BLO   LFT_FWD_DIS
            JSR   INIT_FWD
            JMP   FWD_EXIT
            
NO_SHFT_LT  JSR   STARON
            JSR   PORTON
FWD_STR_DIS LDD   COUNT1
            CPD   #FWD_DIS
            BLO   FWD_STR_DIS
            JSR   INIT_FWD                        
            
FWD_EXIT    JMP MAIN ; return to the MAIN routine
;*******************************************************************
REV_ST      LDAA  COUNT1 ; If Dc>Drev then
            CPD   #REV_DIS
            BLO   REV_ST
            JSR   STARFWD
            LDD   #0
            STD   COUNT1
            
REV_U_TURN  LDD   COUNT1
            CPD   #UTRN_DIS
            BLO   REV_U_TURN
            JSR   INIT_FWD
            LDAA  RETURN
            BNE   BK_TRK_REV
            MOVB  #FWD,CRNT_STATE
            BRA   REV_EXIT
            
BK_TRK_REV  JSR   INIT_FWD
            MOVB  #COMPLETED,CRNT_STATE
                        
REV_EXIT    RTS ; return to the MAIN routine
;*******************************************************************
RGT_TRN_ST     LDD   COUNT2
              CPD   #STR_DIS
              BLO   RGT_TRN_ST
              JSR   STAROFF
              LDD   #0
              STD   COUNT2
            
RGT_TURN_LOOP  LDD   COUNT2
              CPD   #TRN_DIS
              BLO   RGT_TURN_LOOP
              JSR   INIT_FWD
              LDAA  RETURN
              BNE   BK_TRK_RGT_TRN
              MOVB  #FWD,CRNT_STATE
              BRA   RGT_TRN_EXIT
              
BK_TRK_RGT_TRN MOVB  #COMPLETED,CRNT_STATE

RGT_TRN_EXIT   RTS                          

;*******************************************************************
LFT_TRN_ST     LDD   COUNT1
              CPD   #STR_DIS
              BLO   LFT_TRN_ST
              JSR   PORTOFF
              LDD   #0
              STD   COUNT1
            
LFT_TURN_LOOP  LDD   COUNT1
              CPD   #TRN_DIS
              BLO   LFT_TURN_LOOP
              JSR   INIT_FWD
              LDAA  RETURN
              BNE   BK_TRK_LFT_TRN
              MOVB  #FWD,CRNT_STATE
              BRA   LFT_TRN_EXIT
              
BK_TRK_LFT_TRN MOVB  #COMPLETED,CRNT_STATE

LFT_TRN_EXIT   RTS      
;*******************************************************************
BK_TRK_ST     PULD
              BRSET   PORTAD0, $08,NO_BK_BUMP
              JSR     INIT_ALL_STP
              MOVB    #ALL_STP, CRNT_STATE
              JMP     BK_TRK_EXIT
              
NO_BK_BUMP    LDAA    NEXT_D
              BEQ     REG_PATHING
              BNE     IRREG_PATHING             
              
;*******************************************************************

REG_PATHING   LDAA  SENS_D
              BEQ   NO_RGT_TRN
              PULA
              PULA
              STAA  NEXT_D
              JSR   INIT_RGT_TRN
              MOVB  #RGT_TRN,CRNT_STATE
              JMP   BK_TRK_EXIT
              
NO_RGT_TRN     LDAA  SENS_B
              BEQ   RGT_LINE_S
              LDAA  SENS_A
              BEQ   LEFT_TURN
              PULA
              PULA
              STAA  NEXT_D
              BRA   NO_LINE_S
              
LEFT_TURN     PULA
              PULA
              STAA    NEXT_D
              JSR     INIT_LFT_TRN
              MOVB    #LFT_TRN,CRNT_STATE
              JMP     BK_TRK_EXIT                            

;*******************************************************************

IRREG_PATHING   LDAA  SENS_B
                BEQ   NO_LFT_TRN
                PULA
                STAA  NEXT_D
                JSR   INIT_LFT_TRN
                MOVB  #LFT_TRN,CRNT_STATE
                JMP   BK_TRK_EXIT
                
NO_LFT_TRN       LDAA  SENS_D
                BEQ   RGT_LINE_S
                LDAA  SENS_A
                BEQ   RIGHT_TURN
                PULA
                STAA  NEXT_D
                BRA   NO_LINE_S
                
RIGHT_TURN      PULA
                STAA    NEXT_D
                JSR     INIT_RGT_TRN
                MOVB    #RGT_TRN,CRNT_STATE
                JMP     BK_TRK_EXIT                                
;*******************************************************************

RGT_LINE_S     LDAA  SENS_D
              BEQ   LFT_LINE_S
              JSR   PORTON
              
RGT_FWD_D      LDD   COUNT2
              CPD   #INC_DIS
              BLO   RGT_FWD_D
              JSR   INIT_FWD
              JMP   BK_TRK_EXIT
              
LFT_LINE_S     LDAA  SENS_E
              BEQ   NO_LINE_S
              JSR   STARON
              
LFT_FWD_D      LDD   COUNT1
              CPD   #INC_DIS
              BLO   LFT_FWD_D
              JSR   INIT_FWD
              JMP   BK_TRK_EXIT
    
NO_LINE_S     JSR   STARON
              JSR   PORTON

FWD_STR_D     LDD   COUNT1
              CPD   #FWD_DIS
              BLO   FWD_STR_D
              JSR   INIT_FWD

BK_TRK_EXIT   JMP MAIN           
            
;*******************************************************************

ALL_STP_ST    BRSET     PORTAD0, $04,NO_START
              BCLR      PTT,%00110000
              MOVB      #START,CRNT_STATE
              BRA       ALL_STP_EXIT
              
NO_START      NOP
ALL_STP_EXIT      RTS                                              
                    
;*******************************************************************
INIT_FWD    BCLR PTT,%00110000
            LDD #0
            STD COUNT1
            STD COUNT2
            BCLR PORTA,%00000011 ; Set FWD direction for both motors
            RTS
;*******************************************************************
INIT_REV    BSET PORTA,%00000011 ; Set REV direction for both motors
            LDD   #0
            STD COUNT1
            BSET PTT,%00110000 ; Turn on the drive motors
            RTS
            
INIT_RGT_TRN BCLR PORTA,%00000011 ; Set FWD direction for both motors
            LDD #0
            STD COUNT2
            BSET PTT,%00110000 ; Turn on the drive motors
            RTS
            
INIT_LFT_TRN BCLR PORTA,%00000011 ; Set FWD direction for both motors
            LDD   #0
            STD COUNT1
            BSET PTT,%00110000 ; Turn on the drive motors
            RTS                        

INIT_COMPLETE INC RETURN
            PULA
            STAA  NEXT_D
            JSR   INIT_REV
            JSR   REV_ST
            JMP   MAIN
;*******************************************************************
INIT_ALL_STP BCLR PTT,%00110000 ; Turn off the drive motors
             RTS
            
 
 
 ;*******************************************************************
 ;  SENSOR SUBROUTINE
 
UPDT_READING      JSR     G_LEDS_ON
                  JSR     READ_SENSORS
                  JSR     G_LEDS_OFF
                  
                  LDAA    #00
                  STAA    SENS_A
                  STAA    SENS_B
                  STAA    SENS_C
                  STAA    SENS_D
                  STAA    SENS_E
                  STAA    SENS_F
                  
CHK_A           LDAA    SENSOR_BOW
                  CMPA    #THRESH_A
                  BLO     CHK_B
                  INC     SENS_A
           
CHK_B           LDAA    SENSOR_PORT
                  CMPA    #THRESH_B
                  BLO     CHK_C
                  INC     SENS_B
                  
CHK_C           LDAA    SENSOR_MID
                  CMPA    #THRESH_C
                  BLO     CHK_D
                  INC     SENS_C
                  
CHK_D           LDAA    SENSOR_STBD
                  CMPA    #THRESH_D
                  BLO     CHK_E
                  INC     SENS_D
                  
CHK_E           LDAA    SENSOR_LINE
                  CMPA    #THRESH_E
                  BHI     CHK_F
                  INC     SENS_E
                  
CHK_F           LDAA    SENSOR_LINE
                  CMPA    #THRESH_F
                  BLO     UPDT_DONE
                  INC     SENS_F                                                                                   
                  
UPDT_DONE         RTS                  
                                    
;*******************************************************************               
G_LEDS_ON         BSET    PORTA,%00100000
                  RTS
                  
G_LEDS_OFF        BCLR    PORTA,%00100000
                  RTS                  
;*******************************************************************
READ_SENSORS      CLR     SENSOR_NUM
                  LDX     #SENSOR_LINE

RS_MAIN_LOOP      LDAA    SENSOR_NUM
                  JSR     SELECT_SENSOR
                  LDY     #400
                  JSR     del_50us
                  LDAA    #%10000001
                  STAA    ATDCTL5
                  BRCLR   ATDSTAT0, $80,*
                  
                  LDAA    ATDDR0L
                  STAA    0,X
                  CPX     #SENSOR_STBD
                  BEQ     RS_EXIT
                  INC     SENSOR_NUM
                  INX
                  BRA     RS_MAIN_LOOP
                  
RS_EXIT           RTS

;*******************************************************************

SELECT_SENSOR     PSHA                       ;save the current sensor_num
                  LDAA    PORTA              ;clear the sensor selection bits
                  ANDA    #%11100011
                  STAA    TEMP               ;store it to temp
                  PULA                       ;get the sensor_num
                  ASLA                       ;
                  ASLA
                  ANDA    #%00011100
                  ORAA    TEMP
                  STAA    PORTA
                  RTS                                

; utility subroutines
;*******************************************************************
;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display, *
;* turn on display, cursor and blinking off. Shift cursor right. *
;*******************************************************************
initPORTS   BCLR  DDRAD, $FF
            BSET  DDRA,  $FF
            BSET  DDRT, $30
            RTS
                 
initLCD     BSET DDRB,%11111111 ; configure pins PS7,PS6,PS5,PS4 for output
            BSET DDRJ,%11000000 ; configure pins PE7,PE4 for output
            LDY #2000 ; wait for LCD to be ready
            JSR del_50us ; -"-
            LDAA #$28 ; set 4-bit data, 2-line display
            JSR cmd2LCD ; -"-
            LDAA #$0C ; display on, cursor off, blinking off
            JSR cmd2LCD ; -"-
            LDAA #$06 ; move cursor right after entering a character
            JSR cmd2LCD ; -"-
            RTS

;*******************************************************************
;* Clear display and home cursor *
;*******************************************************************
clrLCD      LDAA #$01 ; clear cursor and return to home position
            JSR cmd2LCD ; -"-
            LDY #40 ; wait until "clear cursor" command is complete
            JSR del_50us ; -"-
            RTS


initTCNT    MOVB  #$80,TSCR1
            MOVB  #$00,TSCR2
            MOVB  #$FC,TIOS
            MOVB  #$05,TCTL4
            MOVB  #$03,TFLG1
            MOVB  #$03,TIE
            RTS
            
;*******************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns. *
;*******************************************************************
del_50us:   PSHX ;2 E-clk
eloop:      LDX #300 ;2 E-clk -
iloop:      NOP ;2 E-clk |
            DBNE X,iloop ;3 E-clk -
            DBNE Y,eloop ;3 E-clk
            PULX ;3 E-clk
            RTS ;5 E-clk

;*******************************************************************
;* This function sends a command in accumulator A to the LCD *
;*******************************************************************
cmd2LCD:    BCLR LCD_CNTR,LCD_RS ; select the LCD Instruction Register (IR)
            JSR dataMov ; send data to IR
            RTS
           
;*******************************************************************
;* This function outputs a NULL-terminated string pointed to by X *
;*******************************************************************
putsLCD     LDAA 1,X+ ; get one character from the string
            BEQ donePS ; reach NULL character?
            JSR putcLCD
            BRA putsLCD
donePS      RTS


;*******************************************************************
;* This function outputs the character in accumulator in A to LCD *
;*******************************************************************
putcLCD     BSET LCD_CNTR,LCD_RS ; select the LCD Data register (DR)
            JSR dataMov ; send data to DR
            RTS
           
;*******************************************************************
;* This function sends data to the LCD IR or DR depening on RS *
;*******************************************************************
dataMov     BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
            STAA LCD_DAT ; send the upper 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
            LSLA ; match the lower 4 bits with the LCD data pins
            LSLA ; -"-
            LSLA ; -"-
            LSLA ; -"-
            BSET LCD_CNTR,LCD_E ; pull the LCD E signal high
            STAA LCD_DAT ; send the lower 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
            LDY #1 ; adding this delay will complete the internal
            JSR del_50us ; operation for most instructions
            RTS

initAD      MOVB #$C0,ATDCTL2 ;power up AD, select fast flag clear
            JSR del_50us ;wait for 50 us
            MOVB #$00,ATDCTL3 ;8 conversions in a sequence
            MOVB #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
            BSET ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
            RTS

;*****************************************************************
;* Integer to BCD Conversion Routine
;* This routine converts a 16 bit binary number in .D into
;* BCD digits in BCD_BUFFER.
;* Peter Hiscocks
;* Algorithm:
;* Because the IDIV (Integer Division) instruction is available on
;* the HCS12, we can determine the decimal digits by repeatedly
;* dividing the binary number by ten: the remainder each time is
;* a decimal digit. Conceptually, what we are doing is shifting
;* the decimal number one place to the right past the decimal
;* point with each divide operation. The remainder must be
;* a decimal digit between 0 and 9, because we divided by 10.
;* The algorithm terminates when the quotient has become zero.
;* Bug note: XGDX does not set any condition codes, so test for
;* quotient zero must be done explicitly with CPX.
;* Data structure:
;* BCD_BUFFER EQU * The following registers are the BCD buffer area
;* TEN_THOUS RMB 1 10,000 digit, max size for 16 bit binary
;* THOUSANDS RMB 1 1,000 digit
;* HUNDREDS RMB 1 100 digit
;* TENS RMB 1 10 digit
;* UNITS RMB 1 1 digit
;* BCD_SPARE RMB 2 Extra space for decimal point and string terminator

int2BCD     XGDX                   ; Save the binary number into .X
            LDAA #0   ;lear the BCD_BUFFER
            STAA TEN_THOUS
            STAA THOUSANDS
            STAA HUNDREDS
            STAA TENS
            STAA UNITS
            STAA BCD_SPARE
            STAA BCD_SPARE+1
            
            CPX #0 ;Check for a zero input
            BEQ CON_EXIT ; and if so, exit

            XGDX         ;Not zero, get the binary number back to .D as dividend
            LDX #10      ; Setup 10 (Decimal!) as the divisor
            IDIV         ;Divide: Quotient is now in .X, remainder in .D
            STAB UNITS    ;Store remainder
            CPX #0       ;If quotient is zero,
            BEQ CON_EXIT   ;then exit

            XGDX  ;swap first quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB TENS
            CPX #0
            BEQ CON_EXIT

            XGDX ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            
            IDIV
            STAB HUNDREDS
            CPX #0
            BEQ CON_EXIT

            XGDX ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB THOUSANDS
            CPX #0
            BEQ CON_EXIT

            XGDX    ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB TEN_THOUS

CON_EXIT    RTS ;We?re done the conversion

;****************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ?NO_BLANK? starts cleared and is set once a non-zero
;* digit has been detected.
;* The ?units? digit is never blanked, even if it and all the
;* preceding digits are zero.
;* Peter Hiscocks
BCD2ASC     LDAA  #0               ; Initialize the blanking flag
            STAA NO_BLANK

C_TTHOU     LDAA TEN_THOUS ;Check the ?ten_thousands? digit
            ORAA NO_BLANK
            BNE NOT_BLANK1

ISBLANK1    LDAA #' '             ; It's blank
            STAA TEN_THOUS ;so store a space
            BRA  C_THOU ;and check the ?thousands? digit

NOT_BLANK1  LDAA TEN_THOUS ;Get the ?ten_thousands? digit
            ORAA #$30 ;Convert to ascii
            STAA TEN_THOUS
            LDAA #$1 ;Signal that we have seen a ?non-blank? digit
            STAA NO_BLANK

C_THOU      LDAA THOUSANDS ;Check the thousands digit for blankness
            ORAA NO_BLANK  ;If it?s blank and ?no-blank? is still zero
            BNE  NOT_BLANK2

ISBLANK2    LDAA  #' '             ; Thousands digit is blank
            STAA THOUSANDS ;so store a space
            BRA  C_HUNS ;and check the hundreds digit

NOT_BLANK2  LDAA THOUSANDS ;(similar to ?ten_thousands? case)
            ORAA #$30
            STAA THOUSANDS
            LDAA #$1
            STAA NO_BLANK

C_HUNS      LDAA HUNDREDS ;Check the hundreds digit for blankness
            ORAA NO_BLANK ;If it?s blank and ?no-blank? is still zero
            BNE NOT_BLANK3

ISBLANK3    LDAA  #' '             ; Hundreds digit is blank
            STAA HUNDREDS ;so store a space
            BRA C_TENS ;and check the tens digit

NOT_BLANK3  LDAA HUNDREDS ;(similar to ?ten_thousands? case)
            ORAA #$30
            STAA HUNDREDS
            LDAA #$1
            STAA NO_BLANK

C_TENS      LDAA TENS ;Check the tens digit for blankness
            ORAA NO_BLANK ;If it?s blank and ?no-blank? is still zero
            BNE NOT_BLANK4  ;

ISBLANK4    LDAA  #' '             ; Tens digit is blank
            STAA TENS ;so store a space
            BRA C_UNITS ;and check the units digit

NOT_BLANK4  LDAA TENS ;(similar to ?ten_thousands? case)
            ORAA #$30
            STAA TENS

C_UNITS     LDAA UNITS ;No blank check necessary, convert to ascii.
            ORAA #$30
            STAA UNITS

            RTS ;We?re done
            
;---------------------------------------------------------------------------
; Binary to ASCII
; Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
; character string in accumulator D
; Uses a table-driven method rather than various tricks.
; Passed: Binary value in ACCA
; Returns: ASCII Character string in D
; Side Fx: ACCB is destroyed
HEX_TABLE   FCC '0123456789ABCDEF' ; Table for converting values
BIN2ASC     PSHA ; Save a copy of the input number on the stack
            TAB ; and copy it into ACCB
            ANDB #%00001111 ; Strip off the upper nibble of ACCB
            CLRA ; D now contains 000n where n is the LSnibble
            ADDD #HEX_TABLE ; Set up for indexed load
            XGDX
            LDAA 0,X ; Get the LSnibble character
            PULB ; Retrieve the input number into ACCB
            PSHA ; and push the LSnibble character in its place
            RORB ; Move the upper nibble of the input number
            RORB ; into the lower nibble position.
            RORB
            RORB
            ANDB #%00001111 ; Strip off the upper nibble
            CLRA ; D now contains 000n where n is the MSnibble
            ADDD #HEX_TABLE ; Set up for indexed load
            XGDX
            LDAA 0,X ; Get the MSnibble character into ACCA
            PULB ; Retrieve the LSnibble character into ACCB
            RTS


;*******************************************************************
;* Update Display () *
;*******************************************************************
UPDT_DISPL  LDAA    #$82
            JSR     cmd2LCD
            
            LDAB    CRNT_STATE
            LSLB
            LSLB
            LSLB
            LDX   #tab
            ABX
            JSR   putsLCD
            
            
            LDAA    #$8F
            JSR     cmd2LCD
            LDAA    SENSOR_BOW
            JSR     BIN2ASC
            JSR     putcLCD
            EXG     A,B
            JSR     putcLCD
            
            LDAA    #$92
            JSR     cmd2LCD
            LDAA    SENSOR_LINE
            JSR     BIN2ASC
            JSR     putcLCD
            EXG     A,B
            JSR     putcLCD
            
            LDAA    #$CC
            JSR     cmd2LCD
            LDAA    SENSOR_PORT
            JSR     BIN2ASC
            JSR     putcLCD
            EXG     A,B
            JSR     putcLCD                        
                                                    
            LDAA    #$CF
            JSR     cmd2LCD
            LDAA    SENSOR_MID
            JSR     BIN2ASC
            JSR     putcLCD
            EXG     A,B
            JSR     putcLCD 
            
            LDAA    #$D2
            JSR     cmd2LCD
            LDAA    SENSOR_STBD
            JSR     BIN2ASC
            JSR     putcLCD
            EXG     A,B
            JSR     putcLCD       
            
            MOVB    #$90, ATDCTL5
            BRCLR   ATDSTAT0,   $80,*
            LDAA    ATDDR0L
            LDAB    #39
            MUL
            ADDD    #600
            JSR     int2BCD
            JSR     BCD2ASC
            LDAA    #$C2
            JSR     cmd2LCD
            LDAA    TEN_THOUS
            JSR     putcLCD
            LDAA    THOUSANDS
            JSR     putcLCD
            LDAA    #$2E       ; PERIOD CHARACTER
            JSR     putcLCD
            LDAA    HUNDREDS
            JSR     putcLCD
            
            LDAA    #$C9
            JSR     cmd2LCD
            
            BRCLR   PORTAD0,#%00000100,bowON
            LDAA    #$20
            JSR     putcLCD
            BRA     stern_bump
     bowON: LDAA    #$42
            JSR     putcLCD
            
stern_bump: BRCLR   PORTAD0, #%00001000,sternON
            LDAA    #$20
            JSR     putcLCD
            BRA     UPDT_DISPL_EXIT
   sternON: LDAA    #$53
            JSR     putcLCD
            
UPDT_DISPL_EXIT RTS
    
    
;**************************************************************
; INTERRUPT SERVICE ROUTINES FOR TURNING            
ISR1    MOVB  #$01,TFLG1
        INC   COUNT1
        RTI
        
ISR2    MOVB  #$02,TFLG1
        INC   COUNT2
        RTI                                  
                                                                
;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector  
                 
            ORG $FFEE
            DC.W ISR1 ; Timer Overflow Interrupt Vector
            
            ORG $FFEC
            DC.W  ISR2
