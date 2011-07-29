

/* Embed the configuration word within the C source file.
 * See the data sheet for the meaning of the configuration word bits.
 */
/* CONFIG1H */
#if 0
  /* 
CONFIG OSC = HS, FCMEN = OFF, IESO = OFF
CONFIG2L
     CONFIG PWRT = ON, BOREN = OFF, BORV = 2
     ;Setup CONFIG2H
     CONFIG WDT = OFF, WDTPS = 1024
     ;Setup CONFIG3H
     CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF
     ;Setup CONFIG4L
     CONFIG STVREN = OFF, LVP = OFF, XINST = OFF, DEBUG = OFF
     ;Setup CONFIG5L
     CONFIG CP0 = OFF, CP1 = OFF
     ;Setup CONFIG5H
     CONFIG CPB = OFF, CPD = OFF
     ;Setup CONFIG6L
     CONFIG EBTR0 = OFF
     ;Setup CONFIG6H
     CONFIG EBTR1 = OFF
     ;Setup CONFIG7L
  */
#endif

/* I/O EQUATES AND DEFINES */
#define SW2    PORTA,4
#define SW1    PORTB,0
#define RLED   PORTC,0
#define GLED   PORTC,1
#define ALRM	PORTC,3
#define E	PORTC,5		/* Enable line on LCD */
#define	RS	PORTC,4		/* Instruction/data select */
#define LCD  	PORTB		/* LCD data port */
enum LCD_commands {
  LCD_ClrScr=1, LCD_Home=2, LCD_On=12, LCD_off=8,
  LCD_Left=24, LCD_Right=28, LCD_Line2=0xC0,
};

/* Constants used for MIN/MAX or calculations */
const MT_Max = 200;		/* Max motor temp for error LED */
const CT_Max = 167;		/* Max controller temp for error LED */
const BT_Max = 125;		/* Max battery temp for error LED */
 /* Traction battery capacity in Amp/seconds (mid byte of 24-bit) */
const bcapacityM = 0x5E;
/* traction battery capacity in Amp/seconds (high byte of 24-bit) */
Constant    bcapacityH= 0x06  ; 
Constant    pmile= .32        ; Value=SOC percentage per mile range x 8  e.g. 2.5%SOC drop per mile would require value of 20
Constant    PWM_Max= 0xFF     ; PWM duty cycle for fuel gauge registering "full". Value=duty x 2.55
Constant    Sysval= B'10000000'; bit7= peukert range 1= 1.30~1.20, 0=1.15~1.05
                                ; bit6= temperature adjustment of battery capacity 1=enabled, 0=disabled
                                ; bit5:4= sets peukert exponent, If Bit 7=1 00=1.30, 01=1.25, 10=1.20, If Bit 7=0 00=1.15, 01=1.10, 10=1.05
                                ;                                If bit5:4=11, the bypass peukert adjustment
                                ; bit3= sets units format (temp/distance), 1=Metric, 0= Imperial
                                ; bit2= sets battery amp source, 1= Serial message, 0= Hall sensor (AN1)
                                ; bit1:0= sets Display mode 0=manual, 1=auto i.e."overtemp" shows temp screen, SOC default screen until mAmp>3 
 Constant    ALRM_lim= .21     ; percentage SOC where alarm is tripped
 Constant    PF2_Offset= .26   ;Length of Profile 1 message (in bytes) that establishes offset to access Profile 2
 Constant    Vpack_trim= .00   ;offset adjustment for Pack voltage also corrects for minor gain transfer error in HCNR201. Value=correction x 10
 Constant    Vacc_trim= .70    ;offset adjustment for accessory voltage intended to compensate for diode drop of D1. Value=correction x 100
 Constant    sAmp_scale= .16   ; scales motor amps and battery amps from serial message. Value/16= scale. Value of 16 is 1:1, 20 is 1.25:1,etc
 Constant    cEfficiency= .15   ; battery amp charge efficiency. 16 is 100%, 15=94%, 14=87.5%, 13=81%, 12=75%
 Constant    min_cAmp= .1      ; charge amp level where cTime stops incrementing and mode LED stops blinking
 Constant    D_refresh= .1  ; update rate for displayed information. 1= same as old software. 50=updates once per second


void setup_io_ports(void)
{
  /* Select primary oscillator */
  OSCCON = 0x74;
  PORTA = PORTB = PORTC = 0;

        movlw  0x74          ; select primary oscillator
        movwf  OSCCON 
        CLRF   PORTC           ; Ensure Ports are zero before we enable them
        CLRF   PORTA
        CLRF   PORTB           
        BCF   SSPCON1,5
        MOVLW   0x0F
        MOVWF   TRISB ; Set TRISB register as digital output RB7~RB4 and digital input RB3~RB0.
        MOVLW   0x80
        MOVWF   TRISC ; Set TRISC register as digital input RC7~RC6, all others output.              
        MOVLW   0x3F
        MOVWF   TRISA    ; (use EF)Set TRISA register as digital outputs RA6:RA7, all others analog input
    ; initialize A/D
        MOVLW   0x0A
        MOVWF   ADCON1 ; setup internal A/D reference
    ; initialize USART
        movlw   0x00   ; select high speed baud using 8-bit register
        movwf   BAUDCON
        movlw   0x19  ; set baudrate to 19.2
        movwf   SPBRG
        movlw   0xA4   ; 0xA4 for transmit enable. Test TRMT bit for empty (1)
        movwf   TXSTA
        movlw   0x10
        movwf   RCSTA
        CLRF  PIR1
        CLRF  PIR2 
        MOVLW  B'00100000'
        MOVWF  PIE1
        CLRF   PIE2 
    ; setup interrupts
        MOVLW  B'11100000' ;enable, peripheral and timer0 interrupts
        MOVWF  INTCON
        BSF RCON,7 ; enable interrupt priority mode
        BSF IPR1,5 ; set USART receive as high priority interrupt 
        BCF    INTCON2,2 ; set Timer0 to low priority interrupt 
        MOVLW   .20		; wait for 200ms 
        CALL    MS_DELAY       ;    
        CALL	INIT_LCD	; Initialize the LCD
        MOVLW   CLRSCR		; Clear the screen
        CALL    COMMAND		; Initiate the command in the W reg
        MOVLW   .20		; wait for 200ms
        CALL    MS_DELAY       ;        



;------------------------------------------------------------------------------
; EEPROM INITIALIZATION
;
; The 18F2480 has 1024 bytes of non-volatile EEPROM starting at 0xF00000
; 
;------------------------------------------------------------------------------

DATAEE    ORG  0xF00000 ; Starting address for EEPROM for 18F2480

    DE    0x00,0x06,0x5E        ; Default EEPROM Data

;------------------------------------------------------------------------------
; RESET VECTOR
;------------------------------------------------------------------------------

RES_VECT  ORG     0x0000            ; processor reset vector
          GOTO    START             ; go to beginning of program

;------------------------------------------------------------------------------
; HIGH PRIORITY INTERRUPT VECTOR
;------------------------------------------------------------------------------

ISRH      ORG     0x0008

          ; Run the High Priority Interrupt Service Routine
          GOTO    HIGH_ISR             

;------------------------------------------------------------------------------
; LOW PRIORITY INTERRUPT VECTOR
;------------------------------------------------------------------------------

ISRL      ORG     0x0018
          
          ; Run the High Priority Interrupt Service Routine
          GOTO    LOW_ISR             

;------------------------------------------------------------------------------
; HIGH PRIORITY INTERRUPT SERVICE ROUTINE used for Serial port Receive from controller
;------------------------------------------------------------------------------

HIGH_ISR  
          ; Context Saving for High ISR
          MOVWF   W_TEMP              ; save W register
          MOVFF   STATUS, STATUS_TEMP ; save status register
          MOVFF   BSR, BSR_TEMP       ; save bankselect register
                  
          ; Main RX routine
          BCF PIR1, RCIF
          movlw 0x00
          cpfsgt RX_count ; is this beginning of string?
          bcf LZstat,7 ; if so, clear comm error flag
          INCF RX_count    ; else increment count
          btfsc RCSTA,FERR ; check for receive framing error
          bsf LZstat,7 ; if found, set comm error flag
          MOVFF RCREG, RX_temp ; move character from USART receive reg to RX_temp reg
LF_tst    MOVLW 0x0A
          CPFSGT RX_temp ; compare received character with line feed
          bra LF_Det    ; if it is, branch to LF_Det section
          MOVLW .18
          CPFSEQ RX_count ; else look for 18th character (mAmp 100's)
          bra Char2
          MOVFF RX_temp, CHAR3 ; if found, move it to CHAR3 reg
          bra Exit
Char2     MOVLW .19
          CPFSEQ RX_count  ; look for 19th character (mAmp 10's)
          bra Char3
          MOVFF RX_temp, CHAR2 ; ; if found, move it to CHAR2 reg 
          bra Exit
Char3     MOVLW .20
          CPFSEQ RX_count ; look for 20th character (mAmp 1's)
          bra Char4
          MOVFF RX_temp, CHAR1 ; ; if found, move it to CHAR1 reg
          bra Exit
Char4     MOVLW .33
          CPFSEQ RX_count  ; look for 33rd character (controller temp 100's)
          bra Char5
          MOVFF RX_temp, CHAR6 ; if found, move it to CHAR6 reg
          bra Exit
Char5     Movlw .34
          CPFSEQ RX_count ; look for 34th character (controller temp 10's)
          bra Char6
          MOVFF RX_temp, CHAR5 ; ; if found, move it to CHAR5 reg
          bra Exit
Char6     MOVLW .35       ; look for 35th character (controller temp 1's)
          CPFSEQ RX_count
          bra Char7
          MOVFF RX_temp, CHAR4 ; if found, move it to CHAR4 reg
          bra Exit 
Char7     MOVLW .54
          CPFSEQ RX_count ; look for 54th character (bcAmp 100's)
          bra Char8
          MOVFF RX_temp, CHAR9 ; if found, move it to CHAR9 reg
          bra Exit  
Char8     MOVLW .55
          CPFSEQ RX_count ; look for 55th character (bcAmps 10's)
          bra Char9
          MOVFF RX_temp, CHAR8 ; if found, move it to CHAR8 reg
          bra Exit  
Char9     MOVLW .56
          CPFSEQ RX_count ; look for 56th character (bcAmps 1's)
          bra Exit
          MOVFF RX_temp, CHAR7 ; if found, move it to CHAR7 reg
          bra Exit                                    
LF_Det    movlw .67
          cpfseq RX_count ;is line feed the 67th character?
          bsf LZstat,7 ; if not, set comm error flag
          CLRF RX_count ; else clear count to indicate valid line feed encountered
                   
; Restore Context for High ISR          
Exit:     MOVF    W_TEMP, W           ; restore W register
          MOVFF   STATUS_TEMP, STATUS ; restore status register
          MOVFF   BSR_TEMP, BSR       ; restore bank select register
          
          
          RETFIE 

;---------------------------------------------------------------------------------------------------
; LOW PRIORITY INTERRUPT SERVICE ROUTINE used for 1 second timer (SOC adjust for charge & discharge)
;---------------------------------------------------------------------------------------------------

LOW_ISR
          ; Context Saving for Low ISR
          MOVWF   W_TEMP              ; save W register
          MOVFF   STATUS, STATUS_TEMP ; save status register
          MOVFF   BSR, BSR_TEMP       ; save bankselect register
          MOVFF   Offset,Off_temp
          MOVFF   OffsetH,Off_tempH
          MOVFF   PRODL,PRODL_temp
          MOVFF   PRODH,PRODH_temp
          MOVFF   CNT,CNT_Temp
          MOVFF   Arg2L,Arg2L_temp
          MOVFF   Arg2H,Arg2H_temp

          ; Main 1 second Timer routine
          bcf T0CON,TMR0ON; Disable timer0
          bcf INTCON,TMR0IF ; clear interrupt flag
          tstfsz CM_Delay ; if chargemode delay enable timer is greater
          decf CM_Delay   ; than zero, decrement it
          btfsc LZstat,2 ; check charge mode flag
          bra Charge_it ; if set, go to charge routine
          tstfsz bAmpH ; test for battery amps
          bra Tempmov  ; less than 2 amps
          movlw .1     ; if so, then
          cpfsgt bAmpL ; skip SOC adjust
          goto Alarm ; and writing to EEPROM 
Tempmov   movff bAmpL, bAmptempL ; move values since bAmptemp will be destroyed during divide
          movff bAmpH, bAmptempH
          movlw .3
          movwf CNT
Div8      bcf STATUS,C 
          rrcf bAmptempH,f ; divide bAmptemp by 8 to generate table offset
          rrcf bAmptempL,f
          decfsz CNT,f
          goto Div8
          tstfsz bAmptempL ; test for bAmptemp less than 8 after divide by 8. If so skip next step
          bra PeukTBL
          movff bAmpL, RES0 ; bAmp will directly be used for subtraction from capacity
          clrf RES1
          bra Sub32
PeukTBL   movlw .48
          cpfslt bAmptempL ; test for divided bAmp values greater than 47
          movwf  bAmptempL ; and limit those values to 48
          movff bAmptempL,Offset ;use divided bAmp value for table offset
          btfss  Sys_Config,5 ; test for peukert exponent setting of 1.20
          bra  TST125 ; else go to 1.25 test
          btfss  Sys_Config,4 ; test for peukert bypass setting
          bra TST120 ; else branch to 1.20 offset
          movff bAmpL, RES0 ; bypass peukert calculation, by moving bAmps directly
          movff bAmpH, RES1 ; over to RESx registers
          goto Sub32          
TST120    movlw  .181 ;.165 ; table offset for peukert of 1.20 
          bra   Addoff
TST125    btfss  Sys_Config,4 ; test for peukert exponent setting of 1.25
          bra  TST130         ; else branch to 1.30 offset
          movlw  .133 ;.117 ; table offset for peukert of 1.25 
          bra Addoff
TST130    movlw .85 ;.69   ; table offset for peukert of 1.30 
Addoff    addwf Offset,f
          call GetChar ; get peukert multiplier from table and place in W
          movff bAmpH,Arg2H
          movff bAmpL, Arg2L
          call Fracmult  ; multiply peukert value in W with bAmpH/L              
Sub32     movf    RES0,w ; subtract effective peukert adjusted amps from capacity
	      subwf   capacityL,F
	      movf    RES1,w
	      subwfb  capacityM,F
          movlw   0x00
          subwfb  capacityH,F
          incf EE_count  ;increment eeprom counter
          movlw .60      ; when it hits 60 (1 minute)
          cpfslt EE_count ; then goto the remaining
          call  Capstore  ; capacity storage routine
Alarm     movlw ALRM_lim
          cpfslt SOC
          bra mAmp_test ;Exit2
          btg ALRM
          btg RLED
mAmp_test  btfss LZstat,5
          bra Exit2
          btfsc LZstat,6
          bra Exit2
          movlw .6
          cpfslt Disp_count
          bra Exit2
          incf Disp_count
          movlw .5
          cpfseq Disp_count
          bra Exit2
          bsf LZstat,6
          bra Exit2
Charge_it   movlw min_cAmp 
          cpfsgt bAmpL ; if charge amps are equal or less that min_cAmp
          bra Exit2    ; then exit charge mode routine
          btg GLED
          incf EE_count
          movlw .59
          cpfsgt EE_count
          bra Addchrg
          call Capstore
          incf C_minutes
          movlw .60
          cpfseq C_minutes
          bra Addchrg
          incf C_hours
          clrf C_minutes
Addchrg   movlw .100
          cpfslt SOC ; if SOC is 100%
          bra Exit2  ; then exit and do not increase battery capacity
          movff bAmpL, Arg2L 
          clrf Arg2H         ; take battery amps 
          movlw cEfficiency  ; and multiply by 
          call Fracmult      ;charge efficiency value
          movf    RES0,w 
	      addwf   capacityL,F ; add efficiency adjusted charge amps to capacityL:M:H
	      movlw   0x00
	      addwfc  capacityM,F
          movlw   0x00
          addwfc  capacityH,F
                   
          ; Context Saving for Low ISR
Exit2     movlw  0xED
          movwf  TMR0L ; load timer low byte
          movlw  0x85
          movwf  TMR0H ; load timer high byte (total time= 1 second)
          MOVF    W_TEMP, W           ; restore W register
          MOVFF   STATUS_TEMP, STATUS ; restore status register
          MOVFF   BSR_TEMP, BSR       ; restore bank select register
          MOVFF   Off_temp, Offset
          MOVFF   Off_tempH, OffsetH
          MOVFF   PRODL_temp,PRODL
          MOVFF   PRODH_temp,PRODH
          MOVFF   CNT_Temp,CNT
          MOVFF   Arg2L_temp,Arg2L
          MOVFF   Arg2H_temp,Arg2H
          bsf T0CON,TMR0ON ; enable timer0
          RETFIE

;------------------------------------------------------------------------------
; MAIN PROGRAM
;------------------------------------------------------------------------------

START ;Initialize Ports
        movlw  0x74          ; select primary oscillator
        movwf  OSCCON 
        CLRF   PORTC           ; Ensure Ports are zero before we enable them
        CLRF   PORTA
        CLRF   PORTB           
        BCF   SSPCON1,5
        MOVLW   0x0F
        MOVWF   TRISB ; Set TRISB register as digital output RB7~RB4 and digital input RB3~RB0.
        MOVLW   0x80
        MOVWF   TRISC ; Set TRISC register as digital input RC7~RC6, all others output.              
        MOVLW   0x3F
        MOVWF   TRISA    ; (use EF)Set TRISA register as digital outputs RA6:RA7, all others analog input
    ; initialize A/D
        MOVLW   0x0A
        MOVWF   ADCON1 ; setup internal A/D reference
    ; initialize USART
        movlw   0x00   ; select high speed baud using 8-bit register
        movwf   BAUDCON
        movlw   0x19  ; set baudrate to 19.2
        movwf   SPBRG
        movlw   0xA4   ; 0xA4 for transmit enable. Test TRMT bit for empty (1)
        movwf   TXSTA
        movlw   0x10
        movwf   RCSTA
        CLRF  PIR1
        CLRF  PIR2 
        MOVLW  B'00100000'
        MOVWF  PIE1
        CLRF   PIE2 
    ; setup interrupts
        MOVLW  B'11100000' ;enable, peripheral and timer0 interrupts
        MOVWF  INTCON
        BSF RCON,7 ; enable interrupt priority mode
        BSF IPR1,5 ; set USART receive as high priority interrupt 
        BCF    INTCON2,2 ; set Timer0 to low priority interrupt 
        MOVLW   .20		; wait for 200ms 
        CALL    MS_DELAY       ;    
        CALL	INIT_LCD	; Initialize the LCD
        MOVLW   CLRSCR		; Clear the screen
        CALL    COMMAND		; Initiate the command in the W reg
        MOVLW   .20		; wait for 200ms
        CALL    MS_DELAY       ;        
    ;clear registers
        clrf  Messnum
        clrf RX_count
        clrf Disp_count
        clrf mAmpL
        clrf mAmpH
        clrf cTempL
        clrf cTempH
        clrf bcAmpL
        clrf bcAmpH
        clrf LZstat
        clrf C_hours
        clrf C_minutes
        clrf RF_Count
        movlw '0'   ; want mAmps and Tcont to display zeroes before first transmission
        movwf CHAR1
        movwf CHAR2
        movwf CHAR3
        movwf CHAR4
        movwf CHAR6
        movwf CHAR7
        movwf CHAR8
        movwf CHAR9
        movlw '6' ; Tcont will be 5deg F due to lookup table min value
        movwf CHAR5
        movlw Sysval
        movwf Sys_Config  
        movlw .2
        movwf CM_Delay ; set charge mode enable delay timer for 2 seconds      
    ; setup Fuel Gauge PWM
        bsf TRISC,2
        movlw 0xFF
        movwf PR2
        movlw 0x2C
        movwf CCP1CON
        movlw PWM_Max
        movwf CCPR1L
        movlw 0x06 ; PWM new setting of 488Hz. If using external PWM RC filter, change to 0x04 (7.8kHz)

        movwf T2CON
        bcf TRISC,2
    ; setup timer0
        movlw  0x05   ; set prescaler to 1:64 for Timer0 (32us timer clock)
        movwf  T0CON
        movlw  0xED
        movwf  TMR0L ; load timer low byte
        movlw  0x85
        movwf  TMR0H ; load timer high byte (total time= 1 second)
        clrf EE_count
    ; Test for SOC RESET request
        btfss SW1   ; test if switch 1 is depressed
        bra Reload  ; if so, then reload full capacity        
        movlw 0x01  ; else, read in capacity stored
        call EEread ;in the EEPROM
        movwf capacityH
        movlw 0x02
        call EEread
        movwf capacityM
        clrf  capacityL
        bra SOCdivr
Reload  movlw bcapacityH
        movwf capacityH
        movlw bcapacityM
        movwf capacityM
        clrf  capacityL
        call Capstore
        movlw  .54       ; load table pointer value into offset 
        movwf Offset     ; so we can retrieve and display
RST_MSG call GetChar	 ; message indicating SOC reset
		xorlw	0x00	 
		bz      SOCdivr
        call	DISPLAY
		incf	Offset, f
        goto	RST_MSG
     ; Setup SOC divisors & 20% capacity value
        ; First need SOC divisors
SOCdivr clrf CNT
        clrf CNT1
        movlw bcapacityH
        movwf captempH
        movlw bcapacityM
        movwf captempM
        clrf captempL
Div100  movlw    .100
        subwf   captempL,F
	    clrf WREG
	    subwfb  captempM,F        
        subwfb captempH,F
        bnc Pcnt1
        incf CNT
        btfsc STATUS,C
        incf CNT1
        goto Div100
Pcnt1   movff CNT,pdivL
        movff CNT1,pdivH
    ; then determine 20% capacity values for capacityH/M using divisors
        clrf CNT
        movlw bcapacityH
        movwf captempH
        movlw bcapacityM
        movwf captempM
        clrf captempL
Pcnt2   movf pdivL,w
        subwf   captempL,F
	    movf pdivH,w
	    subwfb  captempM,F
        clrf WREG
        subwfb captempH,F
        incf CNT
        movlw .80
        cpfslt CNT
        bra Pcntmov
        goto Pcnt2
Pcntmov movff captempM, pct20L
        movff captempH, pct20H
        bsf T0CON,TMR0ON ;enable timer
    ;Test for Fuel Gauge calibration request 
        btfsc SW2         ; test if switch 2 is depressed 
        bra CLROFF
        bcf T0CON,TMR0ON  ; stop timer0
        bcf INTCON,TMR0IF ; clear timer0 flag if set
        movf pct20H,W  ; set capacity to 20%
        movwf capacityH
        movf pct20L,W
        movwf capacityM
        clrf Messnum 
SW2_W   btfss SW2 ; wait here for switch to be released
        goto SW2_W
CLROFF  clrf   Offset ; set table pointer to zero for main display routine
        clrf OffsetH
        movlw HOME
        call COMMAND
        movlw 0x11  ; setup AN4 for battery temp
        movwf ADCON0
        bsf ADCON0, GO
Poll_2  btfsc ADCON0, GO
        bra Poll_2
        movff ADRESH,Captempcomp ;place battery temp in Captempcomp
        bsf RCSTA,7 ; enable RS232 receive
        bsf WDTCON,0 ; enable watchdog timer (4 sec)
        
           
        
;*******************************************************************************************
; MAIN LOOP 
;*******************************************************************************************
Repeat         
        clrwdt ; clear watchdog on each loop
        movlw .1
        call MS_DELAY ; 10ms delay
        btfss RCSTA,OERR ; check for receive overflow errors
        bra TSTRX ; if no error skip to RX_count test
        bcf RCSTA,CREN ; else restart USART to clear overflows
        bsf LZstat,7 ; and set comm error flag
        bsf RCSTA,CREN   ; restarting 
TSTRX   tstfsz RX_count ; test for end of message (CRLF)
        bra  A2DConv ; if not do A/D loop
        call RXconvrt ; else convert received characters from BCD ASCII to hex
A2DConv call  A2DLoop      ; samples the six A/D inputs and puts results in appropriate registers
        call SOC_Calc     ; Determine capacity percentage and put in SOC register
        movff mTemp, Temp_ptr   ; Adjust motor temp with lookup table
        movlw .20           
        cpfsgt Temp_ptr ; Test Temp_ptr value
        movwf Temp_ptr  ; and limit to no less than 20
        subwf Temp_ptr,f ; then subtract 20 for proper indexing into table
        call GetTemp     
        movwf mTemp     ; temp correction in WREG moved into motor temp reg
        movff bTemp, Temp_ptr   ; Adjust battery temp with lookup table
        movlw .20          
        cpfsgt Temp_ptr ; Test Temp_ptr value
        movwf Temp_ptr  ; and limit to no less than 20
        subwf Temp_ptr,f ; then subtract 20 for proper indexing into table
        call GetTemp
        movwf bTemp     ; temp correction in WREG moved into battery temp reg
        btfss SW1 ; is the momentary pushbutton SW1 depressed?
        CALL   Button   ; if yes, then debounce and increment message counter
        btfss SW2  ; is momentary pushbutton SW2 depressed?
        call TX_Mode ; if so, go to "profile" transmit routine
        bcf RLED ; set overtemp LED off
        movlw MT_Max    
        cpfslt mTemp    ; test motor temp, if over MT_Max
        bra  TFAULT     ; then set overtemp LED on
        movlw BT_Max
        cpfslt bTemp    ; test battery temp, if over BT_Max
        bra  TFAULT     ; then set overtemp LED on
        movlw CT_Max
        cpfsgt cTempL   ; test controller temp, if over CT_Max
        bra CLR_TF
TFAULT  bsf  RLED       ; then set overtemp LED on 
        btfss Sys_Config,0
        bra Offsave
        btfsc LZstat,4
        bra Offsave
        bsf LZstat,4
        movlw .36
        movwf Offset  ;jump display to 
        movlw .2
        movwf Messnum ;temperature screen
        bra Offsave
CLR_TF  bcf LZstat,4

    ; Display Routine
Offsave incf RF_Count ; increment the display refresh counter
        movlw D_refresh ; if the counter is less than
        cpfsgt RF_Count ;  or equal to D_refresh
        goto Repeat     ; then skip display routine
        clrf RF_Count   ; else clear count and enter display routine
        movff Offset,OffsetH 
        btfsc LZstat,2   ; test for charge mode 
        goto  Charge_Mode  ; and enter if flag set               
Line1   call	GetChar	 ;get a character from the text table
		xorlw	0x00	 ;is it a zero?
		bz    Line2      ; if so, the move to line 2 of LCD
        btfss Sys_Config,3 ; is metric units enabled?
        bra    DIS2       ; if not just display W value as is
        movwf Digtemp     ; else make a copy of WREG
        xorlw 'M'         ; then test for ascii M
        bnz RstrW         ; if not
        movlw 'K'
        bra DIS2
RstrW   movf Digtemp,w
DIS2    call	DISPLAY  ; else display contents of W
		incf	Offset   ; move to next character
        goto	Line1        
        
Line2  	movff OffsetH,Offset ; reset offset to beginning
        call  SECOND_LINE	;move to 2nd row, first column
        movlw .2
        cpfslt  Messnum ; check Messnum counter, if 2
        bra  TEMP3  ; then go to temperature screen
        movlw .1
        cpfslt Messnum
        bra  VSOCM  ; if Messnum is 1, then go to SOC screen
        movff VoltH, NumH ; if zero, then continue with main Volt/Amp screen
        movff VoltL, NumL ; start with pack voltage
        call Hex2BCD  ; convert it from hex to Ascii BCD
        bcf LZstat,0
        movff Dig4, Digtemp ; check for leading zero
        call Zerosupress    ; and suppress if present
        call DISPLAY
        movf Dig3, w
        call  DISPLAY
        movf Dig2, w
        call DISPLAY
        movlw  '.'
        call DISPLAY
        movf Dig1, w
        call DISPLAY
        movlw ' '
        call DISPLAY
        movff mAmpH, NumH ;next is motor amps
        movff mAmpL, NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig4, Digtemp
        call Zerosupress
        call DISPLAY
        movff Dig3, Digtemp
        call Zerosupress
        call  DISPLAY
        movff Dig2, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig1, w
        call DISPLAY
        movlw ' '
        call DISPLAY
        movlw ' '
        call DISPLAY
        movff bAmpH, NumH ; then battery amps
        movff bAmpL, NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call  DISPLAY
        movff Dig2, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig1, w
        call DISPLAY
Home1	movlw HOME
        call  COMMAND
        goto  Repeat

VSOCM   movff avoltH, NumH ; screen 2, first is accessory voltage
        movff avoltL, NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig4, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig3, w
        call  DISPLAY
        movlw  '.'
        call DISPLAY
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call DISPLAY
        movlw ' '
        call DISPLAY
        movlw ' '
        call DISPLAY
        clrf  NumH
        movff SOC, NumL ;then SOC
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call DISPLAY
        movff Dig2, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw '%'
        call DISPLAY
        movlw ' '
        call DISPLAY
        call  Range  ; determine MTE value
        clrf  NumH
        movff Miles, NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call DISPLAY
        movff Dig2, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig1, w       
        call DISPLAY
        goto Home1

TEMP3   btfsc Sys_Config,3 ; check if set for metric
        bra TEMP4          ; if so,branch to TEMP4 section
        clrf  NumH
        movff mTemp, NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw 'F'
        call DISPLAY
        movlw ' '
        call DISPLAY
        movlw ' '
        call DISPLAY
        movff cTempH, NumH
        movff cTempL, NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call  DISPLAY
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call DISPLAY
        movlw 'F'
        call DISPLAY
        clrf  NumH
        movff bTemp, NumL
        call Hex2BCD
        movlw ' '
        call DISPLAY
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw 'F'
        call DISPLAY
        goto Home1		

TEMP4   clrf  NumH
        movf  mTemp,W
        call F2C    ;convert fahrenheit to celcius
        movwf NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress 
        call DISPLAY
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw 'C'
        call DISPLAY
        movlw ' '
        call DISPLAY
        movlw ' '
        call DISPLAY
        clrf NumH
        movf cTempL, w
        call F2C
        movwf NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call  DISPLAY
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call DISPLAY
        movlw 'C'
        call DISPLAY
        clrf  NumH
        movf bTemp, w
        call F2C
        movwf NumL
        call Hex2BCD
        bcf LZstat,0
        movlw ' '
        call DISPLAY
        movff Dig3, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw 'C'
        call DISPLAY
        goto Home1	

Charge_Mode
        movlw .70  ; table offset for charging text
        movwf Offset
LineC1  call	GetChar	 ;get a character from the text table
		xorlw	0x00	 ;is it a zero?
		bz    LineC2      ; if so, the move to line 2 of LCD
        call	DISPLAY  ; else display contents of W
		incf	Offset   ; move to next character
        goto	LineC1     
LineC2 	movff OffsetH,Offset ; reset offset to beginning
        call  SECOND_LINE	;move to 2nd row, first column
        clrf NumH 
        movff C_hours, NumL 
        call Hex2BCD  
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw ':'
        call DISPLAY
        clrf NumH
        movff C_minutes, NumL
        call Hex2BCD
        movf Dig2, w
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw  ' '
        call DISPLAY
        movlw ' '
        call DISPLAY
        movff bAmpH, NumH 
        movff bAmpL, NumL
        call Hex2BCD
        bcf LZstat,0
        movff Dig2, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig1, w
        call DISPLAY
        movlw ' '
        call DISPLAY
        movlw ' '
        call DISPLAY
        clrf  NumH
        movff SOC, NumL 
        call Hex2BCD
        bcf LZstat,0
        movff Dig3, Digtemp
        call Zerosupress
        call DISPLAY
        movff Dig2, Digtemp
        call Zerosupress
        call DISPLAY
        movf Dig1, w
        call  DISPLAY
        movlw '%'
        call DISPLAY
        goto Home1

char text_table[] = "Volts mAmp bAmp \0"
"aVolt  SOC  MTE \0"
"Tmot Tcont Tbat \0"
" RESETTING SOC \0"
"cTime Amps SOC \0";

char TX_Table[] = "rtd-period \0"
"200\0"
"c-rr 15\r"
"bat-amps-lim 350\r\0"
"c-rr 5\r"
"bat-amps-lim 190\r";


/* Omitted: peukert multiplier tables. */

/* Initialize the LCD */
void init_lcd(void)
{
command8(0x30);			/* Software reset */
ms_delay(10);			/* Delay 100ms */
command8(0x30);			/* Reset again */
ms_delay(10);			/* Delay 100ms */
command8(0x30);			/* Reset again */
ms_delay(10);			/* Delay 100ms */

command8(0x28);			/* Set display to 4 bit mode */
command(0x28);			/* Now, in 4 bit mode, set to two lines. */
command(ON);
command(6);			/* Shift along every write */
}

/* Set the LCD to the second line. */
static inline void second_line(void)
{
  command(LIN2);
}

/* Executes a command in the W reg to the LCD Module.
 * Precondition: module set to 4 bit mode.
 */
void command()
{
  BCF RS;                        /* command coming */
  LCDtemp & 0xF0;
     MOVWF LCDtemp                  ;PUT data in temp file
     ANDLW 0xF0                     ; clear lower nibble
     MOVWF LCD     
     BSF E                          ;make sure the enable line is up 
     MOVLW .50                     ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     BCF INTCON,7
     BCF E                          ;clock the data in
     MOVLW .50                    ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     SWAPF LCDtemp
     MOVF  LCDtemp,w
     ANDLW 0xF0
     MOVWF LCD
     BSF E                          ;make sure the enable line is up 
     MOVLW .50                     ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     BCF E                          ;clock the data in
     BSF INTCON,7
     MOVLW .250                     ;delay time
     CALL MDELAY                    ;wait a while
     MOVLW .250                     ;delay time in microsecs
     CALL MDELAY    
     BSF RS                         ;return to a data input
     RETURN

COMMAND8: ; for 8-bit mode
     BCF RS                         ;command coming
     MOVWF LCD     
     BSF E                          ;make sure the enable line is up 
     MOVLW .250                     ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     BCF E                          ;clock the data in
     MOVLW .250                    ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     MOVLW .250                    ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     BSF RS
     
     RETURN
;***************************************************************
; This routine writes characters to the LCD module. The contents 
; of the 'w' register is displayed on the LCD screen.
;***************************************************************
DISPLAY: 
     BSF RS                         ;make sure it will accept data
     MOVWF LCDtemp                      ;send data to lcd
     ANDLW 0xF0                     ; clear lower nibble
     MOVWF LCD
     BSF E                          ;make sure the enable line is up 
     MOVLW .40                     ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     BCF INTCON,7
     BCF E                          ;clock the data in
     MOVLW .40                     ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     SWAPF LCDtemp
     MOVF  LCDtemp,w
     ANDLW 0xF0
     MOVWF LCD
     BSF E                          ;make sure the enable line is up 
     MOVLW .40                     ;delay time in microsecs
     CALL MDELAY                    ;WAIT
     BCF E                          ;clock the data in
     BSF INTCON,7
     MOVLW .250                     ;delay time
     CALL MDELAY                    ;wait a while
     MOVLW .250                     ;delay time in microsecs
     CALL MDELAY
     RETURN
;**********************************************************************************
; This routine loops through 6 A/D Channels, properly scales results and puts them 
; in associated address locations
;**********************************************************************************
A2DLoop:
        movlw 0xBA ;Conversion set to 20TAD cycles with 4us per TAD (80us total)
        movwf ADCON2 ; setup right justifed and conversion time
        movlw 0x01 ; setup ANO for PackVoltage
        movwf ADCON0
        movff VavgL,tempL ;sum of last 16 packvolt low byte
        movff VavgH,tempH ; sum of last 16 packvolt high byte
        movlw .3
        movwf Avg_div
        call A2DAVG ; acquire & pseudo moving average of 16
        movff tempL,VavgL ; new sum of last 16 packvolt low byte
        movff tempH,VavgH ; new sum of last 16 packvolt high byte        
        bcf STATUS,C
        movlw Vpack_trim ;move trim value into W
        btfsc WREG,7     ;and test if it is negative
        bra SUB_trim     ; if so, go to subtraction section
        addwf avgL       ; else add it to packvoltage
        clrf WREG
        addwfc avgH
        bra  Hyst1
SUB_trim bcf WREG,7    ; clear high bit (effectively subtracts 128 from trim value)
        subwf avgL     ; subtract trim from packvoltage
        clrf WREG
        subwfb avgH
        bc Hyst1 ;if result of subtraction creates rollover (no carry)
        clrf avgH  ; then limit pack voltage to zero
        clrf avgL
Hyst1   movff VoltH,tempH ; prepare for hysteresis routine
        movff VoltL,tempL
        call Hysteresis
        movff avgH, VoltH ; new average pack voltage low byte
        movff avgL, VoltL ; new average pack voltage high byte
        btfss Sys_Config,2 ; test for battery amp source
        bra AmpConv        ;if bit is clear then do a/d conversion
        movff bcAmpL,bAmpL ; else load amp value from serial message
        movff bcAmpH,bAmpH
        bra AccVolt        ;then skip to accessory voltage acquisition
AmpConv movlw 0x05 ; setup AN1 for battery amps
        movwf ADCON0       
        movff BavgL,tempL ;sum of last 16 batamp low byte
        movff BavgH,tempH ; sum of last 16 batamp high byte
        movlw .5
        movwf Avg_div
        call A2DAVG ; acquire & pseudo moving average of 16
        movff tempL,BavgL ; new sum of last 16 batamp low byte
        movff tempH,BavgH ; new sum of last 16 batamp high byte
        tstfsz C_hours
        bra bOffset
        movlw .1
        cpfsgt C_minutes ; when C_minutes hits 2 minutes
        bcf LZstat,2     ; then Chargemode is locked in even if bAmps goes to zero (flag can't be cleared)
        cpfslt C_minutes ; when C_minutes hits 1 minute
        bcf Sys_Config, 6 ;then temp compensation of SOC is cleared if enabled
bOffset movlw .30 ; subtract ~300mV offset from battery amps. 
        subwf avgL
        clrf WREG
        subwfb avgH
        bc Tst_avg ;if result of subtraction creates rollover (no carry)
        clrf avgH  ; then enter charge mode
        comf avgL  ; complement negative result in avgL to make positive
        movlw .2
        cpfsgt avgL ; if charge current is less than 3 amps
        bra Cancel  ; then make bAmps =0 and do not enter charge mode (added hysteresis)
        tstfsz CM_Delay ; Check charge mode powerup delay
        bra Tst_avg     ; if it is zero then
        bsf LZstat,2    ; set charge mode flag
        bra Tst_avg
Cancel  clrf avgL
Tst_avg tstfsz avgH ; if high byte is greater than zero, do hysteresis
        bra Hyst2  ;
        movlw .2   ; if not, then test low byte for value less than 2
        cpfslt avgL  ;
        bra Hyst2  ; if 2 or higher, do hysteresis
        bra Xfer   ; otherwise skip hysteresis
Hyst2   movff bAmpL,tempL
        movff bAmpH,tempH
        call Hysteresis
Xfer    movff avgH, bAmpH ; new average battery amp low byte
        movff avgL, bAmpL ; new average battery amp high byte    
;****************** Remove leftmost semicolons on section below if you want to use the LEM HASS x00-S*************
        ;movff bAmpH, Arg2H 
        ;movff bAmpL, Arg2L  
        ;movlw .29 ; value=29 for Hass 300-S, value=19 for Hass 200-S
        ;call Fracmult 
        ;movff RES0, bAmpL 
        ;movff RES1, bAmpH    
AccVolt  movlw 0x09 ; setup AN2 for accessory volts
        movwf ADCON0
        movff aVavgL,tempL ;sum of last 16 acc volt low byte
        movff aVavgH,tempH ; sum of last 16 acc volt high byte
        movlw .3
        movwf Avg_div
        call A2DAVG ; acquire & pseudo moving average of 16
        movff tempL,aVavgL ; new sum of last 16 acc volt low byte
        movff tempH,aVavgH ; new sum of last 16 acc volt high byte
        bcf STATUS,C
        movlw Vacc_trim
        addwf avgL
        clrf WREG
        addwfc avgH
        movff avoltH,tempH
        movff avoltL,tempL
        call Hysteresis
        movff avgH, avoltH ; new average acc voltage low byte
        movff avgL, avoltL ; new average acc voltage high byte
        movlw 0x3A ; Conversion set to 20TAD cycles with 4us per TAD (80us total)
        movwf ADCON2 ; setup left justifed and conversion time
        movlw 0x0D ; setup AN3 for motor temp
        movwf ADCON0
        bsf ADCON0, GO
Poll_4  btfsc ADCON0, GO
        bra Poll_4
        clrf tempH
        clrf avgH
        movff mTemp,tempL
        movff ADRESH,avgL
        call Hysteresis
        movff avgL,mTemp
        movlw 0x11  ; setup AN4 for battery temp
        movwf ADCON0
        bsf ADCON0, GO
Poll_5  btfsc ADCON0, GO
        bra Poll_5
        clrf tempH
        clrf avgH
        movff bTemp,tempL
        movff ADRESH,avgL
        call Hysteresis
        movff avgL,bTemp
        movlw 0x29   ; setup AN10 for FG_Scale potentiometer
        movwf ADCON0
        bsf ADCON0, GO
Poll_6  btfsc ADCON0, GO
        bra Poll_6
        movff ADRESH, FG_Scale
        movlw .202
        cpfslt FG_Scale ; compare A/D result with 202
        movwf FG_Scale  ; if equal or greater than 202, limit to 202
        return
;**************************************************************
; This routine does a moving average of 16 samples. AVG= Old AVG-(Old Avg/16)+ new sample
; returning new 16 sample sum in tempL/H and new average in avgL/H
;**************************************************************
A2DAVG
        bsf ADCON0, GO
Poll_1  btfsc ADCON0, GO
        bra   Poll_1
        movff tempL, avgL
        movff tempH, avgH
        movlw .4    ; number of rotates (divides) to do
        movwf CNT1
AVG1    bcf STATUS,C ; carry is cleared while looping back
        rrcf tempH,f
        rrcf tempL,f
        decfsz CNT1,f
        goto AVG1
        movf tempL,w
        subwf avgL,f
        movf tempH,w
        subwfb avgH
        movf ADRESL,w
        addwf avgL
        movf ADRESH,w
        addwfc avgH
        movff avgH,tempH
        movff avgL,tempL
AVG2    bcf STATUS,C ; carry is cleared while looping back
        rrcf avgH,f
        rrcf avgL,f
        decfsz Avg_div
        goto AVG2
        return

;***********************************************************************
; Routine to implement hysteresis in negative (decreasing) direction only
; Hysteresis amount determined by value loeaded into W reg
;***********************************************************************   
Hysteresis
       movff tempL,RES2
       movff tempH,RES3
       movf avgL,w
       clrf STATUS,C
       subwf RES2
       movf avgH,w
       subwfb RES3
       bc Nextcmp
       return
Nextcmp movlw .2
       cpfslt RES2
       return
       
MVtmp  movff tempH,avgH
       movff tempL,avgL
       return
       

;*****************************************************************************
; Routine to take received BCD ascii in registers "CHAR1~9" and convert to hex
;*****************************************************************************

RXconvrt  btfsc LZstat,7
          return
          bcf INTCON,7 ; temporarily disable interrupts while moving data out of CHAR registers
          MOVFF CHAR1, Dec1 
          MOVFF CHAR2, Dec2
          MOVFF CHAR3, Dec3
          Call BCD2Hex     ; convert motor amps to hex
          movff HexL,Arg2L ; and move to Arg2 regs to prep for multiply
          movff HexH,Arg2H
          movlw sAmp_scale ; move scaler (multiplier) into W
          Call Fracmult    ; and multiply it with motor amps
          MOVFF RES0, mAmpL
          MOVFF RES1, mAmpH
          btfsc Sys_Config,0 ;is display mode set to auto1?
          Call mAmp_stat     ;if so, check if amps<4 and set LZstat accordingly
          MOVFF CHAR4, Dec1
          MOVFF CHAR5, Dec2
          MOVFF CHAR6, Dec3
          CALL BCD2Hex     ; convert controller temp to hex
          movff cTempL,tempL; store old controller temp for hysteresis 
          movff cTempH,tempH
          MOVFF HexH, cTempH
          MOVFF HexL, cTempL 
          MOVFF CHAR7, Dec1
          MOVFF CHAR8, Dec2
          MOVFF CHAR9, Dec3
          CALL BCD2Hex     ; convert battery amps to hex
          MOVFF HexH, Arg2H ; and move to Arg2 regs to prep for multiply
          MOVFF HexL, Arg2L 
          movlw sAmp_scale ; move scaler (multiplier) into W
          Call Fracmult    ; and multiply it with battery amps
          MOVFF RES0,bcAmpL
          MOVFF RES1,bcAmpH
          bsf INTCON,7  ; re-enable interrupts
          movlw .2
          movwf CNT
Shift2    bcf STATUS,C ; carry is cleared while looping back
          rrcf cTempH,f ; divide controller temp by 4 to use as table pointer
          rrcf cTempL,f
          decfsz CNT,f
          goto Shift2
          movff cTempL,avgL
          movff cTempH, avgH
          movlw .2
          call Hysteresis
          movff avgL,cTempL
          movff avgH,cTempH
          clrf Temp_ptrH
          movff cTempL, Temp_ptr   ; Use temp value as initial index pointer for temp lookup table
          movlw .161   ; then add offset value to index table for proper thermistor data
          addwf Temp_ptr ; 
          clrf WREG      ; if offset + index pointer >255, will need
          addwfc Temp_ptrH ; to add carry to high byte
          call GetTemp
          movwf cTempL     ; temp correction in WREG moved into controller temp reg
          return

;***************************************************************************
;This routine is called during Display mode "auto1". It looks for motor amps
;less than 5 and sets flag & display offset & message counter
;***************************************************************************

mAmp_stat
         movlw .5
         subwf RES0
         movlw 0x00
         subwfb RES1
         bnc Set_M
         btfss LZstat,5
         return
         bcf LZstat,5
         bcf LZstat,6
         clrf Disp_count
         clrf Offset
         clrf Messnum         
         return
Set_M    btfsc LZstat,6
         bra Set_Disp
         bsf LZstat,5
         return
Set_Disp movlw .18
         movwf Offset
         movlw .1
         movwf Messnum
         bcf LZstat,6
         return


;*************************************************************
; This routine creates a delay of 'n' microsecond. 'n'= value in w reg
;*************************************************************
MDELAY:
     MOVWF CNT          ;LOAD DELAY TIME
     RRNCF  CNT,1          ;Div2 TO OBTAIN THE DELAY
COUNT1: 
     NOP         ; NO OPERATION
     
     DECFSZ CNT,1       ;DECREMENT THE COUNTER
     GOTO COUNT1         ;GOTO COUNT IF NOT FINISHED
     RETURN


;*************************************************************
; This routine creates a delay of 'n' * 10ms. 'n'= value in w reg
;*************************************************************
MS_DELAY:
     MOVWF CNT           ;LOAD DELAY TIME 
LOOP4: 
     MOVLW .40 ;        ;LOAD W WITH .40
     MOVWF CNT3          ;AND INTO COUNTER
LOOP3: 
     MOVLW .199   ;LOAD W WITH .199
     MOVWF CNT2          ;AND INTO COUNTER
LOOP2:
     DECFSZ CNT2,1       ;DECREMENT THE COUNTER
     GOTO LOOP2          ;GOTO LOOP AGAIN
     DECFSZ CNT3,1       ;DECREMENT THE OTHER COUNTER
     GOTO LOOP3          ;IF NOT FINISHED, GOTO LOOP AGAIN
     DECFSZ CNT,1        ;DECREMENT THE OTHER COUNTER
     GOTO LOOP4          ;IF NOT FINISHED, GOTO LOOP AGAIN
     RETURN

;***********************************************************
;  This routine converts 16-bit hex to 4 digit BCD ASCII
;***********************************************************
Hex2BCD:
      clrf   Dig4
      clrf   Dig3
      clrf   Dig2
      clrf   Dig1
      tstfsz NumH
      goto Thou
      goto Hunds
Thou: movlw .100
      subwf NumL,f
      bc Comp10
      decf NumH
      bz THunds
Comp10: incf Dig3
       movlw 0x0A
       cpfslt Dig3
       bra Carry4
       goto Thou
Carry4: clrf Dig3
        incf Dig4
       goto Thou

Hunds: movlw  .99
      cpfsgt NumL
      goto  Tens
      movlw  .100
      subwf  NumL,f
THunds incf   Dig3
      movlw 0x0A
      cpfseq Dig3
      goto Hunds
      clrf Dig3
      incf Dig4
      goto Hunds
Tens: movlw .9
      cpfsgt NumL
      goto Ones
      movlw .10
      subwf NumL,f
      incf  Dig2
      goto Tens
Ones: movff NumL, Dig1
      movlw 0x30
      addwf Dig1,f
      addwf Dig2,f
      addwf Dig3,f
      addwf Dig4,f
      return 

;***********************************************************
;  This routine converts 3 digit BCD ASCII to 16-bit Hex
;***********************************************************
BCD2Hex:
      movlw 0x30
      xorwf Dec1,f ; remove ascii format from Dec1
      xorwf Dec2,f ; remove ascii format from Dec2
      xorwf Dec3,f ; remove ascii format from Dec3   
      clrf  HexH
      Movff Dec1,HexL ;add ones to Hex low byte
      Movlw 0x0A
      Mulwf Dec2 ; multiply tens by 0Ahex (10 decimal)
      Movff PRODL,WREG
      addwf HexL ; add result of multiply to Hex low byte
      Movlw 0x64 ; multiply hundreds by 64hex (100 decimal)
      Mulwf Dec3
      Movff PRODL,WREG
      addwf HexL ; add low result of multiply to Hex low byte
      bnc BCDnext ;if carry then
      incf HexH   ; increment Hex high byte by one
BCDnext
      movff PRODH,WREG ; add high result of 
      addwf HexH       ; multiply to Hex high byte
      return

;********************************************************************
; This Routine multiplies W with 16-bit variable (Arg2L:Arg2H)then divides by 16. 
; This mimicks multiplying by fractional factors i.e. multiply by 19
; then divide by 16 is the same as multiplying by 1.1875
;********************************************************************
Fracmult
        mulwf Arg2L     
        movff PRODH, RES1 
        movff PRODL, RES0 
        mulwf Arg2H 
        movf PRODL,W
        addwf RES1
        movlw .4
        movwf CNT
Shift3  bcf STATUS,C ; carry is cleared while looping back
        rrcf RES1,f
        rrcf RES0,f
        decfsz CNT,f
        goto Shift3
        return
           
;****************************************************************
; This Routine indexes lookup table for alphanumeric character to display
;****************************************************************
GetChar: 
        movlw   UPPER Text_Table
        movwf   TBLPTRU
        movlw   HIGH Text_Table
        movwf   TBLPTRH
        movlw   LOW Text_Table
        movwf   TBLPTRL
        movf    Offset,w
        addwf   TBLPTRL,f
        clrf    WREG
        addwfc  TBLPTRH,f
        addwfc  TBLPTRU,f
        tblrd   *      ; Read with Post increment
        movf   TABLAT,w  ; Get character into W reg 
        return

;****************************************************************
; This Routine indexes lookup table for adjusted temperature value
;****************************************************************
GetTemp: 
        movlw   UPPER Temp_Table
        movwf   TBLPTRU
        movlw   HIGH Temp_Table
        movwf   TBLPTRH
        movlw   LOW Temp_Table
        movwf   TBLPTRL
        movf    Temp_ptr,w
        addwf   TBLPTRL,f
        clrf    WREG
        addwfc  TBLPTRH,f
        addwfc  TBLPTRU,f
        movf    Temp_ptrH,w
        addwf   TBLPTRH,f
        clrf    WREG
        addwfc  TBLPTRU,f
        tblrd   *      ; Read with Post increment
        movf   TABLAT,w  ; Get character into W reg 
        clrf    Temp_ptrH
        return

;***************************************************************************
; This Routine indexes lookup table for messages to transmit out serial port
;***************************************************************************
GetTX: 
        movlw   UPPER TX_Table
        movwf   TBLPTRU
        movlw   HIGH TX_Table
        movwf   TBLPTRH
        movlw   LOW TX_Table
        movwf   TBLPTRL
        movf    Offset,w
        addwf   TBLPTRL,f
        clrf    WREG
        addwfc  TBLPTRH,f
        addwfc  TBLPTRU,f
        tblrd   *      ; Read with Post increment
        movf   TABLAT,w  ; Get character into W reg 
        return

;****************************************************************
; This Routine calculates battery state of charge based on capacity
; and also uses this result to determine Fuel gauge PWM duty cycle
; PWM duty calc: PWM_Max-[(100-SOC)*FG_Scale/64]
;****************************************************************
SOC_Calc
        bcf INTCON,5
        clrf CNT
        movff capacityH,captempH ;move current capacity to temp storage 
        movff capacityM,captempM ;since it will be destroyed
        movff capacityL,captempL
LoopPc  movf pdivL,w  ;repeated subtraction (divide) by percentage divisor
        subwf   captempL,F
	    movf pdivH,w
	    subwfb  captempM,F
        movlw 0x00
        subwfb captempH,F
        bnc SOCZ    ;exit loop when subtraction done
        incf CNT    ; increment to create the quotient (SOC)
        goto LoopPc
SOCZ    tstfsz CNT ; is SOC zero?
        bra CAPcomp ; if no then continue to capacity compensation
        bcf T0CON,TMR0ON ;else disable Timer0
        clrf capacityL ; and clear capacity
        clrf capacityM
        clrf capacityH
CAPcomp btfss Sys_Config,6 ; is capacity compensation enabled?
        bra PWMset ; if not, then continue with setting PWM
        call CapTempAdj ; else get temp adjustment multiplier and put in WREG
        clrf Arg2H  
        movff CNT,Arg2L ; move SOC into temp register
        call Fracmult   ; then adjust it 
        movff RES0,CNT       ; move adjusted result back into SOC
   ; PWM Duty cycle routine
PWMset  movff CNT,SOC ; Copy SOC value over to SOC reg
        movlw .20
        cpfsgt CNT ;compare SOC with 20%
        movwf CNT  ; if less then fix at 20%
        movlw .100
        subfwb CNT,w ; subtract SOC value from 100
        mulwf FG_Scale ; then take result and multiply with potentiomenter VR2 count
        movff PRODH, RES3 
        movff PRODL, RES2 
        movlw .6
        movwf CNT
Shift5  bcf STATUS,C 
        rrcf RES3,f ; divide result in RES3:2 by 64
        rrcf RES2,f
        decfsz CNT,f
        goto Shift5
        movlw PWM_Max
        tstfsz RES3 ;look for quotient greater than 255
        movwf RES2  ; if so load RES2 with PWM_Max value (this is max to avoid PWM rollover)
        cpfslt RES2 ; else compare RES2 with PWM_Max. 
        movwf RES2 ; If greater, then limit to PWM_Max
        subfwb RES2,f ; subtract result from PWM_Max
        movff RES2, CCPR1L ; then load it into PWM duty register
        bsf INTCON,5
        return

;****************************************************************
; This Routine calculates miles to empty (20%)based on SOC
;****************************************************************
Range
        movff SOC,SOC_temp 
        clrf Miles
        incf Miles
        movlw .20
        cpfsgt SOC_temp
        bra Nomiles
        subwf SOC_temp,F
        movlw .8
        mulwf SOC_temp
Submile movlw pmile 
        subwf PRODL
        movlw 0x00
        subwfb PRODH        
        bnc Done1
        incf Miles
        goto Submile
Done1   return
Nomiles clrf Miles
        return

;****************************************************************
; This Routine uses battery temp measured on powerup (Captempcomp)
; to determine adjustment value (multiplier) for correcting SOC
;****************************************************************
CapTempAdj
       movlw 0x24  ; 32deg F
       cpfsgt Captempcomp
       retlw .10  ; multiplier for 32deg F and lower
       movlw 0x30  ; 40deg F
       cpfsgt Captempcomp
       retlw .11  ; multiplier for 33-40deg F
       movlw 0x31  ; 48deg F
       cpfsgt Captempcomp
       retlw .12  ; multiplier for 41-48deg F
       movlw 0x38  ; 56deg F
       cpfsgt Captempcomp
       retlw .13  ; multiplier for 49-56deg F
       movlw 0x40  ; 64deg F
       cpfsgt Captempcomp
       retlw .14  ; multiplier for 57-64deg F
       movlw 0x48  ; 72deg F
       cpfsgt Captempcomp
       retlw .15  ; multiplier for 65-72deg F
       movlw 0x58  ; 88deg F
       cpfsgt Captempcomp
       retlw .16  ; multiplier for 73-88deg F
       retlw .17  ; multiplier for >88deg F
       
      

;****************************************************************
; This Routine suppresses leading zeros on LCD display. If negative flag is
; set it will reurn with a '-'
;**************************************************************** 
Zerosupress
        btfss LZstat,3 ; check negative flag
        bra Ztest          ; if not set, branch to zero test
        bcf LZstat,3   ; else clear flag
        retlw '-'          ; and return with a '-' in W
Ztest   Movlw '0'
        cpfseq Digtemp    ;test if value is Ascii 0
        bra Dig2W         ;if not branch to Dig2W
        btfss LZstat,0    ; else check if higher order BCD dig was nonzero. 
        retlw ' '         ; if it was zero then return with space in W instead
        
Dig2W   bsf LZstat,0      ; set leading zero reg on nonzero value
        movf  Digtemp,w   ; return with original Ascii value
        return

;**********************************************************************
; This routine debounces pushbutton SW1, then increments message format 
; counter "Messnum" and adjusts "Offset" to retrieve correct text from
; the Lookup Table. 
;**********************************************************************
Button
      MOVLW .5
      CALL  MS_DELAY ; debounce for 50ms
      btfss SW1
      bra Loopbut
      return
Loopbut:
      clrwdt ;ensure watchdog doesn't reset if SW1 held for more than 4 seconds
      btfss SW1 ; wait for pushbutton
      goto Loopbut  ; to be released
      incf  Messnum
      movlw  .3
      cpfslt Messnum ; if counter increments to 3, then
      clrf  Messnum ; reset to zero (valid states 0,1,2)
      movlw .18
      mulwf Messnum      ; Messnum counter * 18 sets
      movff PRODL,Offset ; table pointer offset
      return

;*******************************************************************
; This routine debounces pushbutton SW2, then sends appropriate serial
; command based on status of register Sys_Config, bit 0.
;*******************************************************************
TX_Mode
       MOVLW .5
       CALL  MS_DELAY ; debounce for 50ms
       btfsc SW2
       return 
       btg GLED
       movff Offset, OffsetH
       movlw .3
       movwf CNT
TXLP   clrf WREG
       btfsc LZstat,1 
       ;btfss LZstat,1
       movlw PF2_Offset
       addlw .16
       movwf Offset
TXNXT  call	GetTX	 ;get a character from the text table
	   xorlw	0x00	 ;is it a zero?
	   bz    TX_End
       movwf   TXREG
Wait   btfss TXSTA,TRMT
       goto Wait
	   incf	Offset
       goto	TXNXT             
TX_End  decfsz CNT  
       goto TXLP 
       movff OffsetH,Offset
       btg LZstat,1
SW2T   btfss SW2
       goto SW2T
       return

;*****************************************************************
; Routine takes Farenheit value in W register and converts to Celcius
; then places back in W
;*****************************************************************
        
F2C   bcf LZstat,3 ; clear negative flag
      movwf Arg2L
      movlw .32
      cpfslt Arg2L
      bra Div59
      subwf Arg2L
      comf Arg2L
      bsf LZstat,3
      bra Clr59
Div59 subwf Arg2L
Clr59 clrf Arg2H
      movlw .9
      call Fracmult
      movf RES0,w
      return
      
;****************************************************************
; Routine to store current capacity to EEPROM 
;****************************************************************
Capstore
     clrf EE_count
     MOVLW 0x01 ;
     MOVWF EEADR ; Data Memory Address to read
     MOVFF capacityH,EEDATA ; Data Memory Value to write
     call EEsave
     movlw 0x02
     movwf EEADR
     movff capacityM,EEDATA
     call EEsave
     return

;****************************************************************
; EEPROM write routine
;****************************************************************

EEsave
     BCF  EECON1, EEPGD ; Point to DATA memory
     BCF  EECON1, CFGS ; Access program FLASH or Data EEPROM memory
     BSF  EECON1, WREN ; Enable writes
     BCF  INTCON, GIE ; Disable interrupts
     MOVLW 55h ;
     MOVWF EECON2 ; Write 55h
     MOVLW 0AAh ;
     MOVWF EECON2 ; Write AAh
     BSF  EECON1, WR ; Set WR bit to begin write
EEPOLL    
     btfss PIR2,EEIF
     goto EEPOLL
     bcf PIR2,EEIF
     BSF  INTCON, GIE ; Enable interrupts
     BCF  EECON1, WREN ; Disable writes on write complete (EEIF set)
     return

;****************************************************************
; EEPROM read routine
;****************************************************************

EEread
     movwf EEADR
     BCF  EECON1, EEPGD ; Point to DATA memory
     BCF  EECON1, CFGS ; Access program FLASH or Data EEPROM memory
     BSF  EECON1, RD ; EEPROM Read
     MOVF EEDATA, W ; W = EEDATA
     RETURN

PROG
     nop
     return

     END

        
