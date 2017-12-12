;******************************************************************************
; TITLE: 
; AUTHOR:
; DESCRIPTION: 
;******************************************************************************
  LIST p=18f2331,f=INHX32,r=DEC; Definition du microcontroleur
  #include<p18f2331.inc>       ; Fichier include
  CONFIG OSC = HSPLL,  DEBUG = ON, WDTEN=OFF, LVP = OFF ; PLL enable => 40MHz

;******************************************************************************
;  DEFINITION DE SYMBOLES ET MACRO
;******************************************************************************
movlf MACRO Val, Reg
  movlw Val
  movwf Reg
  endm
 
 
add16 MACRO Val,Rslt
  movf Val,W
  addwf Rslt,F
  btfsc STATUS,C
  incf Rslt+1
  movf Val+1,W
  addwf Rslt+1,F
  endm
 
mul8s MACRO ARG1,ARG2
  MOVF ARG1, W
  MULWF ARG2	    ; ARG1 * ARG2 -> PRODH:PRODL
  BTFSC ARG2, SB    ; Test Sign Bit
  SUBWF PRODH, F    ; PRODH = PRODH - ARG1
  MOVF ARG2, W
  BTFSC ARG1, SB    ; Test Sign Bit
  SUBWF PRODH, F    ; PRODH = PRODH - ARG2
  endm
 
  

; -- QEI
  
  ;#define POSCNT_MAX B'11111010000'
  #define POSCNT_MAX_L B'11010000'
  #define POSCNT_MAX_H B'111'
  ;#define POSCNT_MAX_L B'00000000'
  ;#define POSCNT_MAX_H B'1000'
; -- UART CONFIGURATION
  #define UXTX_LEN 6
  ;Pour Fosc=40Mhz
  ;#define SPBRGVal 11	; 921 600 Baud
  #define SPBRGVal 43 	; 230 400 Baud
  ;#define SPBRGVal 86 	; 115 200 Baud
  ;#define SPBRGVal 172	;  57 600 Baud
; -- PWM CONFIGURATION
  #define PWM_PREL B'11110100'
  #define PWM_PERH 1
  #define PMW_DCmax B'11000000'
  #define PMW_DCmax_H B'111'
; -- Special Event PWM delay
  #define PWM_SEVTCMPL 15
  #define PWM_SEVTCMPH 0
; -- TIMER0 -------
; Base de temps de 4ms pour FOSC = 40MHz et 1:128 Prescale value
  #define TIMER0_RELOADL 0xC7
  #define TIMER0_RELOADH 0xFE
  
  #define I_MAX 30
  #define FORWARD_REVERSE LATB, RB0
  
  
; Definition des Flag de MCC_FLAG
  #define Update 0
  #define QEIUP 1
  #define QEIDOWN 2
  
;******************************************************************************
;  VARIABLE dans ACCESS RAM
;******************************************************************************
  CBLOCK  0x000   ; zone access ram de la banque 0
Time_s:2
Time_Test:2
 
MCC_iMeas:1
MCC_I:2
MCC_I_C: 1
MCC_Vts: 1
MCC_Vts_H:1
MCC_Vts_z: 1
MCC_Vts_C: 1
MCC_Vts_Err:1
MCC_FLAG: 1
MCC_QEI: 1
MCC_QEIH: 1
MCC_QEI_z:1
MCC_QEIH_z:1
MCC_Pst: 2
MCC_DutyCycle: 2
MCC_DutyCycle_Temp:2
  
UxTx_go: 1
UxTx_size: 1
UxTx_iTx: 1
UxRxChar: 1
  
;QEI_Flag:1
  
Corr_KI:1
Corr_KD:1
Corr_KP:1
;Corr_eI:2 INUTILE
Corr_eI_z:2
Corr_uI:2
;Corr_uI_z:2 INUTILE
  
  ENDC            ; fin de la zone de declaration

  CBLOCK  0x100   ; zone access ram de la banque 1
UxTx_str : UXTX_LEN

  ENDC            ; fin de la zone de declaration

  CBLOCK  0x200   ; zone access ram de la banque 2
wreg_lp : 1
bsr_lp : 1
status_lp : 1
  ENDC            ; fin de la zone de declaration
;******************************************************************************
; PROGRAMME
;******************************************************************************
;--- VECTEUR DE RESET ---------------------------------------------------------
  ORG 0x00
  goto main

;--- VECTEUR D'INTERRUPTION HAUTE PRIORITE ------------------------------------
  ORG 0x08
  BTFSC PIR3, IC2QEIF  ; QEI
  BRA IT_QEI
  BTFSC PIR1, ADIF ; ADC
  BRA IT_ADC
  BTFSC PIR3, PTIF; PWM
  BRA IT_PWM  
  retfie FAST
;--- VECTEUR ET ROUTINE D'INTERRUPTION BAS PRIORITE ---------------------------
  ORG 0x18
  MOVFF STATUS, status_lp
  MOVFF WREG, wreg_lp
  BTFSC PIE1, TXIE
  BTFSS PIR1, TXIF  ; EUSART TX
  BRA SKIP_ITTX
  BRA IT_TX
SKIP_ITTX
  BTFSC PIR1, RCIF  ; EUSART RC
  BRA IT_RC
  BTFSC INTCON, TMR0IF; TIMER0
  BRA IT_TIMER0
LowITend
  MOVFF wreg_lp, WREG
  MOVFF status_lp, STATUS
  retfie
;--- ROUTINES D'INTERRUPTION BAS PRIORITE -------------------------------------
; INTERRUPT EUSART RC
IT_RC
;  MOVFF TXREG, POSTDEC1
;  MOVF FSR1L, F
;  BN IT_RC_Label
;  BRA LowITend
;IT_RC_Label
;  LFSR FSR1, UxTx_str+UXTX_LEN-1
  BCF PIE1, RCIE    ; Disable Interrupt
  BRA LowITend
; ---------------------
; INTERRUPT EUSART TX
IT_TX
  incf UxTx_iTx
  MOVFF POSTINC0, TXREG
  movlw UXTX_LEN
  CPFSLT UxTx_iTx
  BCF PIE1, TXIE    ; Disable Interrupt  
  BRA LowITend
; ---------------------
; INTERRUPT TIMER0
IT_TIMER0
  MOVFF WREG, wreg_lp
  BCF INTCON, TMR0IF	; Aquitement
  movlf TIMER0_RELOADL, TMR0L
  movlf TIMER0_RELOADH, TMR0H
  BTG LATC, RC1
  INCF Time_s, F
  BNC IT_T0_Label
  INCF Time_Test+1, F
IT_T0_Label
  MOVFF wreg_lp, WREG
  BRA LowITend
;--- ROUTINES D'INTERRUPTION HAUTE PRIORITE ------------------------------------
; INTERRUPT QEI
IT_QEI
  BCF PIR3, IC2QEIF	; Aquitement
  BTFSS QEICON, UP_DOWN
  BRA IT_QEI_DOWN
IT_QEI_UP
  bsf MCC_FLAG, QEIUP
  bcf MCC_FLAG, QEIDOWN
  retfie FAST
IT_QEI_DOWN
  bcf MCC_FLAG, QEIUP
  bsf MCC_FLAG, QEIDOWN
  retfie FAST
; ---------------------
; INTERRUPT ADC
IT_ADC
  BCF PIR1, ADIF
  movlf PWM_SEVTCMPL, SEVTCMPL
  movlf PWM_SEVTCMPH, SEVTCMPH
  MOVFF ADRESH, MCC_I+1
  MOVFF ADRESL, MCC_I
  btg LATB, 5
  ; Calcul de la vitesse avec le QIE
  ;     Si overflow croissant : MCC.Vts = varQEI + POSCNT_MAX - MCC.QEI;
  ;     Si overflow decroissant :  MCC.Vts = varQEI - POSCNT_MAX - MCC.QEI
  ;     Pas Overflow  : MCC.Vts = varQEI - MCC.QEI;
  
  movff POSCNTL, MCC_QEI
  movff POSCNTH, MCC_QEI+1
  
  BTFSC MCC_FLAG, QEIUP
  BRA ADC_skipUP
  movlw POSCNT_MAX_L
  addwf MCC_Vts, F
  movlw POSCNT_MAX_H
  addwfc MCC_Vts+1, F
  bra ADC_skipDOWN
ADC_skipUP
  BTFSC MCC_FLAG, QEIDOWN
  BRA ADC_skipDOWN
  movlw POSCNT_MAX_L
  subwf MCC_Vts, F ; (f) - (W) -> dest
  movlw POSCNT_MAX_H
  subwfb MCC_Vts+1, F ; (f) - (W) -> dest
ADC_skipDOWN
  movf MCC_QEI, W
  subwf MCC_Vts, F ; (f) - (W) -> dest
  movf MCC_QEI+1, W
  subwfb MCC_Vts+1, F
;  BZ ADC_next
;  movff MCC_Vts_z, MCC_Vts ; si MCC_Vts+1 n'est pas null : valeur invalide ...
ADC_next
  movff MCC_Vts, MCC_QEI
  movff MCC_Vts+1, MCC_QEI+1
  bcf MCC_FLAG, QEIUP
  bcf MCC_FLAG, QEIDOWN
  
  ;movf MCC_Vts_z, W
  ;addlw 50 ; ne tourne que dans un sens pour le moment !!!!  BIZARZ !!!!!!!!!!!
  ;CPFSLT MCC_Vts; Compare f with WREG, Skip <
  ;movff MCC_Vts_z, MCC_Vts
  incf MCC_iMeas, F ; MCC.Pst += (long int)MCC.Vts;
  bsf MCC_FLAG, Update


  
  MOVFF MCC_iMeas, UxTx_str
  MOVFF MCC_Vts, UxTx_str+1
  MOVFF MCC_I, UxTx_str+2
  MOVFF MCC_I+1, UxTx_str+3
  ; Attention a ne pas oublier les \l+\n qui sont dans l'initialisation !!!!!!!!
  LFSR FSR0, UxTx_str
  clrf UxTx_iTx
  BSF PIE1, TXIE    ; Enable Interrupt
  
  retfie FAST
; ---------------------
; INTERRUPT PWM
IT_PWM
  BCF PIR3, PTIF	; Aquitement
  movlf PWM_PREL, PTPERL
  movlf PWM_PERH, PTPERH
  movlW PMW_DCmax_H
  CPFSLT MCC_DutyCycle+1 ; Compare f with WREG, Skip <
  BRA IT_PWM_lim
IT_PWM_nolim
  MOVFF MCC_DutyCycle, PDC0L
  MOVFF MCC_DutyCycle+1, PDC0H  
  retfie FAST
IT_PWM_lim
  movlw PMW_DCmax
  CPFSGT MCC_DutyCycle ; Compare f with WREG, Skip >
  BRA IT_PWM_nolim
  movlf PMW_DCmax, PDC0L
  movlf PMW_DCmax_H, PDC0H
  retfie FAST
;--- ZONE DE DEFINITION DES FONCTION ------------------------------------------

  

;--- PROGRAMME PRINCIPAL ------------------------------------------------------
main
; ------------------------- PIC  CONFIGURATION --------------------------------
; UART CONFIGURATION
    BSF TRISC, TRISC6	;
    BSF TRISC, TRISC7	;
    BSF BAUDCON, BRG16	; 16-bit
    CLRF SPBRGH
    movlf SPBRGVal, SPBRG 	; 115k2 Baud
    movlf 0x24, TXSTA 	; 8-bit, Transmission enabled, High Speed
    BSF TXSTA, TXEN	;
    BSF RCSTA, CREN	; Enable Reception
    BSF RCSTA, SPEN	; Enable UART module
    BCF IPR1, TXIP	; Low priority
    BCF IPR1, RCIP	; Low priority
    BCF PIE1, TXIE	; Disable des interruptions sur TX
;    BSF PIE1, RCIE	; Autorisaiton des interruptions sur RX
    CLRF UxTx_go
    CLRF UxTx_size
    CLRF UxTx_iTx
    LFSR FSR0, UxTx_str
    movlb 1
    movlf  0x0D, UxTx_str+4 ; \r : CR : Carrige Return
    movlf 0x0A, UxTx_str+5  ; \n : LF : Line Feed
    movlb 0
; ---------------------
; ADC CONFIGURATION
    BSF ADCON0, ADON;
    BCF ADCON0, ACONV	; Single-Shot mode
    BCF ADCON0, ACSCH	; Single Channel mode enabled, Multi-Channel mode disabled
    BCF ADCON1, VCFG0	; VREF+ = AVDD, VREF- = AVSS
    BCF ADCON1, VCFG1
    BCF ADCON1, FIFOEN ;	// FIFO is disabled
    BSF TRISA, TRISA0	; TRIS =1
    BSF ANSEL0, ANS0	; Analog Input bits => 1 : Analog input
    BCF ADCHS, GASEL0	; Group A Select bits
    BCF ADCHS, GASEL1	; 00 = AN0
    BCF ADCON2, ADCS0   ; A/D Conversion Clock Select bits
    BSF ADCON2, ADCS1	; FOSC/64
    BSF ADCON2, ADCS2	; ADCS = 0b110
    BSF ADCON2, ADFM	; A/D Result Format Select : Right justified
    BSF ADCON2, ACQT0	; A/D Acquisition Time Select bits : 2 TAD
    movlf B'10000', ADCON3; Power Control PWM module rising edge starts A/D sequence
    BSF PIE1, ADIE	; Enables the A/D interrupt
    BCF PIR1, ADIF	; Clear A/D interrupt flag
    BSF IPR1, ADIP	; High priority A/D Converter Interrupt
; ---------------------
; PWM CONFIGURATION
    CLRF PTCON0		; 1:1 Postscale
			; 1:1 prescale : FOSC/4
			; PWM time base operates in a Free-Running mode
			; Edge-Aligned PWM
    movlf PWM_PREL, PTPERL
    movlf PWM_PERH, PTPERH
    movlf PWM_SEVTCMPL, SEVTCMPL
    movlf PWM_SEVTCMPH, SEVTCMPH
    BCF PWMCON1, SEVTDIR; Special Event Trigger on the upward counting
    BSF PWMCON1, SEVOPS0; 1:10 Postscale => 9=0b1001
    BCF PWMCON1, SEVOPS1; 1:1  Postscale => 0=0b0000
    BCF PWMCON1, SEVOPS2
    BSF PWMCON1, SEVOPS3
    CLRF PDC0L		; Duty cycle INITIAL
    CLRF PDC0H
    BCF PWMCON1, UDIS	; Enable : Updates from Duty Cycle and Period Buffer
    BSF PWMCON0, PWMEN0	; PWM1 pin is enabled for PWM output
    BCF PWMCON0, PWMEN1
    BCF PWMCON0, PWMEN2
    BCF PWMCON0, PMOD0	; PWM I/O pin pair (PWM0, PWM1) is in the Independent mode
    BSF PTCON1, PTEN	; Enable PWM Time Base Timer
    BSF IPR3, PTIP	; High priority : PWM Interrupt Priority
    BSF PIE3, PTIE	; Enable PWM Interrupt
; ---------------------
; QEI CONFIGURATION
    BSF TRISA, TRISA2	; TRIS =1 : entree
    BSF TRISA, TRISA3	; TRIS =1 : entree
    BSF TRISA, TRISA4	; TRIS =1 : entree
    BCF ANSEL0, ANS2	; Analog Input bits => 0 : Digital input
    BCF ANSEL0, ANS3	; Analog Input bits => 0 : Digital input
    BCF ANSEL0, ANS4	; Analog Input bits => 0 : Digital input
    BCF QEICON, VELM	; Velocity mode enabled
    BCF QEICON, PDEC0	; Velocity Pulse Reduction Ratio 1:1
    BCF QEICON, PDEC1
    BSF QEICON, QEIM0	; QEI enabled in 4x Update mode
    BCF QEICON, QEIM1	; INDX resets the position counter
    BSF QEICON, QEIM2
    movlf POSCNT_MAX_H, MAXCNTH; MAXCNT = 500*4 = 0b11111010000
    movlf POSCNT_MAX_L, MAXCNTL
    CLRF POSCNTL
    CLRF POSCNTH
    BSF DFLTCON, FLT3EN	; Enabled Noise Filter Output on QEB input
    BSF DFLTCON, FLT2EN	; Enabled Noise Filter Output on QEA input
    BSF DFLTCON, FLT1EN	; Enabled Noise Filter Output on INDX input
    BCF DFLTCON, FLTCK2	; Noise Filter Clock Divider Ratio bits
    BSF DFLTCON, FLTCK1 ; 011 = 1:16
    BSF DFLTCON, FLTCK0
    BSF IPR3, IC2QEIP	; High priority : QEI Interrupt
    BSF PIE3, IC2QEIE	; Enables QEI Interrupt
; ---------------------
; TIMER0 CONFIGURATION
    BCF T0CON, T016BIT	; Configured as a 16-bit timer/counter
    BCF T0CON, T0CS	; Internal clock (FOSC/4)
    BCF T0CON, T0PS0	; 1:128 Prescale value
    BSF T0CON, T0PS1
    BSF T0CON, T0PS2
    BCF T0CON, PSA	; Enable Prescale
    movlf TIMER0_RELOADL, TMR0L
    movlf TIMER0_RELOADH, TMR0H
    BCF INTCON2, TMR0IP	; High priority TMR0 Overflow Interrupt
    BCF INTCON, TMR0IF	; TMR0 interrupt flag
    BSF INTCON, TMR0IE	; Enables the TMR0 overflow interrupt
    CLRF Time_s
    CLRF Time_s+1
    CLRF Time_Test
    CLRF Time_Test+1
    BSF T0CON, TMR0ON ; Enables Timer0
; ---------------------
    BCF TRISB, TRISB5
    BCF TRISB, TRISB4
; Ajustement des I_O pour les signaux FORWARD_REVERSE/Reverse et BRAKE
    BCF TRISB, TRISB0	; TRIS =0 : sortie => FORWARD_REVERSE
    BCF TRISB, TRISB2	; TRIS =0 : sortie => BRAKE
    BCF FORWARD_REVERSE
    BCF PORTB, RB2	; Disable BRAKE <=> PORTBbits.RB2 = 0
; Initialisation des variables
    CLRF MCC_iMeas
    CLRF MCC_I
    CLRF MCC_I_C
    CLRF MCC_Vts
    CLRF MCC_Vts+1
    CLRF MCC_Vts_C
    CLRF MCC_Vts_C+1
    BCF MCC_FLAG, Update
    CLRF MCC_QEI
    CLRF MCC_QEIH
    CLRF MCC_Pst
    CLRF MCC_Pst+1
    movlf 250, MCC_DutyCycle
    CLRF MCC_DutyCycle+1
    ;movlf PMW_DCmax, MCC_DutyCycle
    ;movlf PMW_DCmax_H, MCC_DutyCycle+1
    
    
    CLRF Corr_eI_z
    CLRF Corr_eI_z+1
    CLRF Corr_uI
    CLRF Corr_uI+1
; ---------------------  
   
    
; INTERRUPT CONFIGURATION
    BSF RCON, IPEN	; Enable priority levels on interrupts
    BSF INTCON, GIEH	; High Interrupt Enable
    BSF INTCON, GIEL	; Low Interrupt Enable bit
; ---------------------  
    
    
    
    
    clrf Corr_KI ; nombre a virgule fixe 0<= Corr_KI <1
    clrf Corr_KD ; nombre a virgule fixe 0<= Corr_KI <1
    movlf 3, Corr_KP ; nombre entier fixe 0<= Corr_KI <= 255
    
;--- Boucle infinie ---
boubleinf
  BTFSS MCC_FLAG, Update
  bra boubleinf
  
  
  ; Calcul de l'erreur
  movff MCC_Vts_C, MCC_Vts_Err
  movf  MCC_Vts, W
  SUBWF MCC_Vts_Err,F ; (f)-(w) -> (f)
  
  
  MOVFF MCC_Vts_Err,MCC_DutyCycle_Temp
  CLRF MCC_DutyCycle_Temp+1
  
  ;  MCC_Vts_Err * Corr_KI -> PRODH:PRODL
  MOVF  Corr_KI, W
  BZ noCorrI
  MULWF MCC_Vts_Err	    ; ARG1 * ARG2 -> PRODH:PRODL
  BTFSC MCC_Vts_Err, 7    ; Test Sign Bit
  SUBWF PRODH, F    ; PRODH = PRODH - ARG1
  MOVF  MCC_Vts_Err, W
  
  ; PRODH:PRODL + Corr_eI_z + Corr_uI -> Corr_uI
  add16 PRODL,Corr_uI
  add16 Corr_eI_z,Corr_uI
  MOVFF PRODL, Corr_eI_z
  MOVFF PRODH, Corr_eI_z+1
  
  
  
  
  
  
  
  ; MCC_Vts_Err + Corr_uI -> MCC_DutyCycle_Temp
  add16 Corr_uI,MCC_DutyCycle_Temp
  
noCorrI
  ;  MCC_DutyCycle_Temp * Corr_KP -> PRODH:PRODL
  MOVF  Corr_KP, W
  MULWF MCC_DutyCycle_Temp+1	    ; ARG1 * ARG2 -> PRODH:PRODL
  BTFSC MCC_DutyCycle_Temp+1, 7    ; Test Sign Bit
  SUBWF PRODH, F    ; PRODH = PRODH - ARG1
  
  ;MOVFF PRODL, MCC_DutyCycle
  ;MOVFF PRODH, MCC_DutyCycle+1
  movff MCC_Vts, MCC_Vts_z
  bcf MCC_FLAG, Update
  ; -------------------
  bra boubleinf

  END      ; Fin
