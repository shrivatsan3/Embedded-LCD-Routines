;;;;;;; template for ASEN 4519/5519 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	Created: Shrivatsan K Chari	
;	
;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
;   Initial
;
;;;;;;; Assembler directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        list  P=PIC18F87K22, F=INHX32, C=160, N=0, ST=OFF, MM=OFF,R=DEC, X=ON
        #include p18f87k22.inc
;		After MPLAB X all configuration bits are set in the code
;		Use mplab help to understand what these directives mean
		CONFIG	FOSC = HS1
		CONFIG	PWRTEN = ON, BOREN = ON, BORV = 1, WDTEN = OFF
		CONFIG	CCP2MX = PORTC, XINST = OFF

;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        
	
	cblock  0x000		; Beginning of Access RAM
        WREG_TEMP		; Temp variables used in Low Pri ISR
        STATUS_TEMP
	CCP_BUFFER_L
	CCP_BUFFER_H
	CCP3_BUFFER_L
	CCP3_BUFFER_H
	ON_COUNT_H
	ON_COUNT_L
	OFF_COUNT_H
	OFF_COUNT_L
	TIME_INCREMENT_H
	TIME_INCREMENT_L
	HARD_UPPER_LIMIT_H
	HARD_UPPER_LIMIT_L
	HARD_LOWER_LIMIT_H
	HARD_LOWER_LIMIT_L
	DIR_RPG			; Direction of RPG
	RPG_TEMP		; Temp variable used for RPG state
	OLDPORTD		; Used to hold previous state of RPG
	COUNT
	CURSOR
	BYTE
	BYTESTR:3		; Display string for binary version of BYTE
	LED_COUNT
	NUMBERS1
        NUMBERS2
	endc

BLINK_COUNT equ 62500
ONE_MS    equ 500
NINETEEN_MS   equ 9500
INCREMENT   equ	5
HARD_UPPER  equ	1000
Bignum  equ   100
COUNTER	equ   0x20
WAIT_MS	equ 65536-40000	
;;;;;;; Macro definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  
MOVLF   macro  literal,dest		; Move literal to file macro
        movlw  literal
        movwf  dest,0
        endm
	
;; POINT taken from Reference: Peatman CH 7 LCD
POINT   macro  stringname		; Load a string into table pointer
        MOVLF  high stringname, TBLPTRH	; Used to put values in program memory
        MOVLF  low stringname, TBLPTRL
        endm

DISPLAY macro  register         ; Displays a given register in binary on LCD
        movff  register,BYTE
        call  ByteDisplay
        endm	
	
;;;;;;; Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        org  0x0000         ; Reset vector
        nop		    ; No operation, wastes one instruction cycle
        goto  Mainline	    ; Send program to Mainline code

        org  0x0008         ; High priority interrupt vector
        goto  $             ; $ returns code to the current program counter

        org  0x0018	    ; Low priority interrupt vector
        goto  LoPriISR     ; Returns. Only here to show code structure.

;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Mainline
       rcall  	Initial     ; Initialize everything
   
loop	
 
       rcall RPG
       rcall LCD_Update
       bra loop

;;;;;;; Initial subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine performs all initializations of variables and registers.

Initial
	
		
	MOVLF   B'11000000',TRISB       ; Set I/O for PORTB
        MOVLF   B'00000000',LATB        ; Initialize PORTB
        MOVLF   B'00000000',TRISC       ; Set I/0 for PORTC
        MOVLF   B'00000000',LATC        ; Initialize PORTC
	MOVLF   B'00000011',TRISD	; Set I/O for PORTD
	MOVLF   B'00000000',LATD	; Initialize PORTD
	
	MOVLF   B'00001000',T0CON       ; Set up Timer0 for a delay of 10 ms
        MOVLF   high WAIT_MS,TMR0H  ; Writing value to timer allowed in initialization routine
        MOVLF   low WAIT_MS,TMR0L	

	
	rcall   InitLCD                 ; Initialize LCD
	rcall   Wait10ms		; 10 ms delay subroutine
	
	POINT   LCDs                    
        rcall   DisplayC		; Display ASEN5519
	
	movlw	0x20
	iorwf	LATD,1	 ; Turn ON RD5
		
	call	Wait1sec ; call subroutine to wait 1 second
	
	movlw	0xcf
	andwf	LATD,1	 ; Turn OFF RD5
	
	movlw	0x40
	iorwf	LATD,1	 ; Turn ON RD6
		
	call	Wait1sec ; call subroutine to wait 1 second
	
	movlw	0xbf
	andwf	LATD,1	 ; Turn OFF RD6
	
	
	movlw	0x80
	iorwf	LATD,1   ; Turn ON RD7
		
	call	Wait1sec ; call subroutine to wait 1 second
	
	movlw	0x7f
	andwf	LATD,1	 ; Turn OFF RD7
	
	
	POINT   LCDs1                    
        rcall   DisplayC		; Display PW
	POINT   LCDs2                    
        rcall   DisplayC		; Display .
	POINT   LCDs3                    
        rcall   DisplayC		; Display ms
	
	
	MOVLF high INCREMENT, TIME_INCREMENT_H	    ; INCREMENT is a count which can increase or decrease pulse width by 0.01 ms
	MOVLF low INCREMENT, TIME_INCREMENT_L

	MOVLF	high HARD_UPPER, HARD_UPPER_LIMIT_H	; Hard upper limit is a count for 2 ms limit
	MOVLF	low HARD_UPPER, HARD_UPPER_LIMIT_L

	MOVLF	high ONE_MS, HARD_LOWER_LIMIT_H		; Hard lower limit is a count for 1 ms limit
	MOVLF	low ONE_MS, HARD_LOWER_LIMIT_L
	
	bsf  RCON,IPEN			; Enable priority levels
	bcf  IPR3,CCP1IP		; and to ECCP1 interrupts
	bcf  IPR4,CCP3IP		; and to ECCP2 interrupts
	
	bsf  INTCON,GIEL		; Enable low-priority interrupts to CPU
        bsf  INTCON,GIEH		; Enable all interrupts
	
	bsf  PIE3,CCP1IE		; Enable ECCP1 interrupts
	bsf  PIE4,CCP3IE		; Enable ECCP2 interrupts
	
	MOVLF  B'00001010',CCP1CON	; Select compare mode
	
	MOVLB 0X0F			; Set BSR to bank F for SFRs outside of access bank				
        movlw	B'00001010'
	movwf	CCP3CON,1
	
	MOVLW  B'01000000'		; NOTE: Macro cannot be used, does not handle when a=1
	MOVWF CCPTMRS0,1		; Set TMR1 for use with ECCP1, a=1!!
	
	MOVLF   high ONE_MS,ON_COUNT_H       ; Initialize with ON pulse = 1 ms and OFF pulse = 19 ms
        MOVLF   low ONE_MS,ON_COUNT_L
	MOVLF   high NINETEEN_MS,OFF_COUNT_H
	MOVLF   low NINETEEN_MS,OFF_COUNT_L
		
	MOVFF   ON_COUNT_H,CCPR1H      ; Since initially RC2 is OFF, add ON COUNT to ECCP1
        MOVFF   ON_COUNT_L,CCPR1L
        MOVFF   ON_COUNT_H,CCP_BUFFER_H   ; Keep track of pulse width, which changes with RPG rotation
        MOVFF   ON_COUNT_L,CCP_BUFFER_L
	
        MOVLF   high BLINK_COUNT,CCP3_BUFFER_H    
        MOVLF   low BLINK_COUNT,CCP3_BUFFER_L
        
	MOVLF	2, LED_COUNT
	
	MOVLF	0, NUMBERS1
	MOVLF	0, NUMBERS2
	
	MOVLF  B'00110011',T1CON	; 16 bit timer and Turn on TMR1
	MOVLF  B'00110011',T3CON	; 16 bit timer and Turn on TMR3

	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	
Wait10ms
					; Subroutine taken from Lab4_Example code  
	bsf     T0CON,7                 ; Turn on Timer0
roll	btfss 	INTCON,TMR0IF           ; Read Timer0 rollover flag and ...
        bra     roll	                ; Loop if timer has not rolled over
        bcf  	INTCON,TMR0IF           ; Clear Timer0 rollover flag
	bcf     T0CON,7                 ; Turn off Timer0
	MOVLF   high WAIT_MS,TMR0H  
        MOVLF   low WAIT_MS,TMR0L	

	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
Wait1sec
	MOVLF	100,COUNT   ;	 Call Wait10ms to get 1 sec delay
one_sec	rcall	Wait10ms    ;	 Only used in initialisation 
	DECF	COUNT,1
	bnz	one_sec
	return
	
;;;;;;; RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Credit: This subroutine modified from Peatman book Chapter 8 - RPG
; This subroutine decyphers RPG changes into values of DIR_RPG of 0, +1, or -1.
; DIR_RPG = +1 for CW change, 0 for no change, and -1 for CCW change.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
RPG
       
	clrf   DIR_RPG          ; Clear for "no change" return value.
        movf   PORTD,W          ; Copy PORTD into W.
        movwf  RPG_TEMP         ;  and RPG_TEMP.
        xorwf  OLDPORTD,W       ; Check for any change?
        andlw  B'00000011'      ; Masks just the RPG pins          
        bz	L8		; If zero, RPG has not moved, ->return
				; But if the two bits have changed then...
				; Form what a CCW change would produce.          	
	rrcf OLDPORTD,W		; Rotate right once into carry bit   
	bnc L9			; If no carry, then bit 0 was a 0 -> branch to L9
        bcf  WREG,1		; Otherwise, bit 0 was a 1. Then clear bit 1
				; to simulate what a CCW change would produce
        bra L10			; Branch to compare if RPG actually matches new CCW pattern in WREG
L9
        bsf  WREG,1		; Set bit 1 since there was no carry
				; again to simulate what CCW would produce
L10				; Test direction of RPG
        xorwf  RPG_TEMP,W       ; Did the RPG actually change to this output?
        andlw  B'00000011'      ; Masks the RPG pins  
        bnz L11			; If not zero, then branch to L11 for CW case
        decf DIR_RPG,F          ; If zero then change DIR_RPG to -1, must be CCW. 
        bra	L8		; Done so branch to return
L11				; CW case 
        incf DIR_RPG,F		; Change DIR_RPG to +1 for CW.
L8
        rcall  UPDATE_PW
	movff  RPG_TEMP,OLDPORTD       	; Save RPG state as OLDPORTD
        return

;;;;;;;;;;; PWM duty cycle update Subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
;This subroutine changes the duty cycle depending on the direcction of rotation 
;of RPG and by the amount it was rotated
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
UPDATE_PW
	
	movlw	0xff
	andwf	DIR_RPG
	bz	NO_CHANGE	;No need to change duty cycle if RPG wasn't rotated
	
	addwf	DIR_RPG, 0	   ; Clockwise rotation => Increase duty cycle
	bz	INCREASE_PW	   ; Counter-Clockwise rotation => Decrease duty cycle
	
	movf	HARD_LOWER_LIMIT_L, 0,0		; Check if ON pulse width is 1ms
	subwf	ON_COUNT_L, 0
	bz	NO_CHANGE			; Cant decrease further if ON Pulse width is 1ms			    
	
	movf	TIME_INCREMENT_L, 0,0		; Decrease ON count to shorten ON pulse width
	subwf	ON_COUNT_L, 1
	movf	TIME_INCREMENT_H, 0,0
	subwfb	ON_COUNT_H, 1
	
	movf	TIME_INCREMENT_L, 0,0	    ; Increase OFF count to lengthen OFF pulse width
	addwf	OFF_COUNT_L, 1
	movf	TIME_INCREMENT_H, 0,0
	addwfc	OFF_COUNT_H, 1
	
	decf	NUMBERS2,1
	bnn	NO_CHANGE
	MOVLF	9, NUMBERS2
	decf	NUMBERS1
	
	return

INCREASE_PW	
	
	movf	HARD_UPPER_LIMIT_L, 0,0	    ; Check if ON pulse width is 2ms
	subwf	ON_COUNT_L, 0		    ; Can't increase further if ON pulse width is 2ms
	bz	NO_CHANGE

	movf	TIME_INCREMENT_L, 0,0  ; Increase ON count to lengthen ON pulse width
	addwf	ON_COUNT_L, 1
	movf	TIME_INCREMENT_H, 0,0
	addwfc	ON_COUNT_H, 1
	
	movf	TIME_INCREMENT_L, 0,0	    ; Decrease OFF count to shorten OFF pulse width
	subwf	OFF_COUNT_L, 1
	movf	TIME_INCREMENT_H, 0,0
	subwfb	OFF_COUNT_H, 1
	
	
	incf	NUMBERS2, 1
	movlw	10
	subwf	NUMBERS2, 0
	bnz	NO_CHANGE
	MOVLF	0,NUMBERS2
	incf	NUMBERS1
			
NO_CHANGE 
	return

LCD_Update
	
	movf	HARD_LOWER_LIMIT_L, 0,0
	subwf	ON_COUNT_L, 0
	bnz	CHECK_UL
	
	MOVLF	0xC2, CURSOR	    ; Since Hard lower limit,
	MOVLF	1, BYTE		    ; Display 1.00ms 
	rcall	ByteDisplay
	MOVLF	0xC4, CURSOR
	MOVLF	0, BYTE		    
	rcall	ByteDisplay
	MOVLF	0xC5, CURSOR
	MOVLF	0, BYTE
	rcall	ByteDisplay
	
	bra	UPDATED

CHECK_UL movf	HARD_UPPER_LIMIT_L, 0,0
	subwf	ON_COUNT_L, 0
	bnz	LCD_UP
	
	MOVLF	0xC2, CURSOR		    ; Since Hard upper limit
	MOVLF	2, BYTE			    ; Display 2.00ms
	rcall	ByteDisplay
	MOVLF	0xC4, CURSOR
	MOVLF	0, BYTE
	rcall	ByteDisplay
	MOVLF	0xC5, CURSOR
	MOVLF	0, BYTE
	rcall	ByteDisplay
	
	bra	UPDATED

LCD_UP	
	
	MOVLF	0xC2, CURSOR
	MOVLF	1, BYTE
	rcall	ByteDisplay
	MOVLF	0xC4, CURSOR
	movff	NUMBERS1, BYTE
	rcall	ByteDisplay
	MOVLF	0xC5, CURSOR
	movff	NUMBERS2, BYTE
	rcall	ByteDisplay
		
UPDATED	return
	
LoPriISR
		
	movff  STATUS,STATUS_TEMP	; Set aside STATUS and WREG
        movwf  WREG_TEMP
	
	btfss PIR3,CCP1IF		; Check for ECCP1 interrupt
	bra  L1
	rcall ECCP1_ISR

L1	btfss PIR4,CCP3IF		; Check for ECCP2 interrupt
	bra  L2
	rcall ECCP2_ISR
	
L2	 movf  WREG_TEMP,W		; Restore WREG and STATUS
        movff  STATUS_TEMP,STATUS
        
	retfie
	
ECCP1_ISR
	
	bcf    PIR3,CCP1IF		; Clear ECCP1 intterupt flag
	btfsc	LATC, 2			; Toggle RC2
	bra	LOAD_OFF_COUNT		; If RC2 was ON previously, Switch it OFF and add OFF COUNT TO ECCP1    
	
        btg	LATC, 2
	
	movf	ON_COUNT_L, 0,0		; RC2 was OFF previously, switch it ON and add ON COUNT to ECCP1 
	addwf	CCP_BUFFER_L, 1
	movf	ON_COUNT_H, 0,0
	addwfc	CCP_BUFFER_H, 1
	
	movff  CCP_BUFFER_H,CCPR1H
	movff  CCP_BUFFER_L,CCPR1L
       
	return			; Return from interrupt, reenabling GIEL

	
LOAD_OFF_COUNT
	
	btg	LATC, 2

	movf	OFF_COUNT_L, 0, 0 ; Get delay from OFF COUNT
	addwf	CCP_BUFFER_L, 1
	movf	OFF_COUNT_H, 0, 0 
	addwfc	CCP_BUFFER_H, 1
	
	movff  CCP_BUFFER_H,CCPR1H
	movff  CCP_BUFFER_L,CCPR1L
        return			; Return from interrupt, reenabling GIEL
	
ECCP2_ISR
	
	bcf    PIR4,CCP3IF
	decf	LED_COUNT
	bnz	AHEAD
	btg	LATD, 4
	MOVLF	2, LED_COUNT
	
AHEAD	movlw	low BLINK_COUNT
	addwf	CCP3_BUFFER_L, 1
	movlw	high BLINK_COUNT
	addwfc	CCP3_BUFFER_H, 1
	
	movff  CCP3_BUFFER_H,CCPR3H
	movff  CCP3_BUFFER_L,CCPR3L
        
	return

	
	
	
	

;;;;;;; InitLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; InitLCD - modified version of subroutine in Reference: Peatman CH7 LCD
; Initialize the LCD.
; First wait for 0.1 second, to get past display's power-on reset time.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
InitLCD
        
	rcall  Wait10ms     ; Call wait10ms 
        bcf     LATB,4	    ; RS=0 for command mode to LCD
        POINT   LCDstr      ; Set up table pointer to initialization string
        tblrd*              ; Get first byte from string into TABLAT
Loop4
	clrf LATB	    ; First set LATB to all zero	
        bsf   LATB,5	    ; Drive E high - enable LCD
	movf TABLAT,W	    ; Move byte from program memory into working register
	andlw 0xF0	    ; Mask to get only upper nibble
	swapf WREG,W	    ; Swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	iorwf PORTB,W	    ; Mask with the rest of PORTB to retain existing RB7:RB4 states
	movwf LATB	    ; Update LATB to send upper nibble
        bcf   LATB,5        ; Drive E low so LCD will process input
        rcall Wait10ms      ; Wait ten milliseconds
	
	clrf LATB	    ; Reset LATB to all zero	    
        bsf  LATB,5         ; Drive E high
        movf TABLAT,W,0	    ; Move byte from program memory into working register
	andlw 0x0F	    ; Mask to get only lower nibble
	iorwf PORTB,W,0	    ; Mask lower nibble with the rest of PORTB
	movwf LATB,0	    ; Update LATB to send lower nibble
        bcf   LATB,5        ; Drive E low so LCD will process input
        rcall Wait10ms      ; Wait ten milliseconds
        tblrd+*             ; Increment pointer and get next byte
        movf  TABLAT,F      ; Check if we are done, is it zero?
        bnz	Loop4
        return
	
;;;;;;; T50 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; T50 modified version of T40 taken from Reference: Peatman CH 7 LCD
; Pause for 50 microseconds or 50/0.25 = 200 instruction cycles.
; Assumes 16/4 = 4 MHz internal instruction rate (250 ns)
; rcall(2) + movlw(1) + movwf(1) + COUNT*3 - lastBNZ(1) + return(2) = 200 
; Then COUNT = 195/3
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
T50
        movlw  195/3          ;Each loop L4 takes 3 ins cycles
        movwf  COUNT		    
L4
        decf  COUNT,F
        bnz	L4
        return
	
;;;;;;;;DisplayC subroutine;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; DisplayC taken from Reference: Peatman CH7 LCD
; This subroutine is called with TBLPTR containing the address of a constant
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position hex to ASCII.
; This subroutine expects a normal one-byte cursor-positioning code, 0xhh, and
; a null byte at the end of the string 0x00
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

DisplayC
        bcf   LATB,4		;Drive RS pin low for cursor positioning code
        tblrd*			;Get byte from string into TABLAT
        movf  TABLAT,F          ;Check for leading zero byte
        bnz	Loop5
        tblrd+*                 ;If zero, get next byte
Loop5
	movlw 0xF0
	andwf LATB,F		;Clear RB0:RB3, which are used to send LCD data
        bsf   LATB,5            ;Drive E pin high
        movf TABLAT,W		;Move byte from table latch to working register
	andlw 0xF0		;Mask to get only upper nibble
	swapf WREG,W		;swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	iorwf PORTB,W		;Mask to include the rest of PORTB
	movwf LATB		;Send upper nibble out to LATB
        bcf   LATB,5            ;Drive E pin low so LCD will accept nibble
	rcall T50
	
	movlw 0xF0
	andwf LATB,F		;Clear RB0:RB3, which are used to send LCD data
        bsf   LATB,5            ;Drive E pin high again
        movf TABLAT,W		;Move byte from table latch to working register
	andlw 0x0F		;Mask to get only lower nibble
	iorwf PORTB,W		;Mask to include the rest of PORTB
	movwf LATB		;Send lower nibble out to LATB
        bcf   LATB,5            ;Drive E pin low so LCD will accept nibble
        rcall T50               ;Wait 50 usec so LCD can process
	
        bsf   LATB,4            ;Drive RS pin high for displayable characters
        tblrd+*                 ;Increment pointer, then get next byte
        movf  TABLAT,F          ;Is it zero?
        bnz	Loop5
        return
	
;;;;;;; DisplayV subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; DisplayV taken from Reference: Peatman CH7 LCD
; This subroutine is called with FSR0 containing the address of a variable
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	

DisplayV
        bcf     LATB,4          ;Drive RS pin low for cursor positioning code
Loop6
	movlw 0xF0
	andwf LATB,F		;Clear RB0:RB3, which are used to send LCD data
        bsf   LATB,5            ;Drive E pin high
        movf INDF0,W		;Move byte from table latch to working register
	andlw 0xF0		;Mask to get only upper nibble
	swapf WREG,W		;swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	iorwf PORTB,W		;Mask to include the rest of PORTB
	movwf LATB		;Send upper nibble out to LATB
        bcf   LATB,5            ;Drive E pin low so LCD will accept nibble
	rcall T50
	
	movlw 0xF0
	andwf LATB,F		;Clear RB0:RB3, which are used to send LCD data
        bsf   LATB,5            ;Drive E pin high again
        movf INDF0,W		;Move byte from table latch to working register
	andlw 0x0F		;Mask to get only lower nibble
	iorwf PORTB,W		;Mask to include the rest of PORTB
	movwf LATB		;Send lower nibble out to LATB
        bcf   LATB,5            ;Drive E pin low so LCD will accept nibble
        rcall T50               ;Wait 50 usec so LCD can process
	  
        bsf   LATB,4            ;Drive RS pin high for displayable characters
        movf  PREINC0,W         ;Increment pointer, then get next byte
        bnz	Loop6
        return	

;;;;;;; ByteDisplay subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Display whatever is in BYTE as a binary number.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	
ByteDisplay
        lfsr    0,BYTESTR+1
string2
         movf	BYTE
	 iorlw	0x30                  ;Convert to ASCII
         movwf POSTDEC0               ; and move to string
         movf  FSR0L,W                ;Done?
         sublw low BYTESTR
        bnz	string2

        lfsr    0,BYTESTR              ;Set pointer to display string
        movff	CURSOR,BYTESTR           ;Add cursor-positioning code
        clrf    BYTESTR+2              ;and end-of-string terminator
        rcall   DisplayV
        return

;;;;;;; Constant strings ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

LCDstr  db  0x33,0x32,0x28,0x01,0x0c,0x06,0x00  ;Initialization string for LCD
LCDs    db  "\x80ASEN5519\x00"
LCDs1   db  "\xC0PW\x00"
LCDs2   db  "\xC3.\x00"   
LCDs3   db  "\xC6ms\x00"   
   
;;;;;;; End of Program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	end


	
