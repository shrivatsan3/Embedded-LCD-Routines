/****** ASEN 4/5519 Lab 6 ******************************************************
 * Author: YOUR NAME HERE
 * Date  : DATE HERE
 *
 * Description
 * "Blinky"
 * The following occurs forever:
 *      RD4 blinks: 1s +/- 10ms ON, then 1s +/- 10ms OFF
 * 	On rollover of TMR0, update LCD to -Rollover-
 *
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

 
#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define _XTAL_FREQ 16000000 // Used for _delay functions in XC8

/* Notice the config settings below are the same that we have used in previous
 labs. We are NOT setting all configuration bits found in the PIC18F87K22 datasheet
 in TABLE 28-1: CONFIGURATION BITS AND DEVICE IDs. You will get several warning 
 messages (1311) at compile time that the remaining config words are not set and 
 the default settings will be used. This could be added here if needed. */

#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
unsigned int Alive_count = 0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR0handler(void);     // Interrupt handler for TMR1

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
      while(1) {                // Repeat mainline loop forever
      
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP0 for compare if desired, and enables LoPri 
 * interrupts for both.
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    //TRISD  = //*** STUDENTS SPECIFY VALUE ***;
    TRISD = 0b00000000; //*** STUDENTS SPECIFY VALUE ***;
    //TRISC  = //*** STUDENTS SPECIFY VALUE ***;
    //LATC = //*** STUDENTS SPECIFY VALUE ***;
    TRISB = 0x00; 
   
    InitLCD();// Init LCD
    __delay_ms(10); // Delay 10ms

    LCD_DATA_LAT = 0x00;
    
    // Initializing TMR0
    //T0CON = //*** STUDENTS SPECIFY VALUE ***;
    T0CON = 0b00000101;              // using a 1:64 prescaler in 16-bit mode   
    TMR0L = 0;                      // Clearing TMR0 registers
    TMR0H = 0;                      // Clear high register if used


    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
}



/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
void __interrupt() HiPriISR (void)
{
   // ISR code would go here 
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR1IF and CCP1IF are clear.
 ******************************************************************************/
void __interrupt(low_priority) LoPriISR (void)
{
    while(1) {
        if( INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }
        break;
    }
}    

/******************************************************************************
 * TMR1handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
void TMR0handler() {

// STUDENTS ADD CODE HERE TO IMPLEMENT THE TIMER SERVICE ROUTINE TO MAKE THE LED BLINK
// Note that the variable Alive_count has been declared with global scope and may be used
// in this handler if desired.

    LATDbits.LD4 ^= 1;      // Toggle RD4
    if(Alive_count == 0){       // if Alive_count is 0 => first roll over
        char rollover[] = {0x80,'-','R','o','l','l','o','v','e','r','-',0x00};
        DisplayV(rollover);     // Display -Rollover-
        Alive_count++;    // increment Alive_count, so we dont have to call DisplayV() routine on every interrupt)
    }
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}
