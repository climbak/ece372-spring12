// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>
#include "lcd.h"

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
		 BKBUG_ON & COE_ON & ICS_PGx1 &
		 FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
		 IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //

// Varible used to indicate that the current configuration of the keypad has been changed,
// and the KeypadScan() function needs to be called.

volatile char scanKeypad = '0';

// ******************************************************************************************* //

int main(void)
{
	char key;

	// TODO: Initialize and configure IOs, LCD (using your code from Lab 2),
	// UART (if desired for debugging), and any other configuration that are needed.

	LCDInitialize();
	KeypadInitialize();
	LCDClear();

	// TODO: Initialize scanKeypad variable.

	// test print
	LCDPrintChar('0');

	//IFS1bits.CNIF = 1;

	while(1)
	{
		
		// TODO: Add functionality for simple calculator that detects two distinct digit ('0' - '9')
		// presses for the first input, a distinct operation press for '+', '-', '*', or '/',
		// and two distinct digit ('0' - '9') presses for the second input.
		//
		// This input should be displayed on the first line of the LCD display as the user enters the
		// operations, e.g. "01+09 =".
		//
		// Notes:
		//        1. Only valid inputs should be allowed by the user such that all invalid inputs
		//           are ignored until a valid input is detected.
		//        2. The user must release all keys of the keypad before the following key press
		//           is processed. This is prevent invalid keypress from being processed if the
		//           users presses multiple keys simultaneously.
		//
		// As soon as the user correctly enters the second number, the result of the calculation
		// should be displayed on second line of the LCD display.

		if( IFS1bits.CNIF == 1 ){   //scanKeypad == '1' ) {
			key = KeypadScan();
			if( key != -1 ) {
				LCDMoveCursor(0,0);
				LCDPrintChar(key);
			}
			IFS1bits.CNIF = 0;   //scanKeypad = '0';
		}
	}
	return 0;
}

// ******************************************************************************************* //
// Defines an interrupt service routine that will execute whenever any enable
// input change notifcation is detected.
//
//     In place of _ISR and _ISRFAST, we can directy use __attribute__((interrupt))
//     to inform the compiler that this function is an interrupt.
//
//     _TCNInterrupt is a macro for specifying the interrupt for input change
//     notification.
//
// The functionality defined in an interrupt should be a minimal as possible
// to ensure additional interrupts can be processed.
/** void _ISR _CNInterrupt(void)
{
	// TODO: Clear interrupt flag
	IFS1bits.CNIF = 0;

	//Once a change is detected scanKeypad will go to 1 so that KeypadScan() will run
	scanKeypad = '1';

	// TODO: Detect if *any* key of the keypad is *pressed*, and update scanKeypad
	// variable to indicate keypad scanning process must be executed.
} */

// ******************************************************************************************* //
