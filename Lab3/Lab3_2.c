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

volatile int scanKeypad = 0;

// ******************************************************************************************* //

int main(void)
{
	//char key;
	char firstDigit1, firstDigit2, secondDigit1, secondDigit2;
	char operator;
	char resultStr[8];
	//char decResultStr[7];
	int number1 = 0, number2 = 0;
	int wholeResult = 0, /*decResult = 0,*/ result = 0;
	int remainder = 0;
	int entry = 0;
	int pressed = 0;
	int i = 0, count = 0;
	int wholeNumAdded = 0;
	//int digitDivisor = 0;
	//int removed = 0;

	// TODO: Initialize and configure IOs, LCD (using your code from Lab 2),
	// UART (if desired for debugging), and any other configuration that are needed.

	LCDInitialize();
	KeypadInitialize();
	LCDClear();

	// TODO: Initialize scanKeypad variable.

	// test print
	//LCDPrintChar('0');

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

		if( IFS1bits.CNIF == 1 ){//   scanKeypad == 1 ) {
			if(!pressed){
				switch(entry){
					case 0: firstDigit1 = KeypadScan();  //key = KeypadScan();
							if( firstDigit1 != -1 && firstDigit1 != '*' && firstDigit1 != '/' 
								&& firstDigit1 != '+' && firstDigit1 != '-' ) {
								LCDMoveCursor(0,0);
								LCDPrintChar(firstDigit1);
								entry++;
							}
							break;
					case 1: secondDigit1 = KeypadScan();
							if( secondDigit1 != -1 && secondDigit1 != '*' && secondDigit1 != '/' 
								&& secondDigit1 != '+' && secondDigit1 != '-' ) {
								LCDMoveCursor(0,1);
								LCDPrintChar(secondDigit1);
								entry++;
							}
							break;
					case 2: operator = KeypadScan();
							if( operator != -1 && (operator == '*' || operator == '/' 
								|| operator == '+' || operator == '-') ) {
								LCDMoveCursor(0,2);
								LCDPrintChar(operator);
								entry++;
							}
							break;
					case 3: firstDigit2 = KeypadScan();  //key = KeypadScan();
							if( firstDigit2 != -1 && firstDigit2 != '*' && firstDigit2 != '/' 
								&& firstDigit2 != '+' && firstDigit2 != '-' ) {
								LCDMoveCursor(0,3);
								LCDPrintChar(firstDigit2);
								entry++;
							}
							break;
					case 4: secondDigit2 = KeypadScan();
							if( secondDigit2 != -1 && secondDigit2 != '*' && secondDigit2 != '/' 
								&& secondDigit2 != '+' && secondDigit2 != '-' ) {
								LCDMoveCursor(0,4);
								LCDPrintChar(secondDigit2);
								entry++;
							}
							break;
				}
				pressed = 1;
			}
			else pressed = 0;

			if(entry == 5){
				number1 = 10 * (firstDigit1-'0') + (secondDigit1-'0');
							number2 = 10 * (firstDigit2-'0') + (secondDigit2-'0');
							switch(operator){
								case '*': result = number1 * number2;
										  LCDMoveCursor(1,0);
										  sprintf(resultStr, "%d", result);
										  LCDPrintString(resultStr);
										  break;
								case '/': if (number2 == 0){
											  LCDMoveCursor(1,0);
											  LCDPrintString("Div by 0");
										  }		  
										  else{
											  result = number1 / number2;
											  remainder = number1 % number2;//(10000*(number1 % number2)) /number2;
											  LCDMoveCursor(1,0);
											  //resultStr = "       ";
											 // if(remainder != 0){
											  wholeNumAdded = 0;
											  count = 0;
											  if(remainder > 0){
												  for(i=0; i<8; i++){
													  if(result >= 10 && !wholeNumAdded){
														  resultStr[0] = (result/10) + '0';
														  //if(result - 10 >= 40) resultStr[1] = (result-40) + '0';
														  //else if(result - 10 >= 30) resultStr[1] = (result - 30) + '0';
														  //else if(result - 10 >= 20) resultStr[1] = (result - 20) + '0';
														  //else if(result - 10 >= 10) resultStr[1] = (result - 10) + '0';
														  resultStr[1] = (result - (result/10)*10) + '0';
														  resultStr[2] = '.';
														  wholeNumAdded = 1;
														  i = i + 3;
													  }
													  else if(result < 10 && !wholeNumAdded){
														  resultStr[0] = result + '0';
														  resultStr[1] = '.';
														  wholeNumAdded = 1;
															  i = i + 2;
													  }
													  if(remainder != 0 && count < 4){
													  	  resultStr[i] = ((remainder * 10) / number2) + '0';
													  	  remainder = (remainder * 10) % number2;
														  count++;
													  }
													  else resultStr[i] = ' ';
												  }
												  resultStr[8] = ' ';
												  LCDPrintString(resultStr);
											  }
											  	 // LCDPrintChar(wholeResult+'0');
												 // LCDMoveCursor(1,1);
												 // LCDPrintChar('.');
												 // digitDivisor = 1000;
											     // for(i=0; i<4; i++){
										         //     decResultStr[i] = (decResult/digitDivisor)+'0';
												//	  decResult = decResult - (decResult/digitDivisor)*digitDivisor;
												//	  digitDivisor = digitDivisor / 10;
												 // }
												  /*i = 3;
												  removed = 1;
												  while(i >= 0 && removed == 1){
													  if(decResultStr[i] == '0'){
														  decResultStr[i] = ' ';
														  removed = 1;
													  }
													  else removed = 0;
													  i--;
												  }
												  LCDMoveCursor(1,2); */
											  //}
											  else{
												  sprintf(resultStr, "%d", wholeResult);
											  	  LCDPrintString(resultStr);
											  }
										  }	
										  break;
								case '+': result = number1 + number2;
										  LCDMoveCursor(1,0);
										  sprintf(resultStr, "%d", result);
										  LCDPrintString(resultStr);
										  break;
								case '-': result = number1 - number2;
										  LCDMoveCursor(1,0);
										  sprintf(resultStr, "%d", result);
										  LCDPrintString(resultStr);
										  break;
							}
			}
			IFS1bits.CNIF = 0;   //scanKeypad = 0;
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
/* void _ISR _CNInterrupt(void)
{
	// TODO: Clear interrupt flag
	IFS1bits.CNIF = 0;

	//Once a change is detected scanKeypad will go to 1 so that KeypadScan() will run
	scanKeypad = 1;

	// TODO: Detect if *any* key of the keypad is *pressed*, and update scanKeypad
	// variable to indicate keypad scanning process must be executed.
} */

// ******************************************************************************************* //
