// ******************************************************************************************* //
// File:         lab4.c   
// Date:         03-22-2012
// Authors:      Jon Englert, Ryan Courreges, Eric Therrio, Brian Suarez
//
// Description:  Control of two motors using a potentiometer. 
//
// Requirements: This software code requires the MPLAB C30Compiler
// ******************************************************************************************* //

#include "p24fj64ga002.h"
#include <stdio.h>

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings. 
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to 
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & 
          BKBUG_ON & COE_ON & ICS_PGx1 & 
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
// Defines to simply UART's baud rate generator (BRG) regiser
// given the osicllator freqeuncy and PLLMODE.

#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE         115200       
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 




int main(void)
{

// ******************************************************************************************* //
// ******************************************************************************************* //
// ******************************************************************************************* //
	//CHANGE THESE PARATEMETERS TO CHANGE THE MOTOR OPERATION

	
// ******************************************************************************************* //
	//CONFIGURATION OF ADC, PWM, and TIMERS
	
	//ADC Configuration settings
	//     ADON          = 0     (ADC off)
	//     FORM<1:0>     = 00    (integer output;  MAX = 1024;  0b0000 00dd dddd dddd)
	//     SSRC<2:0>     = 111   (Auto-sample)
	//     ASAM          = 0     (Auto-sample start if off)
	AD1CON1 = 0x00E0;
	//     VCFG<2:0>     = 000   (AVdd and AGND used for reference;  internal reference)
	//     CSCNA         = 0     (No scanning, select channel with AD1CHS)
	//     SMPI<3:0>     = 0000  (1 sample per conversion)
	//     BUFM			 = 0	 (Buffers hold output as 16-bit word)
	AD1CON2 = 0x0000;
	//     ADRC          = 0     (System clock)
	//     SAMC<4:0>     = 00001 (Tsmp = 1*Tad = 2;  Sample Period)
	//     ADCS<7:0>     = 00000001   (Tad = 1+1 = 2;  Conversion Period)
	AD1CON3 = 0x0101;
	//     Set RB3=AN5 (from potentiometer) to analog input
	AD1PCFG = 0xFFDF;
	TRISBbits.TRISB3 = 1;
	//     Set inputs to scan all off because not scanning
	AD1CSSL = 0x0000;
	//     Set channel for Mux A to AN5
	AD1CHS  = 0x0005;
	//     Clear interrupt flag
	IFS0bits.AD1IF = 0;
	//     Enable ADC interrupt
	IEC0bits.AD1IE = 1;
	
	
	//Frequency Selection Configuration
	//		
	
	
	//TIMER Configuration
	//     Clear Timer 2 value 
	TMR2 = 0;			
 	//     TON           = 0     (Start timer)
	//     TCKPS1<2:0>	 = 00    (Set timer prescaler to 1:1)
	//     TCS           = 0     (Internal clock;  Fosc/2)
	T2CON = 0x0000;
	//     Clear interrupt flag. 
	IFS0bits.T2IF = 0;		
	//     Enable interrupt
	IEC0bits.T2IE = 1;
// ******************************************************************************************* //
/*	//Configure SW1 to be an input that changes the setting of the motors
	//Sequence is IDLE - FORWARD - IDLE - REVERSE - IDLE - FORWARD
	
	//Set SW1 to input (RB5)
	TRISBbits.TRISB5 = 1;
	//Set change notification enable and pull up resistor for input
	CNPU2bits.CN27PUE = 1;
	CNEN2bits.CN27IE = 1;
	//Clear interrupt flag
	IFS1bits.CNIF = 1;
	//Enable ISR
	IEC1bits.CNIE = 1; */
// ******************************************************************************************* //


	AD1CON1bits.ADON = 1;
	T2CONbits.TON = 1;
	
	while (1)
	{
	AD1CON1bits.ASAM = 1;			//Start auto-sampling
	}
	
	return 0;
}


void _ISR _ADC1Interrupt (void)
{
	IFS0bits.AD1IF = 0;
	AD1CON1bits.ASAM = 0;				//Stop auto-sample
	AD1CON1bits.DONE = 0;

	//If potentiometer is in middle buffer holds 512 (0x0200) because in center of refeneces AVdd/AGND.
	if (MotorState == 1)
	{	
		if (ADC1BUF0 > 0x0200) 
		{
			RIGHTWF = (PWMCYCLES + 1);
			LEFTWF = (PWMCYCLES+1) - (((ADC1BUF0 + 1) >> (9-POW)) - (PWMCYCLES + 1));
		}
		else if (ADC1BUF0 < 0x0200)
		{
			RIGHTWF = (ADC1BUF0 + 1) >> (9-POW);
			LEFTWF = (PWMCYCLES+1);
		}
		else if (ADC1BUF0 == 0x0200)
		{
			RIGHTWF = (PWMCYCLES + 1);
			LEFTWF = (PWMCYCLES + 1);
		}
	}
	else if (MotorState == 3)
	{
		if (ADC1BUF0 > 0x0200) 
		{
			RIGHTWR = (PWMCYCLES + 1);
			LEFTWR = (PWMCYCLES+1) - (((ADC1BUF0 + 1) >> (9-POW)) - (PWMCYCLES + 1));
		}
		else if (ADC1BUF0 < 0x0200)
		{
			RIGHTWR = (ADC1BUF0 + 1) >> (9-POW);
			LEFTWR = (PWMCYCLES+1);
		}
		else if (ADC1BUF0 == 0x0200)
		{
			RIGHTWR = (PWMCYCLES + 1);
			LEFTWR = (PWMCYCLES + 1);
		}
	}
	printf("POT = %d\n", PORTBbits.RB3);
	printf("Buffer = %d\n", ADC1BUF0);
	printf("RightWF = %d\n", RIGHTWF);
	printf("LeftWF = %d\n", LEFTWF);
}


void _ISR _T2Interrupt (void)
{
	IFS0bits.T2IF = 0;
}


void _ISR _CNInterrupt (void)
{
	IFS1bits.CNIF = 0;
	
	if(SW1 == 0)
	{
		//IDLE State
		if(MotorState == 0  || MotorState == 2)
		{
			RIGHTE = 0;
			LEFTE = 0;
			MotorState++;
		}
		//FORWARD State
		else if (MotorState == 1)
		{
			RIGHTWF = RIGHTWR;			//Switch duty cycles
			LEFTWF = LEFTWR;	
			RIGHTWR = 0;				//Set reverse duty cycle to zero		
			LEFTWR = 0;
			RIGHTE = 1;					//Enable Motors
			LEFTE = 1;
			MotorState++;
		}
		//REVERSE State
		else if (MotorState == 3)
		{
			RIGHTWR = RIGHTWF;			//Switch duty cycles
			LEFTWR = LEFTWF;	
			RIGHTWF = 0;				//Set forward duty cycle to zero		
			LEFTWF = 0;
			RIGHTE = 1;					//Enable Motors
			LEFTE = 1;
			MotorState = 0;
		}
		else
			MotorState = 0;
	}
}