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
          BKBUG_ON & COE_OFF & ICS_PGx1 & 
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


//volatile int MotorState = 0;
volatile int Reading_BASEIR = 1;
volatile int BASEIR = 0;
volatile int POW;
volatile int RUN = 0;
volatile int success = 0;
volatile int playback = 0, record = 0;
volatile int recordingIndex = 0;
volatile int lastState = 10;
volatile int msCount = 0;
volatile int RIGHTIR  = 0;			//PIN26
volatile int CENTERIR = 0;			//PIN2
volatile int LEFTIR = 0;

volatile enum POSSIBLE_STATES{
				idle,
				forward,
 				hardLeft,
 				slightLeft,
 				hardRight, 
 				slightRight,
 				reverse,
 				search 
 			  } state;

void turnAround(){
	int test = 0;
}

void delay(unsigned int usDelay) {
	PR4 = (14*usDelay);	
	TMR4 = 0;
	IFS1bits.T4IF = 0;
	T4CONbits.TON = 1;	while (IFS1bits.T4IF == 0) {}
	T4CONbits.TON = 0;
	IFS1bits.T4IF = 0;
}

int main(void)
{

// ******************************************************************************************* //
// ******************************************************************************************* //
// ******************************************************************************************* //
	//SETTINGS FOR DIODES
		//Set analog inputs:  RB
		AD1PCFG = 0xFFEA;
		TRISAbits.TRISA0 = 1;			//RA0 - AN0 - PIN2
		TRISBbits.TRISB0 = 1;			//RB0 - AN2 - PIN4
		TRISBbits.TRISB2 = 1;			//RB2 - AN4 - PIN6
		//Set AN11-9 for scan
		AD1CSSL = 0x0015;


		//Thresholds for deteremining whether a slight turn or hard turn should be made
		//Value given in digital steps    ((Vref+ - Vref-)/1024 * THRESHOLD)
		#define THRESHOLD_SLIGHT		10
		#define THRESHOLD_HARD			35


	//CHANGE THESE PARAMETERS TO CHANGE THE MOTOR OPERATION

		// Number of clock cycles per PWM period.
		#define PWMCYCLES          PR2
		//Configure such that the number of cycles is a power of 2 (2,4,8,16...256,512)
		PWMCYCLES = 0x03FF;		//Number of cycles is 1023	
			
		
		//Ports used for output to H-bridge
		//PWM outputs
		RPOR6bits.RP12R = 18;						// Set RB12 to OC1 pin 23
		TRISBbits.TRISB12 = 0;
		RPOR6bits.RP13R = 19;						// Set RB13 to OC2 pin 24
		TRISBbits.TRISB13 = 0;
		RPOR7bits.RP14R = 20;						// Set RB14 to OC3 pin 25
		TRISBbits.TRISB14 = 0;
		RPOR7bits.RP15R = 21;						// Set RB15 to OC4 pin 26
		TRISBbits.TRISB15 = 0;
		//Driver Enable outputs
		#define MOTORE			LATBbits.LATB6      //Physical pin 15
		MOTORE = 0;									//Initialize IDLE state
		
		
		// Voltage levels in digital of potentiometer for Wheel settings
		#define RIGHTWF          OC1RS             //Right Wheel 1 Forward sets OC1 duty cycle
		#define RIGHTWR          OC2RS             //Right Wheel 1 Reverse sets OC2 duty cycle
		#define LEFTWF           OC3RS             //Left Wheel 1 Forward sets OC3 duty cycle
		#define LEFTWR           OC4RS             //Left Wheel 1 Reverse sets OC4 duty cycle
		//Initial duty cycles
		RIGHTWF = 0x0000;
		RIGHTWR = 0x0000;
		LEFTWF = 0x0000;
		LEFTWR = 0x0000;		
		
		//UART1 set up if desired
		RPINR18bits.U1RXR = 9;						// Set UART RX input to RB9
		RPOR4bits.RP8R = 3;							// Set UART TX output to RB8
	
		//Switch 1 definition
		#define SW1				PORTBbits.RB5
		//Front contact switch set up
		#define CONTACT			PORTBbits.RB7
		
// ******************************************************************************************* //
// ******************************************************************************************* //
// ******************************************************************************************* //
/*	//UART CONFIGURATION
	
	// Set UART1's baud rate generator register (U1BRG) to the value calculated above.
	U1BRG  = BRGVAL;

	//     UARTEN        = 1     (enable UART)
	//     PDSEL1:PDSEL0 = 00    (8-bit data, no parity)
	//     STSEL         = 0     (1 stop bit)
	U1MODE = 0x8000; 		

	//     UTXISEL1:UTXISEL0 = 00    (U1TXIF set when character written to trasmit buffer)
	//     UTXEN             = 1     (Trasnmit enabled)
	//     URXISEL1:URXISEL0 = 01    (U1RXIF set when any character is received in receive buffer)
	//     RIDLE             = 0     (Reciver is active)
	U1STA  = 0x0440; 		// Reset status register and enable TX & RX

	// Clear the UART RX interrupt flag. The UART RX interrupt flag can be used  
	// to determine if we have recieved a character from he UART. 
	IFS0bits.U1RXIF = 0;

	// printf by default is mapped to serial communication using UART1.
	//        1. You must specify a heap size for printf. This is required
	//           becuase printf needs to allocate its own memory, which is
	//           allocated on the heap. This can be set in MPLAB by:
	//           a.) Selecting Build Options...->Project from the Project menu.
	//           b.) Selecting the MPLABLINK30 Tab.
	//           c.) Entering the size of heap, e.g. 512, under Heap Size

	*/
	
// ******************************************************************************************* //
	//CONFIGURATION OF ADC, PWM, and TIMERS
	
	//ADC Configuration settings
	//     ADON          = 0     (ADC off)
	//     FORM<1:0>     = 00    (integer output;  MAX = 1024;  0b0000 00dd dddd dddd)
	//     SSRC<2:0>     = 111   (Auto-sample)
	//     ASAM          = 0     (Auto-sample start is off)
	AD1CON1 = 0x00E0;
	//     VCFG<2:0>     = 010   (AVdd and VRef- used for reference;  internal reference)
	//     CSCNA         = 1     (Scanning with Mux A, select channels with AC1CSSL)
	//     SMPI<3:0>     = 0011  (3 sample per conversion)
	//     BUFM			 = 0	 (Buffers hold output as 16-bit word)
	AD1CON2 = 0x040C;
	//     ADRC          = 0     (System clock)
	//     SAMC<4:0>     = 00001 (Tsmp = 1*Tad = 2;  Sample Period)
	//     ADCS<7:0>     = 00000001   (Tad = 1+1 = 2;  Conversion Period)
	AD1CON3 = 0x0101;
	//     No channels for muxes because scanning
	AD1CHS  = 0x0000;
	//     Clear interrupt flag
	IFS0bits.AD1IF = 0;
	//     Enable ADC interrupt
	IEC0bits.AD1IE = 1;
	
	
	//PWM Configuration
	//     DISABLE PWMs
	OC1CON = 0x0000;
	OC2CON = 0x0000;
	OC3CON = 0x0000;
	OC4CON = 0x0000;
	//     Set initial duty cycles
	OC1R = 0x0000;
	OC2R = 0x0000;
	OC3R = 0x0000;
	OC4R = 0x0000;
	//     Set initial duty cycle fill registers
	OC1RS = 0x0000;
	OC2RS = 0x0000;
	OC3RS = 0x0000;
	OC4RS = 0x0000;
	//     ENABLE PWMs
	//     OCTSEL        = 0     (Timer 2 selected)
	//     OCM<2:0>      = 110   (PWM mode enabled)
	OC1CON = 0x0006;
	OC2CON = 0x0006;
	OC3CON = 0x0006;
	OC4CON = 0x0006;
	
// ******************************************************************************************* //	
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

	//		Clear Timer 4 value
	TMR4 = 0;
	//		TON			 = 0     (Start timer)
	//     TCKPS1<2:0>	 = 00    (Set timer prescaler to 1:1)
	//     TCS           = 0     (Internal clock;  Fosc/2)
	T4CON = 0x0000;
	//     Clear interrupt flag. 
	IFS1bits.T4IF = 0;		
	//     Enable interrupt
	IEC1bits.T4IE = 0;

	//		Clear Timer 5 value
	TMR5 = 0;
	//		TON			 = 0     (Start timer)
	//     TCKPS1<2:0>	 = 00    (Set timer prescaler to 1:1)
	//     TCS           = 0     (Internal clock;  Fosc/2)
	T5CON = 0x0000;
	//     Clear interrupt flag. 
	IFS1bits.T5IF = 0;		
	//     Enable interrupt
	IEC1bits.T5IE = 1;
// ******************************************************************************************* //
	//Configure SW1 to be an input that changes whether the program runs or not
	
	//Set SW1 to input (RB5)
	TRISBbits.TRISB5 = 1;
	//Set change notification enable and pull up resistor for input
	CNPU2bits.CN27PUE = 1;
	CNEN2bits.CN27IE = 1;
	//Clear interrupt flag
	IFS1bits.CNIF = 0;
	//Enable ISR
	IEC1bits.CNIE = 1;
// ******************************************************************************************* //
	//Configure RB7 to be an input that detects when the car has made contact with the IR source
	
	//Set RB7 to input (pin 16)
	TRISBbits.TRISB7 = 1;
	//Set change notification enable and pull up resistor for input
	CNPU2bits.CN23PUE = 1;
	CNEN2bits.CN23IE = 1;
	//Interrupt flag already cleared
	//ISR already enabled
// ******************************************************************************************* //

	//set up and initialize the recording arrays
	unsigned short recordedState[1024] = { 0 };
	unsigned int stateTime[1024] = { 0 };
	unsigned int stateIndex = 0;
//	int i;
//	for(i=0; i<1024; i++){
//		recordedState[i] = 0;
//	}
//	for(i=0; i<1024; i++){
//		stateTime[i] = 0;
//	}
	

	RUN = 1;
	AD1CON1bits.ADON = 1;
	T2CONbits.TON = 1;
	
	while (1)
	{
		AD1CON1bits.ASAM = 1;			//Start auto-sampling
		//record the state change
/*		if (state != lastState){
			recordedState[stateIndex] = lastState;
			stateTime[stateIndex++] = msCount;
			msCount = 0;
			lastState = state;
		}
*/		if(RUN && !success){
			switch(state){
			case idle: 			//LEFTE = 0;
								//RIGHTE = 0;
								MOTORE = 0;
								RIGHTWR = 0;
								LEFTWR = 0;
								RIGHTWF = 0;
								LEFTWF = 0;
								break;
			case forward:		//LEFTE = 1;
								//RIGHTE = 1;
								MOTORE = 1;
								RIGHTWR = 0;
								LEFTWR = 0;
								RIGHTWF = (PWMCYCLES + 1);
								LEFTWF = (PWMCYCLES + 1);
								break;
			case hardLeft:		//LEFTE = 1;
								//RIGHTE = 1;
								MOTORE = 1;
								RIGHTWR = 0;
								LEFTWR = 0;
								RIGHTWF = (PWMCYCLES + 1);
								LEFTWF = 0;
								break;
			case slightLeft:	//LEFTE = 1;
								//RIGHTE = 1;
								MOTORE = 1;
								RIGHTWR = 0;
								LEFTWR = 0;
								RIGHTWF = (PWMCYCLES + 1);
								LEFTWF = (PWMCYCLES + 1)/2;
								break;
			case hardRight: 	//LEFTE = 1;
								//RIGHTE = 1;
								MOTORE = 1;
								RIGHTWR = 0;
								LEFTWR = 0;
								RIGHTWF = 0;
								LEFTWF = (PWMCYCLES + 1);
								break;
			case slightRight: 	//LEFTE = 1;
								//RIGHTE = 1;
								MOTORE = 1;
								RIGHTWR = 0;
								LEFTWR = 0;
								RIGHTWF = (PWMCYCLES + 1)/2;
								LEFTWF = (PWMCYCLES + 1);
								break;
			case reverse: 		//LEFTE = 1;
								//RIGHTE = 1;
								MOTORE = 1;
								RIGHTWR = (PWMCYCLES + 1);
								LEFTWR = (PWMCYCLES + 1);
								break;
			case search:		//LEFTE = 1;
								//RIGHTE = 1;
								MOTORE = 1;
								LEFTWR = 0;
								RIGHTWR = 0;
								RIGHTWF = (PWMCYCLES + 1);
								LEFTWF = 0;
								break;
			}
		}
/*		else if(RUN && success){
			turnAround();
			int i;
			for(i=0; i<1024; i++){//while(!finished){
				//set the state to the 
				state = recordedState[i];
				// set the movement to the corresponding settings
				switch(state){
				case forward:		//LEFTE = 1;
									//RIGHTE = 1;
									MOTORE = 1;
									RIGHTWR = 0;
									LEFTWR = 0;
									RIGHTWF = (PWMCYCLES + 1);
									LEFTWF = (PWMCYCLES + 1);
									break;
				case hardLeft:		//LEFTE = 1;
									//RIGHTE = 1;
									MOTORE = 1;
									RIGHTWR = 0;
									LEFTWR = 0;
									RIGHTWF = (PWMCYCLES + 1);
									LEFTWF = 0;
									break;
				case slightLeft:	//LEFTE = 1;
									//RIGHTE = 1;
									MOTORE = 1;
									RIGHTWR = 0;
									LEFTWR = 0;
									RIGHTWF = (PWMCYCLES + 1);
									LEFTWF = (PWMCYCLES + 1)/2;
									break;
				case hardRight: 	//LEFTE = 1;
									//RIGHTE = 1;
									MOTORE = 1;
									RIGHTWR = 0;
									LEFTWR = 0;
									RIGHTWF = 0;
									LEFTWF = (PWMCYCLES + 1);
									break;
				case slightRight: 	//LEFTE = 1;
									//RIGHTE = 1;
									MOTORE = 1;
									RIGHTWR = 0;
									LEFTWR = 0;
									RIGHTWF = (PWMCYCLES + 1)/2;
									LEFTWF = (PWMCYCLES + 1);
									break;
				case reverse: 		//LEFTE = 1;
									//RIGHTE = 1;
									MOTORE = 1;
									RIGHTWR = (PWMCYCLES + 1);
									LEFTWR = (PWMCYCLES + 1);
									break;
				case search:		//LEFTE = 1;
									//RIGHTE = 1;
									MOTORE = 1;
									LEFTWR = 0;
									RIGHTWR = 0;
									RIGHTWF = (PWMCYCLES + 1);
									LEFTWF = 0;
									break;
				case idle: 			//LEFTE = 0;
									//RIGHTE = 0;
									MOTORE = 0;
									RIGHTWR = 0;
									LEFTWR = 0;
									RIGHTWF = 0;
									LEFTWF = 0;
									break;
				}

				// delay the loop until recorded state time is elapsed
				delay(stateTime[i]);
			}
			// reset the RUN and success variables and disable the motors
			RUN = 0;
			success = 0;
			MOTORE = 0;
		}
*/	}
	
	return 0;
}


///*  void _ISR _ADC1Interrupt (void)
//{
//	IFS0bits.AD1IF = 0;
//	AD1CON1bits.ASAM = 0;				//Stop auto-sample
//	AD1CON1bits.DONE = 0;
//
//	if(0 /*condition for straight*/){
//
//	}	
//
//
//	//If potentiometer is in middle buffer holds 512 (0x0200) because in center of refeneces AVdd/AGND.
///*	if (MotorState == 1)
//	{	
//		if (ADC1BUF0 > 0x0200) 
//		{
//			RIGHTWF = (PWMCYCLES + 1);
//			LEFTWF = (PWMCYCLES+1) - (((ADC1BUF0 + 1) >> (9-POW)) - (PWMCYCLES + 1));
//		}
//		else if (ADC1BUF0 < 0x0200)
//		{
//			RIGHTWF = (ADC1BUF0 + 1) >> (9-POW);
//			LEFTWF = (PWMCYCLES+1);
//		}
//		else if (ADC1BUF0 == 0x0200)
//		{
//			state = forward;
//			//RIGHTWF = (PWMCYCLES + 1);
//			//LEFTWF = (PWMCYCLES + 1);
//		}
//	}
//	else if (MotorState == 3)
//	{
//		if (ADC1BUF0 > 0x0200) 
//		{
//			RIGHTWR = (PWMCYCLES + 1);
//			LEFTWR = (PWMCYCLES+1) - (((ADC1BUF0 + 1) >> (9-POW)) - (PWMCYCLES + 1));
//		}
//		else if (ADC1BUF0 < 0x0200)
//		{
//			RIGHTWR = (ADC1BUF0 + 1) >> (9-POW);
//			LEFTWR = (PWMCYCLES+1);
//		}
//		else if (ADC1BUF0 == 0x0200)
//		{
//			RIGHTWR = (PWMCYCLES + 1);
//			LEFTWR = (PWMCYCLES + 1);
//		}
//	}   */
//}  */

void _ISR _ADC1Interrupt (void)
{
	IFS0bits.AD1IF = 0;
	AD1CON1bits.ASAM = 0;				//Stop auto-sample
	AD1CON1bits.DONE = 0;

	RIGHTIR  = ADC1BUF0;			//PIN2
	CENTERIR = ADC1BUF1;			//PIN6
	LEFTIR   = ADC1BUF2;			//PIN7

	//Capture BASEIR

/*	if (Reading_BASEIR < 10)
	{
		int temp;
		temp = (RIGHTIR + CENTERIR + LEFTIR)/3;
		if (Reading_BASEIR == 1)
			BASEIR = temp;
		BASEIR = (BASEIR + temp)/2;
		Reading_BASEIR++;
	}
*/

BASEIR=0;
	
	//Determine State
	if (Reading_BASEIR >= 10)
	{
		if ((RIGHTIR < (BASEIR + 5)) && (CENTERIR < (BASEIR + 5)) && (LEFTIR < (BASEIR + 5)))
		{
			//Recapture IR source
			state = idle;
		}
		else if (LEFTIR < (RIGHTIR + THRESHOLD_SLIGHT)  &&  RIGHTIR < (LEFTIR + THRESHOLD_SLIGHT) && CENTERIR > (BASEIR + 10))
		{
			//Forward
			state = forward;
		}
		else if ((LEFTIR < (RIGHTIR + THRESHOLD_SLIGHT))  &&  (RIGHTIR < (LEFTIR + THRESHOLD_HARD)))
		{	
			//Slight Left
			state = slightLeft;
		}
		else if (LEFTIR > RIGHTIR + THRESHOLD_HARD)
		{
			//Hard Left
			state = hardLeft;
		}
		else if ((RIGHTIR > LEFTIR + THRESHOLD_SLIGHT)  &&  (RIGHTIR < LEFTIR + THRESHOLD_HARD))
		{	
			//Slight Right
			state = slightRight;
		}
		else if (RIGHTIR > LEFTIR + THRESHOLD_HARD)
		{
			//Hard Right
			state = hardRight;
		} 
	}
}


void _ISR _T2Interrupt (void)
{
	IFS0bits.T2IF = 0;
}

void _ISR _T5Interrupt (void)
{
	IFS1bits.T5IF = 0;
	if(record && ! playback){
		msCount++;
	} 
	else if (playback && ! record){
		
	}
}


void _ISR _CNInterrupt (void)
{
	IFS1bits.CNIF = 0;
	
	if(SW1 == 0)
	{
		RUN = ~RUN;	
	}
	else if(CONTACT == 0)
	{
		success = 1;
		state = idle;
	}
	else success = 0;

	/*	//IDLE State
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
			MotorState = 0;   */
	//}
}
