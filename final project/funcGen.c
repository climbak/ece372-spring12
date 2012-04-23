// ******************************************************************************************* //
// File:         lab4.c   
// Date:         04-03-2012
// Author:      Ryan Courreges
//
// Description:  Function generator with control of the frequency using the on board potentiometer 
//
// Requirements: This software code requires the MPLAB C30Compiler
// ******************************************************************************************* //

#include "p24fj64ga002.h"
#include <stdio.h>
#include <math.h>

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
#define PI			    3.14159265			  // Pi

//#define BAUDRATE         115200       
//#define BRGVAL          ((FCY/BAUDRATE)/16)-1 

#define squareOut       LATBbits.LATB6 //set squareOut as the latch for pin RB6, phys pin 15

#define LED1            LATBbits.LATB15
#define LED2            LATBbits.LATB14
#define LED3            LATBbits.LATB13
#define LED4 		    LATBbits.LATB12

// 4th octave note definitions divide/multiply by 2 for stepping octave
#define C				261.63*2
#define Cs				277.18*2
#define Db				277.18*2
#define D               293.66*2
#define Ds				311.13*2
#define Eb				311.13*2
#define	E				329.63*2
#define F				349.23*2
#define Fs				369.99*2
#define Gb				369.99*2
#define G				392.*2
#define Gs				415.3*2
#define Ab				415.3
#define A				440.
#define As				466.16
#define Bb				466.16
#define B				493.88
#define numNotes		241
#define b				150 //one beat length in ms
#define b2				2*b
#define b3				3*b
#define b4				4*b
#define b43				4*b/3
#define b23				2*b/3
#define hb				b/2
#define u				10 //10 ms delay for between same notes
#define R				100000.0 //low freq so "R" beats

volatile int freq = 0; // frequency variable
volatile int t = 0; // time variable
volatile int dt = 0;
volatile int count = 0;
volatile int countms = 0;
volatile float period = 0;
volatile int i = 0;




//volatile int numNotes = 60;
//volatile int noteTime = 500;

//volatile int noteTime[numNotes] = {b,b,b,b,b,b,b,b,b,b};

//volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b3,b,b3,b,b2,b,b2,b,b2,b,b,b,b,b,b,b,b43,b43,b43,b,b,b,b,b,b,b,b,b,b,b2,b,b2,b,b2,b,b2,b,b,b,b,b,b,b,b43,b43,b43,b,b,b,b,b,b,b,b,b,b,b2,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,b,b3,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b2,b,b2,b,b*7,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,*********b,b3,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b2,b,b2,b,7*b,b,u,b,b,b,b,b,b,b,b,b,b,b,b,b3,b,u,b,b,b,b,b,b,b,8*b,b,u,b,b,b,b,b,b,b,b,b,b,b,b};
volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,
								   b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b2,b,b,b2,b4,
								   b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b2,b,b,b2,b4,
								   b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,u,b,b,b,b,b,b,b,b4,b4,b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2};
//volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2,****b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,****b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2,****b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,****b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****
//					****page 2**** b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****
//					****page 3**** b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****
//					****page 4**** b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,****b,u,b,b,b,b,b,b,b,b4,b4,****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,****b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2}
//volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2,****b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b****,b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b****page 2****,b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****,b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2****,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****page 3****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****page 4****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2****bu,,b,b,b,b,b,b,b,b4,b4,****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2****b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2};
/*
volatile int noteTime[numNotes] = {240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,
							240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500};
*/							

int main(void)
{


// ******************************************************************************************* //
	// Pin configurations
	//set square, triange, sin wave pins to out
	TRISBbits.TRISB6 = 0;  //square wave output pin. physical pin 15
//	TRISBbits.TRISB1 = 0;  //triange wave output pin. physical pin 5
//	TRISBbits.TRISB2 = 0;  //sine wave output pin, physical pin 6

	//set squareOut, triangleOut and sinOut to the output latches

	// set LEDs to output
//    TRISBbits.TRISB15 = 0;
//    TRISBbits.TRISB14 = 0;
//    TRISBbits.TRISB13 = 0;
//    TRISBbits.TRISB12 = 0;

	LATB = 0;

    //LATBbits.LATB15 = 1; 
	LED1 = 1;
    //LATBbits.LATB14 = 0; 
	LED2 = 0;
    //LATBbits.LATB13 = 1; 
	LED3 = 1;
    //LATBbits.LATB12 = 0; 
	LED4 = 1;

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
	
	//PWM Configuration
	//     DISABLE PWMs
	OC1CON = 0x0000;
	OC2CON = 0x0000;
	OC3CON = 0x0000;
	//     Set initial duty cycles
	OC1R = 0x0000;
	OC2R = 0x0000;
	OC3R = 0x0000;
	//     Set initial duty cycle fill registers
	OC1RS = 0x0000;
	OC2RS = 0x0000;
	OC3RS = 0x0000;
	//     ENABLE PWMs
	//     OCTSEL        = 0     (Timer 2 selected)
	//     OCM<2:0>      = 110   (PWM mode enabled)
	OC1CON = 0x0006;
	OC2CON = 0x0006;
	OC3CON = 0x0006;

	
	//Frequency Selection Configuration
	//		
	
	
	//TIMER Configuration
	//     Clear Timer 1 value 
	TMR1 = 0;			
 	//     TON           = 0     (Start timer)
	//     TCKPS1<2:0>	 = 00    (Set timer prescaler to 1:1)
	//     TCS           = 0     (Internal clock;  Fosc/2)
	T1CON = 0x0000;
	// Set Timer 1's period value regsiter to value for 250ms. Please note 
    // T1CON's register settings below (internal Fosc/2 and 1:256 prescalar).
    // 
    //    Fosc     = XTFREQ * PLLMODE
    //             = 7372800 * 4
    //             = 29491200
    // 
    //    Fosc/2   = 29491200 / 2
    //             = 14745600
    //
    //    Timer 1 Freq = (Fosc/2) / Prescaler
    //                 = 14745600 / 1
    //                 = 14745600
    //
    //    PR1 = 1 ms / (1 / (T1 Freq)) - 1
    //        = 1e-3 / (1 / 14745600) - 1
    //        = 1e-3 * 14745600 - 1
    //        = 14746 - 1
	PR1 = 14745;
	//     Clear interrupt flag. 
	IFS0bits.T1IF = 0;		
	//     Enable interrupt
	IEC0bits.T1IE = 1;

	//     Clear Timer 2 value 
	TMR2 = 0;			
 	//     TON           = 0     (Start timer)
	//     TCKPS1<2:0>	 = 00    (Set timer prescaler to 1:1)
	//     TCS           = 0     (Internal clock;  Fosc/2)
	T2CON = 0x0000;
	//set PR2 to 1ms
	PR2 = 14745;
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
	CNPUb2its.CN27PUE = 1;
	CNENb2its.CN27IE = 1;
	//Clear interrupt flag
	IFS1bits.CNIF = 1;
	//Enable ISR
	IEC1bits.CNIE = 1; */
// ******************************************************************************************* //


	AD1CON1bits.ADON = 0;
	T1CONbits.TON = 1;
	T2CONbits.TON = 1;
	//    Fosc     = XTFREQ * PLLMODE
    //             = 7372800 * 4
    //             = 29491200
    // 
    //    Fosc/2   = 29491200 / 2
    //             = 14745600
    //
    //    Timer 1 Freq = (Fosc/2) / Prescaler
    //                 = 14745600 / 1
    //                 = 14745600
    //
    //    PR1 = 1/note / (1 / (T1 Freq)) - 1
    //        = 1e-3 / (1 / 14745600) - 1
    //        = 1e-3 * 14745600 - 1
    //        = 14746 - 1


	//int notes[numNotes] = {G*2,G*4};
	 
	
	//float notes[numNotes] =   {E,u,E,R,E,R,C,E,R,G,R,G,R,C,R,G,R,E,R,A,R,B,R,Bb,A,R,G,E,G,A,R,F,G,R,E,R,C,D,B,R,C,R,G,R,E,R,A,R,B,R,Bb,A,G,E,G,A,R,F,G,R,E,R,C,D,B,R,R,G,Gb,F,Ds,R,E,R,Gs,A,C,R,A,C,D,R,G,Gb,F,Ds,R,E,R,C,R,C,u,C,R,R,G,Gb,F,Ds,R,E,R,Gs,A,C,R,A,C,D,R,Eb,R,D,R,C,R,R,G,Gb,F,Ds,R,E,R,Gs,A,C,R,A,C,D,R,G,Gb,F,Ds,R,E,R,C,R,C,u,C,R,R,G,Gb,F,Ds,R,E,Gs,A,C,R,R,A,C,D,R,Eb,R,D,R,C,R,C,u,C,R,C,R,C,D,R,E,C,R,A,G,R,C,u,C,R,C,R,C,D,E,R,C,u,C,R,C,R,C,D,R,E,C,R,A,G,R};
	//float notes[numNotes] =   {E,u,E,R,E,R,C,E,R,G,R,G/2,R,C,R,G/2,R,E/2,R,A,R,B,R,Bb,A,R,G/2,E,G,A*2,R,F,G,R,E,R,C,D,B,R,C,R,G/2,R,E/2,R,A,R,B,R,Bb,A,G/2,E,G,A*2,R,F,G,R,E,R,C,D,B,R,R,G,Gb,F,Ds,R,E,R,Gs/2,A,C,R,A,C,D,R,G,Gb,F,Ds,R,E,R,C,R,C,u,C,R,R,G,Gb,F,Ds,R,E,R,Gs/2,A,C,R,A,C,D,R,Eb,R,D,R,C,R,R,G,Gb,F,Ds,R,E,R,Gs/2,A,C,R,A,C,D,R,G,Gb,F,Ds,R,E,R,C,R,C,u,C,R,R,G,Gb,F,Ds,R,E,Gs,A,C,R,R,A,C,D,R,Eb,R,D,R,C,R,C,u,C,R,C,R,C,D,R,E,C,R,A,G/2,R,C,u,C,R,C,R,C,D,E,R,C,u,C,R,C,R,C,D,R,E,C,R,A,G,R};
	float notes[numNotes] =   {E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A*2,R,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A,R,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,
							   R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
							   R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
							   C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R,C*2,u,C*2,R,C*2,R,C*2,D*2,E*2,R,R,C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R,E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R};
//	float notes[numNotes] =   {E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R,****C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A*2,R,****G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R****,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A,R****,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,
//				****page 2**** R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
//				****page 3**** R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,****R,Eb*2,R,R,D*2,R,C*2,R,R,R,
//				****page 4**** C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,C*2,u,C*2,R,C*2,R,C*2,D*2,E*2,R,R,****C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R};
//	float notes[numNotes] =   {E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R,****C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A*2,R,****G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R****,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A,R****,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R****page 2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,Eb*2,R,R,D*2,R,C*2,R,R,R****page 3****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,Eb*2,R,R,D*2,R,C*2,R,R,R****page 4****,C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,C*2,u,C*2,R,C*2,R,C*2,D*2,E*2,R,R,****C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R};
//float notes[numNotes] = {E,u,E,R,E,R,C,E,R,G,R,G/2,R,C,R,G/2,R,E/2,R,A,R,B,R,Bb,A,R,G/2,E,G,A*2,R,F,G,R,E,R,C,D,B,R,C,R,G/2,R,E,R,A,R,B,R,Bb,A,G/2,E,G,A*2,R,F,G,R,E,R,C,D,B,R,R,G,Gb,F,Ds,R,E,R,Gs/2,A,C,R,A,C,D,R,G,Gb,F,Ds,R,E,R,C,R,C,u,C,R,R,G,Gb,F,Ds,R,E,R,Gs,A,C,R,A,C,D,R,Eb,R,D,R,C,R,R,G,Gb,F,Ds,R,E,R,Gs,A,C,R,A,C,D,R,G,Gb,F,Ds,R,E,R,C,R,C,u,C,R,R,G,Gb,F,Ds,R,E,Gs,A,C,R,R,A,C,D,R,Eb,R,D,R,C,R,C,u,C,R,C,R,C,D,R,E,C,R,A,G/2,R,C,u,C,R,C,R,C,D,E,R,C,u,C,R,C,R,C,D,R,E,C,R,A,G,R};
	

	/*
	volatile int noteTime[numNotes] = {240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,
							240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500};
	

	float notes[numNotes] = {C   ,50,C  ,G  ,50,G  ,A  ,50,A  ,G  ,F  ,50,F  ,E  ,50,E  ,D  ,50,D  ,C  ,G  ,50,G  ,F  ,50,F  ,E  ,50,E  ,D  ,
							G  ,50,G  ,F  ,50,F  ,E  ,50,E  ,D  ,C  ,50,C  ,G  ,50,G  ,A  ,50,A  ,G  ,F  ,50,F  ,E  ,50,E  ,D  ,50,D  ,C  };
	*/
	
	while (1)
	{

	//AD1CON1bits.ASAM = 1;			//Start auto-sampling
	//t = TMR1;
	// 
	PR1 = ((int) (14745600./notes[i]) - 1);
	
	// set square wave values
	//period = 1750*ADC1BUF0/1024+250;

	//	if(TMR2 >= PR2/2) squareOut = 1;
	//else squareOut = 0;

	//set triangle wave values
	//if (TMR1 >= PR1/2) triangleScalar = ();
	//else triangleScalar = -();

	//set sine wave values
	//sinOut = 1.65*sin(2*PI*t)+1.65

	//set LED status states
	if(period <= 250) LED4 = 0;
	else LED4 = 1;
	if(period >= 2000) LED3 = 0;
	else LED3 = 1;

	}
	
	return 0;
}


void _ISR _T1Interrupt (void)
{
	IFS0bits.T1IF = 0;
	squareOut = ~squareOut;
//	TMR1 = 0;
/*	if(count >= period/2){
		count = 0;
 		squareOut = ~squareOut;
		LED1 = ~LED1;
		LED2 = ~LED2;
	}
	else count++; */
//	LATB ^= ((0x1000)<<(3));
}

void _ISR _T2Interrupt (void)
{
	IFS0bits.T2IF = 0;
	//TMR2 = 0;
	if(countms >= noteTime[i]){
		countms = 0;
		i = (i+1) % numNotes;
	}
	else countms++;
	
}