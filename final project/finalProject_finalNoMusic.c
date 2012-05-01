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
          BKBUG_ON & COE_OFF & ICS_PGx1 & 
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
// Defines to simply UART's baud rate generator (BRG) regiser
// given the osicllator freqeuncy and PLLMODE.

#define XTFREQ          7372800                   // On-board Crystal frequency
#define PLLMODE         4                         // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE         115200       
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 

#define squareOut       LATBbits.LATB6 //set squareOut as the latch for pin RB6, phys pin 15

// 4th octave note definitions divide/multiply by 2 for stepping octave
#define C                               261.63*2
#define Cs                              277.18*2
#define Db                              277.18*2
#define D               				293.66*2
#define Ds                              311.13*2
#define Eb                              311.13*2
#define E                               329.63*2
#define F                               349.23*2
#define Fs                              369.99*2
#define Gb                              369.99*2
#define G                               392.*2
#define Gs                              415.3*2
#define Ab                              415.3
#define A                               440.
#define As                              466.16
#define Bb                              466.16
#define B                               493.88

#define numNotes                241

#define b                               150 //one beat length in ms
#define b2                              2*b
#define b3                              3*b
#define b4                              4*b
#define b43                             4*b/3
#define b23                             2*b/3
#define hb                              b/2
#define u                               10 //10 ms delay for between same notes
#define R                               100000. //low freq so "R" beats

volatile int freq = 0; // frequency variable
volatile int t = 0; // time variable
volatile int dt = 0;
volatile int count = 0;
volatile int musicCountms = 0;
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
//                                      ****page 2**** b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****
//                                      ****page 3**** b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****
//                                      ****page 4**** b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,****b,u,b,b,b,b,b,b,b,b4,b4,****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,****b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2}
//volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2,****b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b****,b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b****page 2****,b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****,b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2****,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****page 3****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****page 4****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2****bu,,b,b,b,b,b,b,b,b4,b4,****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2****b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2};
/*

volatile int noteTime[numNotes] = {240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,
                                                        240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500};
*/   
         
        
        volatile float notes[numNotes] =   {E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A*2,R,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A,R,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,
                                   R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
                                   R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
                                   C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R,C*2,u,C*2,R,C*2,R,C*2,D*2,E*2,R,R,C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R,E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R};
//      float notes[numNotes] =   {E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R,****C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A*2,R,****G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R****,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A,R****,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,
//                              ****page 2**** R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
//                              ****page 3**** R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,****R,Eb*2,R,R,D*2,R,C*2,R,R,R,
//                              ****page 4**** C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,C*2,u,C*2,R,C*2,R,C*2,D*2,E*2,R,R,****C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R};
                                        

//volatile int MotorState = 0;
volatile int Reading_BASEIR = 1;
volatile int BASEIR = 0;
volatile int POW;
volatile int RUN = 0;
volatile int success = 0;
volatile int playback = 0, record = 0;
volatile int recordingIndex = 0;
volatile int lastState = 10;
volatile int statemsCount = 0;
volatile int RIGHTIR  = 0;                      //PIN26
volatile int CENTERIR = 0;                      //PIN2
volatile int LEFTIR = 0;
volatile int cnt;
volatile int p=0;
volatile int tempRIGHTIR;
volatile int tempCENTERIR;
volatile int tempLEFTIR;

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
/*
//set up and initialize the recording arrays
volatile unsigned short recordedState[1000] = { 0 };
volatile unsigned int stateTime[1000] = { 0 };
volatile unsigned int stateIndex = 0;
*/

void delay(unsigned int usDelay) ;

void turnAround(void);

		volatile unsigned char recordedState[1500] = { 0 };
		volatile unsigned int stateTime[1500] = { 0 };
		volatile unsigned int stateIndex = 0;


int main(void)
{

// ******************************************************************************************* //
// ******************************************************************************************* //
// ******************************************************************************************* //
        //SETTINGS FOR DIODES
                //Set analog inputs:  RB
                AD1PCFG = 0xFFEA;
                TRISAbits.TRISA0 = 1;                   //RA0 - AN0 - PIN2
                TRISBbits.TRISB0 = 1;                   //RB0 - AN2 - PIN4
                TRISBbits.TRISB2 = 1;                   //RB2 - AN4 - PIN6
                //Set AN11-9 for scan
                AD1CSSL = 0x0015;


                //Thresholds for deteremining whether a slight turn or hard turn should be made
                //Value given in digital steps    ((Vref+ - Vref-)/1024 * THRESHOLD)
                #define THRESHOLD_SLIGHT                50
                #define THRESHOLD_HARD                  65


        //CHANGE THESE PARAMETERS TO CHANGE THE MOTOR OPERATION

                // Number of clock cycles per PWM period.
                #define PWMCYCLES          PR2
                //Configure such that the number of cycles is a power of 2 (2,4,8,16...256,512)
                PWMCYCLES = 0x03FF;   //0x03FF;             //Number of cycles is 1023      
                        
                
                //Ports used for output to H-bridge
                //PWM outputs
                RPOR6bits.RP12R = 18;                                           // Set RB12 to OC1 pin 23
                TRISBbits.TRISB12 = 0;
                RPOR6bits.RP13R = 19;                                           // Set RB13 to OC2 pin 24
                TRISBbits.TRISB13 = 0;
                RPOR7bits.RP14R = 20;                                           // Set RB14 to OC3 pin 25
                TRISBbits.TRISB14 = 0;
                RPOR7bits.RP15R = 21;                                           // Set RB15 to OC4 pin 26
                TRISBbits.TRISB15 = 0;
                //Driver Enable outputs
                #define MOTORE                  LATBbits.LATB10      			 //Physical pin 17
                MOTORE = 0;                                                      //Initialize IDLE state
                
                
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
                RPINR18bits.U1RXR = 9;                                          // Set UART RX input to RB9
                RPOR4bits.RP8R = 3;                                                     // Set UART TX output to RB8
        
                //Switch 1 definition
                #define SW1                             PORTBbits.RB5
                //Front contact switch set up
                #define CONTACT                 PORTBbits.RB7
                
// ******************************************************************************************* //
// ******************************************************************************************* //
// ******************************************************************************************* //
      //UART CONFIGURATION
        
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
        U1STA  = 0x0440;                // Reset status register and enable TX & RX

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
        //     SMPI<3:0>     = 0010  (3 sample per conversion)
        //     BUFM          = 0     (Buffers hold output as 16-bit word)
        AD1CON2 = 0x0408;
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
        //     TCKPS1<2:0>   = 00    (Set timer prescaler to 1:1)
        //     TCS           = 0     (Internal clock;  Fosc/2)
        T2CON = 0x0000;
        //     Clear interrupt flag. 
        IFS0bits.T2IF = 0;              
        //     Enable interrupt
        IEC0bits.T2IE = 1;

        //              Clear Timer 3 value
        TMR3 = 0;
        //     TON           = 0     (Start timer)
        //     TCKPS1<2:0>   = 00    (Set timer prescaler to 1:1)
        //     TCS           = 0     (Internal clock;  Fosc/2)
        T3CON = 0x0000;
        // set PR3 to default value
        PR3 = 14745;
        //     Clear interrupt flag. 
        IFS0bits.T3IF = 0;              
        //     Enable interrupt
        IEC0bits.T3IE = 1;

        //              Clear Timer 4 value
        TMR4 = 0;
        //     TON           = 0     (Start timer)
        //     TCKPS1<2:0>   = 00    (Set timer prescaler to 1:1)
        //     TCS           = 0     (Internal clock;  Fosc/2)
        T4CON = 0x0000;
        // set PR4 to 1ms
        PR4 = 14745;
        //     Clear interrupt flag. 
        IFS1bits.T4IF = 0;              
        //     Enable interrupt
        IEC1bits.T4IE = 1;

        //              Clear Timer 5 value
        TMR5 = 0;
        //     TON           = 0     (Start timer)
        //     TCKPS1<2:0>   = 00    (Set timer prescaler to 1:1)
        //     TCS           = 0     (Internal clock;  Fosc/2)
        T5CON = 0x0000;
        //     Clear interrupt flag. 
        IFS1bits.T5IF = 0;              
        //     Enable interrupt
        IEC1bits.T5IE = 1;
		PR2 = 14745;

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
        //Configure RB5 to be an input that detects when the car has made contact with the IR source
        
        //Set RB5 to input (pin 16)
        TRISBbits.TRISB5 = 1;
        //Set change notification enable and pull up resistor for input
        CNPU2bits.CN27PUE = 1;
        CNEN2bits.CN27IE = 1;
        //Interrupt flag already cleared
        //ISR already enabled
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
        //Configure RB6 to be an output to the speaker for music
        
        //Set RB6 to output (pin 15)
        TRISBbits.TRISB6 = 0;
// ******************************************************************************************* //
        //Configure RB10 to be an output for the motor enable bit
        
        //Set RB10 to output (pin 21)
        TRISBbits.TRISB10 = 0;
// ******************************************************************************************* //

        LATB = 0;
        RUN = 0;

        AD1CON1bits.ADON = 1;
        T2CONbits.TON = 1;
        T3CONbits.TON = 1;
        T4CONbits.TON = 1;
        

        while (1)
        {
                AD1CON1bits.ASAM = 1;                   //Start auto-sampling
                //record the state change
                if ((state != lastState)  && RUN  && !success){
/*
					if(lastState == 2)
						lastState = 4;
					else if (lastState == 4)
						lastState = 2;
*/
                        recordedState[stateIndex] = lastState;
                        stateTime[stateIndex] = statemsCount;
                        statemsCount = 0;
                        lastState = state;
						stateIndex++;
					
						//printf("state: %d \n", recordedState[stateIndex-1]);
						//printf("time: %d \n", stateTime[stateIndex-1]);
						//printf("Index %d \n", (stateIndex-1));
                }
		
                PR3 = ((int) (14745600./notes[i]) - 1);

				if (RUN == 0)
				{
		         		  MOTORE = 0;
                          RIGHTWR = 0;
                          LEFTWR = 0;
                          RIGHTWF = 0;
                          LEFTWF = 0;
						statemsCount=0;
				}


                if(RUN && !success){
                        switch(state){
                        case idle:   
		                             MOTORE = 0;
		                             RIGHTWR = 0;
		                             LEFTWR = 0;
		                             RIGHTWF = 0;
		                             LEFTWF = 0;
		                             break;
                        case forward:  
		                             MOTORE = 1;
		                             RIGHTWR = 0;
		                             LEFTWR = 0;
		                             RIGHTWF = (PWMCYCLES + 1)*2/3;
		                             LEFTWF = (PWMCYCLES + 1)*2/3;
									 AD1CON1bits.ASAM = 0;
									 delay(4);
									 AD1CON1bits.ASAM = 1;
		                             break;
                        case hardLeft: 
		                             MOTORE = 1;
		                             RIGHTWR = 0;
		                             LEFTWR = 0;
		                             RIGHTWF = (PWMCYCLES + 1)*2/3;
		                             LEFTWF = 0;
		                             break;
                        case slightLeft:
		                             MOTORE = 1;
		                             RIGHTWR = 0;
		                             LEFTWR = 0;
		                             RIGHTWF = (PWMCYCLES + 1);
		                             LEFTWF = (PWMCYCLES + 1)*7/8;
		                             break;
                        case hardRight: 
                                     MOTORE = 1;
                                     RIGHTWR = 0;
                                     LEFTWR = 0;
                                     RIGHTWF = 0;
                                     LEFTWF = (PWMCYCLES + 1)*2/3;
                                     break;
                        case slightRight:
                                     MOTORE = 1;
                                     RIGHTWR = 0;
                                     LEFTWR = 0;
                                     RIGHTWF = (PWMCYCLES + 1);
                                     LEFTWF = (PWMCYCLES + 1);
                                     break;
                        case reverse:
                                     MOTORE = 1;
                                     RIGHTWR = (PWMCYCLES + 1);
                                     LEFTWR = (PWMCYCLES + 1)*2/3;
                                     break;
                        case search:
                                     MOTORE = 1;
                                     LEFTWR = 0;
                                     RIGHTWR = 0;
                                     RIGHTWF = (PWMCYCLES+1)*2/3;
                                     LEFTWF = 0;
                                     break;

                        }
              }
            else if(RUN && success){
				AD1CON1bits.ADON = 0;
                AD1CON1bits.ASAM = 0;
                turnAround();
				success = 0;
				RUN = 0;
                int j;
/*
				for(j=stateIndex; j>=stateIndex-20; j--){
					recordedState[j] = forward;
				}
*/ 
                MOTORE = 1;
                RIGHTWR = 0;
                LEFTWR = 0;
                RIGHTWF = (PWMCYCLES + 1)*2/3;
                LEFTWF = (PWMCYCLES + 1)*2/3;
				delay(150);

                for(j=stateIndex; j>=0; j--)
				{
                 	//set the state to the 
                 	state = recordedState[j];
                 	// set the movement to the corresponding settings
                 	switch(state){
                              case forward:           //LEFTE = 1;
                                                                      //RIGHTE = 1;
                                                                      MOTORE = 1;
                                                                      RIGHTWR = 0;
                                                                      LEFTWR = 0;
                                                                      RIGHTWF = (PWMCYCLES + 1)*2/3;
                                                                      LEFTWF = (PWMCYCLES + 1)*2/3;
																	  delay(4);
                                                                      break;
                              case hardLeft:          //LEFTE = 1;
                                                                      //RIGHTE = 1;
                                                                      MOTORE = 1;
                                                                      RIGHTWR = 0;
                                                                      LEFTWR = 0;
                                                                      RIGHTWF = (PWMCYCLES + 1)*2/3;
                                                                      LEFTWF = 0;
                                                                      break;
                              case slightLeft:        //LEFTE = 1;
                                                                      //RIGHTE = 1;
                                                                      MOTORE = 1;
                                                                      RIGHTWR = 0;
                                                                      LEFTWR = 0;
                                                                      RIGHTWF = (PWMCYCLES + 1);
                                                                      LEFTWF = (PWMCYCLES + 1)/2;
                                                                      break;
                              case hardRight:         //LEFTE = 1;
                                                                      //RIGHTE = 1;
                                                                      MOTORE = 1;
                                                                      RIGHTWR = 0;
                                                                      LEFTWR = 0;
                                                                      RIGHTWF = 0;
                                                                      LEFTWF = (PWMCYCLES + 1)*2/3;
                                                                      break;
                              case slightRight:       //LEFTE = 1;
                                                                      //RIGHTE = 1;
                                                                      MOTORE = 1;
                                                                      RIGHTWR = 0;
                                                                      LEFTWR = 0;
                                                                      RIGHTWF = (PWMCYCLES + 1)/2;
                                                                      LEFTWF = (PWMCYCLES + 1);
                                                                      break;
                              case reverse:           //LEFTE = 1;
                                                                      //RIGHTE = 1;
                                                                      MOTORE = 1;
                                                                      RIGHTWR = (PWMCYCLES + 1);
                                                                      LEFTWR = (PWMCYCLES + 1);
                                                                      break;
                              case search:            //LEFTE = 1;
                                                                      //RIGHTE = 1;
                                                                      MOTORE = 1;
                                                                      LEFTWR = 0;
                                                                      RIGHTWR = 0;
                                                                      RIGHTWF = (PWMCYCLES + 1)*2/3;
                                                                      LEFTWF = 0;
                                                                      break;
                              case idle:                    //LEFTE = 0;
                                                                      //RIGHTE = 0;
                                                                      MOTORE = 0;
                                                                      RIGHTWR = 0;
                                                                      LEFTWR = 0;
                                                                      RIGHTWF = 0;
                                                                      LEFTWF = 0;
                                                                      break;
                              }
                              // delay the loop until recorded state time is elapsed
                              delay(stateTime[j]);

                  }

                      MOTORE = 1;
                      RIGHTWR = 0;
                      LEFTWR = PWMCYCLES+1;
                      RIGHTWF = PWMCYCLES+1;
                      LEFTWF = 0;
						delay(65000);
                      RUN = 0;
                      success = 0;
                } 
      }     
        return 0;
}


void _ISR _ADC1Interrupt (void)
{
        IFS0bits.AD1IF = 0;
        AD1CON1bits.ASAM = 0;                           //Stop auto-sample
        AD1CON1bits.DONE = 0;

		if (p<10)
		{
	      	tempRIGHTIR  += ADC1BUF0;                    //PIN2
	        tempCENTERIR += ADC1BUF1;                    //PIN4
	        tempLEFTIR   += ADC1BUF2;                    //PIN6
			p++;
		}
		else 
		{
			RIGHTIR  = tempRIGHTIR/10;                    //PIN2
	        CENTERIR = tempCENTERIR/10;                    //PIN4
	        LEFTIR   = tempLEFTIR/10;                    //PIN6
			p = tempRIGHTIR = tempCENTERIR = tempLEFTIR = 0;
		}


        //Capture BASEIR

        if (Reading_BASEIR < 100  && p == 0)
        {
                int temp;
                temp = (RIGHTIR + CENTERIR + LEFTIR)/3;
                if (Reading_BASEIR == 1)
                        BASEIR = temp;
                BASEIR = (BASEIR + temp)/2;
                Reading_BASEIR++;
        }

        //Determine State
        if(RUN){
			if (Reading_BASEIR >= 100 && p == 0)
	        {
	                if ((RIGHTIR < (BASEIR + 10)) && (CENTERIR < (BASEIR + 10)) && (LEFTIR < (BASEIR + 10)))
	                {
	                        //Recapture IR source
	                        state = search;
	                }
	                else if (LEFTIR < (RIGHTIR + THRESHOLD_SLIGHT)  &&  RIGHTIR < (LEFTIR + THRESHOLD_SLIGHT) && CENTERIR > (BASEIR + 10))
	                {
	                        //Forward
	                        state = forward;
	                }
	
/*	                else if ((LEFTIR < (RIGHTIR + THRESHOLD_SLIGHT))  &&  (RIGHTIR < (LEFTIR + THRESHOLD_HARD)))
	                {       
	                        //Slight Left
	                        state = slightLeft;
	                }
*/	
	                else if (LEFTIR > RIGHTIR + THRESHOLD_HARD)
	                {
	                        //Hard Left
	                        state = hardLeft;
	                }
/*	                else if ((RIGHTIR > LEFTIR + THRESHOLD_SLIGHT)  &&  (RIGHTIR < LEFTIR + THRESHOLD_HARD))
	                {       
	                        //Slight Right
	                        state = slightRight;
	                }
*/	
	                else if (RIGHTIR > LEFTIR + THRESHOLD_HARD)
	                {
	                        //Hard Right
	                        state = hardRight;
	                }         
			}
		}
	else state = idle;
	
/*		if(p==0)
		{
			RIGHTIR  = 0;                    //PIN2
	        CENTERIR = 0;                    //PIN4
	        LEFTIR   = 0;                    //PIN6
		}
*/
}

//Timer 2 is used for the PWM output.  PR2 is constantly set to 1024.
void _ISR _T2Interrupt (void)
{
        IFS0bits.T2IF = 0;
}

// Timer 3 controls audio output frequency
void _ISR _T3Interrupt (void)
{
        IFS0bits.T3IF = 0;
        squareOut = ~squareOut;
}

//Timer 4 controls how long each note of a song is played
void _ISR _T4Interrupt (void)
{
		cnt++;
		statemsCount++;
        IFS1bits.T4IF = 0;

        if(musicCountms >= noteTime[i]){
                musicCountms = 0;
                i = (i+1) % numNotes;
        		PR3 = ((int) (14745600./notes[i]) - 1);
        }
        else musicCountms++;
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
				recordedState[stateIndex] = lastState;
                stateTime[stateIndex] = statemsCount;
        }
       // else success = 0;
}


void delay(unsigned int usDelay) 
{
		cnt = 0;
		while (cnt < usDelay);
}

void turnAround(void)
{

        MOTORE = 0;
        RIGHTWR = 0;
        LEFTWR = 0;
        RIGHTWF = 0;
        LEFTWF = 0;
		delay(500);

		MOTORE = 1;
        RIGHTWR = (PWMCYCLES + 1);
        LEFTWR = (PWMCYCLES + 1)*3/4;
		delay(280);

        MOTORE = 0;
        RIGHTWR = 0;
        LEFTWR = 0;
        RIGHTWF = 0;
        LEFTWF = 0;
		delay(500);

		MOTORE = 1;
        RIGHTWR = PWMCYCLES*3/4;
        LEFTWR = 0;
        RIGHTWF = 0;
        LEFTWF = PWMCYCLES/2;
		delay(750);

        MOTORE = 0;
        RIGHTWR = 0;
        LEFTWR = 0;
        RIGHTWF = 0;
        LEFTWF = 0;
		delay(500);

/*		RIGHTWR = 0;
		LEFTWR = 0;
		RIGHTWF = (PWMCYCLES + 1);
		LEFTWF = (PWMCYCLES + 1)-850; 
		MOTORE = 1;
		delay(6000);
		MOTORE = 0;   */
}
