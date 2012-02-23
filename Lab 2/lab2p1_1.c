// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

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

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB. 
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.

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

// ******************************************************************************************* //

void DebounceDelay(void) {
	//T1CONbits.TON = 1;
	//while(IFS0bits.T1IF = 0) {
	int i = 0;
 	for(i=0; i<5000; i++) {
		
	}
	// Clear Timer 1 interrupt flag to allow another Timer 1 interrupt to occur.
	
}
volatile int sw_count = 0;
volatile int sw_is_running = 0;

void sw_reset(){
	if(!sw_is_running){
		sw_count = 0;
		sw_is_running = 0;
	}
}

void sw_toggle_pause()
{
	sw_is_running = !sw_is_running;
}

// ******************************************************************************************* //

int main(void)
{
	// ****************************************************************************** //
	// Setup the UART to enable use of  printf() for debugging your code, if needed. 
	//
	// printf by default is mapped to serial communication using UART1.
	// NOTES:
	//        1. You must specify a heap size for printf. This is required
	//           becuase printf needs to allocate its own memory, which is
	//           allocated on the heap. This can be set in MPLAB by:
	//           a.) Selecting Build Options...->Project from the Project menu.
	//           b.) Selecting the MPLABLINK30 Tab.
	//           c.) Entering the size of heap, e.g. 512, under Heap Size
	//        2. printf function is advanced and using printf may require 
	//           significant code size (6KB-10KB).   

	// RPINR18 is a regsiter for selectable input mapping (see Table 10-2) for 
	// for UART1. U1RX is 8 bit value used to specifiy connection to which
	// RP pin. RP9 is used for this configuration. Physical Pin 18.
	RPINR18bits.U1RXR = 9;	

	// RPOR4 is a register for selctable ouput mapping (see Regsiter 1019) for
	// pins RP9 and RP8. The register for RP8 is assigned to 3 to connect 
	// the U1TX output for UART1 (see table 10-3). Physical Pin 17.
	RPOR4bits.RP8R = 3;		

	// Set UART1's baud rate generator register (U1BRG) to the value calculated above.
	U1BRG  = BRGVAL;

	// Set UART1's mode register to 8-bit data, no parity, 1 stop bit, enabled.
	//     UARTEN        = 1     (enable UART)
	//     PDSEL1:PDSEL0 = 00    (8-bit data, no parity)
	U1MODE = 0x8000; 		

	// Set UART2's status and control register
	//     UTXISEL1:UTXISEL0 = 00    (U1TXIF set when character 
	//                                written to trasmit buffer)
	//     UTXEN             = 1     (trasnmit enabled)
	//     URXISEL1:URXISEL0 = 01    (U1RXIF set when any character 
	//                                is received in receive buffer)
	//     RIDLE             = 0     (Receiver is active)
	U1STA  = 0x0440; 		// Reset status register and enable TX & RX

	// Clear the UART RX interrupt flag. Althouhg we are not using a ISR for 
	// the UART receive, the UART RX interrupt flag can be used to deermine if 
	// we have recived a character from the UART. 
	IFS0bits.U1RXIF = 0;

	// ****************************************************************************** //
	// TODO: Configure TRIS register bits for Right and Left LED outputs.
	// Configure IO1 and IO2 corresponding to RB0 and RB1 to outputs.
	TRISAbits.TRISA0 = 0; // red LED
	TRISAbits.TRISA1 = 0; // green LED

	// TODO: Configure AD1PCFG register for configuring input pins between analog input
	// and digital IO.
	// Set all analog/digital pins to digital IO.
	AD1PCFG = 0xFFFF;

	// TODO: Configure LAT register bits to initialize Right (Green) LED to on.
	LATAbits.LATA0 = 0;
	LATAbits.LATA1 = 1;

	// TODO: Configure ODC register bits to use open drain configuration for Right
	// and Left LED output.
	ODCAbits.ODA0 = 1;
	ODCAbits.ODA1 = 1;

	// TODO: Configure TRIS register bits for swtich input at RB2.
	TRISBbits.TRISB2 = 1;

	// Configure SW1 on the start board as input on RB5
	TRISBbits.TRISB5 = 1;

	// TODO: Configure CNPU register bits to enable internal pullup resistor for switch 
	// input.
	CNPU1bits.CN6PUE = 1;

	// TODO: Set Timer 1's period value regsiter to value for 5 ms. 
	// Set Timer 1's period value regsiter to value for 5ms. Please note 
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
	//                 = 14745600 / 64
	//                 = 230400
	//
	//    PR1 = 1 ms / (1 / (T1 Freq))
	//        = 1e-3 / (1 / 57600) 
	//        = 1e-3 * 57600
	//        = 230 
	PR1 = 229;		
	
	// TODO: Setup Timer 1 to use internal clock (Fosc/2).
	// Setup Timer 1 control register (T1CON) to:
 	//     TON           = 1     (Timer1 initially off)
	//     TCKPS1:TCKPS2 = 10    (set timer prescaler to 1:64)
	//     TCS           = 0     (Fosc/2)
	//T1CON = 0b1000000000100000;
	T1CON = 0x8020;

	// TODO: Clear Timer 1 value and enable Timer 1 interrupt and reset interrupt flag	
	TMR1 = 0;	
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;				
	
	// Create an interrupt that triggers when the added switch is pressed.  I am not going to
	// use an ISR but I will use it to find when the user has pressed the attached switch.
	// Use change notification (CN) interrupt enable (CNEN) register to record changes
	// at RB2 (pin 14 - CN27) and send and interrupt request when switch changes.
	// Set priority to high.
	CNEN1bits.CN6IE = 1; // Interrupt enable for switch we added
	CNEN2bits.CN27IE = 1; // Interrupt enable for SW1
	IFS1bits.CNIF = 0;
	IPC4bits.CNIP = 7;

	//printf("Hi");

	// intitialize counters for stopwatch ( this will hold hundredths of a second )
	int sw_time = 0;

	while(1)
	{

		sw_time = sw_count % 10;


		// TODO: For each distinct button press, alternate which
		// LED is illumintate (on).
		// TODO: Use DebounceDelay() function to debounce button press 
		// and button release in software.
		if (IFS1bits.CNIF == 1) {

			if (PORTBbits.RB2 == 0 && LATAbits.LATA0 == 0) {
				LATAbits.LATA0 = 1;
				LATAbits.LATA1 = 0;
				sw_toggle_pause();
				DebounceDelay();
				while(PORTBbits.RB2 == 0);
				DebounceDelay();
			}

			else if (PORTBbits.RB2 == 0 && LATAbits.LATA0 == 1) {
				LATAbits.LATA0 = 0;
				LATAbits.LATA1 = 1;
				sw_toggle_pause();
				DebounceDelay();
				while(PORTBbits.RB2 == 0);
				DebounceDelay();
			}

			else if (PORTBbits.RB2 == 1) {
				DebounceDelay();
				//printf("release");
			}
			IFS1bits.CNIF =0;
		}
		if(sw_is_running){
			printf("timing");
		}
		else{
			printf("stopped");
		}
	}
	return 0;
}

// ******************************************************************************************* //
// Defines an interrupt service routine that will execute whenever Timer 1's
// count reaches the specfied period value defined within the PR1 register.
// 
//     _ISR and _ISRFAST are macros for specifying interrupts that 
//     automatically inserts the proper interrupt into the interrupt vector 
//     table
//
//     _T1Interrupt is a macro for specifying the interrupt for Timer 1
//
// The functionality defined in an interrupt should be a minimal as possible
// to ensure additional interrupts can be processed. 
void _ISR _T1Interrupt(void)
{
	// reset interrupt flag
	IFS0bits.T1IF = 0;
	//Reset TMR1 to 0 and turn off timer
	TMR1 = 0;
	T1CONbits.TON = 0;

	if (sw_is_running){
		sw_count++;
	}
}




// ******************************************************************************************* //
