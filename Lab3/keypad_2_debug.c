// ******************************************************************************************* //

#include "p24fj64ga002.h"
#include "keypad.h"

// ******************************************************************************************* //
//Define TRIS bits
#define KEYPAD_C1			TRISBbits.TRISB0
#define KEYPAD_C2			TRISAbits.TRISA0
#define KEYPAD_C3			TRISBbits.TRISB2
#define KEYPAD_R1			TRISAbits.TRISA1
#define KEYPAD_R2			TRISBbits.TRISB10
#define KEYPAD_R3			TRISBbits.TRISB3
#define KEYPAD_R4			TRISBbits.TRISB1

//#define SW1                 TRISBbits.TRISB5
#define KP_WATCH			TRISBbits.TRISB5

//Define keypad presses
#define KEY1

char ScanRows(int colActive, char lastChar);
void SetColActive(int col);
char KeypadScan();
void KeypadInitialize();

volatile int keyPressed = 0;
char lastKey;

void KeypadInitialize() {

	// TODO: Configure IOs and Change Notificaiton interrupt for keypad scanning. This
	// configuration should ensure that if any key is pressed, a change notification interrupt
	// will be generated.

	//Set inputs and outputs
	KEYPAD_C1 = 0;
	KEYPAD_C2 = 0;
	KEYPAD_C3 = 0;
	KEYPAD_R1 = 1;
	KEYPAD_R2 = 1;
	KEYPAD_R3 = 1;
	KEYPAD_R4 = 1;

	KP_WATCH = 0;

	_LATB5 = keyPressed % 2;

	

	// enable the CNPU resistors
	CNPU1bits.CN3PUE = 1;
    CNPU2bits.CN16PUE = 1;
	CNPU1bits.CN7PUE = 1;
	CNPU1bits.CN5PUE = 1;

	//Set change notification for flag enable at read pins
	CNEN1bits.CN3IE  = 1;
	CNEN2bits.CN16IE = 1;
	CNEN1bits.CN7IE  = 1;
	CNEN1bits.CN5IE  = 1;
	IFS1bits.CNIF = 0;
    //IPC4bits.CNIP = 1;

	//Configure SW1 as an input
	//SW1 = 1;


}

// ******************************************************************************************* //


char KeypadScan() {
	char key = -1;

	// TODO: Implement the keypad scanning procedure to detect if exactly one button of the
	// keypad is pressed. The function should return:
	//
	//      0          : Return 0 if no keys are pressed.
	//      '0' - '9'  : Return the ASCII character '0' to '9' if one of the
	//                   numeric (0 - 9) keys are pressed.
	//      '+'        : Return the ASCII character '+' if the A key is pressed.
	//      '-'        : Return the ASCII character '-' if the B key is pressed.
	//      '*'        : Return the ASCII character '*' if the C key is pressed.
	//      '/'        : Return the ASCII character '/' if the D key is pressed.
	//       -1        : Return -1 if the * or # keys are pressed, or if more than
	//                   one key is pressed simultaneously.

    //used to verify that only one key is pressed
    keyPressed = 0;
    int i = 0;
    for(i = 0; i < 3; i++) {
        SetColActive(i); //set only one column to HI at once
        key = ScanRows(i, lastKey); //read each row when a specific col is set HI
    }
	lastKey = key;
	SetColActive(4);
	if(keyPressed == 1) return key;         //return the key pressed
    else if (keyPressed == 0) return lastKey;     //return 0 b/c no key was pressed
    else return '7';                         //return -1 because more than one key was pressed

}

char ScanRows(int colActive, char lastChar) {

    //the character to return
    char returnChar = 0;

    //if statments depending on which col is set HI
    if(colActive == 0) {
        //if statments based on which row is HI
        //need to use if, not if/else if b/c multiple key presses could be possible on different rows
        if(PORTAbits.RA1 == 0) {
            keyPressed++;
            returnChar = '1';
        }
        if(PORTBbits.RB10 == 0){
            keyPressed++;
            returnChar = '4';
        }
		if(PORTBbits.RB3 == 0){
            keyPressed++;
            returnChar = '7';
        }
        
        //using SW1 to implement two functions on one button
/*        if(PORTBbits.RB1 == 0 && PORTBbits.RB5 == 1){ //SW1 not pressed then multiply
            keyPressed++;
            returnChar = '*';
        }
        else if(PORTBbits.RB1 == 0 && PORTBbits.RB5 == 0){ //SW pressed then devided
            keyPressed++;
            returnChar = '/';
        } */
    }

    else if(colActive == 1) {
        if(PORTAbits.RA1 == 0) {
            keyPressed++;
            returnChar = '2';
        }
        if(PORTBbits.RB10 == 0){
            keyPressed++;
            returnChar = '5';
        }
        if(PORTBbits.RB3 == 0){
            keyPressed++;
            returnChar = '8';
        }
/*        if(PORTBbits.RB1 == 0){
            keyPressed++;
            returnChar = '0';
        } */
    }

    else if(colActive == 2) {
        if(PORTAbits.RA1 == 0) {
            keyPressed++;
            returnChar = '3';
        }
        if(PORTBbits.RB10 == 0){
            keyPressed++;
            returnChar = '6';
        }
        if(PORTBbits.RB3 == 0){
            keyPressed++;
            returnChar = '9';
        }
  /*      if(PORTBbits.RB1 == 0 && PORTBbits.RB5 == 1){ //SW1 not pressed then add
            keyPressed++;
            returnChar = '+';
        }
        else if(PORTBbits.RB1 == 0 && PORTBbits.RB5 == 0){ //SW pressed then subtract
            keyPressed++;
            returnChar = '-';
        }*/
    }

    else returnChar = 0;

    if(keyPressed == 1) return returnChar;  //return the key pressed
    else if (keyPressed == 0) return lastChar;     //return 0 b/c no key was pressed
    else return -1;                         //return -1 because more than one key was pressed

}

//setting just one col HI and the others LO
void SetColActive(int col) {
    if(col == 0) {
        //_LATB0 = 0;
        _LATA0 = 1;
        _LATB2 = 1;
    }
    else if(col == 1) {
        //_LATB0 = 1;
        _LATA0 = 0;
        _LATB2 = 1;
    }
    else if(col == 2) {
        //_LATB0 = 1;
        _LATA0 = 1;
        _LATB2 = 0;
    }
    //default set all off
    else {
        //_LATB0 = 1;
        _LATA0 = 0;
        _LATB2 = 0;
    }
}

// ******************************************************************************************* //

