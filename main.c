// Muntaser Ahmed, Chad Becker, Usman Chaudhry, Gautam Kanumuru

#include <msp430.h>
#include "useful.h"			// Using Useful.h
#define greenled	BIT6	// Green LED
#define button		BIT3	// Button
#define LATCH		BIT0	// Bit for Latch
#define SIN			BIT7	// Bit for Input
#define SCLK		BIT5	// Bit for Clock
#define BLANK		BIT4	// Bit for Blank
#define MSB			0x80	// Most Significant Bit
#define LSB			0x01	// Least Significant Bit
#define SENSITIVITY	10		//Sensitivity for the weighted inputs
#define SENSITIVITY2 5		// Sensitivity for the unweighted inputs (leveled range)

/*
 * Type definition to represent the different game states
 */

typedef enum gameStates {Calibrate, P1Init, Player1, P2Init, Player2, Won, Lost } gameState;

/*
 * This structure represents the eight lights that are available on the header board.
 */

typedef struct ledStruct {

	char ledStatus;		// 8-bit data structure
	gameState state; 	// defines which state the program is in

} ledInfo;

typedef enum DbStatesPossible { DbExpectHigh, DbValidateHigh, DbExpectLow, DbValidateLow } DbState;		// state of a particular button
typedef enum SwitchValues { Low, High } SwitchStatus;	// indicates if the button is Low (not presses) or High (pressed)

typedef struct {						// assigns modifiable variables to a variable declared as a SwitchDefine
	DbState ControlState;				// assigns a state to a particular button
	SwitchStatus CurrentSwitchReading;	// the current (valid or not valid) Low or High status of a particular button
	unsigned int timer_2;				// stores the time of the global timer when a particular button is to be validated as high or low. Used to check if the transition is valid.
	unsigned int TIME_VALID;			// the time that the button must be in a "High/pushed" state to validate it as being "High/pushed"
	unsigned int TIME_VALID_RELEASED;	// the time that the button must be in a "Low/released" state to validate it as being "Low/released"
	unsigned int IndivPresses;			// counts the number of button presses for an individual button
} SwitchDefine;

/*
 * Forward Declarations
 */

SwitchStatus GetSwitch();						// returns the instantaneous value of the selected switch
DbState Debouncer(SwitchDefine *PushButton);	// debounces a switch input. Appropriately updates the state of a button and counts individual and total button presses.
void initPorts();								// Initialize Ports
void send_instruction(byte instruction);		// Bit-Banging to the LEDs
void InitTimer();								// Initialize the Timer with the right interrupts
gameState stateMachine(ledInfo _myInfo, gameState _myStatus);		// The state machine
void initVars();													// Set up the variables
void ConfigureAdc(void);											// Declares the function that configures the ADC
void average(void);													// Declares the function that averages the samples.
int getSide();														// Returns the side of the board that chosen

/*
 * Global Variables
 */

int p1Moves[8];			// holds the sequence of eight sides that player 1 sets
int whoseTurn = 1;		// keeps track of which player's turn it is (1 or 2)
int p2Move = 0;			// keeps track of the side that player 2 picks
int i1 = 0;				// this variable keeps track of the index in the p1Moves array
int p2Score = 0;		// used to determine whether or not player 2 entered the sequence correctly. It is also used as an index in the p1Moves array.

ledInfo myInfo;			// Representation of Lights
int count;				// Count used in interrupt.

int xsum; 	// running sum for x-axis
int ysum;	// running sum for y-axis
int zsum;	// running sum for z-axis
int xavg;	// average digital value for x
int yavg;	// average digital value for y
int zavg;	// average digital value for z
volatile unsigned int baja[3];		// holds the ADC conversions using the DTC

/*
 * "Memory buffer" to get rid of a weird error we were getting from reading values off of the accelerometer
 */

int r;
int s;
int t;
int u;
int v;
volatile unsigned int boxCartX[8];		// holds the samples for x
volatile unsigned int boxCartY[8];		// holds the samples for y
volatile unsigned int boxCartZ[8];		// holds the samples for z
unsigned int counter;					// used to determine if the initial samples have been taken
unsigned int boolean;					// indicates that the initial samples have been taken
signed int inc;							// array pointer that replaces old samples for x, y, and z with the new samples

int buttonPresses;						// Variable used to keep track of button presses.
unsigned int timer;						// used as a continuous counter to determine if a transition is valid
SwitchDefine PushButton1;				// represents a button with individual values for its structure variables

/*
 * Calibration values
 */
int xZero;
int yZero;
int zZero;
int xMin;
int yMin;
int zMin;
int xMax;
int yMax;
int zMax;
int calibrate;

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	/*
	 * Ensure that the MSP is fully "set-up"
	 */

	initVars();
	InitTimer();
	initPorts();
	ConfigureAdc();

	ADC10CTL0 |= ENC + ADC10SC; 	// Enables and starts the conversion and sampling process.
	__bis_SR_register(GIE);			// General interrupt enable. Enables maskable interrupts.

	while (1) {

		ADC10CTL0 &= ~ENC;				// disables conversion for determining the average
		while (ADC10CTL1 & BUSY);       // Wait if ADC10 core is active
		average();						// finds the average of the ADC-converted x, y, and z values from the accelerometer after each new sample is taken
		ADC10SA = (int) &(baja); 		// selects the starting address where the DTC results are stored
		ADC10CTL0 |= ENC + ADC10SC;		// Enables and starts the conversion process for continued sampling.
		_NOP();                         // space for debugger
		_NOP();                         // Set Breakpoint here to read ADC

		myInfo.state = stateMachine(myInfo, myInfo.state);		// Update state machine

		PushButton1.CurrentSwitchReading = GetSwitch();			// updates the current Low/High status of PushButton1 by polling it
		PushButton1.ControlState = Debouncer(&PushButton1);		// debounces PushButton1, updates its state, and counts button presses
	}
}

void average(void) {		// Function that calculates the average of the samples using the boxcar algorithm. It keeps a running sum--subtracting the oldest sample, adding the newest sample, and finding the new average.

	if (boolean == 0) {				// True if the samples to be averaged are the initial ones.
		xsum += baja[2];			// The running sum of the initial x-axis samples is calculated by dereferencing the pointer.
		boxCartX[inc] = baja[2];	// stores the x sample in its own array of size 8
		ysum += baja[1];			// The running sum of the initial y-axis samples is calculated by dereferencing the pointer.
		boxCartY[inc] = baja[1];	// stores the y sample in its own array of size 8
		zsum += baja[0];			// The running sum of the initial z-axis samples is calculated by dereferencing the pointer.
		boxCartZ[inc] = baja[0];	// stores the z sample in its own array of size 8
		inc++;						// increment the pointer to fill the arrays
		if (counter >= 8) {			// if the initial 8 samples for x, y, and z have been generated and stored, the first average is to be calculated
			xavg = (xsum >> 3);		// divide the running sum by 8 (the size of the array) to calculate the average
			yavg = (ysum >> 3);		// divide the running sum by 8 (the size of the array)
			zavg = (zsum >> 3);		// divide the running sum by 8 (the size of the array)
			boolean = 1;			// indicates the initial samples have been gathered and the box car average logic can be executed
			inc = 0;				// resets counter for stepping through the arrays to begin at the oldest sample
		}
	}
	else {							// True if the samples to be averaged are not the initial ones. This is the moving boxcar filter.
		counter = 0;				// never to be used again, it is set to zero to ensure it doesn't continue incrementing in main
		xsum = xsum - boxCartX[inc] + baja[2];	// the running sum is updated by subtracting the oldest sample and adding the newest sample.
		ysum = ysum - boxCartY[inc] + baja[1];	// the running sum is updated by subtracting the oldest sample and adding the newest sample.
		zsum = zsum - boxCartZ[inc] + baja[0];	// the running sum is updated by subtracting the oldest sample and adding the newest sample.
		boxCartX[inc] = baja[2];		// the newest sample replaces the oldest
		boxCartY[inc] = baja[1];		// the newest sample replaces the oldest
		boxCartZ[inc] = baja[0];		// the newest sample replaces the oldest
		inc++;						// increment the array pointer to point to the now oldest sample in the arrays
		if (inc >= 8) {				// if the array pointer reaches the end of the arrays, send it to the beginning
			inc = 0;				// points to the first sample in the arrays
		}
		xavg = (xsum >> 3);			// divide the running sum by 8 (the size of the array) to calculate the average
		yavg = (ysum >> 3);			// divide the running sum by 8 (the size of the array) to calculate the average
		zavg = (zsum >> 3);			// divide the running sum by 8 (the size of the array) to calculate the average
	}
	counter++;						// increments the counter used to determine of the initial samples have been taken
}

void ConfigureAdc(void) {			// Function that sets the ADC's reference voltages, sets the ADC's clock speed, and selects the source that is converted (the temp. sensor or signal generator).

	ADC10CTL1 = INCH_2 + ADC10SSEL_3 + CONSEQ_3;		// selects input channels A2/A1/A0 (z, y, and x axes), runs on the SMCLK, and selects the repeat multi channel option
	ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE; 	// sets the sample-and-hold time for 16 × ADC10CLKs, selects multiple sample and conversion option, turns the ADC10 on, and enables the interrupt.
	ADC10DTC0 = 0;										// sets the DTC to one-block non-continuous transfer mode
	ADC10AE0 = (BIT0 + BIT1 + BIT2); 					// enables x, y, and z pins for analog conversion
	ADC10DTC1 = 0x08;                					// selects the number of transfers per block
}

#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR(void) {						// interrupt requested by the DTC. The interrupt flag is automatically set low.
}

/*
 * Returns the instantaneous value of the selected switch
 */

SwitchStatus GetSwitch() {
	if ((P1IN & button)== 0 ){ 							// true if the passed-in switch is pressed. Switches are physically active low.
		return High;									// the button is pressed
	} else {											// true if the passed-in switch is released. Switches are physically active low.
		return Low;										// the button is released
	}
}

/*
 * Debounces a switch input. Appropriately updates the state of a button and counts individual and total button presses.
 */

DbState Debouncer(SwitchDefine *PushButton) {
	SwitchStatus CurrentSwitch = PushButton->CurrentSwitchReading;	// assigns the button's current Low/High status to a local variable
	DbState MyState = PushButton->ControlState;						// assigns the button's current state to a local variable
	unsigned int timer2 = PushButton->timer_2;						// assigns the button's validation timer to a local variable
	unsigned int TimeValid = PushButton->TIME_VALID;				// assigns the button's valid High time to a local variable
	unsigned int TimeValidReleased = PushButton->TIME_VALID_RELEASED; // assigns the button's valid Low time to a local variable
	unsigned int IndPresses = PushButton->IndivPresses;				// assigns the button's individual press count to a local variable

	switch (MyState) {						// checks the button's current state
	case DbExpectHigh:						// if Low / Expect High state
		if (CurrentSwitch == High) {		// true if the button is polled as High
			MyState = DbValidateHigh;		// the transition should now be checked to be valid
			timer2 = timer;					// records the time that the validation process starts
		}
		break;
	case DbValidateHigh:					//if in a High Validation state
		if ((timer - timer2) >= TimeValid && CurrentSwitch == High) {	//true if the button signal has been High passed the threshold time (button has been pressed)
			MyState = DbExpectLow;

			if(calibrate == 0) {
				if (buttonPresses == 0) {			// If the button has not been pressed, the first value to be recorded is x value when x-axis is level
						xZero = xavg;
						zZero = zavg;
						send_instruction(1); 			// 00000001, xMax, This turns on LED 0, prompting the user to calibrate the x axis when it is at its max
					}
					else if (buttonPresses == 1) {		// If the button has been pressed once, we have stored the xZero value, and the next value we want stored is xMax
						xMax = xavg;
						send_instruction(16);			// 00001000, xMin, This turns on LED 4, prompting the user to calibrate the x axis when it is at its min
					}
					else if (buttonPresses == 2) {		// Following the same logic as above, we record xMin's value when button presses is equal to 2
						xMin = xavg;
						send_instruction(68); 			// 01000100, yZero, This turns on LED 2 and 6, prompting the user to calibrate the y axis when it is level
					}
					else if (buttonPresses == 3) {		// Record yZero's value when button presses is 3
						yZero = yavg;
						send_instruction(64); 			// 01000000, yMax, This turns on LED 6, prompting the user to calibrate the y axis when it is at its max
					}
					else if (buttonPresses == 4) {		// Record yMax's value when button presses is 4
						yMax = yavg;
						send_instruction(4); 			// 00000100, yMin, This turns on LED 2, prompting the user to calibrate the y axis when it is at its min
					}
					else if (buttonPresses == 5) {		// Record yMin's value when button presses is 5
						yMin = yavg;
						send_instruction(0);			// This turns off all the LEDs after the calibration
						calibrate = 1;
						buttonPresses = 0;
						myInfo.state = P1Init;

					}
			}

			else {
				if (whoseTurn == 1) {					// If it is player 1's turn, we will record the sides that he/she chooses in the p1Moves array on every button press

					if (buttonPresses == 0) {
						p1Moves[0] = getSide();			// store the side in the first spot in the array
					} else if (buttonPresses == 1) {
						p1Moves[1] = getSide();			// store the side in the second spot in the array
					} else if (buttonPresses == 2) {
						p1Moves[2] = getSide();			// store the side in the third spot in the array
					} else if (buttonPresses == 3) {
						p1Moves[3] = getSide();			// store the side in the fourth spot in the array
					} else if (buttonPresses == 4) {
						p1Moves[4] = getSide();			// store the side in the fifth spot in the array
					} else if (buttonPresses == 5) {
						p1Moves[5] = getSide();			// store the side in the sixth spot in the array
					} else if (buttonPresses == 6) {
						p1Moves[6] = getSide();			// store the side in the seventh spot in the array
					} else if (buttonPresses == 7) {
						p1Moves[7] = getSide();			// store the side in the last spot in the array
						whoseTurn = 2;					// Change the flag to represent player 2's turn
					}
				} else {								// otherwise, we  it is player 2's turn, and we need to verify that player 2's moves match each spot in the array

					p2Move = getSide();					// here we get player 2's current side

					if (p1Moves[i1] == p2Move) {		// if player 2's current side is equal to the correct spot in the array, we increment the index and score and continue
						i1++;
						p2Score++;
					} else {							// otherwise, player 2's current side is not equal to the spot in the array, so he/she lost
						myInfo.state = Lost;			// switch the game state to Lost

						i1 = 0;							// reset the index and other variables to prepare for a new game
						buttonPresses = 0;				// reset buttonPresses
						whoseTurn = 1;					// change the turn flag to represent player 1's turn to prepare for a new game
					}

					if (p2Score == 8) {					// here we check if p2's score has reached 8, which means he/she has completed the entire sequence correctly
						myInfo.state = Won;				// we change the gamestate to Won

						i1 = 0;							// and reset the index and other variables to prepare for a new game
						buttonPresses = 0;				// reset buttonPresses
						p2Score = 0;					// reset player 2's score
						whoseTurn = 1;					// change the turn flag to represent player 1's turn to prepare for a new game

					}
				}
			}

			buttonPresses++;						// increments the total count of button presses
			IndPresses++;							// increments the individual count of button presses
			PushButton->IndivPresses = IndPresses;	// stores the individual count for the passed-in button
		} else if (CurrentSwitch == Low) {			// true if the button signal switches back to Low (bounce)
			MyState = DbExpectHigh;					// sets the state back to Expect High (not valid High)
		}
		break;
	case DbExpectLow:								// if High / Expect Low state
		if (CurrentSwitch == Low) {					// true if the button is polled as Low
			MyState = DbValidateLow;				// the transition should now be checked to be valid
			timer2 = timer;							// records the time that the validation process starts
		}
		break;
	case DbValidateLow:								// if in a Low Validation state
		if ((timer - timer2) >= TimeValidReleased && CurrentSwitch == Low) {		// true if the button signal has been Low passed the threshold time (button has been released)
			MyState = DbExpectHigh;					// sets the state to Low / Expect High
		} else if (CurrentSwitch == High) {			// true if the button signal switches back to High (bounce)
			MyState = DbExpectLow;					// sets the state back to Expect Low (not valid Low)
		}
		break;
	default:
		MyState = MyState;							// default set state to current state
	}
	return MyState;									// returns the button's updated state
}

/*
 * The function below initializes the variables in the beginning in order to iron out any potential starting errors.
 */

void initVars() {

	myInfo.ledStatus = 0x00;

	/*
	 * Set all relevant values to zero in the beginning.
	 */

	count = 0;
	buttonPresses = 0;
	timer = 0;

	counter = 0;
	xsum = 0;
	ysum = 0;
	zsum = 0;
	xavg = 0;
	yavg = 0;
	zavg = 0;
	boolean = 0;
	inc = -1;

	xZero = 0;
	yZero = 0;
	zZero = 0;
	xMin = 0;
	yMin = 0;
	zMin = 0;
	xMax = 0;
	yMax = 0;
	zMax = 0;

	calibrate = 0;

	myInfo.state = Calibrate;

	PushButton1.ControlState = DbExpectHigh ;	//assume PushButton1 is not pressed at start of program (Is Low / Expect High state)
	PushButton1.CurrentSwitchReading = GetSwitch() ;	//updates the current Low/High status of PushButton1 by polling it (should be Low at program start)
	PushButton1.timer_2 = 0;					//validation timer for PushButton1 has not yet started
	PushButton1.TIME_VALID = 10;				//sets the time for PushButton1 to be validated as High to 10ms for the timer configuration
	PushButton1.TIME_VALID_RELEASED = 23;		//sets the time for PushButton1 to be validated as Low to 23ms for the timer configuration
	PushButton1.IndivPresses = 0;				//PushButton1 has not been pressed yet
}

void initPorts(void) {
	P1OUT &= ~greenled;			// Green LED is off if pin is made to be an output.
	P1DIR |= greenled;

	P1OUT &= ~SIN;				// Set the input of the acceleramotor as low
	P1DIR |= SIN;				// Set the input as on output.

	P1OUT &= ~SCLK;				// Set SCLK low
	P1DIR |= SCLK;				// Set SCLK as output

	P1OUT &= ~BLANK;			// Set Blank bit low
	P1DIR |= BLANK;				// Set Blank set as output

	P2OUT &= ~LATCH;			// Set the Latch as low initially
	P2DIR |= LATCH;				// Set the Latch as an output.

	P1REN |= button;			// Resistor enable for the button
	P1SEL &= ~button;			// Selects the I/O as an input
	P1OUT |= button;			// makes button2's resistor a pullup resistor so that it is active low (high when not pressed, low when pressed)
	P1DIR &= ~button;			// Select the button as an input

	P1IFG &= ~button;			// Clear the flag bits
	P1IE &= ~button;			// Initially turn off the button interrupt
}

/*
 * Set up the timer and interrupts correctly for the MSP. Initializes the timer to run on the SMCLK at 1MHz and sets its interrupt frequency to 2000 times a second.
 */

void InitTimer() {
	BCSCTL1 = CALBC1_1MHZ;		// calibrates the basic clock to 1MHz. Sets the range.
	DCOCTL = CALDCO_1MHZ;		// calibrates the DCO to 1MHz. Sets the DCO step and modulation.
	TACTL = TASSEL_2 | ID_0 | MC_1 | TACLR;		// sets the SMCLK to drive the timer, divides the SMCLK frequency by 1 (1MHz), sets the timer for counting up to TACCR0 and then resetting, and clears the timer.

	TACCR0 = 500;				// Set the count up limit of the interrupt so that it is called 2000 times every second
	TACCTL0 = CM_0 | CCIE;		// Set capture compare off and enable the interrupt
	TACCTL0 &= ~CCIFG;			// no interrupt pending initially
}

/*
 * This is the backbone of the program. These states dictate what happens when it is a player's turn, and when player 2 wins or loses
 */

gameState stateMachine(ledInfo _myInfo, gameState _myStatus) {

	switch (_myStatus) {

	/*
	 * State for getting the calibrating values
	 */
	case Calibrate: {
		if(buttonPresses == 0) {
			send_instruction(17);
		}

		break;
	}

	case P1Init: {				// This is the initialization state for player 1's turn

		TACCTL0 &= ~CCIE;		// Turn off interrupts so that the delays do not interfere

		p2Score = 0;			// reset player 2's score

		int z;
		char ledPosition = 1;		// This keeps track of which LED to turn on

		send_instruction(ledPosition);	// Initially turn on LED 0 at the start of initialization
		_delay_cycles(500000);

		for (z = 1; z < 8; z++) {		// Loop through all LEDs one by one for the LED pattern, provides visual feedback

			send_instruction(ledPosition << 1);		// Shift the LED position to the next LED
			ledPosition = ledPosition << 1;			// Update the position of the LED
			_delay_cycles(500000);

		}

		send_instruction(0);				// Turn off the LEDs

		buttonPresses = 0;					// Reset the number of button presses

		_myStatus = Player1;				// P1Init is done, change to Player 1's turn

		break;
	}

	case Player1: {							// We stay in the Player 1 state as long as he/she has yet to complete a sequence of 8 moves

		if (buttonPresses == 8) {
			_myStatus = P2Init;				// if a sequence of 8 has been entered, we switch to the player 2 initialization state
		}

		break;
	}

	case P2Init: {

		int z2;
		char ledPosition2 = BIT7;			// This keeps track of which LED to turn on

		send_instruction(ledPosition2);		// Initially turn on LED 7 at the start of initialization
		_delay_cycles(500000);

		for (z2 = 1; z2 < 8; z2++) {		// Loop through all LEDs one by one for the LED pattern, provides visual feedback

			send_instruction(ledPosition2 >> 1);		// Shift the LED position to the next LED
			ledPosition2 = ledPosition2 >> 1;			// Update the position of the LED
			_delay_cycles(500000);

		}

		send_instruction(0);

		_myStatus = Player2;							// switch to player 2's turn

		break;

	}

	case Player2: {										// This state is a wait-state. And is dictated by the if-else logic in the debouncer
		// if the player 2 has completed the sequence correctly, it goes to the Won state
		// otherwise it goes to the Lost state

		break;
	}
	case Won: {											// the state in which the program enters if player 2 wins

		int i3;

		for (i3 = 0; i3 < 5; i3++) {					// we flash all the LEDs to notify the user of success
			send_instruction(255);						// Bit bang the instruction to the light. 255 represents eight 1's (all LEDs)
			_delay_cycles(500000);
			send_instruction(0);
			_delay_cycles(500000);

		}

		_myStatus = P1Init;								// Since the game is over, we start a new game by sending the program to the first state

		break;
	}
	case Lost: {										// the state in which the program enters if player 2 loses. the side the player should have chosen
		// repeatedly lights up to signify an incorrect entry

		int correctMove = p1Moves[p2Score];				// we keep track of the correct move the player should've entered when he/she lost
		int correctBit = 0;								// a variable to keep track of which light needs to be turned on for feedback

		if (correctMove == 1) {							// if player 2 should've chosen side 1, we set correctBit to represent BIT0, which will light up the top LED if sent to send_instruction
			correctBit = BIT4;
		} else if (correctMove == 2) {					// if player 2 should've chosen side 1, we set correctBit to represent BIT2, which will light up the right LED if sent to send_instruction
			correctBit = BIT6;
		} else if (correctMove == 3) {					// if player 2 should've chosen side 1, we set correctBit to represent BIT4, which will light up the bottom LED if sent to send_instruction
			correctBit = BIT0;
		} else if (correctMove == 4) {					// if player 2 should've chosen side 1, we set correctBit to represent BIT6, which will light up the left LED if sent to send_instruction
			correctBit = BIT2;
		}

		int i4;

		for (i4 = 0; i4 < 5; i4++) {
			send_instruction(correctBit);				// Bit bang the instruction to the light to light up the side the player should've chosen
			_delay_cycles(500000);
			send_instruction(0);
			_delay_cycles(500000);

		}

		_myStatus = P1Init;								// Since the game is over, we start a new game by sending the program to the first state
		p2Score = 0;

		break;
	}
	}

	return _myStatus;
}

/*
 * Bit Banging method from previous lab to send instructions to the LED board.
 */

void send_instruction(byte instruction) {

	/*
	 * For loop to go through each bit in the byte (goes from MSB to LSB)
	 */

	int i;
	for (i = 0; i < 8; i++) {

		/*
		 * Check the LSB, and send the relevant value (done with if-else statement)
		 */

		if ((instruction & LSB) == LSB) {
			P1OUT |= SIN;
		} else {
			P1OUT &= ~SIN;
		}
		instruction >>= 1;		// Shift to the right by 1, so that the MSB is now the next bit to be sent

		/*
		 * Make the SCLK go from low to high (with relevant delay cycles) because data is sent on the rising edge of the clock.
		 * The reason we do P1OUT |= SCK then P1OUT &= ~SCK is because SCK is already in low state, so this produces a rising edge, then a falling edge.
		 */

		__delay_cycles(8);
		P1OUT &= ~SCLK;
		__delay_cycles(8);
		P1OUT |= SCLK;
		__delay_cycles(8);
	}

	/*
	 * Set the Latch high then low so that the signal goes through and is displayed on the lights.
	 */

	P2OUT |= LATCH;
	P2OUT &= ~LATCH;
}

/*
 * This interrupt deals with the LEDs. By using a count variable that counts up to the myInfo.period value, we create a mock PWM signal.
 */

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timerRoutine(void) {	 // Interrupt service routine for lights
	timer++;							 // global timer is incremented every 1ms.

}

int getSide(void) {
	if ((yavg >= (yMax - SENSITIVITY)) && ((xavg <= (xZero + SENSITIVITY2)) && (xavg >= (xZero - SENSITIVITY2)))) {		// if the adc raw values fall within this range, then side 2 is chosen (right side points up)
		return 2;
	}

	else if ((xavg >= (xMax - SENSITIVITY)) && ((yavg <= (yZero + SENSITIVITY2)) && (yavg >= (yZero - SENSITIVITY2)))) {	// if the adc raw values fall within this range, then side 3 is chosen (lower side points up)
		return 3;
	}

	else if ((yavg <= (yMin + SENSITIVITY)) && ((xavg <= (xZero + SENSITIVITY2)) && (xavg >= (xZero - SENSITIVITY2)))) {	// if the adc raw values fall within this range, then side 4 is chosen (left side points up)
		return 4;
	}

	else if ((xavg <= (xMin + SENSITIVITY)) && ((yavg <= (yZero + SENSITIVITY2)) && (yavg >= (yZero - SENSITIVITY2 - 9)))) {	// if the adc raw values fall within this range, then side 1 is chosen (upper side points up)
		return 1;
	}

	return 0;
}
