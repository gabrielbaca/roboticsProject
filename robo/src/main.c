/*********************************************************************
 *
 *                  
 *
 **********************************************************************
 * FileName:        main.c
 * Dependencies:
 * Processor:       PIC32
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE
 *
 **********************************************************************/

/***********************************************************************************
									 README
 ***********************************************************************************
 * Objective: 
 *
 * Tools:
 *			1. MPLAB with PIC32MX support
 *
 *
 ***********************************************************************************
 ***********************************************************************************/

#include <plib.h>					/* Peripheral Library */

// *****************************************************************************
// *****************************************************************************
// Section: Configuration bits
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 10 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
// *****************************************************************************
// *****************************************************************************
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_8


/******************************************************************************/
// Section: System Macros
/******************************************************************************/
#define	GetSystemClock() 	(80000000ul)
#define SYS_FREQ 			(80000000L)
#define PB_DIV         		8
#define PRESCALE       		1
#define TOGGLES_PER_SEC		10000	//Hz
#define TOGGLES_PER_SEC2	100000 	//Hz
#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define T2_TICK				(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC2)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())
#define BAUD    	(9600)      //The desired UART BaudRate

#define SPC_front	11.4777		//Steps per cm.
#define SPC_side	14.7059
#define SPD			3.408 		//Steps per degree

/*****************************************************************************/


/********PROCEDURE declarations*********/
void WriteString(const char *string);
void PutCharacter(const char character);
void step (unsigned char data, unsigned char motorNumber);
void go (char code);
void goSlow();
void goFast();
int abs(int number) { return (number < 0) ? -number : number; }

//main algorithm routines
#define RESET 1
#define GO 0
unsigned char InitialOrientation(char reset);
unsigned char InvInitialOrientation(char reset);
unsigned char GoToCenter(char reset);
unsigned char GoToRoom2(char reset);
unsigned char InspectRoom2RightSide(char reset);
unsigned char GoToRoom3FromRoom2(char reset);
unsigned char GoToStartFromRoom3(char reset);
unsigned char GoToCenterFromRoom3(char reset);
unsigned char GoToCenterFromRoom3afterExt(char reset);
unsigned char TestDog(char reset);
unsigned char GoToRoom4short(char reset);
unsigned char GoToRoom4long(char reset);
unsigned char BackToStart(char reset);
unsigned char GoToRoom1(char reset);
unsigned char GoToCenterFromRoom1(char reset);
unsigned char GoToStartFromCenter(char reset);
unsigned char GoToStartFromRoom4short(char reset);
unsigned char GoToStartFromRoom4shortAfterExt(char reset);
unsigned char GoToStartFromRoom4long(char reset);
unsigned char GoToStartFromRoom4longAfterExt(char reset);
unsigned char GoToCenterFromRoom2(char reset);
unsigned char GoToCenterFromRoom2point1(char reset);	//NOT USED anymore

unsigned char ExploreRightRoom1(char reset);
unsigned char ExploreCenterRoom1(char reset);

int ScanRoom(char reset);

unsigned char ReachFlame(char reset);
unsigned char Extinguish(char reset);
/***************************************/


/*************GLOBALS**********************/
char COMMAND = '0';
int commandtime = 0;
//unsigned char lastPositionR1;	no longer needed, when using scan
int degreesTurned = 0;

// for motors
int Robo_State = 0, debug_state = 0;
unsigned char MotorsON = 0;
unsigned char M1forward = 1, M2forward = 1, M3forward = 1, M4forward = 1;
unsigned int M1_counter = 0, M2_counter = 0, M3_counter = 0, M4_counter = 0;
unsigned int M1_stepPeriod = 50, M2_stepPeriod = 50, M3_stepPeriod = 50, M4_stepPeriod = 50;
char directionNow = 0, countingDirection;
unsigned int step_counter[2];

// for ultrasonics
unsigned int counterDistanceMeasure = 0, counterTrigger=6;
unsigned short frontDistance=0, backDistance=0, leftDistance=0, rightDistance=0; //in cms
unsigned short timeFront = 0, timeBack = 0, timeLeft = 0, timeRight=0;
unsigned short delayFront = 0, delayBack = 0, delayLeft = 0, delayRight=0;
unsigned volatile char timesReadIC1, timesReadIC2, timesReadIC3, timesReadIC4 = 0;
unsigned volatile int D8, D9, D10, D11 = 0;

// for servos
#define SERVOMAXPERIOD 200 // * 100us = 20ms
unsigned int servo1_counter = 0, servo1_period = 200;
int servo1_angle = 0;
unsigned int servo2_counter = 0, servo2_period = 200;
int servo2_angle = 0;

// general time counters
int xtime = 0;
int auxcounter = 10000; //for the servo, just for testing
/******************************************/

int main(void)
{
//LOCALS
	unsigned int temp;
	unsigned int channel1, channel2;
	M1_stepPeriod = M2_stepPeriod = M3_stepPeriod = M4_stepPeriod = 50; // in tens of u-seconds
	unsigned char M1_state = 0, M2_state = 0, M3_state = 0, M4_state = 0;

	SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

/* TIMER1 - now configured to interrupt at 10 khz (every 100us) */
	OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, T1_TICK);
	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
/* TIMER2 - 100 khz interrupt for distance measure*/
	OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, T2_TICK);
	ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_3); //It is off until trigger

/* PORTA b2 and b3 for servo-PWM */
	mPORTAClearBits(BIT_2 | BIT_3);
	mPORTASetPinsDigitalOut(BIT_2 | BIT_3);

/* ULTRASONICS: some bits of PORTB for ultrasonic sensors */
	PORTResetPins(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11 );	
	PORTSetPinsDigitalOut(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11); //trigger
/* Input Capture pins for echo signals */
	//interrupt on every risging/falling edge starting with a rising edge
	PORTSetPinsDigitalIn(IOPORT_D, BIT_8| BIT_9| BIT_10| BIT_11); //INC1, INC2, INC3, INC4 Pin
	mIC1ClearIntFlag();
	OpenCapture1(  IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_ON );//front
	ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_4 | IC_INT_SUB_PRIOR_3);
	OpenCapture2(  IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_ON );//back
	ConfigIntCapture2(IC_INT_ON | IC_INT_PRIOR_4 | IC_INT_SUB_PRIOR_3);
	OpenCapture3(  IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_ON );//left
	ConfigIntCapture3(IC_INT_ON | IC_INT_PRIOR_4 | IC_INT_SUB_PRIOR_3);
	OpenCapture4(  IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_ON );//right
	ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_4 | IC_INT_SUB_PRIOR_3);

/* PINS used for the START (RD13) BUTTON */
    PORTSetPinsDigitalIn(IOPORT_D, BIT_13);
	#define CONFIG          (CN_ON | CN_IDLE_CON)
	#define INTERRUPT       (CHANGE_INT_ON | CHANGE_INT_PRI_2)
	mCNOpen(CONFIG, CN19_ENABLE, CN19_PULLUP_ENABLE);
	temp = mPORTDRead();

/* PORT D and E for motors */
	//motor 1
	mPORTDSetBits(BIT_4 | BIT_5 | BIT_6 | BIT_7); 		// Turn on PORTD on startup.
	mPORTDSetPinsDigitalOut(BIT_4 | BIT_5 | BIT_6 | BIT_7);	// Make PORTD output.
	//motor 2
	mPORTCSetBits(BIT_1 | BIT_2 | BIT_3 | BIT_4); 		// Turn on PORTC on startup.
	mPORTCSetPinsDigitalOut(BIT_1 | BIT_2 | BIT_3 | BIT_4);	// Make PORTC output.
	//motor 3 and 4
	mPORTESetBits(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7); 		// Turn on PORTE on startup.
	mPORTESetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7);	// Make PORTE output.

// UART2 to connect to the PC.
	// This initialization assumes 36MHz Fpb clock. If it changes,
	// you will have to modify baud rate initializer.
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, GetPeripheralClock(), BAUD);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
	// Configure UART2 RX Interrupt
	INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);


/* PORTD for LEDs - DEBUGGING */
	mPORTDClearBits(BIT_0 | BIT_1 | BIT_2);
	mPORTDSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2);

	

// Congifure Change/Notice Interrupt Flag
	ConfigIntCN(INTERRUPT);
// configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
// enable interrupts
    INTEnableInterrupts();


	counterDistanceMeasure=600; //measure ULTRASONICS distance each 60 ms

	unsigned char dogFound = 0;
	int lastRoom = 0;
	

	while (1) {
	
/***************** Robot MAIN state machine *****************/
		unsigned char ret = 0;
		switch (Robo_State) {
			case 0:
				MotorsON = 0;
				Robo_State = 0;
				servo2_angle = 0;	//debugging
				goFast();

				InvInitialOrientation(RESET);
				TestDog(RESET);
				GoToRoom4short(RESET);
				GoToRoom4long(RESET);
				GoToStartFromRoom4short(RESET);
				GoToStartFromRoom4shortAfterExt(RESET);
				GoToStartFromRoom4long(RESET);
				GoToStartFromRoom4longAfterExt(RESET);
				BackToStart(RESET);
				InitialOrientation(RESET);
				GoToCenter(RESET);
				GoToRoom2(RESET);
				InspectRoom2RightSide(RESET);
				GoToRoom3FromRoom2(RESET);
				GoToCenterFromRoom3(RESET);
				GoToCenterFromRoom3afterExt(RESET);
				GoToRoom1(RESET);
				GoToCenterFromRoom1(RESET);
				GoToStartFromCenter(RESET);
				GoToCenterFromRoom2(RESET);
				GoToCenterFromRoom2point1(RESET);

				ExploreRightRoom1(RESET);
				ExploreCenterRoom1(RESET);

				ScanRoom(RESET);

				ReachFlame(RESET);
				Extinguish(RESET);
				break;
			case 1:
				ret = InitialOrientation(GO);
				if (ret == 1) {
					Robo_State = 2;
				}
				break;
			case 2:
				ret = TestDog(GO);
				if (ret == 1) {			//dog not detected
					dogFound = 0;
					Robo_State = 3;
				} else if (ret == 2) {	//dog detected
					dogFound = 1;
					Robo_State = 6;
				}
				break;
			case 3:
				ret = GoToRoom4short(GO);
				if (ret == 1) {
					Robo_State = 35;
				}
				break;
			case 35:
				degreesTurned = ScanRoom(GO);
				if (degreesTurned == 200) {	// room scaned: no flame found
					Robo_State = 4;	// just to confirm
				} else if (degreesTurned != 300) {	//room scanned: flame found at 'degreesTurned'
					Robo_State = 4;
				}
				break;
			case 4:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 4;
				} else if (ret == 2) {	//flame not found
					Robo_State = 5;
				}
				break;
			case 5:
				ret = GoToStartFromRoom4short(GO);
				if (ret == 1) {
					Robo_State = 7;
				}
				break;
			case 6:
				ret = BackToStart(GO);
				if (ret == 1) {
					Robo_State = 7;
				}
				break;
			case 7:
				ret = InitialOrientation(GO);
				if (ret == 1) {
					Robo_State = 8;
				}
				break;
			case 8:
				ret = GoToCenter(GO);
				if (ret == 1) {
					Robo_State = 9;		///14 for room 3, 17 for room 4
				}
				break;
			case 9:
				ret = GoToRoom2(GO);
				if (ret == 1) {
					Robo_State = 95;
				}
				break;
			case 95:
				degreesTurned = ScanRoom(GO);
				if (degreesTurned == 200) {	// room scaned: no flame found
					Robo_State = 10;	// just to confirm
				} else if (degreesTurned != 300) {	//room scanned: flame found at 'degreesTurned'
					Robo_State = 10;
				}
				break;
			case 10:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 2;
				} else if (ret == 2) {	//flame not found
					Robo_State = 11;
				}
				break;
			/*case 51:	NO LONGER NECESSARY WITH SCAN
				ret = InspectRoom2RightSide(GO);
				if (ret == 1) {
					Robo_State = 52;
				}
				break;
			case 52:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 21;
				} else if (ret == 2) {	//flame not found
					Robo_State = 11;
				}
				break;*/
			case 11:
				ret = GoToRoom3FromRoom2(GO);
				if (ret == 1) {
					Robo_State = 115;
				}
				break;
			case 115:
				degreesTurned = ScanRoom(GO);
				if (degreesTurned == 200) {	// room scaned: no flame found
					Robo_State = 12;	// just to confirm
				} else if (degreesTurned != 300) {	//room scanned: flame found at 'degreesTurned'
					Robo_State = 12;
				}
				break;
			case 12:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 3;
				} else if (ret == 2) {	//flame not found
					Robo_State = 13;
				}
				break;
			case 13:
				ret = GoToCenterFromRoom3(GO);
				if (ret == 1) {
					Robo_State = 14;
				}
				break;
			case 14:
				ret = GoToRoom1(GO);
				if (ret == 1) {
					Robo_State = 145;
					//lastPositionR1 = 1;		// first position NO LONGER NECESSARY TO TRACE
				}
				break;
			case 145:
				degreesTurned = ScanRoom(GO);
				if (degreesTurned == 200) {	// room scaned: no flame found
					Robo_State = 15;	// just to confirm
				} else if (degreesTurned != 300) {	//room scanned: flame found at 'degreesTurned'
					Robo_State = 15;
				}
				break;
			case 15:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 1;
				} else if (ret == 2) {	//flame not found
					Robo_State = 16;
				}
				break;

		/*		// OTHER EXPLORATION CASES FOR ROOM 1
			case 30:
				ret = ExploreRightRoom1(GO);
				if (ret == 1) {
					Robo_State = 31;
					lastPositionR1 = 2;		// second position
				}
				break;
			case 31:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 1;
				} else if (ret == 2) {	//flame not found
					Robo_State = 32;
				}
				break;
			case 32:
				ret = ExploreCenterRoom1(GO);
				if (ret == 1) {
					Robo_State = 33;
					lastPositionR1 = 3;		// third position
				}
				break;
			case 33:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 1;
				} else if (ret == 2) {	//flame not found
					Robo_State = 16;
				}
				break;
		*/		////////////////////////////////////////
				
			case 16:
				degreesTurned = 0;	// this is done so that we can reuse the GoToCenterFromRoom1 function
				ret = GoToCenterFromRoom1(GO);
				if (ret == 1) {
					if (dogFound) {
						Robo_State = 17;
					} else {	//already explored room 4
						Robo_State = 18;
					}
				}
				break;
			case 17:
				ret = GoToRoom4long(GO);
				if (ret == 1) {
					Robo_State = 175;
				}
				break;
			case 175:
				degreesTurned = ScanRoom(GO);
				if (degreesTurned == 200) {	// room scaned: no flame found
					Robo_State = 19;	// just to confirm
				} else if (degreesTurned != 300) {	//room scanned: flame found at 'degreesTurned'
					Robo_State = 19;
				}
				break;

			case 18:
				ret = GoToStartFromCenter(GO);
				if (ret == 1) {
					Robo_State = 0;
				}
				break;

			case 19:
				ret = ReachFlame(GO);
				if (ret == 1) {			//flame found approach finished -> extinguish
					Robo_State = 100;		//EXTINGUISH STATE is 100
					lastRoom = 41;	//room 41 is room 4 but having arrived from the long road
				} else if (ret == 2) {	//flame not found
					Robo_State = 20;
				}
				break;
			case 20:
				ret = GoToStartFromRoom4long(GO);
				if (ret == 1) {
					Robo_State = 0;
				}
				break;

			case 100:
				ret = Extinguish(GO);
				if (ret == 1) {
					Robo_State = 101;
				}
				break;
			case 101:
				switch (lastRoom) {		// THESE are the functions that use 'degreesTurned'
				case 1:
					ret = GoToCenterFromRoom1(GO);
					if (ret == 1) {
						Robo_State = 102;
					}
					break;
				case 2:
					ret = GoToCenterFromRoom2(GO);
					if (ret == 1) {
						Robo_State = 102;
					}
					break;
				/*case 21:
					ret = GoToCenterFromRoom2point1(GO);
					if (ret == 1) {
						Robo_State = 102;
					}
					break;*/
				case 3:
					ret = GoToCenterFromRoom3afterExt(GO);
					if (ret == 1) {
						Robo_State = 102;
					}
					break;
				case 4:
					ret = GoToStartFromRoom4shortAfterExt(GO);
					if (ret == 1) {
						Robo_State = 0;
					}
					break;
				case 41:
					ret = GoToStartFromRoom4longAfterExt(GO);
					if (ret == 1) {
						Robo_State = 0;
					}
					break;
				}
				break;
			case 102:
				ret = GoToStartFromCenter(GO);
				if (ret == 1) {
					Robo_State = 0;
				}
				break;
		}

		if (frontDistance < 30 || backDistance < 30 || leftDistance < 30 || rightDistance < 30)
			mPORTDSetBits(BIT_0);
		else 
			mPORTDClearBits(BIT_0);
/***************************************************************/


/***************** Motors State Machine ************************/
		if (MotorsON) {
			/****************************
			MOTOR MAP
				M1 O-------------O M2   ON EVEN MOTORS, STEPS MUST BE INVERTED
					|	 /\		|			i.e. FORWARD IS BACKWARD
					|	/  \	|
					|	 || 	|
					|	 ||		|
				M3 O-------------O M4
			*****************************/
			if (M1_counter == 0) {
				switch (M1_state) {
					case 0: // set 0011
						step (0x3 , 1);
						if (M1forward)
							M1_state = 1;
						else
							M1_state = 3;
						break;
					case 1: // set 1001
						step (0x9 , 1);
						if (M1forward)
							M1_state = 2;
						else
							M1_state = 0;
						break;
					case 2: // set 1100
						step (0xC , 1);
						if (M1forward)
							M1_state = 3;
						else
							M1_state = 1;
						break;
					case 3: // set 0110
					default:
						step (0x6 , 1);
						if (M1forward)
							M1_state = 0;
						else
							M1_state = 2;
						break;	
				}
				M1_counter = M1_stepPeriod;
				if (step_counter[0] > 0) {
					step_counter[0]--;
				} else if (step_counter[0] < 0) {
					step_counter[0] = 0;
				}
				if (directionNow == countingDirection)
					step_counter[1]--;
			}
			
			if (M2_counter == 0) {
				switch (M2_state) {
					case 0: // set 0011
						step (0x3 , 2);
						if (M2forward)
							M2_state = 1;
						else
							M2_state = 3;
						break;
					case 1: // set 0110
						step (0x6 , 2);
						if (M2forward)
							M2_state = 2;
						else
							M2_state = 0;
						break;
					case 2: // set 1100
						step (0xC , 2);
						if (M2forward)
							M2_state = 3;
						else
							M2_state = 1;
						break;
					case 3: // set 1001
					default:
						step (0x9 , 2);
						if (M2forward)
							M2_state = 0;
						else
							M2_state = 2;
						break;	
				}
				M2_counter = M2_stepPeriod;
			}

			if (M3_counter == 0) {
				switch (M3_state) {
					case 0: // set 0011
						step (0x3 , 3);
						if (M3forward)
							M3_state = 1;
						else
							M3_state = 3;
						break;
					case 1: // set 1001
						step (0x9 , 3);
						if (M3forward)
							M3_state = 2;
						else
							M3_state = 0;
						break;
					case 2: // set 1100
						step (0xC , 3);
						if (M3forward)
							M3_state = 3;
						else
							M3_state = 1;
						break;
					case 3: // set 0110
					default:
						step (0x6 , 3);
						if (M3forward)
							M3_state = 0;
						else
							M3_state = 2;
						break;	
				}
				M3_counter = M3_stepPeriod;
			}
			
			if (M4_counter == 0) {
				switch (M4_state) {
					case 0: // set 0011
						step (0x3 , 4);
						if (M4forward)
							M4_state = 1;
						else
							M4_state = 3;
						break;
					case 1: // set 0110
						step (0x6 , 4);
						if (M4forward)
							M4_state = 2;
						else
							M4_state = 0;
						break;
					case 2: // set 1100
						step (0xC , 4);
						if (M4forward)
							M4_state = 3;
						else
							M4_state = 1;
						break;
					case 3: // set 1001
					default:
						step (0x9 , 4);
						if (M4forward)
							M4_state = 0;
						else
							M4_state = 2;
						break;	
				}
				M4_counter = M4_stepPeriod;
			}
		} else {
			//motors off
			mPORTDSetBits(BIT_4 | BIT_5 | BIT_6 | BIT_7);
			mPORTCSetBits(BIT_1 | BIT_2 | BIT_3 | BIT_4);
			mPORTESetBits(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7);
		}
/***************************************************************/
		

/******* TEST CODE, toggles the servos (from 90 deg. to -90 deg.) every 1 s. ********/
/*		if (auxcounter == 0) {
			
			servo1_angle = 0;

			if (servo2_angle == 90)
				servo2_angle = -90;
			else
				servo2_angle = 90;

			auxcounter = 20000;		// toggle angle every 2 s.
		}
*/

		servo1_angle = 0;
		//servo2_angle = -90;
	/*
		if (frontDistance > 13 && frontDistance < 17) {
			servo2_angle = 90;
		}
		else
			servo2_angle = -90;
	*/
/*******************************************************************/


/****************** SERVO CONTROL ******************/
		/*
			Changing the global servoX_angle at any point in the code will 
			move the servo to the desired angle.
		*/
		servo1_counter = (servo1_angle + 90)*(18)/180 + 6; // between 600 and 2400 us
		if (servo1_period == 0) {
			mPORTASetBits(BIT_2);
			servo1_period = SERVOMAXPERIOD; 		/* 200 * 100us = 20000us period  */
		}

		servo2_counter = (servo2_angle + 90)*(18)/180 + 6; // between 600 and 2400 us
		if (servo2_period == 0) {
			mPORTASetBits(BIT_3);
			servo2_period = SERVOMAXPERIOD; 		/* 200 * 100us = 20000us period  */
		}
/*****************************************************/
	
	} /* end of while(1)  */
		
	return 0;
}

/************** INTERRUPT HANDLERS *******************/

/* TIMER 1 Interrupt Handler - configured to 100us periods */
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

	if (M1_counter != 0)
		M1_counter--;
	if (M2_counter != 0)
		M2_counter--;
	if (M3_counter != 0)
		M3_counter--;
	if (M4_counter != 0)
		M4_counter--;
	
	if (servo1_period != 0) {
		servo1_period--;
		if (servo1_period == (SERVOMAXPERIOD - servo1_counter))
			mPORTAClearBits(BIT_2);
	}
	if (servo2_period != 0) {
		servo2_period--;
		if (servo2_period == (SERVOMAXPERIOD - servo2_counter))
			mPORTAClearBits(BIT_3);
	}

	if(counterDistanceMeasure !=0)
		counterDistanceMeasure--;
	else {
		counterTrigger = 6;
		counterDistanceMeasure = 600;
	}


	if (auxcounter != 0)
		auxcounter--;

	if (xtime != 0)
		xtime--;

	if (commandtime == 0) {
		COMMAND = '0';
	} else {
		commandtime--;
	}
}

/* TIMER 2 Interrupt Handler - configured to 50us periods */
void __ISR(_TIMER_2_VECTOR, ipl3) Timer2Handler(void)
{
    // clear the interrupt flag
    mT2ClearIntFlag();
	counterTrigger--;
	if(counterTrigger==5){
		mPORTBSetBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Sends trigger signal to the four sensors
	}
	if(counterTrigger == 0){
		mPORTBClearBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Shut down trigger signal
	}
	
	delayFront++;
	delayBack++;
	delayRight++;
	delayLeft++;

}

/******************** ULTRASONICS ***********************/
void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl4) IC1Handler(void)
{
    // clear the interrupt flag
    //mIC5ClearIntFlag();
	INTClearFlag(INT_IC1);
	//mPORTDSetBits(BIT_2); 	//DEBUGGING
	D8 = PORTReadBits(IOPORT_D, BIT_8);
	if(D8>0 && timesReadIC1==0){
		//mPORTDSetBits(BIT_0); 
		timeFront=delayFront;
		timesReadIC1++;
	}
	
	if(D8==0 && timesReadIC1==1){
		//mPORTDSetBits(BIT_1); 
		timeFront= delayFront - timeFront;
		timesReadIC1=0;
		frontDistance= timeFront * 10.0 / 58.0; //counterTimer2*periodTimer2 (10us)/58 = cm;
		delayFront=0;
	}

}

void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl4) IC2Handler(void)
{
    // clear the interrupt flag
    //mIC5ClearIntFlag();
	INTClearFlag(INT_IC2);
	//mPORTDSetBits(BIT_2); 	//DEBUGGING
	D9 = PORTReadBits(IOPORT_D, BIT_9);
	if(D9>0 && timesReadIC2==0){
		//mPORTDSetBits(BIT_0); 
		timeBack=delayBack;
		timesReadIC2++;
	}
	
	if(D9==0 && timesReadIC2==1){
		//mPORTDSetBits(BIT_1); 
		timeBack= delayBack - timeBack;
		timesReadIC2=0;
		backDistance= timeBack * 10.0 / 58.0; //counterTimer2*periodTimer2 (10us)/58 = cm;
		delayBack=0;
	}

}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl4) IC3Handler(void)
{
    // clear the interrupt flag
    //mIC5ClearIntFlag();
	INTClearFlag(INT_IC3);
	//mPORTDSetBits(BIT_2); 	//DEBUGGING
	D10 = PORTReadBits(IOPORT_D, BIT_10);
	if(D10>0 && timesReadIC3==0){
		//mPORTDSetBits(BIT_0); 
		timeLeft=delayLeft;
		timesReadIC3++;
	}
	
	if(D10==0 && timesReadIC3==1){
		//mPORTDSetBits(BIT_1); 
		timeLeft= delayLeft - timeLeft;
		timesReadIC3=0;
		leftDistance= timeLeft * 10.0 / 58.0; //counterTimer2*periodTimer2 (10us)/58 = cm;
		delayLeft=0;
	}

}

void __ISR(_INPUT_CAPTURE_4_VECTOR, ipl4) IC4Handler(void)
{
    // clear the interrupt flag
    //mIC5ClearIntFlag();
	INTClearFlag(INT_IC4);
	//mPORTDSetBits(BIT_2); 	//DEBUGGING
	D11 = PORTReadBits(IOPORT_D, BIT_11);
	if(D11>0 && timesReadIC4==0){
		//mPORTDSetBits(BIT_0); 
		timeRight=delayRight;
		timesReadIC4++;
	}
	
	if(D11==0 && timesReadIC4==1){
		//mPORTDSetBits(BIT_1); 
		timeRight= delayRight - timeRight;
		timesReadIC4=0;
		rightDistance= timeRight * 10.0 / 58.0; //counterTimer2*periodTimer2 (10us)/58 = cm;
		delayRight=0;
	}

}
/***********************END of ULTRASONICS********************/


// UART 2 interrupt handler
// it is set at priority level 2

void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
	char temp;
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART2)))
	{

		// Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART2));

		// Code to be executed on RX interrupt:
		temp = UARTGetDataByte(UART2);

		if (temp >= '0' && temp <= '5') {	//ignore unknown chars
			COMMAND = temp;
			commandtime = 30000;
		}
		
		// Echo what we just received.
		PutCharacter(temp);
	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART2)) )
	{
		// Clear the TX interrupt Flag
		INTClearFlag(INT_SOURCE_UART_TX(UART2));

		// Code to be executed on TX interrupt:
			//none
	}
}


// Configure the CN interrupt handler

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void)
{
	unsigned int temp;

    // clear the mismatch condition
    temp = mPORTDRead();

    // clear the interrupt flag
    mCNClearIntFlag();

    // .. things to do .. 
    if ( !(temp & (1<<13)) ) { //button on RD13 is pressed
		if (Robo_State == 0) {
			Robo_State = 1;
			servo2_angle = -90;		// ONLY FOR THE EXTINGUISH TEST
		}
		else {
			debug_state = Robo_State;
			Robo_State = 0;
		}
	}
	
}
/********************************************/


/************ PROCEDURES ********************/
void step (unsigned char data, unsigned char motorNumber) {
	data = data & 0x0F; // make sure only the LS-nibble will be overwritten
	unsigned int lectura;

	switch (motorNumber) {
		case 1:
			lectura = mPORTDRead() & (~(0xF << 4));
			mPORTDWrite(lectura | (data << 4));
			break;
		case 2:
			lectura = mPORTCRead() & (~(0xF << 1));
			mPORTCWrite(lectura | (data << 1));
			break;
		case 3:
		case 4:
			motorNumber -= 3;
			lectura = mPORTERead() & (~(0xF << motorNumber*4));
			mPORTEWrite(lectura | (data << motorNumber*4));
			break;
	}
}

void go (char code) {
	directionNow = code;
	switch (code) {
		case 'F':
			M1forward = M2forward = M3forward = M4forward= 1;
			break;
		case 'B':
			M1forward = M2forward = M3forward = M4forward= 0;
			break;
		case 'L':
			M1forward = M4forward = 0;
			M2forward = M3forward = 1;
			break;
		case 'R':
			M1forward = M4forward = 1;
			M2forward = M3forward = 0;
			break;
		case 'O':	// CW
			M1forward = M3forward = 1;
			M2forward = M4forward = 0;
			break;
		case 'K':	// CCW
			M1forward = M3forward = 0;
			M2forward = M4forward = 1;
			break;
	}
}

void goSlow() {
	M1_stepPeriod = M2_stepPeriod = M3_stepPeriod = M4_stepPeriod = 150; // in tens of u-seconds
}

void goFast() {
	M1_stepPeriod = M2_stepPeriod = M3_stepPeriod = M4_stepPeriod = 50; // in tens of u-seconds
}

void WriteString(const char *string)
{
    while(*string != '\0')
    {
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, *string);

        string++;

        while(!UARTTransmissionHasCompleted(UART2))
            ;
    }
}

void PutCharacter(const char character)
{
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, character);


        while(!UARTTransmissionHasCompleted(UART2))
            ;
}


unsigned char InitialOrientation(char reset) {
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				if ( !(rightDistance < 25 && backDistance < 25 && frontDistance > 25) ) {
					MotorsON = 1;
					step_counter[0] = 90 * SPD;
					go('O');
					state = 1;
				} else {
					return 1;
				}
				break;
			default:
				if (step_counter[0] == 0) {
					MotorsON = 0;
					state = 0;
				}
				break;
		}
	}
	return 0;
}

unsigned char InvInitialOrientation(char reset) {
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				if ( !(rightDistance > 25 && backDistance < 25 && leftDistance < 25) ) {
					MotorsON = 1;
					step_counter[0] = 90 * SPD;
					go('K');
					state = 1;
				} else {
					go('F');
					return 1;
				}
				break;
			default:
				if (step_counter[0] == 0) {
					MotorsON = 0;
					state = 0;
				}
				break;
		}
	}
	return 0;
}

unsigned char GoToCenter(char reset) {
	static int state = 0;
	if (reset) {
		state = 0;
	} else {

		switch (state) {
			case 0:
				go('F');
				MotorsON = 1;
				step_counter[1] = 30 * SPC_front;
				countingDirection = 'F';
				state = 1;
				break;
			case 1:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 2;
				}/*
				else if (rightDistance > 8 && rightDistance < 10) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 3;
				}*/
				else if (rightDistance > 11 || leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 4;
				}

				if (step_counter[1] == 0) {
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('F');
					state = 1;
				}
				break;
			case 5:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 6;
				}
				else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 7;
				}
				else if (leftDistance > 12 && leftDistance < 40) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 8;
				}
				
				if (leftDistance >= 40) {
					step_counter[0] = 15 * SPC_front;
					go('F');
					state = 9;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 8;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 8;
				}
				break;
			case 8:
				if (step_counter[0] == 0) {
					go('F');
					state = 5;
				}
				break;
			case 9:			//on CENTER
				if (step_counter[0] == 0) {
					state = 10;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToRoom2(char reset) {
	static int state = 0;
	if (reset) 
		state = 0;
	else {
		switch (state) {
			case 0:
				go('F');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 99;
				}
				else if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 98;
				}

				if (frontDistance < 10) {
					state = 3;
				}
				break;
			case 99:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 2;
				}
				break;
			case 98:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 2;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					go('F');
					state = 1;
				}
				break;
			case 3:
				step_counter[0] = 90 * SPD;
				go('O');
				state = 4;
				break;
			case 4:
				if (step_counter[0] == 0) {
					state = 5;
				}
				break;
			case 5:
				step_counter[1] = 40 * SPC_front;
				countingDirection = 'F';
				go ('F');
				state = 6;
				break;
			case 6:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 96;
				} else if (leftDistance > 11) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 97;
				}
				if (step_counter[1] == 0) {
					state = 8;
				}
				break;
			case 97:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 7;
				}
				break;
			case 96:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 7;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					go('F');
					state = 6;
				}
				break;
			
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char InspectRoom2RightSide(char reset) {
	static int state = 0;
	if (reset) 
		state = 0;
	else {
		switch (state) {
			case 0:
				MotorsON = 1;
				state = 1;
				step_counter[0] = 85 * SPD;
				go('O');
				break;
			case 1:
				if (step_counter[0] == 0) {
					go('B');
					state = 2;
				}
				break;
			case 2:
				if (backDistance < 12) {
					state = 3;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToRoom3FromRoom2(char reset) {
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				go('B');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 2;
				}/*
				else if (frontDistance > 7 && frontDistance < 9) {
					step_counter[0] = 1 * SPC_front;
					go('F');
					state = 3;
				}*/
				else if (leftDistance >= 12) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 4;
				}

				if (backDistance < 15) {
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 5:
				step_counter[0] = 90 * SPD;
				go('O');
				state = 6;
				break;
			case 6:
				if (step_counter[0] == 0) {
					state = 7;
				}
				break;
			case 7:
				go ('F');
				state = 8;
				break;
			case 8:
				if (rightDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 9;
				}/*
				else if (rightDistance > 7 && rightDistance < 9) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 10;
				}*/
				else if (rightDistance >= 10 && rightDistance < 20) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 11;
				}

				if (rightDistance >= 20) {
					state = 12;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 11;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 11;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					go('F');
					state = 8;
				}
				break;
			case 12:
				step_counter[0] = 15 * SPC_front;
				go('F');
				state = 99;
				break;
			case 99:
				if (step_counter[0] == 0) {
					state = 13;
				}
				break;
			case 13:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 14;
				}/*
				else if (leftDistance > 7 && leftDistance < 9) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 15;
				}*/
				else if (leftDistance >= 10 && leftDistance < 30) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 16;
				}
				if (leftDistance >= 30) {
					step_counter[0] = 25 * SPC_front;
					go('F');
					state = 17;
				}
				break;
			case 14:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 16;
				}
				break;
			case 15:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 16;
				}
				break;
			case 16:
				if (step_counter[0] == 0) {
					go('F');
					state = 13;
				}
				break;
			case 17:	// around the corner to room 3
				if (step_counter[0] == 0) {
					step_counter[0] = 90 * SPD;
					go('K');
					state = 18;
				}
				break;
			case 18:
				if (step_counter[0] == 0) {
					state = 19;
				}
				break;
			case 19:
				go('F');
				state = 20;
				break;
			case 20:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 21;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 22;
				}

				if (frontDistance < 10) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 24;
				}
				break;
			case 21:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 23;
				}
				break;
			case 22:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 23;
				}
				break;
			case 23:
				if (step_counter[0] == 0) {
					go('F');
					state = 20;
				}
				break;
			case 24:
				if (step_counter[0] == 0) { //enter room 3
					step_counter[1] = 25 * SPC_front;
					go('F');
					state = 25;
				}
				break;
			case 25:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 26;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 27;
				}

				if (frontDistance < 23 || step_counter[1] == 0) {
					//step_counter[0] = 30 * SPD;
					//go('O');
					state = 29;
				}
				break;
			case 26:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 28;
				}
				break;
			case 27:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 28;
				}
				break;
			case 28:
				if (step_counter[0] == 0) {
					go('F');
					state = 25;
				}
				break;
			/*case 29:
				if (step_counter[0] == 0) {
					state = 30;
				}
				break;*/
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToStartFromRoom3(char reset) {
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				go('B');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 2;
				}/*
				else if (leftDistance > 9 && leftDistance < 10) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 3;
				}*/
				else if (leftDistance >= 10) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 4;
				}

				if (backDistance < 10) {
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 5:
				go('R');
				state = 6;
				break;
			case 6:
				if (backDistance < 8) {
					step_counter[0] = 1 * SPC_front;
					go('F');
					state = 7;
				}/*
				else if (backDistance > 9 && backDistance < 10) {
					step_counter[0] = 1 * SPC_front;
					go('B');
					state = 8;
				}*/
				else if (backDistance > 10 && backDistance < 20) {
					step_counter[0] = 1 * SPC_front;
					go('B');
					state = 9;
				}

				if (backDistance >= 20) { 
					step_counter[0] = 27 * SPC_side;
					go('R');
					state = 10;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 9;
				}
				break;
			case 8:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 9;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					go('R');
					state = 6;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					state = 11;
				}
				break;
			case 11:
				go('F');
				state = 12;
				break;
			case 12:
				if (leftDistance < 20 || rightDistance < 20) { //until it finds the wall----------------
					state = 13;
				}
				break;
			case 13:
				go('F');
				state = 14;
				break;
			case 14:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 15;
				}else if (rightDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 16;
				}

				if (frontDistance < 10) {
					state = 18;
				}
				break;
			case 15:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 17;
				}
				break;
			case 16:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 17;
				}
				break;
			case 17:
				if (step_counter[0] == 0) {
					go('F');
					state = 14;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToCenterFromRoom3(char reset) {
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				go('B');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				}else if (leftDistance > 11) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 3;
				}

				if (backDistance < 12) {
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 5:
				step_counter[0] = 90 * SPD;
				go('O');
				state = -2;
				break;
			case -2:
				if (step_counter[0] == 0) {
					go('F');
					state = 6;
				}
				break;
			case 6:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 7;
				} else if (rightDistance > 11 && rightDistance < 20) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 8;
				}

				if (rightDistance >= 20) { 
					step_counter[0] = 25 * SPC_front;
					go('F');
					state = 10;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 9;
				}
				break;
			case 8:
				if (step_counter[0] == 0) {
					go('O');
					state = 9;
					step_counter[0] = 2 * SPD;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					go('F');
					state = 6;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 11;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					state = 12;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToCenterFromRoom3afterExt(char reset) {
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				if (degreesTurned < 0) {
					go('B');
					state = 1;
				} else {
					step_counter[0] = degreesTurned * SPD;
					go('K');
					state = -2;
				}
				MotorsON = 1;
				break;
			case -2:
				if (step_counter[0] == 0) {
					go('L');
					state = -1;
				}
				break;
			case -1:
				if (leftDistance < 13) { //(step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				}else if (leftDistance > 11) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 3;
				}

				if (backDistance < 11) {
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 5:
				step_counter[0] = 90 * SPD;
				go('O');
				state = -22;
				break;
			case -22:
				if (step_counter[0] == 0) {
					go('F');
					state = 6;
				}
				break;
			case 6:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 7;
				} else if (rightDistance > 11 && rightDistance < 20) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 8;
				}

				if (rightDistance >= 20) { 
					step_counter[0] = 25 * SPC_front;
					go('F');
					state = 10;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 9;
				}
				break;
			case 8:
				if (step_counter[0] == 0) {
					go('O');
					state = 9;
					step_counter[0] = 2 * SPD;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					go('F');
					state = 6;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 11;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					state = 12;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char TestDog(char reset) {
	// Returns 2 if it finds the dog while walking 20 cm to the front
	static int state = 0;
	if (reset) {
		state = 0;
	} else {

		switch (state) {
			case 0:
				go('L');
				goSlow();
				MotorsON = 1;
				//step_counter[1] = 30 * SPC_front;
				//countingDirection = 'F';
				state = 99;
				break;
			case 99:
				if (rightDistance > 20) {
					go('B');
					state = 1;
				}
			case 1:
				if (backDistance < 6) {
					state = 2;
				}

				if (leftDistance < 45) {
					state = 6;	// DOG FOUND
				}
				break;
			case 2:
				go('F');
				state = 3;
				break;
			case 3:
				if (backDistance > 20) {
					state = 5;
				}
				if (leftDistance < 40) {
					state = 6;	// DOG FOUND
				}
				break;
		/*
			case 4:
				if (step_counter[0] == 0) {
					go('F');
					state = 1;
				}
				break;
		*/
			case 5:
				MotorsON = 0;
				state = 0;
				goFast();
				return 1;	// Dog was not detected
				break;
			case 6:
				MotorsON = 0;
				state = 0;
				goFast();
				return 2;	// Dog was detected
				break;
		}
	}
	return 0;
}

unsigned char GoToRoom4short(char reset) {
	static int state = 0;
	if (reset) 
		state = 0;
	else {
		switch (state) {
			case 0:
				step_counter[0] = 90 * SPD;
				go('K');
				MotorsON = 1;
				state = -1;
				break;
			case -1:
				if (step_counter[0] == 0) {
					step_counter[1] = 20 * SPC_front;
					countingDirection = 'F';
					go('F');
					state = 99;
				}
				break;
			case 99:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 98;
				}/*
				else if (leftDistance > 8 && leftDistance < 10) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 97;
				}*/
				else if (leftDistance > 11) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 96;
				}

				if (step_counter[1] == 0) {
					go('F');
					state = 1;
				}
				break;
			case 98:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 96;
				}
				break;
			case 97:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 96;
				}
				break;
			case 96:
				if (step_counter[0] == 0) {
					go('F');
					state = 99;
				}
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				}/*
				else if (leftDistance > 8 && leftDistance < 10) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 3;
				}*/
				else if (leftDistance > 11) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 4;
				}

				if (rightDistance > 30) {
					step_counter[1] = 25 * SPC_front;
					countingDirection = 'F';
					go('F');
					state = 6;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('F');
					state = 1;
				}
				break;
			case 5:
				go ('F');
				state = 6;
				break;
			case 6:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 95;
				}/*
				else if (leftDistance > 8 && leftDistance < 10) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 94;
				}*/
				else if (leftDistance > 12) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 94;
				}

				if (step_counter[1] == 0) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 7;
				}
				break;
			case 95:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 93;
				}
				break;
			case 94:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 93;
				}
				break;
			case 93:
				if (step_counter[0] == 0) {
					go('F');
					state = 6;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					go('F');
					state = 8;
				}
				break;
			case 8:
				if (frontDistance < 30) {
					state = 10;
				}

				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 9;
				} else if (rightDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 9;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					go('F');
					state = 8;
				}
				break;
			/*case 10:
				step_counter[0] = 30 * SPD;
				go('O');
				state = 11;
				break;
			case 11:
				if (step_counter[0] == 0) {
					state = 12;
				}*/
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToRoom4long(char reset) {
	static int state = 0;
	if (reset) 
		state = 0;
	else {
		switch (state) {
			case 0:
				step_counter[0] = 90 * SPD;
				go('K');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (step_counter[0] == 0) {
					go('F');
					state = 2;
				}
				break;
			case 2:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 3;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 4;
				}

				if (frontDistance < 11) {
					step_counter[0] = 90 * SPD;
					go('K');
					state = 6;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 5;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 5;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					go('F');
					state = 2;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					go('F');
					state = 7;
				}
				break;
			case 7:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 8;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 9;
				}

				if (frontDistance < 11) {
					step_counter[0] = 90 * SPD;
					go('K');
					state = 11;
				}
				break;
			case 8:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 10;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 10;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					go('F');
					state = 7;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					go('F');
					state = 12;
				}
				break;
			case 12:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 50;
				}

				if (leftDistance < 25) { // passing the thin wall
					step_counter[1] = 23 * SPC_front;
					countingDirection = 'F';
					go('F');
					state = 14;
				}
				break;
			case 50:	//aux
				if (step_counter[0] == 0) {
					step_counter[0] = 1 * SPD;
					go('K');
					state = 13;
				}
				break;
			case 13:
				if (step_counter[0] == 0) {
					go('F');
					state = 12;
				}
				break;
			case 14:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 51;
				}

				if (step_counter[1] == 0) { // passed the wall
					step_counter[0] = 90 * SPD;
					go('K');
					state = 16;
				}
				break;
			case 51:	//aux
				if (step_counter[0] == 0) {
					step_counter[0] = 1 * SPD;
					go('K');
					state = 15;
				}
				break;
			case 15:
				if (step_counter[0] == 0) {
					go('F');
					state = 14;
				}
				break;
			case 16:
				if (step_counter[0] == 0) {
					go('F');
					state = 17;
				}
				break;
			case 17:
				if (frontDistance < 30) {
					//step_counter[0] = 30 * SPD;
					//go('O');
					state = 19;
				}

				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 18;
				} else if (rightDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 18;
				}
				break;
			case 18:
				if (step_counter[0] == 0) {
					go('F');
					state = 17;
				}
				break;
			/*case 19:
				if (step_counter[0] == 0) {
					state = 20;
				}
				break;*/
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char BackToStart(char reset) {
	static int state = 0;
	if (reset) {
		state = 0;
	} else {
		switch (state) {
			case 0:
				go('R');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (backDistance < 8) {
					step_counter[0] = 0.5 * SPC_front;
					go('F');
					state = 2;
				}/*
				else if (leftDistance > 8 && leftDistance < 10) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 3;
				}*/
				else if (backDistance > 11) {
					step_counter[0] = 0.5 * SPC_front;
					go('B');
					state = 4;
				}

				if (rightDistance < 11) {
					state = 6;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 1 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 1 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('R');
					state = 1;
				}
				break;
			/*case 5:
				if (step_counter[0] == 0) {
					state = 6;
				}
				break;*/
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToRoom1(char reset) {
	static int state = 0;
	if (reset) 
		state = 0;
	else {
		switch (state) {
			case 0:
				go('F');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 99;
				}
				else if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 98;
				}

				if (leftDistance < 25) { // passing the thin wall
					step_counter[1] = 22 * SPC_front;
					countingDirection = 'F';
					go('F');
					state = 3;
				}
				break;
			case 99:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 2;
				}
				break;
			case 98:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 2;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					go('F');
					state = 1;
				}
				break;
			case 3:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 4;
				} else if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 4;
				}

				if (step_counter[1] == 0) { // passed the wall
					step_counter[0] = 90 * SPD;
					go('K');
					state = 5;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('F');
					state = 3;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					step_counter[1] = 41 * SPC_front;
					countingDirection = 'F';
					go('F');
					state = 6;
				}
				break;
			case 6:
				if (rightDistance < 15) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 7;
				} else if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 8;
				}

				if (backDistance > 56 || step_counter[1] == 0) {
					//step_counter[0] = 20 * SPD;	// to avoid noise
					//go('O');
					state = 10;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 9;
				}
				break;
			case 8:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 9;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					go('F');
					state = 6;
				}
				break;
			/*case 10:
				if (step_counter[0] == 0) {
					state = 11;
				}
				break;*/
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToCenterFromRoom1(char reset) {
	static int state = 0;
	if (reset) {
		state = 0;
	} else {
		switch (state) {
			case 0:
				goFast();
				if (degreesTurned < 0) {
					step_counter[0] = abs(degreesTurned) * SPD;
					go('O');
					state = -1;
				} else if (degreesTurned < 1){
					go('B');
					state = -2;
				} else {
					step_counter[0] = degreesTurned * SPD;
					go('K');
					state = -1;
				}
				MotorsON = 1;
				break;
			case -1:
				if (step_counter[0] == 0) {
					go('B');
					state = -2;
				}
				break;
			case -2:
				if (frontDistance > 45 || backDistance < 10 || leftDistance < 10) {
					go('L');
					state = -3;
				}
				break;
			case -3:
				if (leftDistance < 9) {
					go('B');
					state = 1;
				}
				break;
			case 1:
				if (leftDistance < 9) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				}
				if (rightDistance < 8 || (leftDistance > 12 && leftDistance < 30)) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 3;
				}

				if (backDistance < 11) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					step_counter[0] = 10 * SPC_front;
					go('B');
					state = 6;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					state = 7;
				}
				break;
			case 7:
				if (rightDistance < 10) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 8;
				}/*
				else if (leftDistance > 7 && leftDistance < 9) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 9;
				}*/
				else if (rightDistance > 12 && rightDistance < 30) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 9;
				}
				if (rightDistance >= 30) {
					state = 11;
				}
				break;
			case 8:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 10;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 10;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					go('B');
					state = 7;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToStartFromCenter(char reset) {
	static int state = 0;
	if (reset) {
		state = 0;
	} else {
		switch (state) {
			case 0:
				MotorsON = 1;
				go('B');
				state = 1;
				break;
			case 1:
				if (leftDistance < 20 || rightDistance < 20) { //until it finds the wall----------------
					state = 2;
				}
				break;
			case 2:
				go('B');
				state = 3;
				break;
			case 3:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 4;
				}else if (rightDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 5;
				}

				if (backDistance < 10) {
					state = 7;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 6;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 6;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					go('B');
					state = 3;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToStartFromRoom4short(char reset) {
	static int state = 0;
	if (reset) 
		state = 0;
	else {
		switch (state) {
			case 0:
				go('B');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 2;
				}

				if (backDistance < 15) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 3;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 4:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 5;
				}
				else if (rightDistance > 12) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 6;
				}

				if (frontDistance < 15) {
					state = 8;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 7;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 7;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 8:
				step_counter[0] = 90 * SPD;
				go ('K');
				state = 9;
				break;
			case 9:
				if (step_counter[0] == 0) {
					state = 10;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToStartFromRoom4shortAfterExt(char reset) {
	static int state = 0;
	if (reset) 
		state = 0;
	else {
		switch (state) {
			case 0:
				if (degreesTurned < 0) {
					step_counter[0] = abs(degreesTurned) * SPD;
					go('O');
				} else {
					step_counter[0] = degreesTurned * SPD;
					go('K');
				}
				
				MotorsON = 1;
				state = -1;
				break;
			case -1:
				if(step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 2;
				}

				if (backDistance < 11) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 3;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 4:
				if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 5;
				}
				else if (rightDistance > 12) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 6;
				}

				if (frontDistance < 15) {
					state = 8;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 7;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 7;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 8:
				step_counter[0] = 90 * SPD;
				go ('K');
				state = 9;
				break;
			case 9:
				if (step_counter[0] == 0) {
					state = 10;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToStartFromRoom4long(char reset) {
	static int state = 0;
	static int nextstate = 0;
	if (reset) {
		state = 0;
		nextstate = 0;
	}
	else {
		switch (state) {
			case 0:
				step_counter[0] = 30 * SPD;
				go('B');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 2;
				}

				if (backDistance < 10) {
					step_counter[0] = 90 * SPD;
					go('K');
					state = 3;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 4:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 5;
				}
				else if (leftDistance > 13) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 6;
				}

				if (frontDistance < 12) {
					state = 8;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 7;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 7;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 8:
				step_counter[0] = 90 * SPD;
				go('O');
				state = 9;
				break;
			case 9:
				if (step_counter[0] == 0) {
					go('F');
					state = 10;
					nextstate = 14;
				}
				break;
			case 10:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 11;
				}
				else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 12;
				}

				if (rightDistance > 40 && backDistance > 55) {
					step_counter[0] = 23 * SPC_front;
					go('F');
					state = nextstate;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 13;
				}
				break;
			case 12:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 13;
				}
				break;
			case 13:
				if (step_counter[0] == 0) {
					go('F');
					state = 10;
				}
				break;
			case 14:
				if (step_counter[0] == 0) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 15;
				}
				break;

			case 15:
				if (step_counter[0] == 0) {
					go('F');
					state = 10;
					nextstate = 16;
				}
				break;
			case 16:
				if (step_counter[0] == 0) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 17;
				}
				break;
			case 17:
				if (step_counter[0] == 0) {
					go('F');
					state = 18;
				}
				break;
			case 18:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 19;
				}
				else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 20;
				}

				if (frontDistance < 15) {
					step_counter[0] = 180 * SPD;
					go('O');
					state = 22;
				}
				break;
			case 19:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 21;
				}
				break;
			case 20:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 21;
				}
				break;
			case 21:
				if (step_counter[0] == 0) {
					go('F');
					state = 18;
				}
				break;
			case 22:
				if (step_counter[0] == 0) {
					state = 23;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				nextstate = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToStartFromRoom4longAfterExt(char reset) {
	static int state = 0;
	static int nextstate = 0;
	if (reset) {
		state = 0;
		nextstate = 0;
	}
	else {
		switch (state) {
			case 0:
				if (degreesTurned < 0) {
					go('O');
					step_counter[0] = abs(degreesTurned) * SPD;
				} else {
					go('K');
					step_counter[0] = degreesTurned * SPD;
				}
				state = -1;
				MotorsON = 1;
				break;
			case -1:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 2;
				} else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 2;
				}

				if (backDistance < 10) {
					step_counter[0] = 90 * SPD;
					go('K');
					state = 3;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 4:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 5;
				}
				else if (leftDistance > 12) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 6;
				}

				if (frontDistance < 12) {
					state = 8;
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 7;
				}
				break;
			case 6:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 7;
				}
				break;
			case 7:
				if (step_counter[0] == 0) {
					go('F');
					state = 4;
				}
				break;
			case 8:
				step_counter[0] = 90 * SPD;
				go('O');
				state = 9;
				break;
			case 9:
				if (step_counter[0] == 0) {
					go('F');
					state = 10;
					nextstate = 14;
				}
				break;
			case 10:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 11;
				}
				else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 12;
				}

				if (rightDistance > 40 && backDistance > 55) {
					step_counter[0] = 23 * SPC_front;
					go('F');
					state = nextstate;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 13;
				}
				break;
			case 12:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 13;
				}
				break;
			case 13:
				if (step_counter[0] == 0) {
					go('F');
					state = 10;
				}
				break;
			case 14:
				if (step_counter[0] == 0) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 15;
				}
				break;

			case 15:
				if (step_counter[0] == 0) {
					go('F');
					state = 10;
					nextstate = 16;
				}
				break;
			case 16:
				if (step_counter[0] == 0) {
					step_counter[0] = 90 * SPD;
					go('O');
					state = 17;
				}
				break;
			case 17:
				if (step_counter[0] == 0) {
					go('F');
					state = 18;
				}
				break;
			case 18:
				if (leftDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 19;
				}
				else if (rightDistance < 8) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 20;
				}

				if (frontDistance < 15) {
					step_counter[0] = 180 * SPD;
					go('O');
					state = 22;
				}
				break;
			case 19:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 21;
				}
				break;
			case 20:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 21;
				}
				break;
			case 21:
				if (step_counter[0] == 0) {
					go('F');
					state = 18;
				}
				break;
			case 22:
				if (step_counter[0] == 0) {
					state = 23;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				nextstate = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToCenterFromRoom2(char reset) {
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				if (degreesTurned < 0) {
					step_counter[0] = abs(degreesTurned) * SPD;
					go('O');
					state = -3;
				} else if (degreesTurned <= 90) {
					go('O');
					step_counter[0] = (90 - degreesTurned) * SPD;
					state = -1;
				} else if (degreesTurned > 90) {
					go('K');
					step_counter[0] = (degreesTurned - 90) * SPD;
					state = -1;
				}
				MotorsON = 1;
				break;
			case -1:
				if (step_counter[0] == 0) {
					go('B');
					state = -2;
				}
				break;
			case -2:
				if (backDistance < 12) {
					step_counter[0] = 90 * SPD;
					go('K');
					state = -3;
				}
				break;
			case -3:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 2;
				}/*
				else if (leftDistance > 10 && leftDistance < 13) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 3;
				}*/
				else if (leftDistance >= 13) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 4;
				}

				if (backDistance < 15) {
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 5:
				step_counter[0] = 90 * SPD;
				go('O');
				state = 6;
				break;
			case 6:
				if (step_counter[0] == 0) {
					state = 7;
				}
				break;
			case 7:
				go ('F');
				state = 8;
				break;
			case 8:
				if (rightDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 9;
				}/*
				else if (rightDistance > 7 && rightDistance < 9) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 10;
				}*/
				else if (rightDistance >= 10 && rightDistance < 20) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 11;
				}

				if (rightDistance >= 20) {
					state = 12;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 11;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 11;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					go('F');
					state = 8;
				}
				break;
			case 12:
				step_counter[0] = 15 * SPC_front;
				go('F');
				state = 99;
				break;
			case 99:
				if (step_counter[0] == 0) {
					state = 13;
				}
				break;
			case 13:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 14;
				}/*
				else if (leftDistance > 7 && leftDistance < 9) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 15;
				}*/
				else if (leftDistance >= 10 && leftDistance < 30) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 16;
				}
				if (leftDistance >= 30) {
					step_counter[0] = 180 * SPD;
					go('O');
					state = 17;
				}
				break;
			case 14:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 16;
				}
				break;
			case 15:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 16;
				}
				break;
			case 16:
				if (step_counter[0] == 0) {
					go('F');
					state = 13;
				}
				break;
			case 17:
				if (step_counter[0] == 0) {
					state = 18;
				}
				break;
			
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char GoToCenterFromRoom2point1(char reset) {	// NOT USED anymore
	static int state = 0;
	if (reset) state = 0;
	else {
		switch (state) {
			case 0:
				go('B');
				MotorsON = 1;
				state = -2;
				break;
			case -2:
				if (backDistance < 15) {
					go('K');
					step_counter[0] = 83 * SPD; //////
					state = -1;
				}
				break;
			case -1:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 1:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 2;
				}
				else if (leftDistance > 12 && leftDistance < 15) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 3;
				}
				else if (leftDistance >= 15) {
					step_counter[0] = 2 * SPC_side;
					go('L');
					state = 4;
				}

				if (backDistance < 13) {
					state = 5;
				}
				break;
			case 2:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 4;
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 4;
				}
				break;
			case 4:
				if (step_counter[0] == 0) {
					go('B');
					state = 1;
				}
				break;
			case 5:
				step_counter[0] = 90 * SPD;
				go('O');
				state = 6;
				break;
			case 6:
				if (step_counter[0] == 0) {
					state = 7;
				}
				break;
			case 7:
				go ('F');
				state = 8;
				break;
			case 8:
				if (rightDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 9;
				}/*
				else if (rightDistance > 7 && rightDistance < 9) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 10;
				}*/
				else if (rightDistance >= 10 && rightDistance < 20) {
					step_counter[0] = 2 * SPC_side;
					go('R');
					state = 11;
				}

				if (rightDistance >= 20) {
					state = 12;
				}
				break;
			case 9:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 11;
				}
				break;
			case 10:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 11;
				}
				break;
			case 11:
				if (step_counter[0] == 0) {
					go('F');
					state = 8;
				}
				break;
			case 12:
				step_counter[0] = 15 * SPC_front;
				go('F');
				state = 99;
				break;
			case 99:
				if (step_counter[0] == 0) {
					state = 13;
				}
				break;
			case 13:
				if (leftDistance < 8) {
					step_counter[0] = 1 * SPC_side;
					go('R');
					state = 14;
				}/*
				else if (leftDistance > 7 && leftDistance < 9) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 15;
				}*/
				else if (leftDistance >= 10 && leftDistance < 30) {
					step_counter[0] = 1 * SPC_side;
					go('L');
					state = 16;
				}
				if (leftDistance >= 30) {
					step_counter[0] = 180 * SPD;
					go('O');
					state = 17;
				}
				break;
			case 14:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('O');
					state = 16;
				}
				break;
			case 15:
				if (step_counter[0] == 0) {
					step_counter[0] = 2 * SPD;
					go('K');
					state = 16;
				}
				break;
			case 16:
				if (step_counter[0] == 0) {
					go('F');
					state = 13;
				}
				break;
			case 17:
				if (step_counter[0] == 0) {
					state = 18;
				}
				break;
			
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char ReachFlame(char reset) {
	static int state = 0;
	if (reset) {
		state = 0;
	} else {
		switch (state) {
			case 0:
				xtime = 15000;
				state = 1;
				break;
			case 1:
				if (xtime == 0) {
					if (COMMAND == '0') {	//flama no encontrada
						MotorsON = 0;
						state = 0;
						goFast();
						return 2;	// TERMINA
					} else {				//iniciar approach
						state = 2;
						go('F');
						MotorsON = 1;
						goSlow();
					}
				}
				break;
			case 2:
				if (xtime == 0) {
					MotorsON = 1;
					if (rightDistance < 8) {
						step_counter[0] = 2 * SPC_side;
						go('L');
						state = 3;
					} else if (leftDistance < 8) {
						step_counter[0] = 2 * SPC_side;
						go('R');
						state = 3;
					}

					if (COMMAND == '1') {
						step_counter[0] = 10 * SPD;
						go('K');
						state = 3;
					} else if (COMMAND == '2') {
						step_counter[0] = 5 * SPD;
						go('K');
						state = 3;
					} else if (COMMAND == '4') {
						step_counter[0] = 5 * SPD;
						go('O');
						state = 3;
					} else if (COMMAND == '5') {
						step_counter[0] = 10 * SPD;
						go('O');
						state = 3;
					}

					if (frontDistance < 15) {	//flame base found
						state = 4;
					}
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					go('F');
					xtime = 20000;
					MotorsON = 0;
					state = 2;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char Extinguish(char reset) {
	static int state = 0;
	static int tries = 0; //Numero de intentos de apagar la flama, para cambiar la estrategia
	static int flameDistanceLimit = 15;
	if (reset) {
		state = 0;
		tries = 0;
		flameDistanceLimit = 15;
	} else {
		switch (state) {
			case 0:
				if (frontDistance > 11 && frontDistance < flameDistanceLimit && COMMAND == '3') {
					// ready to extinguish
					servo2_angle = 90;
					xtime = 30000;
					state = 1;
				}
				if (COMMAND == '1' || COMMAND == '2') {
					go('K');
					MotorsON = 1;
					state = 3;
					step_counter[0] = 1 * SPD;
				} else if (COMMAND == '4' || COMMAND == '5') {
					go('O');
					MotorsON = 1;
					state = 3;
					step_counter[0] = 1 * SPD;
				}
				if (frontDistance <= 11) {
					step_counter[0] = 0.5 * SPC_front;
					go('B');
					MotorsON = 1;
					state = 3;
				}
				if (frontDistance > flameDistanceLimit) {
					step_counter[0] = 0.5 * SPC_front;
					go('F');
					MotorsON = 1;
					state = 3;
				}
				if (xtime == 0 && tries > 0) {
					state = 1;
				}
				break;
			case 1:
				if (xtime == 0) {
					servo2_angle = -90;
					xtime = 20000;
					state = 2;
				}
				break;
			case 2:
				if (xtime == 0) {
					if (COMMAND == '0') { // flame was extinguished
						state = 99;
					} else {
						if(tries % 4 < 3) {	// try again
							state = 0;
							tries++;
							xtime = 30000;
						} else { 		//retrocede por si la vela quedó muy cerca
							go('B');
							MotorsON = 1;
							tries++;
							flameDistanceLimit += 2;
							state = 3;
							step_counter[0] = 2 * SPC_front;
						}
					}
				}
				break;
			case 3:
				if (step_counter[0] == 0) {
					MotorsON = 0;
					state = 0;
					xtime = 30000;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				tries = 0;
				flameDistanceLimit = 15;
				goFast();
				return 1;
				break;
		}
	}
	return 0;
}

unsigned char ExploreRightRoom1(char reset) {
	static int state = 0;
	if (reset) {
		state = 0;
	} else {
		switch (state) {
			case 0:
				step_counter[0] = 70 * SPD;
				go('O');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (step_counter[0] == 0) {
					MotorsON = 0;
					state = 0;
					return 1;
				}
				break;
		}
	}
	return 0;
}

unsigned char ExploreCenterRoom1(char reset) {
	static int state = 0;
	if (reset) {
		state = 0;
	} else {
		switch (state) {
			case 0:
				step_counter[0] = 50 * SPD;
				go('K');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (step_counter[0] == 0) {
					go('F');
					step_counter[0] = 30 * SPC_front;
					state = 2;
				}
				break;
			case 2:
				if (step_counter[0] == 0 || frontDistance < 20 || COMMAND != '0') {
					state = 3;
				}
				break;
			default:
				MotorsON = 0;
				state = 0;
				goFast();
				return 1;
				break;
		}
	}
	return 0;
}

int ScanRoom(char reset) {
// Returns 200 if no flame was found
// Returns degrees from -60 to 110 where the flame was found
// Returns 300 if nothing happened. General state machine should not switch state
	const int compensation = 5 * SPD;
	static int state = 0;
	static int degrees = 200;
	int temp;
	if (reset) {
		state = 0;
		degrees = 200;
	} else {
		switch (state) {
			case 0:
				goSlow();
				step_counter[0] = 110 * SPD;
				go('O');
				MotorsON = 1;
				state = 1;
				break;
			case 1:
				if (step_counter[0] == 0) {	// no flame was seen
					goFast();
					MotorsON = 0; //stop
					xtime = 10000; //1 sec. pause to process camera
					state = 2;
				}
				if (COMMAND != '0') {	// flame was seen
					degrees = 110 - (step_counter[0] / SPD) - compensation; //'degrees' holds the angle until flame was seen
					if (degrees < 0) degrees = 0;
					if (COMMAND == '3'){
						state = 100;
					} else if (COMMAND == '4' || COMMAND == '5') {
						step_counter[0] = 60 * SPD;	//turn only 60 degrees more
						// go O, not necessary
						state = 3;
					} else if (COMMAND == '1' || COMMAND == '2') {
						step_counter[0] = 60 * SPD;	//turn only 60 degrees more
						go('K');
						state = 4;
					}
				}
				break;
			case 2:
				if (xtime == 0) {
					go('K');	// Return to original orientation
					MotorsON = 1;
					step_counter[0] = 110 * SPD;
					state = 21;
				}
				if (COMMAND != '0') {	// flame was seen
					degrees = 110 - (step_counter[0] / SPD) - compensation; //'degrees' holds the angle until flame was seen
					if (degrees < 0) degrees = 0;
					if (COMMAND == '3'){
						state = 100;
					} else if (COMMAND == '4' || COMMAND == '5') {
						step_counter[0] = 60 * SPD;	//turn only 60 degrees more
						// go O, not necessary
						MotorsON = 1;
						state = 3;
					} else if (COMMAND == '1' || COMMAND == '2') {
						step_counter[0] = 60 * SPD;	//turn only 60 degrees more
						go('K');
						MotorsON = 1;
						state = 4;
					}
				}
				break;
			case 21:
				if (step_counter[0] == 0) {
					state = 100;
				}
				break;

			case 3:
				if (COMMAND == '3') {
					degrees = degrees + (60 - (step_counter[0] / SPD));
					state = 100;
				}
				if (COMMAND == '0' || step_counter[0] == 0) {
				//ERROR, false alarm. Return steps and go to case 0
					degrees = degrees + (60 - (step_counter[0] / SPD));
					step_counter[0] = degrees * SPD;
					go('K');
					state = 5;
				}
				break;
			case 4:
				if (COMMAND == '3') {
					degrees = degrees - (60 - (step_counter[0] / SPD));
					state = 100;
				}
				if (COMMAND == '0' || step_counter[0] == 0) {
				//ERROR, false alarm. Return steps and go to case 0
					degrees = degrees - (60 - (step_counter[0] / SPD));
					if (degrees < 0) {
						step_counter[0] = abs(degrees) * SPD;
						go('O');
						state = 5;
					} else {
						step_counter[0] = degrees * SPD;
						go('K');
						state = 5;
					}
					
				}
				break;
			case 5:
				if (step_counter[0] == 0) {
					state = 0;
				}
				break;

			default:
				temp = degrees;
				MotorsON = 0;
				state = 0;
				degrees = 200;
				goFast();
				return temp;
				break;
		}
	}
	return 300;	//Return value if nothing happened
}

/**************************************************/
