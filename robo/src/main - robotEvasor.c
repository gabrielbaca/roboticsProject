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
/***************************************/


/*************GLOBALS**********************/
unsigned char COMMAND;
unsigned char Robo_State = 0;
unsigned char MotorsON = 0;
unsigned char M1forward = 1, M2forward = 1, M3forward = 1, M4forward = 1;
unsigned int M1_counter = 0, M2_counter = 0, M3_counter = 0, M4_counter = 0;

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

int auxcounter = 10000; //for the servo, just for testing
/******************************************/

int main(void)
{
//LOCALS
	unsigned int temp;
	unsigned int channel1, channel2;
	unsigned int M1_stepPeriod, M2_stepPeriod, M3_stepPeriod, M4_stepPeriod;
	M1_stepPeriod = M2_stepPeriod = M3_stepPeriod = M4_stepPeriod = 1000; // in tens of u-seconds
	unsigned char M1_state = 0, M2_state = 0, M3_state = 0, M4_state = 0;
	unsigned int step_counter;

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

	while (1) {

/*
		//Process UART command
		switch (COMMAND) {
			case 'q':
				if (MotorsON)
					MotorsON = 0;
				else
					MotorsON = 1;
				break;
			case 'w':
				M1forward = M2forward = 1;
				break;
			case 's':
				M1forward = M2forward = 0;
				break;
			case 'a': //left
				M1forward = 0;
				M2forward = 1;
				break;
			case 'd': //right
				M1forward = 1;
				M2forward = 0;
				break;
			
			case 0:
			default:
				break;
		}
		COMMAND = 0;
*/	
		/***************** Robot MAIN state machine *****************/
		switch (Robo_State) {
			case 0:
				MotorsON = 0;
				Robo_State = 0;
				step_counter = 0;
				break;
			case 1:	//forward
				/*
				if (step_counter == 0) {
					step_counter = 50 * SPC_front; //1 cm es aprox. = 11.3636 pasos
					MotorsON = 1;
					M1forward = M2forward = M3forward = M4forward= 1;
					Robo_State = 2;
				}
				*/
				go ('F');
				MotorsON = 1;
				Robo_State = 2;
				break;
			case 2:
				if (frontDistance < 20) {
					go('B');
				}
				if (backDistance < 20) {
					go('F');
				}
				if (rightDistance < 20) {
					go('L');
				}
				if (leftDistance < 20) {
					go('R');
				}
				break;
			default:
				if (step_counter == 0) {
					Robo_State = 0;
				}
				break;
		}
		if (frontDistance < 20 || backDistance < 20 || leftDistance < 20 || rightDistance < 20)
			mPORTDSetBits(BIT_0);
		else 
			mPORTDClearBits(BIT_0);
		/***************************************************************/

		M1_stepPeriod = 50; // value between 50 and 100 in tens of u-seconds (step period)
		M2_stepPeriod = 50;
		M3_stepPeriod = 50; // influences the speed of the wheels
		M4_stepPeriod = 50;
			
		if (MotorsON) {
			/****************************
			MOTOR MAP
				M1 O-------------O M2   ON EVEN MOTORS, STEPS MUST BE INVERTE
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
				step_counter--;
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
			if (step_counter == 0)
				MotorsON = 0;
		} else {
			//motors off
			mPORTDSetBits(BIT_4 | BIT_5 | BIT_6 | BIT_7);
			mPORTCSetBits(BIT_1 | BIT_2 | BIT_3 | BIT_4);
			mPORTESetBits(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7);
		}
		
		/******* TEST CODE, toggles the servos (from 90 deg. to -90 deg.) every 1 s. ********/
		if (auxcounter == 0) {
			if (servo1_angle == 90)
				servo1_angle = -90;
			else
				servo1_angle = 90;

			if (servo2_angle == 90)
				servo2_angle = -90;
			else
				servo2_angle = 90;

			auxcounter = 10000;		// toggle angle every 1 s.
		}
		/***********************************************************************************/

		/******* SERVO CONTROL ********/
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
		/*******************************/
	
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
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART2)))
	{
		unsigned char databyte;

		// Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART2));

		// Code to be executed on RX interrupt:
		databyte = UARTGetDataByte(UART2);
		databyte++;
		
		// Echo what we just received.
		PutCharacter(databyte);
	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART2)) )
	{
		// Clear the TX interrupt Flag
		INTClearFlag(INT_SOURCE_UART_TX(UART2));

		// Code to be executed on TX interrupt:

		
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
		if (Robo_State == 0)
			Robo_State = 1;
		else {
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
			M1forward = M3forward = 0;
			M2forward = M4forward = 1;
			break;
		case 'K':	// CCW
			M1forward = M3forward = 1;
			M2forward = M4forward = 0;
			break;
	}
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

/**************************************************/
