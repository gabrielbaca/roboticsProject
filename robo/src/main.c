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
#define PB2_DIV				4
#define PRESCALE       		1
#define TOGGLES_PER_SEC		10000
#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define T2_TICK				(SYS_FREQ/PB2_DIV/PRESCALE/TOGGLES_PER_SEC)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())
#define DESIRED_BAUDRATE    	(9600)      //The desired UART BaudRate

#define SPC_front	11.4777		//Steps per cm.
#define SPC_side	14.7059
#define SPD			3.408 		//Steps per degree

/*****************************************************************************/


/********PROCEDURE declarations*********/
void WriteString(const char *string);
void PutCharacter(const char character);
void step (unsigned char data, unsigned char motorNumber);
/***************************************/


/*************GLOBALS**********************/
unsigned char COMMAND;
unsigned char Path_State = 0;
unsigned char MotorsON = 0;
unsigned char M1forward = 1, M2forward = 1, M3forward = 1, M4forward = 1;
unsigned int M1_counter = 0, M2_counter = 0, M3_counter = 0, M4_counter = 0;
unsigned int counterDistanceMeasure = 0, counterTrigger=0, counterEcho=0;
unsigned char delayFront = 0, delayBack = 0, delayLeft = 0, delayRight=0;
unsigned char frontDistance=0, backDistance=0, leftDistance=0, rightDistance=0; //in cms

#define SERVOMAXPERIOD 200
unsigned int servo_counter = 0, servo_period = 200;
int servo_angle = 0;

int auxcounter = 10000; //for the servo?
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
/* TIMER2 - 20 khz interrupt for distance measure*/
	OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, T2_TICK);
	ConfigIntTimer2(T2_INT_OFF | T2_INT_PRIOR_2); //It is off until trigger

/* PORTA.0 for servo-PWM */
	mPORTAClearBits(BIT_2);
	mPORTASetPinsDigitalOut(BIT_2);

/* some bits of PORTB for ultrasonic sensors */
	PORTResetPins(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11 );	
	PORTSetPinsDigitalOut(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11); //trigger
	//PORTSetPinsDigitalIn(IOPORT_B, BIT_12 | BIT_13| BIT_14 | BIT_15); //echo

/* external interrupts for echo signals */
	ConfigINT1(EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_DISABLE); //RE8, highest priority
	ConfigINT2(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_DISABLE); //RE9
	ConfigINT3(EXT_INT_PRI_3 | RISING_EDGE_INT | EXT_INT_DISABLE); //RA14
	ConfigINT4(EXT_INT_PRI_4 | RISING_EDGE_INT | EXT_INT_DISABLE); //RA15

/* PINS used for the buttons */
    PORTSetPinsDigitalIn(IOPORT_D, BIT_13);
	#define CONFIG          (CN_ON | CN_IDLE_CON)
	#define INTERRUPT       (CHANGE_INT_ON | CHANGE_INT_PRI_2)
	mCNOpen(CONFIG, CN19_ENABLE, CN19_PULLUP_ENABLE);
	temp = mPORTDRead();

/* Analog input */
	CloseADC10();
	//#define PARAM1 ADC_MODULE_ON | ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
	//#define PARAM2 ADC_VREF_AVDD_AVSS | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_BUF_16 | ADC_ALT_INPUT_OFF
	//#define PARAM3 ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_5
	//#define PARAM4	ENABLE_AN0_ANA | ENABLE_AN1_ANA
	//#define PARAM5	SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
	//SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN0);
	//OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );
	//EnableADC10();

/* PORT D and E for motors */
	mPORTDSetBits(BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11); 		// Turn on PORTD on startup.
	mPORTDSetPinsDigitalOut(BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11);	// Make PORTD output.
	mPORTESetBits(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7); 		// Turn on PORTE on startup.
	mPORTESetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7);	// Make PORTE output.
/* PORTD for LEDs - DEBUGGING */
/*	mPORTDClearBits(BIT_0 | BIT_1 | BIT_2);
	mPORTDSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2);
*/

// Explorer-16 uses UART2 to connect to the PC.
	// This initialization assumes 36MHz Fpb clock. If it changes,
	// you will have to modify baud rate initializer.
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
	// Configure UART2 RX Interrupt
	INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);


	
// Congifure Change/Notice Interrupt Flag
	ConfigIntCN(INTERRUPT);
// configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
// enable interrupts
    INTEnableInterrupts();

	counterDistanceMeasure=600; //measure distance each 60 ms

	// Let interrupt handler do the work
	while (1) {

		if(counterDistanceMeasure==0){ //Measure distance with ultrasonic sensors
			counterTrigger=1; //Sends trigger signal during 1 interrupt of timer2 (50us)
			/*Reset echo time*/
			delayFront=0;
			delayBack=0;
			delayLeft=0;
			delayRight=0;
			mPORTBSetBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Sends trigger signal to the four sensors
//		mPORTDSetBits(BIT_0); 	//DEBUGGING
//		mPORTDClearBits(BIT_1);
			ConfigIntTimer2(T2_INT_ON); //Starts timer2
			/*Enable interrupts for echo signals*/
			ConfigINT1(EXT_INT_ENABLE); //RE8, biggest priority
			ConfigINT2(EXT_INT_ENABLE); //RE9
			ConfigINT3(EXT_INT_ENABLE); //RA14
			ConfigINT4(EXT_INT_ENABLE); //RA15
			counterDistanceMeasure=600; //measure distance again
		}
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
		/***************** Robot path state machine *****************/
		switch (Path_State) {
			case 0:
				MotorsON = 0;
				Path_State = 0;
				step_counter = 0;
				break;
			case 1:	//forward
				if (step_counter == 0) {
					step_counter = 50 * SPC_front; //1 cm es aprox. = 11.3636 pasos
					MotorsON = 1;
					M1forward = M2forward = M3forward = M4forward= 1;
					Path_State = 2;
				}
				break;
			case 2:	//180 grados positivos (counter clockwise)
				if (step_counter == 0) {
					step_counter = 180 * SPD; //1 grado = 3.3889 pasos
					MotorsON = 1;		//610 pasos giran aprox 180 grados
					M1forward = 0;
					M2forward = 1;
					M3forward = 0;
					M4forward = 1;
					Path_State = 3;
				}
				break;
			case 3:
				if (step_counter == 0) {
					step_counter = 50 * SPC_front;
					MotorsON = 1;
					M1forward = M2forward = M3forward = M4forward= 1;
					Path_State = 4;
				}
				break;
			case 4:
				if (step_counter == 0) {
					step_counter = 90 * SPD;
					MotorsON = 1;
					M1forward = 1;
					M2forward = 0;
					M3forward = 1;
					M4forward = 0;
					Path_State = 5;
				}
				break;
			case 5:		//lateral derecha
				if (step_counter == 0) {
					step_counter = 50 * SPC_side; //1 cm lateral = 14.7059 pasos
					MotorsON = 1;
					M1forward = M4forward = 1;
					M2forward = M3forward = 0;
					Path_State = 6;
				}
				break;
			case 6:
				if (step_counter == 0) {
					Path_State = 0;
				}
				break;
		}
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
			mPORTDSetBits(BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11);
			mPORTESetBits(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7);
		}
		
		/******* SERVO CONTROL ********/
		if (auxcounter == 0) {
			if (servo_angle == 90)
				servo_angle = -90;
			else
				servo_angle = 90;
			auxcounter = 10000;		// toggle angle every 1 s.
		}

		servo_counter = (servo_angle + 90)*(18)/180 + 6; // between 600 and 2400 us

		if (servo_period == 0) {
			mPORTASetBits(BIT_2);
			servo_period = SERVOMAXPERIOD; 		/* 200 * 100us = 20000us period  */
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
	
	if (servo_period != 0) {
		servo_period--;
		if (servo_period == (SERVOMAXPERIOD - servo_counter))
			mPORTAClearBits(BIT_2);
	}

	if (auxcounter != 0)
		auxcounter--;
}

/* TIMER 2 Interrupt Handler - configured to 50us periods */
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // clear the interrupt flag
    mT2ClearIntFlag();
//						mPORTDSetBits(BIT_2); 	//DEBUGGING
	counterTrigger--;
	if(counterTrigger == 0){
		mPORTBClearBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Shut down trigger signal
	}

	//Until there is no echo answer
	delayFront++;
	delayBack++;
	delayLeft++;
	delayRight++;

}

void __ISR(_EXTERNAL_1_VECTOR, ipl7) INT1Interrupt() //Front sensor
{ 
   mINT1IntEnable(0);
   mINT1ClearIntFlag();
   frontDistance= delayFront*50/58; //us/58=cm
mPORTDSetBits(BIT_1); 	//DEBUGGING
mPORTDClearBits(BIT_0);
}
void __ISR(_EXTERNAL_2_VECTOR, ipl7) INT2Interrupt() 
{ 
   mINT2IntEnable(0);
   mINT2ClearIntFlag();
   backDistance=delayBack*50/58; //us/58=cm
} 
void __ISR(_EXTERNAL_3_VECTOR, ipl7) INT3Interrupt() 
{ 
   mINT3IntEnable(0);
   mINT3ClearIntFlag();
   leftDistance = delayLeft * 50/58; //us/58=cm
} 
void __ISR(_EXTERNAL_4_VECTOR, ipl7) INT4Interrupt() 
{ 
   mINT4IntEnable(0);
   mINT4ClearIntFlag();
   rightDistance = delayRight * 50/58; //us/58=cm
}

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
		COMMAND = databyte;
		
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
		if (Path_State == 0)
			Path_State = 1;
		else {
			Path_State = 0;
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
		case 2:
			lectura = mPORTDRead() & (~(0xF << motorNumber*4));
			mPORTDWrite(lectura | (data << motorNumber*4));
			break;
		case 3:
		case 4:
			motorNumber -= 3;
			lectura = mPORTERead() & (~(0xF << motorNumber*4));
			mPORTEWrite(lectura | (data << motorNumber*4));
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
