/*********************************************************************
 *
 *                  
 *
 *********************************************************************
 * FileName:        .c
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

#include <plib.h>					// Peripheral Library


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
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

// *****************************************************************************
// *****************************************************************************
// Section: System Macros
// *****************************************************************************
// *****************************************************************************
#define	GetSystemClock() 	(80000000ul)
#define SYS_FREQ 			(80000000L)
#define PB_DIV         		1
#define PRESCALE       		1
#define TOGGLES_PER_SEC		1
#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())

#define DESIRED_BAUDRATE    	(9600)      //The desired UART BaudRate

/*********************************/
void WriteString(const char *string);
void PutCharacter(const char character);

void Delay(unsigned int count)
{
	while(--count);
}
//UNA SOLA FUNCION PARA AMBOS MOTORES? TIMERS?
void adelante(unsigned int pasos4, int tiempo, unsigned int motornum){
	motornum--;
	unsigned int lectura = mPORTDRead() & (~(0xF << motornum * 4));
	do{
	mPORTDWrite(lectura | (0x6 << motornum*4)); //0110
	Delay(tiempo);
	mPORTDWrite(lectura | (0x3 << motornum*4)); //0011
	Delay(tiempo);
	mPORTDWrite(lectura | (0x9 << motornum*4)); //1001
	Delay(tiempo); 
	mPORTDWrite(lectura | (0xC << motornum*4)); //1100
	Delay(tiempo); 
	//Delay(140000*4 - tiempo*4);//lo restante para que el total sea constante
	pasos4--;
	
	}while (pasos4>0);

}

void atras(unsigned int pasos4, int tiempo, unsigned int motornum){
	motornum--;
	unsigned int lectura = mPORTDRead() & (~(0xF << motornum * 4));
	do{
	mPORTDWrite(lectura | (0x6 << motornum*4)); //0110
	Delay(tiempo);
	mPORTDWrite(lectura | (0xC << motornum*4)); //1100
	Delay(tiempo);
	mPORTDWrite(lectura | (0x9 << motornum*4)); //1001
	Delay(tiempo); 
	mPORTDWrite(lectura | (0x3 << motornum*4)); //0011
	Delay(tiempo); 
	//Delay(140000*4 - tiempo*4);//lo restante para que el total sea constante
	pasos4--;
	
	}while (pasos4>0);
}

unsigned char M1forward = 1, MotorsON = 0;
unsigned char M2forward = 1;

int main(void)
{
	unsigned int temp;

	// Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states, RAM
	// wait state and enable prefetch cache but will not change the PBDIV.
	// The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

/*PORT CONFIGURATION for UART*/
	// Blinky LED for Uart activity
	//mPORTAClearBits(BIT_7);
	//mPORTASetPinsDigitalOut(BIT_7);

// PINS used for the buttons
    PORTSetPinsDigitalIn(IOPORT_B, BIT_2 | BIT_3 | BIT_4);
	#define CONFIG          (CN_ON | CN_IDLE_CON)
	#define INTERRUPT       (CHANGE_INT_ON | CHANGE_INT_PRI_2)
	mCNOpen(CONFIG, CN4_ENABLE | CN5_ENABLE | CN6_ENABLE,
			CN4_PULLUP_ENABLE | CN5_PULLUP_ENABLE | CN6_PULLUP_ENABLE);
	temp = mPORTBRead();

//Analog input
	CloseADC10();
	#define PARAM1 ADC_MODULE_ON | ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
	#define PARAM2 ADC_VREF_AVDD_AVSS | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_BUF_16 | ADC_ALT_INPUT_OFF
	#define PARAM3 ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_5
	#define PARAM4	ENABLE_AN0_ANA | ENABLE_AN1_ANA
	#define PARAM5	SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN0);
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );
	EnableADC10();

//PORT D for motors
	mPORTDClearBits(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11 |
					BIT_12 | BIT_13 | BIT_14 | BIT_15); 		// Turn off PORTD on startup.
	mPORTDSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3 |
					BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11 |
					BIT_12 | BIT_13 | BIT_14 | BIT_15);	// Make PORTD output.


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


	WriteString("*** UART Interrupt-driven Example ***\r\n");

	unsigned int channel1, channel2;
	unsigned int motor1Time, motor2Time;
	// Let interrupt handler do the work
	while (1) {
		
		while ( ! mAD1GetIntFlag() );

		channel1 = ReadADC10(0);
		channel2 = ReadADC10(1);
		motor1Time = (channel1*(60000)/(1023) + 80000);
		motor2Time = (channel2*(60000)/(1023) + 80000);
		//maximo valor de motorTime = 140000

		//use motor time for stepping delay		
		if (MotorsON) {
			if (M1forward) {
				adelante(1, motor1Time, 1);
			} else {
				atras(1, motor1Time, 1);
			}

			if (M2forward) {
				adelante(1, motor2Time, 3);
			} else {
				atras(1, motor2Time, 3);
			}

		}
	
			mAD1ClearIntFlag();
		}
			
		return 0;
}

// UART 2 interrupt handler
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART2)))
	{
		// Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART2));

	// Code to be executed on RX interrupt:

		// Echo what we just received.
		PutCharacter(UARTGetDataByte(UART2));
		// Toggle LED to indicate UART activity
		mPORTAToggleBits(BIT_7);

	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART2)) )
	{
		// Clear the TX interrupt Flag
		INTClearFlag(INT_SOURCE_UART_TX(UART2));

	// Code to be executed on TX interrupt:

		
	}
}

// helper functions
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


// Configure the CN interrupt handler
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void)
{
	unsigned int temp;

    // clear the mismatch condition
    temp = mPORTBRead();

    // clear the interrupt flag
    mCNClearIntFlag();

    // .. things to do .. 
    if ( !(temp & (1<<2)) ) { //button on RB2 is pressed
		if (MotorsON == 0)
			MotorsON = 1;
		else
			MotorsON = 0;
	}
	
	if ( temp & (1<<3) ) {
		M1forward = 1;
	} else if ( !(temp & (1<<3)) ) {
		M1forward = 0;
	}

	if ( temp & (1<<4) ) {
		M2forward = 1;
	} else if ( !(temp & (1<<4)) ) {
		M2forward = 0;
	}
	
	
}
