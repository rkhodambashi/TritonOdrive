//----------------------------------------------------------------------------------
//	FILE:			PM_endat22-Main.C
//
//	Description:	Example project for using PM EnDat22 Library.
//					Includes PM_endat22_lib library and correspoding include files.
//					Initializes the encoders and performs delay compensation.
//					Runs endat21 and endat22 command set.
//					Continuously Read position value in an infinite loop
//
//	Version: 		1.0
//
//  Target:  		TMS320F28377D,
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// Sep 2017  - Example project for PM EnDat22 Library TIDM-1008
//----------------------------------------------------------------------------------
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "endat.h"

uint16_t crc5_self;
volatile uint32_t endatMainLoopCount = 0;

#define SERIAL_STREAM_DECIMATION 20UL

static void Serial_Init(void);
static void Serial_WriteChar(char value);
static void Serial_WriteString(const char *value);
static void Serial_WriteUint32(uint32_t value);
static void Serial_StreamEndatData(void);

void main(void) {

// Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
	InitSysCtrl();
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
	DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
	InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
	InitPieVectTable();

	Serial_Init();
	Serial_WriteString("loop,pos_hi,pos_lo,pos32,position_clocks,error1,error2,timeout_step,timeout_loops\r\n");

//Initialization routine for endat operation - defined in endat.c
//Configures the peripherals and enables clocks for required modules
//Configures GPIO and XBar as needed for EnDat operation
//Sets up the SPI peripheral in endat data structure and enables interrupt
	EnDat_Init();

// The TI demo command sweeps are useful for library validation, but they can
// stop before the continuous position-read loop. Skip them for first hardware bring-up.
//	endat21_runCommandSet();
//	if (ENCODER_TYPE == 22)
//	{
//		endat22_runCommandSet();
//	}
//Enables 2 additional datas in endat22 operation
//This is also optional in real applications. Function defined in endat.c

	endat22_setupAddlData();

//Peforms cable propagation delay calculation.
//This is required for long cable lengths and higher EnDat Clock frequencies
//Function defined in endat.c

	EnDat_initDelayComp();

//Switch to high frequency - 8.3MHz	(=200/4*ENDAT_RUNTIME_FREQ_DIVIDER)
	PM_endat22_setFreq(ENDAT_RUNTIME_FREQ_DIVIDER);
	DELAY_US(800L); 	//Delay 800us

	while(1)
	{
		endatMainLoopCount++;
		if (ENCODER_TYPE == 22)
		{
//Read position data in EnDat22 mode with 2 additional data enabled
//earlier by endat22_setupAddlData function call.
//Function defined in endat.c
			endat22_readPositionWithAddlData();
		}
		else
		{
//Read position data in EnDat21 mode. Function defined in endat.c
			endat21_readPosition();
		}

		if ((endatMainLoopCount % SERIAL_STREAM_DECIMATION) == 0UL)
		{
			Serial_StreamEndatData();
		}
	}
}

static void Serial_Init(void)
{
	GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15);
	GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
	GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
	GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_PUSHPULL);

	SciaRegs.SCICCR.all = 0x0007;
	SciaRegs.SCICTL1.all = 0x0003;
	SciaRegs.SCICTL2.all = 0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
	SciaRegs.SCIHBAUD.all = 0x0000;
	SciaRegs.SCILBAUD.all = 0x001A;
	SciaRegs.SCICTL1.all = 0x0023;
}

static void Serial_WriteChar(char value)
{
	while (SciaRegs.SCICTL2.bit.TXRDY == 0) {}
	SciaRegs.SCITXBUF.all = (uint16_t)value;
}

static void Serial_WriteString(const char *value)
{
	while (*value != '\0')
	{
		Serial_WriteChar(*value);
		value++;
	}
}

static void Serial_WriteUint32(uint32_t value)
{
	char digits[10];
	uint16_t count = 0;

	if (value == 0UL)
	{
		Serial_WriteChar('0');
		return;
	}

	while ((value > 0UL) && (count < 10U))
	{
		digits[count] = (char)('0' + (value % 10UL));
		value /= 10UL;
		count++;
	}

	while (count > 0U)
	{
		count--;
		Serial_WriteChar(digits[count]);
	}
}

static void Serial_StreamEndatData(void)
{
	uint32_t pos32 = endat22Data.position_lo;

	Serial_WriteUint32(endatMainLoopCount);
	Serial_WriteChar(',');
	Serial_WriteUint32(endat22Data.position_hi);
	Serial_WriteChar(',');
	Serial_WriteUint32(endat22Data.position_lo);
	Serial_WriteChar(',');
	Serial_WriteUint32(pos32);
	Serial_WriteChar(',');
	Serial_WriteUint32(endat22Data.position_clocks);
	Serial_WriteChar(',');
	Serial_WriteUint32(endat22Data.error1);
	Serial_WriteChar(',');
	Serial_WriteUint32(endat22Data.error2);
	Serial_WriteChar(',');
	Serial_WriteUint32(endatDataReadyTimeoutStep);
	Serial_WriteChar(',');
	Serial_WriteUint32(endatDataReadyTimeoutLoops);
	Serial_WriteString("\r\n");
}

// End of file





