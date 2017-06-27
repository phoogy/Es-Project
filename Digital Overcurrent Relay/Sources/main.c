/* ###################################################################
 **     Filename    : main.c
 **     Project     : Lab6
 **     Processor   : MK70FN1M0VMJ12
 **     Version     : Driver 01.01
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 6.0
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

// CPU module - contains low level hardware initialization routines
#include <math.h>
#include "Cpu.h"
#include "OS.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "packet.h"
#include "LEDs.h"
#include "Flash.h"
#include "PIT.h"
#include "Semaphore.h"
#include "analog.h"

#define THREAD_STACK_SIZE 100		// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define NB_ANALOG_CHANNELS 3
#define NB_SAMPLES 16

#define BAUD_RATE (uint32_t)115200	// BaudRate
#define DEFAULT_TOWER_NUMBER 6681	// Last four digits of student number to be used as tower number
#define SAMPLING_CLOCK 0
#define TIMING_CLOCK 1
#define TIMING_CHANNEL 0
#define TRIP_CHANNEL 1
#define DOR_IDLE (int16_t)0
#define DOR_ACTIVE (int16_t)16383

#define CMD_TOWER_STARTUP 0x04		//Tower Startup command in hex
#define CMD_TOWER_VERSION 0x09		//Tower Version command in hex
#define CMD_TOWER_NUMBER 0x0B		//Tower Number command in hex
#define CMD_TOWER_MODE 0x0D		//Tower Mode command in hex
#define CMD_READ_BYTE 0x08		//Read Byte command in hex
#define CMD_PROGRAM_BYTE 0x07		//Program Byte command in hex
#define CMD_TIME 0x0C			//Time command in hex
#define CMD_ACCEL 0x10			//Accelerometer command in hex
#define CMD_PROTOCOL_MODE 0x0A		//Protocol Mode command in hex
#define CMD_DOR 0x0E
//gfhfdghgf
/* Thread stacks */
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the Init thread. */
static uint32_t PacketModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PITModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t TimingModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

const uint8_t ANALOG_THREAD_PRIORITIES[3] = {
		4,
		5,
		6 };

/* Declared functions */

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
		OS_ECB* semaphore;
		uint8_t channelNb;
} TAnalogThreadData;

typedef enum
{
	INVERSE = (uint8_t) 0,
	VERY_INVERSE = (uint8_t) 1,
	EXTREMELY_INVERSE = (uint8_t) 2
} TInverseMode;

/*! @brief Data structure used to calculate the timing used for tripping
 *
 */

typedef struct Characteristic
{
		TInverseMode mode;
		double k;
		double a;
} TCharacteristic;

/*! @brief Characteristic data
 *
 */
static TCharacteristic Characteristic[3] =
{
	{
		.mode = INVERSE,
		.k = 0.14,
		.a = 0.02 },
	{
		.mode = VERY_INVERSE,
		.k = 13.5,
		.a = 1 },
	{
		.mode = EXTREMELY_INVERSE,
		.k = 80,
		.a = 2 }
};

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[3] =
{
	{
		.semaphore = NULL,
		.channelNb = 0 },
	{
		.semaphore = NULL,
		.channelNb = 1 },
	{
		.semaphore = NULL,
		.channelNb = 2 }
};

/* Declared variables */
static volatile uint16union_t *NvTowerNb; /*!< The non-volatile Tower number. */
static volatile uint16union_t *NvTowerMode; /*!< The non-volatile Tower Mode. */
static volatile uint8_t *NvInverseMode; /*!< The non-volatile Inverse Mode. */
static volatile uint16union_t *NvTimesTripped; /*!< The non-volatile number of times tripped. */

uint8_t LastFaultType;
uint8_t LastCurrent[3];

bool TimingStarted = false;
bool TimingActive = false;

uint32_t TimingCounter[NB_ANALOG_CHANNELS];

/* Semaphores */
static OS_ECB* IRMSSemaphore;



static void IRMSGet(int16_t * samples, double * const dataPtr)
{
	OS_SemaphoreWait(IRMSSemaphore, 0);

	// Initialise variables for vrms calculation
	double sum = 0;
	double average = 0;
	double vrms = 0;

	// Calculate sum of squares
	int j = sizeof(samples);

	for (int i = 0; i < sizeof(samples); i++)
	{
		sum = sum + ((double)samples[i] * (double)samples[i]);
	}

	// Calculate average
	average = (sum / sizeof(samples));
	vrms = sqrt(average);

	// Vrms --> Irms
	// 0.350mV ~ 1
	*dataPtr = vrms / 0.35;

	OS_SemaphoreSignal(IRMSSemaphore);
}


/*
 * @brief Sends or Sets the Tower Number.
 * @return bool TRUE if packet was successfully put into the TxFIFO buffer.
 */
static bool TowerNumber(void)
{
	bool packetFlag = false;
	if (Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
		packetFlag = Packet_Put(CMD_TOWER_NUMBER, 1, NvTowerNb->s.Lo, NvTowerNb->s.Hi);
	if (Packet_Parameter1 == 2)
		packetFlag = Flash_Write16((uint16_t *) NvTowerNb, Packet_Parameter23);
	return packetFlag;
}

/*
 * @brief Sends a Tower Startup packets.
 * @return bool TRUE if packet was successfully put into the TxFIFO buffer.
 */
static bool TowerStartup(void)
{
	bool packetFlag = false;
	if (Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
	{
		packetFlag = Packet_Put(CMD_TOWER_STARTUP, 0, 0, 0);
		packetFlag &= Packet_Put(CMD_TOWER_VERSION, 118, 1, 0);
		packetFlag &= Packet_Put(CMD_TOWER_NUMBER, 1, NvTowerNb->s.Lo, NvTowerNb->s.Hi);
		packetFlag &= Packet_Put(CMD_TOWER_MODE, 1, NvTowerMode->s.Lo, NvTowerMode->s.Hi);
	}
	return packetFlag;
}

/*
 * @brief Sends a Tower Version packet.
 * @return bool TRUE if packet was successfully put into the TxFIFO buffer.
 */
static bool TowerVersion(void)
{
	bool packetFlag = false;
	if (Packet_Parameter1 == 118 && Packet_Parameter2 == 120 && Packet_Parameter3 == 13)
		packetFlag = Packet_Put(CMD_TOWER_VERSION, 118, 1, 0);
	return packetFlag;
}

/*
 * @brief Sends or sets the Tower Mode..
 * @return bool TRUE if packet was successfully put into the TxFIFO buffer.
 */
static bool TowerMode(void)
{
	bool packetFlag = false;
	if (Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
		packetFlag = Packet_Put(CMD_TOWER_MODE, 1, NvTowerMode->s.Lo, NvTowerMode->s.Hi);
	if (Packet_Parameter1 == 2)
		packetFlag = Flash_Write16((uint16_t *) NvTowerMode,
				Packet_Parameter23);
	return packetFlag;
}

/*
 * @brief Sends an ACK or NAK Response based on @param ackFlag.
 * @param bool ackFlag The flag in which to send a ACK or NAK
 * @return void.
 */
static void ACKNAK(bool ackFlag)
{
	if (ackFlag)
		Packet_Put(Packet_Command |= 0x80, Packet_Parameter1, Packet_Parameter2,
				Packet_Parameter3);
	else
		Packet_Put(Packet_Command &= ~0x80, Packet_Parameter1,
				Packet_Parameter2, Packet_Parameter3);
}

/*
 * @brief Erases the flash or program new data in to a byte.
 * @return bool TRUE if packet was successfully put into the TxFIFO buffer or erase flash was successful.
 */
static bool ProgramByte(void)
{
	bool packetFlag = false;
	if (Packet_Parameter1 >= 0 && Packet_Parameter1 <= 8 && Packet_Parameter2 == 0)
	{
		if (Packet_Parameter1 == 8)
			packetFlag = Flash_Erase();
		else
			packetFlag = Flash_Write8((uint8_t *) (FLASH_DATA_START + Packet_Parameter1),
					Packet_Parameter3);
	}
	return packetFlag;
}

/*
 * @brief Sends the data for a byte from flash in a packet.
 * @return bool TRUE if packet was successfully put into the TxFIFO buffer.
 */
static bool ReadByte(void)
{
	bool packetFlag = false;
	if (Packet_Parameter1 >= 0 && Packet_Parameter1 <= 7 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
		packetFlag = Packet_Put(CMD_READ_BYTE, Packet_Parameter1, 0, _FB(FLASH_DATA_START + Packet_Parameter1));
	return packetFlag;
}

static bool DOR(void)
{
	bool packetFlag = false;
	if (Packet_Parameter1 == 0 && Packet_Parameter3 == 0)
	{
		switch (Packet_Parameter2)
		{
			case 0:
				//Send Characteristic
				packetFlag = Packet_Put(CMD_DOR, Packet_Parameter2, *NvInverseMode, 0);
				break;
			case 1:
				// TODO get current
				packetFlag = Packet_Put(CMD_DOR, Packet_Parameter2, 0, 0);
				// send current
				//packetFlag = Packet_Put(CMD_DOR, Packet_Parameter2, ,0);
				break;
			case 2:
				// send number of times tripped
				packetFlag = Packet_Put(CMD_DOR, Packet_Parameter2, NvTimesTripped->s.Hi, NvTimesTripped->s.Lo);
				break;
			case 3:
				// TODO type of last fault detected
				packetFlag = Packet_Put(CMD_DOR, Packet_Parameter2, 0, 0);
				break;
		}
	} else if (Packet_Parameter1 == 1 && Packet_Parameter3 == 0 && (Packet_Parameter2 == 0 || Packet_Parameter2 == 1 || Packet_Parameter2 == 2))
	{
		packetFlag = Flash_Write8((uint8_t *) NvInverseMode, Packet_Parameter2);
	}

	//if (Packet_Parameter1 >= 0 && Packet_Parameter1 <= 7 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
	// packetFlag = Packet_Put(CMD_READ_BYTE, Packet_Parameter1, 0, _FB(FLASH_DATA_START + Packet_Parameter1));
	return packetFlag;
}

/*
 * @brief Determines the packet's instructions and calls the relevant function.
 * @return void.
 */
static void HandlePacket(void)
{
	bool ackFlag = false;
	switch (Packet_Command & 0x7F)
	{
		case CMD_TOWER_STARTUP:
			ackFlag = TowerStartup();
			break;
		case CMD_TOWER_VERSION:
			ackFlag = TowerVersion();
			break;
		case CMD_TOWER_NUMBER:
			ackFlag = TowerNumber();
			break;
		case CMD_TOWER_MODE:
			ackFlag = TowerMode();
			break;
		case CMD_READ_BYTE:
			ackFlag = ReadByte();
			break;
		case CMD_PROGRAM_BYTE:
			ackFlag = ProgramByte();
			break;
		case CMD_DOR:
			ackFlag = DOR();
			break;

			//    case CMD_TIME:
			//      ackFlag = SetTime();
			//      break;
			//    case CMD_PROTOCOL_MODE:
			//      ackFlag = ProtocolMode();
			//      break;
	}
	if ((Packet_Command & 0x80) == 0x80)
		ACKNAK(ackFlag);
}

/*! @brief Initialises the modules.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
	OS_DisableInterrupts();

	/* Local variable definitions */
	bool initialised = true;

	// Generate the global analog semaphores
	for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
		AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

	/* Initialise Modules */
	initialised &= Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ);			// Initialise Packet module
	initialised &= Flash_Init();					// Initialise Flash module
	initialised &= LEDs_Init();						// Initialise Led module
	initialised &= PIT_Init(CPU_BUS_CLK_HZ);			// Initialise PIT module

	/* Allocate Flash Memory for Tower Number and Tower Mode */
	initialised &= Flash_AllocateVar((volatile void**) &NvTowerNb, sizeof(*NvTowerNb));		// Allocate Flash space for Tower number
	initialised &= Flash_AllocateVar((volatile void**) &NvTowerMode, sizeof(*NvTowerMode));	// Allocate Flash space for Tower mode

	/* Allocate Flash Memory for Inverse Mode */
	initialised &= Flash_AllocateVar((volatile void**) &NvInverseMode, sizeof(*NvInverseMode));
	initialised &= Flash_AllocateVar((volatile void**) &NvTimesTripped, sizeof(*NvTimesTripped));

	/* If the tower number and tower mode is cleared then write default to flash */
	if (_FH(FLASH_DATA_START) == 0xFFFF)
	{
		initialised &= Flash_Write16((uint16_t *) NvTowerNb, (uint16_t) DEFAULT_TOWER_NUMBER);
		initialised &= Flash_Write16((uint16_t *) NvTowerMode, (uint16_t) 1);

		initialised &= Flash_Write8((uint8_t *) NvInverseMode, INVERSE);
		initialised &= Flash_Write16((uint16_t *) NvTimesTripped, 0);
	}

	///* If no valid Inverse mode is set then set to default INVERSE*/

	PIT_Set(SAMPLING_CLOCK, 1250000, true);			// Set Pit with 1.25ms
	//PIT_Set(TIMING_CLOCK, 1000000, true);

	// Analog
	(void) Analog_Init(CPU_BUS_CLK_HZ);

	if (initialised)
		LEDs_On(LED_ORANGE);	// Turn on orange led if everything initialised

	TowerStartup();			// Send Tower Startup Packets

	IRMSSemaphore = OS_SemaphoreCreate(1);


	OS_EnableInterrupts();

	// We only do this once - therefore delete this thread
	OS_ThreadDelete(OS_PRIORITY_SELF);
}

static void PacketModuleThread(void* pData)
{
	for (;;)
	{
		if (Packet_Get())
			HandlePacket();
	}
}

static void TimingModuleThread(void* pData)
{
	for (;;)
	{
		OS_SemaphoreWait(PITReady[TIMING_CLOCK], 0);
		//    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
		//    {
		//	  if (TimingCounter[analogNb] > 0)
		//	  {
		//		TimingCounter[analogNb]--;
		//		if (TimingCounter[analogNb] == 0)
		//		{
		//		  Analog_Put(TRIP_CHANNEL, DOR_ACTIVE);
		//		  Flash_Write16((uint16_t *)NvTowerNb, (NvTimesTripped->l + 1));
		//		}
		//	  }
		//    }
	}
}

void AnalogLoopbackThread(void* pData)
{
	// Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

	// Number of samples to take
	//static const uint8_t NB_SAMPLES = 16;

	// Array of array of samples for each analog channel
	int16_t analogInputValue[NB_ANALOG_CHANNELS][NB_SAMPLES];

	// Array of sampleCount for each analog channel
	uint8_t sampleCount[NB_ANALOG_CHANNELS];

	// Initialise sampleCount to 0
	sampleCount[analogData->channelNb] = 0;

	// An array of previous total time for each analog channel
	uint32_t previousTotalTime[NB_ANALOG_CHANNELS];

	// An array of timing Cycle Percentage for each analog channel
	double timingCyclePercentage[NB_ANALOG_CHANNELS];

	// Initialise timing cycle percentage to 0
	timingCyclePercentage[analogData->channelNb] = 0;

	for (;;)
	{

		// Wait for semaphore
		(void) OS_SemaphoreWait(analogData->semaphore, 0);

		// Get analog sample into array


		uint8_t testVar = sampleCount[analogData->channelNb];

		int16_t *testVar3 = &analogInputValue[analogData->channelNb][testVar];
		Analog_Get(analogData->channelNb, testVar3);
//		Analog_Get(analogData->channelNb, analogInputValue[analogData->channelNb][testVar]);

		int16_t testVar2 = analogInputValue[analogData->channelNb][testVar];
		// TODO FIX IT SHOULD NOT BE DOING THIS IF IT ALREADY TRIPPED
		// If timer is not running
		if (TimingCounter[analogData->channelNb] == 0)
		{
			// Set Timing Counter to high number 10 seconds
			TimingCounter[analogData->channelNb] = 10000;

			// Set previous total time to 10 seconds
			previousTotalTime[analogData->channelNb] = 10000;
		}

		// Increment sample Count
		sampleCount[analogData->channelNb]++;

		// If collected 16 samples reset sample count to 0
		if (sampleCount[analogData->channelNb] == NB_SAMPLES)
			sampleCount[analogData->channelNb] = 0;


//		// Initialise variables for vrms calculation

		double sum = 0;
		double average = 0;
		double vrms = 0;

		// Calculate sum of squares
		for (int i = 0; i < NB_SAMPLES; i++)
		{
			sum = sum + ((double)(analogInputValue[analogData->channelNb][i]) * (double)(analogInputValue[analogData->channelNb][i]));
		}

		// Calculate average
		average = (sum / (double)NB_SAMPLES);
		vrms = sqrt(average);
		// Vrms --> Irms
		// 0.350mV ~ 1
		double irms = vrms / 0.35;

		//double irms;

		//irms = 1000;
		//IRMSGet(analogInputValue[analogData->channelNb][analogData->channelNb], &irms);


		//		/* 1.03 Irms is the current used to activate the trip
		//		 * When V = 10pp
		//		 * 1.03Irms = 65534 / 10 * 1.03
		//		 * 1.03Irms = 6750.002
		//		 */


		if (irms < 6750.002)
		{
			TimingCounter[analogData->channelNb] = 0;
			Analog_Put(TIMING_CHANNEL, DOR_IDLE);
			Analog_Put(TRIP_CHANNEL, DOR_IDLE);

		} else
		{
			 //time = k / ((Irms^2)-1) This is in Seconds so we need to change it into milliseconds


//			Analog_Put(TIMING_CHANNEL, DOR_ACTIVE);
//			uint32_t time = (Characteristic[*NvInverseMode].k / (uint32_t)((pow(irms, Characteristic[*NvInverseMode].a) - 1)) * (double)1000);
//
//			uint32_t timeLeft = TimingCounter[analogData->channelNb];
//
//			timingCyclePercentage[analogData->channelNb] = timingCyclePercentage[analogData->channelNb] + (timeLeft / previousTotalTime[analogData->channelNb]);
//			time = timeLeft * (1 - timingCyclePercentage[analogData->channelNb]);
//			previousTotalTime[analogData->channelNb] = time;
//
//
//
//			if (time < 1)
//				time = 1;
//			TimingCounter[analogData->channelNb] = (uint32_t)time;


		}
	}
}

static void PITModuleThread(void* pData)
{
	for (;;)
	{
		OS_SemaphoreWait(PITReady[SAMPLING_CLOCK], 0);
		for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
			(void) OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
	}
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
	OS_ERROR error;

	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
	PE_low_level_init();
	/*** End of Processor Expert internal initialization.                    ***/

	/* Initialize the RTOS - without flashing the orange LED "heartbeat" */
	OS_Init(CPU_CORE_CLK_HZ, false);

	error = OS_ThreadCreate(InitModulesThread,
													NULL,
													&InitModulesThreadStack[THREAD_STACK_SIZE - 1],
													0); // Highest priority

	error = OS_ThreadCreate(PacketModuleThread,
														NULL,
														&PacketModuleThreadStack[THREAD_STACK_SIZE - 1],
														1);

	error = OS_ThreadCreate(PITModuleThread,
														NULL,
														&PITModuleThreadStack[THREAD_STACK_SIZE - 1],
														2);

	error = OS_ThreadCreate(TimingModuleThread,
														NULL,
														&TimingModuleThreadStack[THREAD_STACK_SIZE - 1],
														3);


	for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
	{
		error = OS_ThreadCreate(AnalogLoopbackThread,
														&AnalogThreadData[threadNb],
														&AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
														ANALOG_THREAD_PRIORITIES[threadNb]);
	}


	OS_Start();

	/*** Don't write any code pass this line, or it will be deleted during code generation. ***/
	/*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
#ifdef PEX_RTOS_START
	PEX_RTOS_START(); /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
#endif
	/*** End of RTOS startup code.  ***/
	/*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
	for (;;)
	{
	}
	/*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
 ** @}
 */
/*
 ** ###################################################################
 **
 **     This file was created by Processor Expert 10.5 [05.21]
 **     for the Freescale Kinetis series of microcontrollers.
 **
 ** ###################################################################
 */

