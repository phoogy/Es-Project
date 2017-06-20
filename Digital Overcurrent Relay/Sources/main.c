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
//#include "RTC.h"
//#include "FTM.h"
//#include "median.h"
//#include "I2C.h"
//#include "accel.h"
#include "Semaphore.h"
#include "analog.h"


#define THREAD_STACK_SIZE 100		// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define NB_ANALOG_CHANNELS 3
#define BAUD_RATE (uint32_t)115200	// BaudRate
#define DEFAULT_TOWER_NUMBER 6681	// Last four digits of student number to be used as tower number

#define CMD_TOWER_STARTUP 0x04		//Tower Startup command in hex
#define CMD_TOWER_VERSION 0x09		//Tower Version command in hex
#define CMD_TOWER_NUMBER 0x0B		//Tower Number command in hex
#define CMD_TOWER_MODE 0x0D		//Tower Mode command in hex
#define CMD_READ_BYTE 0x08		//Read Byte command in hex
#define CMD_PROGRAM_BYTE 0x07		//Program Byte command in hex
#define CMD_TIME 0x0C			//Time command in hex
#define CMD_ACCEL 0x10			//Accelerometer command in hex
#define CMD_PROTOCOL_MODE 0x0A		//Protocol Mode command in hex

/* Thread stacks */
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the Init thread. */
static uint32_t PacketModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t RTCModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t FTMModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PITModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t AccelPollModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t AccelIntModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t I2CModuleThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {4, 5, 6};

/* Declared functions */
//static bool ProtocolMode(void);



/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1
  },
  {
    .semaphore = NULL,
    .channelNb = 2
  }
};



/* Declared variables */
static volatile uint16union_t *NvTowerNb; 	/*!< The non-volatile Tower number. */
static volatile uint16union_t *NvTowerMode;	/*!< The non-volatile Tower Mode. */
bool DataReady;

/* Semaphores */
OS_ECB* PacketReady;
OS_ECB* PITReady;
OS_ECB* ByteReady;


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
    packetFlag = Flash_Write16((uint16_t *)NvTowerNb, Packet_Parameter23);
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
    packetFlag = Flash_Write16((uint16_t *)NvTowerMode, Packet_Parameter23);
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
    Packet_Put(Packet_Command |= 0x80, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  else
    Packet_Put(Packet_Command &= ~0x80, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
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
      packetFlag = Flash_Write8((uint8_t *)(FLASH_DATA_START + Packet_Parameter1), Packet_Parameter3);
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
  //AccelMode = ACCEL_POLL;
  DataReady = true;

  /* Create Semaphores */
  PacketReady = OS_SemaphoreCreate(0);
  //RTCReady = OS_SemaphoreCreate(0);
  PITReady = OS_SemaphoreCreate(0);
  //FTMReady = OS_SemaphoreCreate(0);
  //ACCELIntReady = OS_SemaphoreCreate(0);
  //I2CReady = OS_SemaphoreCreate(0);
  //ACCELPollReady = OS_SemaphoreCreate(0);


  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);


  /* Initialise Modules */
  initialised &= Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ);		// Initialise Packet module
  initialised &= Flash_Init();						// Initialise Flash module
  initialised &= LEDs_Init();						// Initialise Led module
  initialised &= PIT_Init(CPU_BUS_CLK_HZ, NULL, NULL);			// Initialise PIT module
  //initialised &= RTC_Init(NULL, NULL);					// Initialise RTC module
  //initialised &= FTM_Init();						// Initialise FTM module
  //initialised &= Accel_Init(&accelSetup);				// Initialise Accel module

  /* Allocate Flash Memory for Tower Number and Tower Mode */
  initialised &= Flash_AllocateVar((volatile void**)&NvTowerNb, sizeof(*NvTowerNb));		// Allocate Flash space for Tower number
  initialised &= Flash_AllocateVar((volatile void**)&NvTowerMode, sizeof(*NvTowerMode));	// Allocate Flash space for Tower mode

  /* If the tower number and tower mode is cleared then write default to flash */
  if (_FH(FLASH_DATA_START) == 0xFFFF)
  {
    initialised &= Flash_Write16((uint16_t *)NvTowerNb, (uint16_t)DEFAULT_TOWER_NUMBER);
    initialised &= Flash_Write16((uint16_t *)NvTowerMode, (uint16_t)1);
  }

  PIT_Set(1250000,true);			// Set Pit with 500ms
  //FTM_Set(&FTMChannel0);		// Set FTM





  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);




  if (initialised)
    LEDs_On(LED_ORANGE);		// Turn on orange led if everything initialised






  TowerStartup();			// Send Tower Startup Packets

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


void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)
  int16_t analogInputValue[16];
  uint8_t sampleCount = 0;
  for (;;)
  {
    //int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue[sampleCount]);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue[sampleCount]);

    sampleCount++;

    if (sampleCount == sizeof(analogInputValue))
    {
    	int32_t sum = 0;
    	double average = 0;
    	for (int i = 0; i < sizeof(analogInputValue); i++)
    	{
    		sum = sum + (analogInputValue[i] * analogInputValue[i]);
    	}
    	average = (double)(sum/sizeof(analogInputValue));

		double vrms = average/3, last, diff = 1;
		if (average <= 0) return 0;
		do {
			last = vrms;
			vrms = (vrms + average / vrms) / 2;
			diff = vrms - last;
		} while (diff > MINDIFF || diff < -MINDIFF);

		vrms

		double vrms

    	//Packet_Put(0x10, , 0, 0);
    	sampleCount = 0;
    }
//    if (analogData->channelNb == 0)
//    {
//    	y = ((analogInputValue*100)/35)*1000;
//    	if (y < 1030)
//    		x = 254;
//    	else if (y > 1030)
//    		x = 0;
//	    Packet_Put(0x10, analogInputValue, x, 0);
//    }
//    // Test data
//    if (analogData->channelNb == 0)
//      x = analogInputValue;
//    else if (analogData->channelNb == 1)
//      y = analogInputValue;
//    else if (analogData->channelNb == 2)
//          z = analogInputValue;
//    Packet_Put(0x10, x, y, z);
  }
}


//static void RTCModuleThread(void* pData)
//{
//  uint8_t hours, minutes, seconds;	// Declare variables to get time
//  for (;;)
//  {
//    OS_SemaphoreWait(RTCReady, 0);
//    RTC_Get(&hours, &minutes, &seconds);		// Get time variables
//    Packet_Put(CMD_TIME, hours, minutes, seconds);	// Send Time
//    LEDs_Toggle(LED_YELLOW);				// Toggle Yellow Led
//  }
//}

static void PITModuleThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(PITReady,0);
    //LEDs_Toggle(LED_GREEN);
//    OS_SemaphoreSignal(ACCELPollReady);
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
  }
}

static void FTMModuleThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(FTMReady,0);
    LEDs_Toggle(LED_BLUE);			// Toggle Blue Led
  }
}

//static void AccelPollModuleThread(void* pData)
//{
//  for (;;)
//  {
//    OS_SemaphoreWait(ACCELPollReady,0);
//    if (AccelMode == ACCEL_POLL)
//    {
//      PreviousAccelData.axes.x = CurrentAccelData.axes.x;
//      PreviousAccelData.axes.y = CurrentAccelData.axes.y;
//      PreviousAccelData.axes.z = CurrentAccelData.axes.z;
//
//      Accel_ReadXYZ(CurrentAccelData.bytes);	// Read Accel Data
//
//      if (PreviousAccelData.axes.x != CurrentAccelData.axes.x || PreviousAccelData.axes.y != CurrentAccelData.axes.y || PreviousAccelData.axes.z != CurrentAccelData.axes.z)
//      {
//	Packet_Put(CMD_ACCEL, CurrentAccelData.axes.x, CurrentAccelData.axes.y, CurrentAccelData.axes.z);
//	LEDs_Toggle(LED_GREEN);			// Toggle Green Led
//      }
//    }
//  }
//}

//static void AccelIntModuleThread(void* pData)
//{
//  for (;;)
//  {
//    OS_SemaphoreWait(ACCELIntReady,0);
//    if (DataReady)
//    {
//      // DataReady set to false so new data isn't initialised half way through a reading
//      DataReady = false;
//      for (int i = 2; i > 0; i--)
//      {
//	AccelData[i].axes.x = AccelData[i-1].axes.x;
//	AccelData[i].axes.y = AccelData[i-1].axes.y;
//	AccelData[i].axes.z = AccelData[i-1].axes.z;
//      }
//
//      // Call I2C_IntRead
//      I2C_IntRead(0x01, AccelData[0].bytes, 3);
//    }
//  }
//}

//static void I2CModuleThread(void* pData)
//{
//  for (;;)
//  {
//    OS_SemaphoreWait(I2CReady,0);
//    // Median filter the last 3 sets of data
//    CurrentAccelData.axes.x = Median_Filter3(AccelData[0].axes.x,AccelData[1].axes.x,AccelData[2].axes.x);
//    CurrentAccelData.axes.y = Median_Filter3(AccelData[0].axes.y,AccelData[1].axes.y,AccelData[2].axes.y);
//    CurrentAccelData.axes.z = Median_Filter3(AccelData[0].axes.z,AccelData[1].axes.z,AccelData[2].axes.z);
//
//    // Sends the data
//    Packet_Put(CMD_ACCEL, CurrentAccelData.axes.x, CurrentAccelData.axes.y, CurrentAccelData.axes.z);
//
//    // Toggle Led
//    LEDs_Toggle(LED_GREEN);
//
//    // DataReady set to true to receive next data
//    DataReady = true;
//  }
//}

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

//  error = OS_ThreadCreate(RTCModuleThread,
//    			  NULL,
//    			  &RTCModuleThreadStack[THREAD_STACK_SIZE - 1],
//    			  2);

//  error = OS_ThreadCreate(FTMModuleThread,
//  			  NULL,
//  			  &FTMModuleThreadStack[THREAD_STACK_SIZE - 1],
//  			  3);

  error = OS_ThreadCreate(PITModuleThread,
  			  NULL,
  			  &PITModuleThreadStack[THREAD_STACK_SIZE - 1],
  			  2);

//  error = OS_ThreadCreate(I2CModuleThread,
//    			  NULL,
//    			  &I2CModuleThreadStack[THREAD_STACK_SIZE - 1],
//    			  5);

//  error = OS_ThreadCreate(AccelIntModuleThread,
//  			  NULL,
//  			  &AccelIntModuleThreadStack[THREAD_STACK_SIZE - 1],
//  			  6);

//  error = OS_ThreadCreate(AccelPollModuleThread,
//  			  NULL,
//  			  &AccelPollModuleThreadStack[THREAD_STACK_SIZE - 1],
//  			  7);


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
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for (;;){}
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






