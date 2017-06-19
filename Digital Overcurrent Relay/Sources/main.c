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

// Simple OS
#include "OS.h"

// Analog functions
#include "analog.h"

// Packet functions
#include "packet.h"

#include "PIT.h"
// ----------------------------------------
// Packet set up
// ----------------------------------------
#define BAUD_RATE (uint32_t)115200  // BaudRate

TPacket Packet;
// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
OS_THREAD_STACK(PacketModuleThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PitModuleThreadStack, THREAD_STACK_SIZE);
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {4, 5, 6};

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
 * @brief Determines the packet's instructions and calls the relevant function.
 * @return void.
 */
static void HandlePacket(void)
{
  bool ackFlag = false;
//  switch (Packet_Command & 0x7F)
//  {
//    case CMD_TOWER_STARTUP:
//      ackFlag = TowerStartup();
//      break;
//    case CMD_TOWER_VERSION:
//      ackFlag = TowerVersion();
//      break;
//    case CMD_TOWER_NUMBER:
//      ackFlag = TowerNumber();
//      break;
//    case CMD_TOWER_MODE:
//      ackFlag = TowerMode();
//      break;
//    case CMD_READ_BYTE:
//      ackFlag = ReadByte();
//      break;
//    case CMD_PROGRAM_BYTE:
//      ackFlag = ProgramByte();
//      break;
//    case CMD_TIME:
//      ackFlag = SetTime();
//      break;
//    case CMD_PROTOCOL_MODE:
//      ackFlag = ProtocolMode();
//      break;
//  }
  if ((Packet_Command & 0x80) == 0x80)
    ACKNAK(ackFlag);
}

/*! @brief PIT signals other threads to run.
 *
 */
static void PitModuleThread(void* pData)
{
  OS_SemaphoreWait(PITReady,0);
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
}

/*! @brief Initialises modules.
 *
 */
static void InitModulesThread(void* pData)
{
  (void)Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ);

  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // 16 samples per cycle at 50Hz
  // 16 * 50 = 800 samples per 1000ms
  // 1000 / 800 = 1.25
  // We need to tick every 1.25 ms = 1250 ns

  // Initialise the PIT to tick every 1250 ns
  PIT_Init(CPU_BUS_CLK_HZ, NULL, NULL);
  PIT_Set(1250,true);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Gets and handles packets.
 *
 */
static void PacketModuleThread(void* pData)
{
  for (;;)
  {
    if (Packet_Get())
      HandlePacket();
  }
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)
  uint8_t x,z = 0;
  uint32_t y = 0;
  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);

    if (analogData->channelNb == 0)
    {
    	y = ((analogInputValue*100)/35)*1000;
    	if (y < 1030)
    		x = 254;
    	else if (y > 1030)
    		x = 0;
	    Packet_Put(0x10, analogInputValue, x, 0);
    }
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

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          0); // Highest priority

  error = OS_ThreadCreate(PacketModuleThread,
                            NULL,
                            &PacketModuleThreadStack[THREAD_STACK_SIZE - 1],
                            1); // Highest priority

  error = OS_ThreadCreate(PitModuleThread,
                          NULL,
                          &PitModuleThreadStack[THREAD_STACK_SIZE - 1],
                          3); // Third priority
  // Create threads for 3 analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogLoopbackThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }

  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
