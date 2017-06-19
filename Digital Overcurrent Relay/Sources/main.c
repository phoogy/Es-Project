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

// PIT functions
#include "PIT.h"

// Non-volatile Memory
#include "Flash.h"


#define AC_TO_DC(x) (double)((x-32768)/32768)*10
#define DC_TO_AC(x) (uint16_t)((32768*x)/10)+32768

// Test parameters
static uint8_t x,y,z;

// ----------------------------------------
// Packet set up
// ----------------------------------------
#define BAUD_RATE (uint32_t)115200  // BaudRate

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the Init thread. */
OS_THREAD_STACK(PacketModuleThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(TestModuleThreadStack, THREAD_STACK_SIZE);
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

static OS_ECB* SemX;
//static OS_ECB* SemY;
//static OS_ECB* SemZ;
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
  // Flash
  (void)Flash_Init();


  // Packet
  (void)Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ);
  SemX = OS_SemaphoreCreate(0);

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
    {
      bool ackFlag = false;
        switch (Packet_Command & 0x7F)
        {
//          case CMD_TOWER_STARTUP:
//            ackFlag = TowerStartup();
//            break;
//          case CMD_TOWER_VERSION:
//            ackFlag = TowerVersion();
//            break;
//          case CMD_TOWER_NUMBER:
//            ackFlag = TowerNumber();
//            break;
//          case CMD_TOWER_MODE:
//            ackFlag = TowerMode();
//            break;
//          case CMD_READ_BYTE:
//            ackFlag = ReadByte();
//            break;
//          case CMD_PROGRAM_BYTE:
//            ackFlag = ProgramByte();
//            break;
//          case CMD_TIME:
//            ackFlag = SetTime();
//            break;
//          case CMD_PROTOCOL_MODE:
//            ackFlag = ProtocolMode();
//            break;
          case 0x09:    // Get values
          {
            //if (Packet_Put)
          }
        }
        if ((Packet_Command & 0x80) == 0x80)
          ACKNAK(ackFlag);
    }
      //HandlePacket();
  }
}

/*! @brief Test thread.
 *
 */
static void TestModuleThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(SemX,0);
//    (void)OS_SemaphoreWait(SemY,0);
//    (void)OS_SemaphoreWait(SemZ,0);
    Packet_Put(0x10, x, y, z);
  }
}


/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)
  TAnalogInput channel[NB_ANALOG_CHANNELS];
  uint8_t valueArrayPos[NB_ANALOG_CHANNELS];
  for (uint8_t i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    valueArrayPos[i] = 0;
    channel[i].putPtr = valueArrayPos[i];
  }
  for (;;)
  {
    int16_t analogInputValue;



    (void)OS_SemaphoreWait(analogData->semaphore, 0);

    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);


    channel[analogData->channelNb].oldValue = channel[analogData->channelNb].value;
    channel[analogData->channelNb].value = analogInputValue;
    channel

    // TODO
    // Analog_Put replaced with
    // for outputChannel1 and outputChannel2
    // timing and

    // Test data
    switch(analogData->channelNb)
    {
      case 0:
        if (AC_TO_DC(analogInputValue) < 1.03)
          x = 0;
        else if (AC_TO_DC(analogInputValue) > 1.03)
          x = 254;
        x = (uint8_t)analogInputValue;
        y = (uint8_t)(AC_TO_DC(analogInputValue));
        z = (uint8_t)((AC_TO_DC(analogInputValue)/10)*254);
        (void)OS_SemaphoreSignal(SemX);
        break;
//      case 1:
//        y = analogInputValue;
//        (void)OS_SemaphoreSignal(SemY);
//        break;
//      case 2:
//        z = analogInputValue;
//        (void)OS_SemaphoreSignal(SemZ);
//        break;
      default:
        break;
    }
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
                          1); // Second priority

  error = OS_ThreadCreate(TestModuleThread,
                          NULL,
                          &TestModuleThreadStack[THREAD_STACK_SIZE - 1],
                          2); // Third priority
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
