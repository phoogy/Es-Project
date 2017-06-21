/*! @file PIT.c
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-05-17
 */

/*!
 *  @addtogroup pit_module PIT module documentation
 *  @{
 */

#include "PIT.h"
#include "MK70F12.h"
#include "OS.h"
//#include "Semaphore.h"

static uint32_t ClockPeriod;
static uint32_t TimerPeriod;
OS_ECB* PITReady[4];

bool PIT_Init(const uint32_t moduleClk)
{
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;		// Enable PIT clock gating


  /* Convert from seconds to milliseconds */
  ClockPeriod = 1000000000 / moduleClk; 	// Convert module clock to get clock period in nanoseconds

  PIT_MCR &= ~PIT_MCR_MDIS_MASK;		// Enable Pit Module
  PIT_MCR |= PIT_MCR_FRZ_MASK;			// Freeze timer during debug mode

  /* Initialise NVIC */
  NVICICPR2 = (1 << 4); 			// Clear pending interrupts on PIT module
  NVICISER2 = (1 << 4); 			// Enable interrupts from Pit module

  for (int i=0; i < sizeof(PITReady); i++)
    PITReady[i] = OS_SemaphoreCreate(0);

  return true;
}

void PIT_Set(const uint8_t pitClock, const uint32_t period, const bool restart)
{
  TimerPeriod = period;
  switch(pitClock)
  {
    case 0:
      PIT_LDVAL0 = (TimerPeriod / ClockPeriod) - 1;  // Calculate number of cycles and set LDVAL
      break;
    case 1:
      PIT_LDVAL1 = (TimerPeriod / ClockPeriod) - 1;  // Calculate number of cycles and set LDVAL
      break;
    case 2:
      PIT_LDVAL2 = (TimerPeriod / ClockPeriod) - 1;  // Calculate number of cycles and set LDVAL
      break;
    case 3:
      PIT_LDVAL3 = (TimerPeriod / ClockPeriod) - 1;  // Calculate number of cycles and set LDVAL
      break;
  }
  if (restart)
  {
    PIT_Enable(pitClock, false);				// Disable the PIT Timer
    PIT_Enable(pitClock, true);				// Enable the PIT timer for new LDVAl value
  }
}

void PIT_Get(const uint8_t pitClock, uint32_t * const dataPtr)
{
  switch(pitClock)
  {
    case 0:
      *dataPtr = (uint32_t)(ClockPeriod*(PIT_CVAL0 + 1));
      break;
    case 1:
      *dataPtr = (uint32_t)(ClockPeriod*(PIT_CVAL1 + 1));
      break;
    case 2:
      *dataPtr = (uint32_t)(ClockPeriod*(PIT_CVAL2 + 1));
      break;
    case 3:
      *dataPtr = (uint32_t)(ClockPeriod*(PIT_CVAL3 + 1));
      break;
  }
}

void PIT_Update(const uint8_t pitClock, const uint32_t updatedperiod)
{
  PIT_Set(pitClock, updatedperiod, true);
  PIT_Set(pitClock, TimerPeriod, false);
}

void PIT_Enable(const uint8_t pitClock, const bool enable)
{
  if (enable)
  {
    switch(pitClock)
    {
      case 0:
        PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;   // Enable the timer interrupt
        PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;     // Enable timer
        break;
      case 1:
        PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;   // Enable the timer interrupt
        PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;     // Enable timer
        break;
      case 2:
        PIT_TCTRL2 |= PIT_TCTRL_TIE_MASK;   // Enable the timer interrupt
        PIT_TCTRL2 |= PIT_TCTRL_TEN_MASK;     // Enable timer
        break;
      case 3:
        PIT_TCTRL3 |= PIT_TCTRL_TIE_MASK;   // Enable the timer interrupt
        PIT_TCTRL3 |= PIT_TCTRL_TEN_MASK;     // Enable timer
        break;
    }
  }
  else
  {
    switch(pitClock)
    {
      case 0:
        PIT_TCTRL0 &= ~PIT_TCTRL_TIE_MASK;    // Disable the timer interrupt
        PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;    // Disable timer
        break;
      case 1:
        PIT_TCTRL1 &= ~PIT_TCTRL_TIE_MASK;    // Disable the timer interrupt
        PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK;    // Disable timer
        break;
      case 2:
        PIT_TCTRL2 &= ~PIT_TCTRL_TIE_MASK;    // Disable the timer interrupt
        PIT_TCTRL2 &= ~PIT_TCTRL_TEN_MASK;    // Disable timer
        break;
      case 3:
        PIT_TCTRL3 &= ~PIT_TCTRL_TIE_MASK;    // Disable the timer interrupt
        PIT_TCTRL3 &= ~PIT_TCTRL_TEN_MASK;    // Disable timer
        break;
    }

  }
}

void __attribute__ ((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();
  if (PIT_TFLG0 & PIT_TFLG_TIF_MASK)
  {
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;   // Clear interrupt flag 0
    OS_SemaphoreSignal(PITReady[0]);
  }

  if (PIT_TFLG1 & PIT_TFLG_TIF_MASK)
  {
    PIT_TFLG1 |= PIT_TFLG_TIF_MASK;   // Clear interrupt flag 1
    OS_SemaphoreSignal(PITReady[1]);
  }

  if (PIT_TFLG2 & PIT_TFLG_TIF_MASK)
  {
    PIT_TFLG2 |= PIT_TFLG_TIF_MASK;   // Clear interrupt flag 2
    OS_SemaphoreSignal(PITReady[2]);
  }

  if (PIT_TFLG3 & PIT_TFLG_TIF_MASK)
  {
    PIT_TFLG3 |= PIT_TFLG_TIF_MASK;   // Clear interrupt flag 3
    OS_SemaphoreSignal(PITReady[3]);
  }
  OS_ISRExit();
}

/*!
 * @}
*/
