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
#include "Semaphore.h"

static uint32_t ClockPeriod;
static void (*CallbackFunction)(void *);
static void *CallbackArguments;

bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;		// Enable PIT clock gating

  CallbackFunction  = userFunction;		// Storing userFunction
  CallbackArguments = userArguments;		// Storing UserArguments

  /* Convert from seconds to milliseconds */
  ClockPeriod = 1000000000 / moduleClk; 	// Convert module clock to get clock period in nanoseconds

  PIT_MCR &= ~PIT_MCR_MDIS_MASK;		// Enable Pit Module
  PIT_MCR |= PIT_MCR_FRZ_MASK;			// Freeze timer during debug mode

  /* Initialise NVIC */
  NVICICPR2 = (1 << 4); 			// Clear pending interrupts on PIT module
  NVICISER2 = (1 << 4); 			// Enable interrupts from Pit module

  return true;
}

void PIT_Set(const uint32_t period, const bool restart)
{
  PIT_LDVAL0 = (period * 1000000 / ClockPeriod) - 1;	// Calculate number of cycles and set LDVAL
  if (restart)
  {
    PIT_Enable(false);				// Disable the PIT Timer
    PIT_Enable(true);				// Enable the PIT timer for new LDVAl value
  }
}

void PIT_Enable(const bool enable)
{
  if (enable)
  {
    PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;		// Enable the timer interrupt
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;  		// Enable timer
  }
  else
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TIE_MASK;		// Disable the timer interrupt
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK; 		// Disable timer
  }
}

void __attribute__ ((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;		// Clear interrupt flag
  OS_SemaphoreSignal(PITReady);
  OS_ISRExit();
}

/*!
 * @}
*/
