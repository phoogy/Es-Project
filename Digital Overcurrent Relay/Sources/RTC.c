/*! @file RTC.c
 *  @brief Routines for controlling the Real Time Clock (RTC) on the TWR-K70F120M.
 *  This contains the functions for operating the real time clock (RTC).
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-05-17
 */

/*!
 *  @addtogroup rtc_module RTC module documentation
 *  @{
 */

#include "RTC.h"
#include "MK70F12.h"
#include "OS.h"
#include "Semaphore.h"

static void (*CallbackFunction)(void *);	// Variable for pointer to the callback function
static void *CallbackArguments;			// Variable for pointer to the callback variable

bool RTC_Init(void (*userFunction)(void*), void* userArguments)
{
  SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;		// Enable RTC clock

  CallbackFunction  = userFunction;		// Storing userFunction
  CallbackArguments = userArguments;		// Storing UserArguments

  RTC_LR |= RTC_LR_LRL_MASK;			// Unlock Control Register

  RTC_CR = RTC_CR_SWR_MASK; 			// Reset the RTC Registers
  RTC_CR &= ~RTC_CR_SWR_MASK;			// Clear the Software Reset bit

  /* Tower Schematics states 18pF for the configuration load so we enable 16 and 2 */
  RTC_CR |= RTC_CR_SC2P_MASK; 			// Enable Oscillator 2pF
  RTC_CR |= RTC_CR_SC16P_MASK;			// Enable Oscillator 16pF

  RTC_CR |= RTC_CR_OSCE_MASK; 			// Enable the 32.768 kHz oscillator
  int i;
  for (i = 0; i < 5000000; i++);
  // Need to wait 500ms for it to become stable page 33 of the datasheet

  /* If the time is invalid then set to 1 and clear the invalid flag */
  if (RTC_SR & RTC_SR_TIF_MASK)
  {
    RTC_TSR = 1;				// Set time to 1
    RTC_SR &= ~RTC_SR_TIF_MASK;			// Clear invalid flag
  }
  RTC_LR &= ~RTC_LR_CRL_MASK; 			// Lock Control Register

  RTC_IER |= RTC_IER_TSIE_MASK; 		// Enabling the Interrupt to occur every second
  RTC_IER &= ~RTC_IER_TAIE_MASK;		// Disable Time Alarm
  RTC_IER &= ~RTC_IER_TOIE_MASK;		// Disable Overflow Interrupt
  RTC_IER &= ~RTC_IER_TIIE_MASK;		// Disable Time Invalid Interrupt

  RTC_SR |= RTC_SR_TCE_MASK;         		// Enable the Timer

  /* Initialise NVIC */
  NVICICPR2 = (1 << 3);       			// Clear pending interrupts on RTC module
  NVICISER2 = (1 << 3);       			// Enable interrupts from RTC module

  /* IRQ 67 Mod 32 = 3
   * pg.90 for value
   * pg.92 for formula */

  return true;
}

/*! @brief Sets the value of the real time clock.
 *
 *  @param hours The desired value of the real time clock hours (0-23).
 *  @param minutes The desired value of the real time clock minutes (0-59).
 *  @param seconds The desired value of the real time clock seconds (0-59).
 *  @note Assumes that the RTC module has been initialized and all input parameters are in range.
 */
void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  static uint32_t time;
  time = (hours * 60 * 60) + (minutes * 60) + seconds;	// Calculate time
  RTC_SR &= ~RTC_SR_TCE_MASK;        		// Disable Timer.
  RTC_TSR = time;           	 		// Write the new time value to RTC_TSR
  RTC_SR |= RTC_SR_TCE_MASK;         		// Enable the Timer
}

/*! @brief Gets the value of the real time clock.
 *
 *  @param hours The address of a variable to store the real time clock hours.
 *  @param minutes The address of a variable to store the real time clock minutes.
 *  @param seconds The address of a variable to store the real time clock seconds.
 *  @note Assumes that the RTC module has been initialized.
 */
void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  static uint32_t time;
  static uint32_t time2;
  // Page 1400 ish. Need to read RTC_TSR twice....... compare and make sure they match
  time = RTC_TSR; // do while
  time2 = RTC_TSR;
  while (time != time2)
  {
    time = RTC_TSR;
    time2 = RTC_TSR;
  }
  *hours   = ((time / 3600) % 24);    		// Calculate current hours
  *minutes = ((time % 3600) / 60);    		// Calculate current minutes
  *seconds = ((time % 3600) % 60);    		// Calculate current seconds

}

/*! @brief Interrupt service routine for the RTC.
 *
 *  The RTC has incremented one second.
 *  The user callback function will be called.
 *  @note Assumes the RTC has been initialized.
 */
void __attribute__ ((interrupt)) RTC_ISR(void)
{
  OS_ISREnter();
  OS_SemaphoreSignal(RTCReady);
  OS_ISRExit();
}

/*!
 * @}
*/
