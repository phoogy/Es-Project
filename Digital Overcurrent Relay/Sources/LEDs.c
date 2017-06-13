/*! @file LEDs.c
 *  @brief Routines to access the LEDs on the TWR-K70F120M.
 *  This contains the functions for operating the LEDs.
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-05-17
 */

/*!
 *  @addtogroup leds_module LEDs module documentation
 *  @{
 */

#include "LEDs.h"
#include "MK70F12.h"

/*! @brief Sets up the LEDs before first use.
 *
 *  @return BOOL - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void)
{
  /* Enable PortA */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  /* Enable all LEDs through multiplexing */
  PORTA_PCR11 |= PORT_PCR_MUX(1);	// Enable Orange LED
  PORTA_PCR28 |= PORT_PCR_MUX(1);	// Enable Yellow LED
  PORTA_PCR29 |= PORT_PCR_MUX(1);	// Enable Green LED
  PORTA_PCR10 |= PORT_PCR_MUX(1);	// Enable Blue LED

  /* Setting as Active-Low, so they don't light up when initialised */
  GPIOA_PDOR |= (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE);

  /* Set the pins as outputs for LEDs */
  GPIOA_PDDR |= (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE);

  return true;
}

/*! @brief Turns an LED on.
 *
 *  @param color The color of the LED to turn on.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_On(const TLED color)
{
  GPIOA_PCOR |= color; /* Use the Port Clear Output */
}

/*! @brief Turns off an LED.
 *
 *  @param color The color of the LED to turn off.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Off(const TLED color)
{
  GPIOA_PSOR |= color; /* Use the Port Set Output */
}

/*! @brief Toggles an LED.
 *
 *  @param color The color of the LED to toggle.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Toggle(const TLED color)
{
  GPIOA_PTOR |= color; /* Use the Port Toggle Output */
}

/*!
 * @}
*/
