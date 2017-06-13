/*! @file FTM.c
 *  @brief Routines for setting up the FlexTimer module (FTM) on the TWR-K70F120M.
 *  This contains the functions for operating the FlexTimer module (FTM).
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-06-06
 */

/*!
 *  @addtogroup ftm_module FTM module documentation
 *  @{
 */

#include "FTM.h"
#include "MK70F12.h"
#include "OS.h"

#define NB_OF_CHANNELS 8

/* Define callback object */
typedef struct {
  void (*callbackFunc)(void *);
  void *callbackArgs;
} TCallback;

/* Create array of n callback objects where n = number of channels */
static TCallback ChannelCallback[NB_OF_CHANNELS];

/* Enables the FTM as a free running 16-bit counter. */
bool FTM_Init()
{
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;		// Enable FTM0 clock gating
  FTM0_MODE |= FTM_MODE_WPDIS_MASK;		// Disable Write Protection to configure FTM0
  FTM0_MODE |= FTM_MODE_FTMEN_MASK;		// Enable the timer module in FTM0 mode
  FTM0_CNTIN = 0;				// Initial counter value
  FTM0_MOD = FTM_MOD_MOD_MASK;			// Maximum value of MOD
  FTM0_CNT = 0;					// Writing to CNT to set it to FTM0_CNTIN
  FTM0_SC = FTM_SC_CLKS(2);			// Set clock source to fixed frequency pg.1210
  NVICICPR1 = (1 << 30);			// Clear any pending interrupts on FTM0
  NVICISER1 = (1 << 30);			// Enable interrupts from FTM0 module

  /* FTM0 IRQ = 62 mod 32 = 30
   * pg.90 for value
   * pg.92 for formula */

  return true;
}

/*! @brief Sets up a timer channel.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *    channelNb is the channel number of the FTM to use.
 *    delayCount is the delay count (in module clock periods) for an output compare event.
 *    timerFunction is used to set the timer up as either an input capture or an output compare.
 *    ioType is a union that depends on the setting of the channel as input capture or output compare:
 *      outputAction is the action to take on a successful output compare.
 *      inputDetection is the type of input capture detection.
 *    userFunction is a pointer to a user callback function.
 *    userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the timer was set up successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
  /*Set channel Mode according to aFTMChannel*/
  if (aFTMChannel->timerFunction == TIMER_FUNCTION_OUTPUT_COMPARE)
  {
    /* Channel Mode Select set for input */
    FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_MSB_MASK;
    FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_MSA_MASK;
  }
  else
  {
    /* Channel(n) Status and Controls (RM: Page 1219 - 1223) */
    FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_MSB_MASK;			// Sets control for output
    FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_MSA_MASK;			// Channel Mode Select set for output
  }

  /* Setup ioType input detection Page 1219 */
  switch (aFTMChannel->ioType.inputDetection)
  {
    case 1:	// Capture on rising edge only
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSA_MASK;
      break;
    case 2:	// Capture on falling edge only
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSA_MASK;
      break;
    case 3:	// Capture on rising or falling edge
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSA_MASK;
      break;
    default:	// Pin not used for FTM, revert to GPIO
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSA_MASK;
      break;
  }

  /*Store callback information for the channel*/
  ChannelCallback[aFTMChannel->channelNb].callbackFunc = aFTMChannel->userFunction;
  ChannelCallback[aFTMChannel->channelNb].callbackArgs = aFTMChannel->userArguments;

  FTM0_MODE |= FTM_MODE_WPDIS_MASK;						// Disable Write protection
  FTM0_CnV(aFTMChannel->channelNb) = FTM0_CNT + aFTMChannel->delayCount;	// Set the timer
  FTM0_CnSC(aFTMChannel->channelNb) = FTM_CnSC_MSA_MASK ; 			// Enable Ch as output compare with interrupts enabled
  FTM0_MODE &= ~FTM_MODE_WPDIS_MASK;						// Enable Write protection

  return true;
}

/*! @brief Starts a timer if set up for output compare.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *  @return bool - TRUE if the timer was started successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{
  FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_CHF_MASK;			// Clear interrupt flag
  FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_CHIE_MASK;			// Enables the Channel Interrupt Enable
  return true;
}

/*! @brief Interrupt service routine for the FTM.
 *
 *  If a timer channel was set up as output compare, then the user callback function will be called.
 *  @note Assumes the FTM has been initialized.
 */
void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  OS_ISREnter();
  uint8_t i;
  /*Determine which channel triggered the interrupt*/
  for (i = 0; i < NB_OF_CHANNELS; i++)
  {
    if ((FTM0_CnSC(i) & FTM_CnSC_CHF_MASK) != 0)				// If the flag is set
    {
      FTM0_CnSC(i) &= ~FTM_CnSC_CHF_MASK;					// Clear Flag
      FTM0_CnSC(i) &= ~FTM_CnSC_CHIE_MASK;					// Disable interrupts
      ChannelCallback[i].callbackFunc(ChannelCallback[i].callbackArgs);		// Initiate callback
      break;
    }
  }
  OS_ISRExit();
}

/*!
 * @}
*/
