/*! @file
 *
 *  @brief HAL for the accelerometer.
 *
 *  This contains the functions for interfacing to the MMA8451Q accelerometer.
 *
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-06-06
 */

/*!
 *  @addtogroup accel_module Accel module documentation
 *  @{
 */

// Accelerometer functions
#include "accel.h"

// Inter-Integrated Circuit
#include "I2C.h"

// Median filter
#include "median.h"

// K70 module registers
#include "MK70F12.h"

// CPU and PE_types are needed for critical section variables and the defintion of NULL pointer
#include "CPU.h"
#include "PE_types.h"

#include "OS.h"
#include "Semaphore.h"

// Accelerometer registers
#define ADDRESS_OUT_X_MSB 0x01

#define ADDRESS_INT_SOURCE 0x0C
#define ADDRESS_ACCELEROMETER 0x1D

static union
{
  uint8_t byte;			/*!< The INT_SOURCE bits accessed as a byte. */
  struct
  {
    uint8_t SRC_DRDY   : 1;	/*!< Data ready interrupt status. */
    uint8_t            : 1;
    uint8_t SRC_FF_MT  : 1;	/*!< Freefall/motion interrupt status. */
    uint8_t SRC_PULSE  : 1;	/*!< Pulse detection interrupt status. */
    uint8_t SRC_LNDPRT : 1;	/*!< Orientation interrupt status. */
    uint8_t SRC_TRANS  : 1;	/*!< Transient interrupt status. */
    uint8_t SRC_FIFO   : 1;	/*!< FIFO interrupt status. */
    uint8_t SRC_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt status. */
  } bits;			/*!< The INT_SOURCE bits accessed individually. */
} INT_SOURCE_Union;

#define INT_SOURCE     			INT_SOURCE_Union.byte
#define INT_SOURCE_SRC_DRDY		INT_SOURCE_Union.bits.SRC_DRDY
#define INT_SOURCE_SRC_FF_MT		CTRL_REG4_Union.bits.SRC_FF_MT
#define INT_SOURCE_SRC_PULSE		CTRL_REG4_Union.bits.SRC_PULSE
#define INT_SOURCE_SRC_LNDPRT		CTRL_REG4_Union.bits.SRC_LNDPRT
#define INT_SOURCE_SRC_TRANS		CTRL_REG4_Union.bits.SRC_TRANS
#define INT_SOURCE_SRC_FIFO		CTRL_REG4_Union.bits.SRC_FIFO
#define INT_SOURCE_SRC_ASLP		CTRL_REG4_Union.bits.SRC_ASLP

#define ADDRESS_CTRL_REG1 0x2A

typedef enum
{
  DATE_RATE_800_HZ,
  DATE_RATE_400_HZ,
  DATE_RATE_200_HZ,
  DATE_RATE_100_HZ,
  DATE_RATE_50_HZ,
  DATE_RATE_12_5_HZ,
  DATE_RATE_6_25_HZ,
  DATE_RATE_1_56_HZ
} TOutputDataRate;

typedef enum
{
  SLEEP_MODE_RATE_50_HZ,
  SLEEP_MODE_RATE_12_5_HZ,
  SLEEP_MODE_RATE_6_25_HZ,
  SLEEP_MODE_RATE_1_56_HZ
} TSLEEPModeRate;

static union
{
  uint8_t byte;			/*!< The CTRL_REG1 bits accessed as a byte. */
  struct
  {
    uint8_t ACTIVE    : 1;	/*!< Mode selection. */
    uint8_t F_READ    : 1;	/*!< Fast read mode. */
    uint8_t LNOISE    : 1;	/*!< Reduced noise mode. */
    uint8_t DR        : 3;	/*!< Data rate selection. */
    uint8_t ASLP_RATE : 2;	/*!< Auto-WAKE sample frequency. */
  } bits;			/*!< The CTRL_REG1 bits accessed individually. */
} CTRL_REG1_Union;

#define CTRL_REG1     		    	CTRL_REG1_Union.byte
#define CTRL_REG1_ACTIVE	    	CTRL_REG1_Union.bits.ACTIVE
#define CTRL_REG1_F_READ  	  	CTRL_REG1_Union.bits.F_READ
#define CTRL_REG1_LNOISE  	  	CTRL_REG1_Union.bits.LNOISE
#define CTRL_REG1_DR	    	  	CTRL_REG1_Union.bits.DR
#define CTRL_REG1_ASLP_RATE	  	CTRL_REG1_Union.bits.ASLP_RATE

#define ADDRESS_CTRL_REG2 0x2B

#define ADDRESS_CTRL_REG3 0x2C

static union
{
  uint8_t byte;			/*!< The CTRL_REG3 bits accessed as a byte. */
  struct
  {
    uint8_t PP_OD       : 1;	/*!< Push-pull/open drain selection. */
    uint8_t IPOL        : 1;	/*!< Interrupt polarity. */
    uint8_t WAKE_FF_MT  : 1;	/*!< Freefall/motion function in SLEEP mode. */
    uint8_t WAKE_PULSE  : 1;	/*!< Pulse function in SLEEP mode. */
    uint8_t WAKE_LNDPRT : 1;	/*!< Orientation function in SLEEP mode. */
    uint8_t WAKE_TRANS  : 1;	/*!< Transient function in SLEEP mode. */
    uint8_t FIFO_GATE   : 1;	/*!< FIFO gate bypass. */
  } bits;			/*!< The CTRL_REG3 bits accessed individually. */
} CTRL_REG3_Union;

#define CTRL_REG3     		    	CTRL_REG3_Union.byte
#define CTRL_REG3_PP_OD		    	CTRL_REG3_Union.bits.PP_OD
#define CTRL_REG3_IPOL		    	CTRL_REG3_Union.bits.IPOL
#define CTRL_REG3_WAKE_FF_MT		CTRL_REG3_Union.bits.WAKE_FF_MT
#define CTRL_REG3_WAKE_PULSE		CTRL_REG3_Union.bits.WAKE_PULSE
#define CTRL_REG3_WAKE_LNDPRT		CTRL_REG3_Union.bits.WAKE_LNDPRT
#define CTRL_REG3_WAKE_TRANS		CTRL_REG3_Union.bits.WAKE_TRANS
#define CTRL_REG3_FIFO_GATE	  	CTRL_REG3_Union.bits.FIFO_GATE

#define ADDRESS_CTRL_REG4 0x2D

static union
{
  uint8_t byte;			/*!< The CTRL_REG4 bits accessed as a byte. */
  struct
  {
    uint8_t INT_EN_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t               : 1;
    uint8_t INT_EN_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_EN_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_EN_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_EN_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_EN_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_EN_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG4 bits accessed individually. */
} CTRL_REG4_Union;

#define CTRL_REG4            		CTRL_REG4_Union.byte
#define CTRL_REG4_INT_EN_DRDY	  	CTRL_REG4_Union.bits.INT_EN_DRDY
#define CTRL_REG4_INT_EN_FF_MT		CTRL_REG4_Union.bits.INT_EN_FF_MT
#define CTRL_REG4_INT_EN_PULSE		CTRL_REG4_Union.bits.INT_EN_PULSE
#define CTRL_REG4_INT_EN_LNDPRT		CTRL_REG4_Union.bits.INT_EN_LNDPRT
#define CTRL_REG4_INT_EN_TRANS		CTRL_REG4_Union.bits.INT_EN_TRANS
#define CTRL_REG4_INT_EN_FIFO	  	CTRL_REG4_Union.bits.INT_EN_FIFO
#define CTRL_REG4_INT_EN_ASLP	  	CTRL_REG4_Union.bits.INT_EN_ASLP

#define ADDRESS_CTRL_REG5 0x2E

static union
{
  uint8_t byte;			/*!< The CTRL_REG5 bits accessed as a byte. */
  struct
  {
    uint8_t INT_CFG_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t                : 1;
    uint8_t INT_CFG_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_CFG_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_CFG_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_CFG_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_CFG_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_CFG_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG5 bits accessed individually. */
} CTRL_REG5_Union;

#define CTRL_REG5     		      	CTRL_REG5_Union.byte
#define CTRL_REG5_INT_CFG_DRDY		CTRL_REG5_Union.bits.INT_CFG_DRDY
#define CTRL_REG5_INT_CFG_FF_MT		CTRL_REG5_Union.bits.INT_CFG_FF_MT
#define CTRL_REG5_INT_CFG_PULSE		CTRL_REG5_Union.bits.INT_CFG_PULSE
#define CTRL_REG5_INT_CFG_LNDPRT	CTRL_REG5_Union.bits.INT_CFG_LNDPRT
#define CTRL_REG5_INT_CFG_TRANS		CTRL_REG5_Union.bits.INT_CFG_TRANS
#define CTRL_REG5_INT_CFG_FIFO		CTRL_REG5_Union.bits.INT_CFG_FIFO
#define CTRL_REG5_INT_CFG_ASLP		CTRL_REG5_Union.bits.INT_CFG_ASLP

static TAccelData AccelData[3];
static void (*DataReadyCallbackFunction)(void*);
static void *DataReadyCallbackArguments;

/*! @brief Initializes the accelerometer by calling the initialization routines of the supporting software modules.
 *
 *  @param accelSetup is a pointer to an accelerometer setup structure.
 *  @return BOOL - TRUE if the accelerometer module was successfully initialized.
 */
bool Accel_Init(const TAccelSetup* const accelSetup)
{
  TI2CModule i2CModule;
  i2CModule.baudRate = 100000;
  i2CModule.primarySlaveAddress = ADDRESS_ACCELEROMETER;
  i2CModule.readCompleteCallbackFunction = accelSetup->readCompleteCallbackFunction;
  i2CModule.readCompleteCallbackArguments = accelSetup->readCompleteCallbackArguments;

  DataReadyCallbackFunction = accelSetup->dataReadyCallbackFunction;
  DataReadyCallbackArguments = accelSetup->dataReadyCallbackArguments;

  if (I2C_Init(&i2CModule,accelSetup->moduleClk))
  {
    CTRL_REG1_ACTIVE = 0;				// Disable Active mode
    I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);		// Initiate the I2C write function

    //Set for Accelerometer Control Register 1
    CTRL_REG1_DR = DATE_RATE_1_56_HZ; //Desired frequency for Synchronous mode
    CTRL_REG1_F_READ = 1; 				// Fast read mode for 8 bit precision - auto increment counter will skip over the LS bytes
    CTRL_REG1_LNOISE = 0; 				// Reduced noise mode disabled
    CTRL_REG1_ASLP_RATE = SLEEP_MODE_RATE_1_56_HZ; 	// Set sleep mode the same as date rate selection speed
    I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);		// Initiate the I2C write function

    //Set for Accelerometer Control Register 3
    CTRL_REG3_PP_OD = 0; 				// Set for push-pull
    CTRL_REG3_IPOL = 1; 				// Set for Active High
    I2C_Write(ADDRESS_CTRL_REG3, CTRL_REG3); 		// Initiate the I2C write function

    //Control Register 4
    CTRL_REG4_INT_EN_DRDY = 0; 				// Disable Data Ready Interrupt (default)
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4); 		// Initiate the I2C write function

    //Control Register 5
    CTRL_REG5_INT_CFG_DRDY = 1;				// Send data ready interrupt to INT1 pin
    I2C_Write(ADDRESS_CTRL_REG5, CTRL_REG5); 		// Initiate the I2C write function

    CTRL_REG1_ACTIVE = 1;				// Enable Active mode
    I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);		// Initiate the I2C write function

    Accel_SetMode(ACCEL_POLL);				// Set default mode to poll

    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;			// Enable port b
    PORTB_PCR4 |= PORT_PCR_MUX(1);			// Set port b 4 to alt 1 GPIO
    PORTB_PCR4 |= PORT_PCR_ISF_MASK;			// Clear interrupt flag
    PORTB_PCR4 |= PORT_PCR_IRQC(9);			// Set to interrupt on rising

    NVICICPR2 = (1 << 24);				// Clear pending interrupts
    NVICISER2 = (1 << 24);				// Enable interrupts

    return true;
  }
  else
    return false;
}

/*! @brief Reads X, Y and Z accelerations.
 *
 *  @param data is a an array of 3 bytes where the X, Y and Z data are stored.
 */
void Accel_ReadXYZ(uint8_t data[3])
{
  for (int i = 2; i > 0; i--)
  {
    AccelData[i].axes.x = AccelData[i-1].axes.x;
    AccelData[i].axes.y = AccelData[i-1].axes.y;
    AccelData[i].axes.z = AccelData[i-1].axes.z;
  }

  I2C_PollRead(ADDRESS_OUT_X_MSB, AccelData[0].bytes, 3);					// Initiate PollRead from I2C module

  data[0] = Median_Filter3(AccelData[0].axes.x,AccelData[1].axes.x,AccelData[2].axes.x);	// Gets the middle number for x
  data[1] = Median_Filter3(AccelData[0].axes.y,AccelData[1].axes.y,AccelData[2].axes.y);	// Gets the middle number for y
  data[2] = Median_Filter3(AccelData[0].axes.z,AccelData[1].axes.z,AccelData[2].axes.z);	// Gets the middle number for z
}

/*! @brief Set the mode of the accelerometer.
 *
 *  @param mode specifies either polled or interrupt driven operation.
 */
void Accel_SetMode(const TAccelMode mode)
{
  CTRL_REG1_ACTIVE = 0;				// Disable Active mode
  I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);	// Initiate the I2C write function
  if (mode == ACCEL_POLL)
  {
    CTRL_REG4_INT_EN_DRDY = 0; 			// Data ready interrupt disabled
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4); 	// I2C write the interrupt enable register
  }
  else if (mode == ACCEL_INT)
  {
    CTRL_REG4_INT_EN_DRDY = 1; 			// Data ready interrupt enable
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4); 	// I2C write the interrupt enable register
  }
  CTRL_REG1_ACTIVE = 1;
  I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);
}

/*! @brief Interrupt service routine for the accelerometer.
 *
 *  The accelerometer has data ready.
 *  The user callback function will be called.
 *  @note Assumes the accelerometer has been initialized.
 */
void __attribute__ ((interrupt)) AccelDataReady_ISR(void)
{
  OS_ISREnter();
  PORTB_ISFR |= PORT_ISFR_ISF(4);				// Clear interrupt flag
  OS_SemaphoreSignal(ACCELIntReady);
  OS_ISRExit();
}

/*!
 * @}
*/
