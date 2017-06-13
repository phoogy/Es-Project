/*! @file
 *
 *  @brief I/O routines for the K70 I2C interface.
 *
 *  This contains the functions for operating the I2C (inter-integrated circuit) module.
 *
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-06-06
 */

/*!
 *  @addtogroup i2c_module I2C module documentation
 *  @{
 */

#include "I2C.h"
#include "MK70F12.h"
#include "PE_Types.h"
#include "Cpu.h"
#include "OS.h"
#include "Semaphore.h"

static void (*ReadCompleteCallbackFunction)(void *);
static void *ReadCompleteCallbackArguments;
static uint8_t SlaveAddress;
static uint8_t RxNbBytes;
static uint8_t *RxData;
static uint8_t InterruptCount;

static uint8_t GetFrequencyDivider(const uint32_t baudRate,const uint32_t moduleClk);
static void WaitBusy(void);
static void Wait(void);

static OS_ECB* I2CWRITESEMAPHORE;

/*! @brief Sets up the I2C before first use.
 *
 *  @param aI2CModule is a structure containing the operating conditions for the module.
 *  @param moduleClk The module clock in Hz.
 *  @return BOOL - TRUE if the I2C module was successfully initialized.
 */
// pg 1887
bool I2C_Init(const TI2CModule* const aI2CModule, const uint32_t moduleClk)
{
  /*
   * Chapter 55 Inter-Integrated Circuit (I2C)
   * K70 Sub-Family Reference Manual, Rev. 3, November 2014
   * Freescale Semiconductor, Inc. 1887
   */

  ReadCompleteCallbackFunction = aI2CModule->readCompleteCallbackFunction;
  ReadCompleteCallbackArguments = aI2CModule->readCompleteCallbackArguments;

  I2CWRITESEMAPHORE = OS_SemaphoreCreate(1);
  SIM_SCGC4 |= SIM_SCGC4_IIC0_MASK;           				// Enable I2C0 clock gate control
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;					// Enable PORTE clock gate control
  PORTE_PCR18 |= PORT_PCR_MUX(4); 					// Set PORTE_PCR18 to Alt 4 for I2C0_SDA pg.280
  PORTE_PCR18 |= PORT_PCR_ODE_MASK; 					// Enable open drain pg.184

  PORTE_PCR19 |= PORT_PCR_MUX(4); 					// Set PORTE_PCR19 to Alt 4 for I2C0_SCL
  PORTE_PCR19 |= PORT_PCR_ODE_MASK; 					// Enable open drain

  I2C0_F = GetFrequencyDivider(aI2CModule->baudRate, moduleClk);	// Write: Frequency Divider register to set the I2C baud rate

  I2C0_C1 &= ~I2C_C1_MST_MASK;						//Set for Master Mode Select - Slave mode

  I2C_SelectSlaveDevice(aI2CModule->primarySlaveAddress);		// Select Save Device

  NVICICPR0 = (1 << 24); 						// Clear pending interrupts on I2C
  NVICISER0 = (1 << 24); 						// Set interrupts on I2C

  I2C0_C1 |= I2C_C1_IICEN_MASK;						// Write: Control Register 1 to enable the I2C module

  return true;
}

/*! @brief Selects the current slave device
 *
 * @param slaveAddress The slave device address.
 */
void I2C_SelectSlaveDevice(const uint8_t slaveAddress)
{
  SlaveAddress = slaveAddress;						// Store slave address in global variable
}

/*! @brief Write a byte of data to a specified register
 *
 * @param registerAddress The register address.
 * @param data The 8-bit data to write.
 */
void I2C_Write(const uint8_t registerAddress, const uint8_t data)
{
  OS_SemaphoreWait(I2CWRITESEMAPHORE, 0);
  WaitBusy();
  I2C0_C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);  			// Start and set to Transmit
  I2C0_D = (SlaveAddress << 1);			              		// Slave address transmission with write
  Wait();
  I2C0_D = registerAddress;			                  	// Register address to write to
  Wait();
  I2C0_D = data;				                        // Data to write to the address
  Wait();
  I2C0_C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK); 			// Stop and set to Receive
  OS_SemaphoreSignal(I2CWRITESEMAPHORE);
}

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses polling as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{
  WaitBusy();
  I2C0_C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK); 			// Start and set to Transmit
  I2C0_D = (SlaveAddress << 1);						// Slave address transmission with write
  Wait();
  I2C0_D = registerAddress;						// Register address to write to
  Wait();
  I2C0_C1 |= I2C_C1_RSTA_MASK;						// Repeat Start
  I2C0_D = ((SlaveAddress << 1) + 1);					// Slave address transmission with read
  Wait();
  I2C0_C1 &= ~I2C_C1_TX_MASK;						// Write: Control Register 1 to enable RX
  I2C0_C1 &= ~I2C_C1_TXAK_MASK;						// Write: Control Register 1 to send acknowledge signal ? do we need to send AK? Not Sure

  data[0] = I2C0_D;							// Clear initial data from register by reading from it.
  Wait();

  for (int i = 0; i < nbBytes - 1; i++)
  {
    data[i] = I2C0_D;							// Store data to variable
    Wait();
  }

  I2C0_C1 |= I2C_C1_TXAK_MASK;						// Write: Control Register 1 to send Not acknowledge signal
  data[nbBytes-1] = I2C0_D;						// Store last byte data into variable
  Wait();
  I2C0_C1 &= ~I2C_C1_MST_MASK;						// Stop
}


/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses interrupts as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{
  RxNbBytes = nbBytes;							// Store nbBytes in a global variable
  RxData = data;							// Store pointer of data into a global variable
  InterruptCount = 0;							// Set InterruptCount to 0

  WaitBusy();
  I2C0_C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK); 			// Start and set to Transmit
  I2C0_D = (SlaveAddress << 1);						// Slave address transmission with write
  Wait();
  I2C0_D = registerAddress;						// Register address to write to
  Wait();
  I2C0_C1 |= I2C_C1_RSTA_MASK;						// Repeat Start
  I2C0_D = ((SlaveAddress << 1) + 1);					// Slave address transmission with read
  Wait();

  I2C0_C1 &= ~I2C_C1_TX_MASK;						// Write: Control Register 1 to enable RX
  I2C0_C1 &= ~I2C_C1_TXAK_MASK;						// Write: Control Register 1 to send acknowledge signal

  I2C0_C1 |= I2C_C1_IICIE_MASK;						// Enable Interrupts
  RxData[0] = I2C0_D;							// Clear initial data from register by reading from it.
}

/*! @brief Interrupt service routine for the I2C.
 *
 *  Only used for reading data.
 *  At the end of reception, the user callback function will be called.
 *  @note Assumes the I2C module has been initialized.
 */
void __attribute__ ((interrupt)) I2C_ISR(void)
{
  OS_ISREnter();
  I2C0_S |= I2C_S_IICIF_MASK;
  if (!(I2C0_C1 & I2C_C1_TX_MASK))
  {
    if (RxNbBytes > 1)
      RxData[InterruptCount] = I2C0_D;						// Read Data
    else
    {
      I2C0_C1 &= ~I2C_C1_IICIE_MASK;						// Disable Interrupts
      I2C0_C1 |= I2C_C1_TXAK_MASK;						// Send Not acknowledge paclet
      RxData[InterruptCount] = I2C0_D;						// Read Data
      Wait();									// Wait
      I2C0_C1 &= ~I2C_C1_MST_MASK;						// Stop

      OS_SemaphoreSignal(I2CReady);
    }
    RxNbBytes--;
    InterruptCount++;
  }
  OS_ISRExit();
}

/*! @brief Returns the FrequencyDivider for I2Cx_F from baudRate and moduleClk
 *
 * @param frequencyDivider The variable to return
 * @param smallestBaudRateDifference The smallest baudrate calculated/found.
 * @param calculatedBaudRateDifference The calculated baudrate difference between the original baudrate.
 * @param calculatedBaudRate The calculated baudrate.
 * @param multiplier An array of multipliers.
 * @param sclDivider An array of 64 values containing the scl dividers.
 * @return uint8_t - frequencyDivider after obtaining the closest frequency divider.
 */
static uint8_t GetFrequencyDivider(const uint32_t baudRate,const uint32_t moduleClk)
{
  /* Initiate variables to be used to calculate closest Frequency Divider */
  static uint8_t frequencyDivider;                          						// Variable to return.
  static float smallestBaudRateDifference, calculatedBaudRateDifference, calculatedBaudRate;		// Baud rate variables used in calculation
  static uint32_t multiplier[3] = {1, 2, 4};								// Multipler used in the Frequency Divider calculation.
  static uint32_t sclDivider[64] = {20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56, 	// SCL Divider values in the order of ICR values. pg 1878 Table 55-41
      68, 48, 56, 64, 72, 80, 88, 104, 128, 80, 96, 112, 128, 144, 160, 192, 240, 160, 192, 224,
      256, 288, 320, 384, 480, 320, 384, 448, 512, 576, 640, 768, 960, 640, 768, 896, 1024, 1152,
      1280, 1536, 1920, 1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840};

  smallestBaudRateDifference = baudRate;					          		// Set smallest difference to the max value;

  for (int i = 0; i < 3; i++)							                  	// Loop through the multiplier values
  {
    for (int j = 0; j < 64; j++)						                	// Loop through the scl Divider values
    {
      calculatedBaudRate = moduleClk / (multiplier[i] * sclDivider[j]);		 			// I2C baud rate = I2C module clock speed (Hz)/(mul × SCL divider)
      if (baudRate >= calculatedBaudRate)					          		// Gets the difference in a positive value for easier comparison
        calculatedBaudRateDifference = baudRate - calculatedBaudRate;
      else
        calculatedBaudRateDifference = calculatedBaudRate - baudRate;
      if (calculatedBaudRateDifference <= smallestBaudRateDifference)		   			// Sets new frequency Divider if baudrate difference is smaller
      {
        smallestBaudRateDifference = calculatedBaudRateDifference;
        frequencyDivider = (I2C_F_MULT(multiplier[i]) | I2C_F_ICR(j));
      }
    }
  }
  return frequencyDivider;
}

/*! @brief Waits until the I2C0 data register is not busy
 * @return void
 */
static void WaitBusy(void)
{
  while (I2C0_S & I2C_S_BUSY_MASK);
}

/*! @brief Waits until the I2C0 interrupt flag is set then clears it
 * @return void
 */
static void Wait(void)
{
  while (!(I2C0_S & I2C_S_IICIF_MASK));
  I2C0_S |= I2C_S_IICIF_MASK;
}

/*!
* @}
*/
