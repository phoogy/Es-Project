/*! @file Flash.c
 *  @brief Routines for erasing and writing to the Flash.
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-05-17
 */

/*!
 *  @addtogroup flash_module Flash module documentation
 *  @{
 */

#include "Flash.h"
#include "MK70F12.h"

#define FCMD_PROGRAM_PHRASE 0x07	// Hex command to phrase
#define FCMD_ERASE_SECTOR 0x09		// Hex command to erase sector

///* Union to efficiently access hi and lo parts of a "phrase" */
//typedef union
//{
//  uint64_t l;
//  struct
//  {
//    uint32_t Lo;
//    uint32_t Hi;
//  } s;
//} uint64union_t;

/* Union to efficiently access byte data in "phrase" */
typedef union
{
  uint64_t phrase;
  uint8_t byte[8];
} phrase;

/* Union to efficiently access address byte data in "address" */
typedef union
{
  uint32_t l;
  struct
  {
    uint8_t address3;
    uint8_t address2;
    uint8_t address1;
    uint8_t address0;
  } s;
} address;

/* Structure of a Flash Command */
typedef struct
{
  uint8_t command;
  address address;
  phrase data;
} TFCCOB;

static bool AvailableSpace[8];

static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase);
static bool WritePhrase(const uint32_t address, const uint64union_t phrase);
static bool EraseSector(uint32_t address);
static bool LaunchCommand(TFCCOB* commonCommandObject);

bool Flash_Init(void)
{
  static uint8_t i;
  for (i = 0; i < sizeof(AvailableSpace); i++)
    AvailableSpace[i] = true;
  return true;
}

bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  static bool spaceFound;
  spaceFound = false;

  /* Loop through each address within start and end to find space */
  static uint8_t i;
  for (i = 0; i < sizeof(AvailableSpace); i += size)
  {
    switch (size)
    {
      case 1:
	/* Space Found if data at the address is blank.(All bits in space set) */
	if (AvailableSpace[i])
	{
	  spaceFound = true;
	  AvailableSpace[i] = false;
	}
	break;
      case 2:
	/* Space Found if data at the address is blank.(All bits in space set) */
	if (AvailableSpace[i])
	  if (AvailableSpace[i+1])
	  {
	    spaceFound = true;
	    AvailableSpace[i] = false;
	    AvailableSpace[i+1] = false;
	  }
	break;
      case 4:
	/* Space Found if data at the address is blank.(All bits in space set) */
	if (AvailableSpace[i])
	  if (AvailableSpace[i+1])
	    if (AvailableSpace[i+2])
	      if (AvailableSpace[i+3])
	      {
		spaceFound = true;
		AvailableSpace[i] = false;
		AvailableSpace[i+1] = false;
		AvailableSpace[i+2] = false;
		AvailableSpace[i+3] = false;
	      }
	break;
    }
    if (spaceFound)
    {
      /* allocate address of space to variable address pointer and return space found  */
	*variable = (void *)(FLASH_DATA_START + i);
      return spaceFound;
    }
  }
  return spaceFound;
}

bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  /* Check if address of word is divisible by 4 (word aligned) */
  if ((uint32_t)address % 4 != 0)
    return false;

  /* Initiate phrase and new address to be passed onto modify phrase */
  static uint64union_t phrase;
  static uint32_t *newAddress;

  /* If position of the address is in the hi or lo part of the phrase */
  if ((uint32_t)address % 8 == 0)
  {
    phrase.s.Lo = data;
    phrase.s.Hi = *(address + 1);
    newAddress = (uint32_t *)address;
  }
  else
  {
    phrase.s.Lo = *(address - 1);
    phrase.s.Hi = data;
    newAddress = (uint32_t *)(address - 1);
  }
  return ModifyPhrase((uint32_t)newAddress, phrase);
}

bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  /* Check if address of half-word is an even number */
  if ((uint32_t)address % 2 != 0)
    return false;

  /* Initiate word and new address to be passed onto Flash_Write32() */
  static uint32union_t word;
  static uint32_t *newAddress;

  /* If position of the address is in the hi or lo part of the word */
  if ((uint32_t)address % 4 == 0)
  {
    word.s.Lo = data;
    word.s.Hi = *(address + 1);
    newAddress = (uint32_t *)address;
  }
  else
  {
    word.s.Lo = *(address - 1);
    word.s.Hi = data;
    newAddress = (uint32_t *)(address - 1);
  }
  return Flash_Write32(newAddress, word.l);
}

bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  /* Initiate halfWord and new address to be passed onto Flash_Write16() */
  static uint16union_t halfWord;
  static uint16_t *newAddress;

  /* If position of the address is in the hi or lo part of the halfWord */
  if ((uint32_t)address % 2 == 0)
  {
    halfWord.s.Lo = data;
    halfWord.s.Hi = *(address + 1);
    newAddress = (uint16_t *)address;
  }
  else
  {
    halfWord.s.Lo = *(address - 1);
    halfWord.s.Hi = data;
    newAddress = (uint16_t *)(address - 1);
  }
  return Flash_Write16(newAddress, halfWord.l);
}

bool Flash_Erase(void)
{
  /* Erase sector starting from  */
  return EraseSector(FLASH_DATA_START);
}

/*! @brief Loads the common command (with all parameters) into the FCCOB register block,
 *         then executes it.
 *
 *  @param commonCommandObject an object that contains the command and all the parameters.
 *  @return BOOL - TRUE if the command was executed successfully.
 */
static bool LaunchCommand(TFCCOB* commonCommandObject)
{
  /* Check if the address is 8byte(64 bit) aligned */
  if (commonCommandObject->address.l % 8 != 0)
    return false;

  /* Wait until previous command is finished */
  while (!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK));

  /* Clear any error flags */
  FTFE_FSTAT = FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_FPVIOL_MASK | FTFE_FSTAT_RDCOLERR_MASK;

  /* Load the command into FCCOB registers */
  FTFE_FCCOB0 = commonCommandObject->command;

  /* Load the address into FCCOB registers */
  FTFE_FCCOB1 = commonCommandObject->address.s.address1;
  FTFE_FCCOB2 = commonCommandObject->address.s.address2;
  FTFE_FCCOB3 = commonCommandObject->address.s.address3;

  /* Load the data into FCCOB registers. */
  FTFE_FCCOB4 = commonCommandObject->data.byte[3];
  FTFE_FCCOB5 = commonCommandObject->data.byte[2];
  FTFE_FCCOB6 = commonCommandObject->data.byte[1];
  FTFE_FCCOB7 = commonCommandObject->data.byte[0];
  FTFE_FCCOB8 = commonCommandObject->data.byte[7];
  FTFE_FCCOB9 = commonCommandObject->data.byte[6];
  FTFE_FCCOBA = commonCommandObject->data.byte[5];
  FTFE_FCCOBB = commonCommandObject->data.byte[4];

  /* Set the CIFF flag to launch the command */
  FTFE_FSTAT |= FTFE_FSTAT_CCIF_MASK;

  /* Wait until command is finished */
  while (!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK));

  /* Check if any error flags are set within the FSTAT register */
  bool errorFlag = false;
  errorFlag = FTFE_FSTAT & FTFE_FSTAT_ACCERR_MASK;	// Access Error Flag
  errorFlag |= FTFE_FSTAT & FTFE_FSTAT_FPVIOL_MASK;	// Flash Protection Violation Flag
  errorFlag |= FTFE_FSTAT & FTFE_FSTAT_MGSTAT0_MASK;	// Memory Controller Command Completion Status Flag

  /* If an error has occurred, return false */
  return (!errorFlag);
}

/*! @brief Writes the phrase to the Flash memory at the specified address.
 *
 *  @param address The address to be written to.
 *  @param phrase The data that is to be written.
 *  @return BOOL - TRUE if the phrase was written successfully to Flash.
 *  @note address must be 64 aligned.
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  /* Create Flash Command for write phrase */
  static TFCCOB flashCommand;
  flashCommand.command = FCMD_PROGRAM_PHRASE;
  flashCommand.address.l = address;
  flashCommand.data.phrase = phrase.l;
  return LaunchCommand(&flashCommand);
}

/*! @brief Erases the specified sector of Flash memory.
 *
 *  @param address The start address of the sector to be erased.
 *  @return BOOL - TRUE if the sector was erased successfully.
 *  @note address must be 64 aligned.
 */
static bool EraseSector(const uint32_t address)
{
  /* Create Flash Command for erase sector */
  static TFCCOB flashCommand;
  flashCommand.command = FCMD_ERASE_SECTOR;
  flashCommand.address.l = address;
  return LaunchCommand(&flashCommand);
}

/*! @brief Modifies the phrase of Flash memory.
 *
 *  @param address The address of the phrase to be modified.
 *  @param phrase The data that is to be written to the phrase.
 *  @return BOOL - TRUE if the phrase was modified successfully in Flash.
 *  @note address must be 64 aligned.
 */
static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase)
{
  if (EraseSector(address))		// Erases Sector
    if (WritePhrase(address, phrase))	// Writes new phrase
      return true;
  return false;
}

/*!
 * @}
*/
