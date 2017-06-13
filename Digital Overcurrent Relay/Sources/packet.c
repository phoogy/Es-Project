/*! @file packet.c
 *  @brief Contains the functions for handling 5-byte packets
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-05-17
 */

/*!
 *  @addtogroup packet_module Packet module documentation
 *  @{
 */

#include "packet.h"
#include "UART.h"

static uint8_t NumBytes;
TPacket Packet;

bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  /* Initialise Packet Variables*/
  NumBytes = 0;
  Packet_Command = 0;
  Packet_Parameter1 = 0;
  Packet_Parameter2 = 0;
  Packet_Parameter3 = 0;
  Packet_Checksum = 0;

  /* Initialise UART */
  return UART_Init(baudRate, moduleClk);
}

bool Packet_Get(void)
{
  // Receives packet and returns true if successful
  while (UART_InChar(&Packet_Checksum))
  {
    NumBytes++;

    // Reset counter if valid packet is received
    if (NumBytes == 5)
    {
      if (Packet_Checksum == (Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3))
      {
	NumBytes = 0;
	Packet_Checksum = 0;
	return true;
      }
      else
	NumBytes--;
    }
    // Shifts the bytes forward if any one fails to transmit
    Packet_Command = Packet_Parameter1;
    Packet_Parameter1 = Packet_Parameter2;
    Packet_Parameter2 = Packet_Parameter3;
    Packet_Parameter3 = Packet_Checksum;
  }
  return false;
}

bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  // Puts each byte into the FIFO and returns true if all is successful
  return (UART_OutChar(command) && UART_OutChar(parameter1) && UART_OutChar(parameter2) && UART_OutChar(parameter3) && UART_OutChar(command ^ parameter1 ^ parameter2 ^ parameter3));
}

/*!
 * @}
*/
