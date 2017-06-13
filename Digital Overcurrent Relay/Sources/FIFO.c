/*! @file FIFO.c
 *  @brief Contains the functions for accessing a byte-wide FIFO.
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-06-06
 */

/*!
 *  @addtogroup fifo_module FIFO module documentation
 *  @{
 */

#include "FIFO.h"
#include "PE_Types.h"
#include "Cpu.h"
#include "OS.h"

static OS_ECB* PutSemaphore;
static OS_ECB* GetSemaphore;

/*! @brief Initialize the FIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return void
 */
void FIFO_Init(TFIFO * const FIFO)
{
  EnterCritical();
  FIFO->Start = 0;
  FIFO->End = 0;
  FIFO->NbBytes = 0;
  PutSemaphore = OS_SemaphoreCreate(1);
  GetSemaphore = OS_SemaphoreCreate(1);
  ExitCritical();
}

/*! @brief Put one character into the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return bool - TRUE if data is successfully stored in the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
  OS_SemaphoreWait(PutSemaphore,0);
  EnterCritical();
  // Returns false if array is full
  if (FIFO->NbBytes == FIFO_SIZE)
  {
    OS_SemaphoreSignal(PutSemaphore);
    ExitCritical();
    return false;
  }

  // Puts data into buffer and increments counter
  FIFO->Buffer[FIFO->Start] = data;
  FIFO->NbBytes++;
  FIFO->Start++;
  // Resets counter when data saved is the last one on the array
  if (FIFO->Start == FIFO_SIZE)
    FIFO->Start = 0;
  OS_SemaphoreSignal(PutSemaphore);
  ExitCritical();
  return true;
}

/*! @brief Get one character from the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct with data to be retrieved.
 *  @param dataPtr A pointer to a memory location to place the retrieved byte.
 *  @return bool - TRUE if data is successfully retrieved from the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr)
{
  OS_SemaphoreWait(GetSemaphore,0);
  EnterCritical();
  // Returns false if array is empty
  if (FIFO->NbBytes == 0)
  {
    OS_SemaphoreSignal(GetSemaphore);
    ExitCritical();
    return false;
  }

  // Retrieves data from buffer and decrements counter
  *dataPtr = FIFO->Buffer[FIFO->End];
  FIFO->NbBytes--;
  FIFO->End++;
  // Resets counter when data retrieved is the last one on the array
  if (FIFO->End == FIFO_SIZE)
    FIFO->End = 0;
  OS_SemaphoreSignal(GetSemaphore);
  ExitCritical();
  return true;
}

/*!
 * @}
*/
