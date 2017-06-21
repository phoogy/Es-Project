/*! @file
 *
 *  @brief Global Semaphores used for RTOS.
 *
 *  This contains the Semaphores used in the implementation of the RTOS.
 *
 *  @author Phong 10692820 & Emily 12016681
 *  @date 2017-05-17
 */

#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include "OS.h"

extern OS_ECB* PacketReady;
extern OS_ECB* RTCReady;
//extern OS_ECB* PITReady;
extern OS_ECB* FTMReady;
extern OS_ECB* ACCELIntReady;
extern OS_ECB* I2CReady;
extern OS_ECB* ACCELPollReady;

#endif
