/* SPDX-License-Identifier: MIT */

#ifndef BGT60LTR11_PLATFORMSPECIFIC_H_
#define BGT60LTR11_PLATFORMSPECIFIC_H_

#include "BGT60LTR11.h"

#include <stdint.h>
#include <stddef.h>

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_Init();
BGT60LTR11_Status BGT60LTR11_PlatformSpecific_Deinit();
BGT60LTR11_Status BGT60LTR11_PlatformSpecific_TransmitReceive(BGT60LTR11_Device* dev, uint8_t* txBuffer, uint8_t* rxBuffer, size_t bufferSize);
BGT60LTR11_Status BGT60LTR11_PlatformSpecific_DelayUs(uint16_t us);

#endif
