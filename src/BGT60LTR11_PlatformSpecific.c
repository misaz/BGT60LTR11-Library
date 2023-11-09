/* SPDX-License-Identifier: MIT */

#include "BGT60LTR11_PlatformSpecific.h"

#include "BGT60LTR11.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cyhal.h"

static cy_stc_scb_spi_context_t spi;

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_Init() {
    // Initialize SPI Driver here. Use CPHA=0 and CPOL=1 mode. 
    // Do not issue any transaction to the device here.
    // Return BGT60LTR11_Status_Ok or BGT60LTR11_Status_SpiError

    return BGT60LTR11_Status_NotImplemented;
}

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_Deinit() {
    // Deinitialize SPI Driver here.
    // Return BGT60LTR11_Status_Ok or BGT60LTR11_Status_SpiError

    return BGT60LTR11_Status_NotImplemented;
}

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_TransmitReceive(BGT60LTR11_Device* dev, uint8_t* txBuffer, uint8_t* rxBuffer, size_t bufferSize) {
    // 1.) drive Chip Select pin low. Use dev->slaveSelect for determining which pin you should drive.
    // 2.) Isues transaction on the SPI bus. Transmit data from txBuffer while receive data to
    //     the rxBuffer at the same time. Both buffers has size of bufferSize bytes.
    // 3.) drive Chip Select pin high.
    // 4.) Return BGT60LTR11_Status_Ok if everything was OK. Otherwise return BGT60LTR11_Status_SpiError

    return BGT60LTR11_Status_NotImplemented;
}

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_DelayUs(uint16_t us) {
    // Delay execution by busy waiting for specified amount of time (value is in microseconds unit)
    // You can wait more time, but you should never wait less than specified amount of time
    // Return BGT60LTR11_Status_Ok if everything was OK. Otherwise return BGT60LTR11_Status_SpiError.

    return BGT60LTR11_Status_NotImplemented;
}
