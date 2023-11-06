#include "BGT60LTR11_PlatformSpecific.h"

#include "BGT60LTR11.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cyhal.h"

static cy_stc_scb_spi_context_t spi;

// This is not the real name of this function. It is macro which contains hardened SCB number.
void BGT60LTR11_SPI_IRQHandler(void) {
    Cy_SCB_SPI_Interrupt(BGT60LTR11_SPI_HW, &spi);
}

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_Init() {
    cy_en_scb_spi_status_t cyStatus;

    cyStatus = Cy_SCB_SPI_Init(BGT60LTR11_SPI_HW, &BGT60LTR11_SPI_CONFIG, &spi);
    if (cyStatus != CY_SCB_SPI_SUCCESS) {
        return BGT60LTR11_Status_SpiError;
    }

    Cy_SCB_SPI_Enable(BGT60LTR11_SPI_HW);
	
    NVIC_ClearPendingIRQ(BGT60LTR11_SPI_IRQn);
    NVIC_EnableIRQ(BGT60LTR11_SPI_IRQn);

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_Deinit() {
    Cy_SCB_SPI_DeInit(BGT60LTR11_SPI_HW);
    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_TransmitReceive(BGT60LTR11_Device* dev, uint8_t* txBuffer, uint8_t* rxBuffer, size_t bufferSize) {
    cy_en_scb_spi_status_t cyStatus;

    Cy_SCB_SPI_SetActiveSlaveSelect(BGT60LTR11_SPI_HW, dev->slaveSelect);

    cyStatus = Cy_SCB_SPI_Transfer(BGT60LTR11_SPI_HW, txBuffer, rxBuffer, bufferSize, &spi);
    if (cyStatus != CY_SCB_SPI_SUCCESS) {
        return BGT60LTR11_Status_SpiError;
    }

    while (Cy_SCB_SPI_GetTransferStatus(BGT60LTR11_SPI_HW, &spi) & CY_SCB_SPI_TRANSFER_ACTIVE) {
    }

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_PlatformSpecific_DelayUs(uint16_t us) {
    Cy_SysLib_DelayUs(us);
    return BGT60LTR11_Status_Ok;
}
