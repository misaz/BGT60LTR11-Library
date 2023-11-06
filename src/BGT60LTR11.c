#include "BGT60LTR11.h"

#include <stdint.h>
#include <string.h>

#include "BGT60LTR11_PlatformSpecific.h"

static int devicesCount = 0;

BGT60LTR11_Status BGT60LTR11_WriteReg(BGT60LTR11_Device* dev, uint8_t address, uint16_t data) {
    BGT60LTR11_Status status;

    uint8_t txBuffer[3] = {(address << 1) | 0x01, (data & 0xFF00) >> 8, data & 0xFF};
    uint8_t rxBuffer[3] = {0, 0, 0};

    status = BGT60LTR11_PlatformSpecific_TransmitReceive(dev, txBuffer, rxBuffer, sizeof(txBuffer));
    if (status) {
        return status;
    }

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_ReadReg(BGT60LTR11_Device* dev, uint8_t address, uint16_t* data) {
    BGT60LTR11_Status status;

    uint8_t txBuffer[3] = {(address << 1), 0, 0};
    uint8_t rxBuffer[3] = {0, 0, 0};

    status = BGT60LTR11_PlatformSpecific_TransmitReceive(dev, txBuffer, rxBuffer, sizeof(txBuffer));
    if (status) {
        return status;
    }

    *data = (((uint16_t)rxBuffer[1]) << 8) | ((uint16_t)rxBuffer[2]);

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_ReadRegBurst(BGT60LTR11_Device* dev, uint8_t address, uint16_t* data, size_t dataSize) {
    BGT60LTR11_Status status;

    uint8_t txBuffer[57 * 2] = {0xFF, address << 1};
    uint8_t rxBuffer[57 * 2] = {};

    // buffer too large
    if (dataSize > sizeof(txBuffer)) {
        return BGT60LTR11_Status_BadArg;
    }

    // non 2-byte data (odd value)
    if (dataSize & 1) {
        return BGT60LTR11_Status_BadArg;
    }

    status = BGT60LTR11_PlatformSpecific_TransmitReceive(dev, txBuffer, rxBuffer, dataSize + 2);
    if (status) {
        return status;
    }

    for (size_t i = 0; i < dataSize / 2; i++) {
        uint16_t b1 = rxBuffer[(i + 1) * 2];
        uint16_t b2 = rxBuffer[(i + 1) * 2 + 1];

        data[i] = (b1 << 8) | b2;
    }

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_Init(BGT60LTR11_Device* dev, uint32_t slaveSelect) {
    BGT60LTR11_Status status;

    if (devicesCount == 0) {
        status = BGT60LTR11_PlatformSpecific_Init();
        if (status) {
            return status;
        }
    }

    dev->slaveSelect = slaveSelect;
	dev->isAutonomous = 0;

	status = BGT60LTR11_SoftReset(dev);
	if (status) {
		return status;
	}

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_Deinit(BGT60LTR11_Device* dev) {
    BGT60LTR11_Status status;

    if (--devicesCount == 0) {
        status = BGT60LTR11_PlatformSpecific_Deinit();
        if (status) {
            return status;
        }
    }

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_SoftReset(BGT60LTR11_Device* dev) {
    BGT60LTR11_Status status;

    uint16_t val = BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_SOFT_RESET_FIELD, 1);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG15_DIGITAL_CTRL, val);
	if (status) {
		return status;
	}

	status = BGT60LTR11_PlatformSpecific_DelayUs(2000);
	if (status) {
		return status;
	}

	return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_LoadDefaultConfiguration(BGT60LTR11_Configuration* config) {
    config->detector.threshold = 400;
    config->detector.thresholdOffset = 0;
    config->detector.holdTime = 16;
    config->detector.holdMultipliedBy32 = 0;
    config->detector.directionInactiveLevel = BGT60LTR11_DirectionInactiveLevel_PDetHigh;
    config->detector.directionPolarity = BGT60LTR11_DirectionPolarity_PDetLowWhenDeparting;
    config->detector.motionPolarity = BGT60LTR11_MotionPolarity_TDetIsLowWhenActive;
    config->detector.disableDirectionHystheresis = 0;
    config->detector.dectedDirectionOnlyOnMotion = 0;
    config->detector.swapIq = 0;
    config->detector.monitorRadarPulse = 0;
    config->detector.directionPhaseThreshold = BGT60LTR11_DirectionPhaseThreshold_NoChange;

    config->pll.frequency = BGT60LTR11_Frequency_61_1_GHz;
    config->pll.disablePllBias = 0;
    config->pll.loopfilterIsolationMode = 1;
    config->pll.loopfilterR4 = BGT60LTR11_LoopfilterR4_0_1_kOhm;
    config->pll.loopfilterR2 = BGT60LTR11_LoopfilterR2_18_7_kOhm;
    config->pll.closedLoopInPulsedMode = BGT60LTR11_ClosedLoopInPulsedMode_ClosedLoop;
    config->pll.xtalOscilatorMode = BGT60LTR11_XtalOscilatorMode_Amplitude1;
    config->pll.feedbackDivderCounter = BGT60LTR11_FeedbackDividerCounter_Cnt_21_Freq_38_4_Mhz;
    config->pll.chargePumpCurrent = BGT60LTR11_ChargePumpCurrent_55_uA;
    config->pll.chargePumpBiasMode = BGT60LTR11_ChargePumpBiasMode_BiasRegulationMode;
    config->pll.pfdResetDelay = BGT60LTR11_PfdResetDelay_375_ps;
    config->pll.enableLockDetection = 1;
    config->pll.lockDetectionTimeWindow = BGT60LTR11_LockDetectionTimeWindow_1_5_ns;
    config->pll.lockDetectionTiming = BGT60LTR11_LockDetectionTiming_LockTime_24_Cycles_Delay_3_57_us;

    config->timing.enableHighPulseRepetitionTime = 0;
    config->timing.enableAdaptivePulseRepetitionTime = 0;
    config->timing.adaptivePulseRepetitionTimeMultiplier = BGT60LTR11_AdaptivePulseRepetitionTimeMultiplier_4;
    config->timing.dutyCycleRepetitionRate = BGT60LTR11_DutyCycleRepetitionRate_500_us;
    config->timing.dutyCyclePulseOnLength = BGT60LTR11_DutyCyclePulseOnLength_5_us;
    config->timing.vcoToPllDelay = BGT60LTR11_VcoToPllDelay_1000_ns;
    config->timing.mpaSampleHoldDelay = BGT60LTR11_MpaSampleHoldDelay_1000_ns;
    config->timing.enablePd = 0;
    config->timing.mediumPowerAmplifierGainControl = BGT60LTR11_MediumPowerAmplifierGainControl_Plus_4_5_dBm;

    config->externalClock.enableDividerOutput = 0;
    config->externalClock.enableDividerTestMode = 0;
    config->externalClock.divider = BGT60LTR11_ExternalClockOutputFrequencyDivider_InternalClock;

    config->baseband.highPassFilterResistor = BGT60LTR11_HighPassFilterResistor_4_MOhm;
    config->baseband.clockChopFrequency = BGT60LTR11_ClockChopFrequency_192_kHz;
    config->baseband.lowPassFilter = BGT60LTR11_LowPassFilter_10_kHz;
    config->baseband.pga = BGT60LTR11_BasebandPga_50_dB;
    config->baseband.enableQs4Output = 0;
    config->baseband.qs4Output = BGT60LTR11_Qs4Output_BasebandBandgap;

    config->bite.enable = 0;
    config->bite.enablePowerDetector = 0;
    config->bite.phase = BGT60LTR11_BitePhase_0_deg;

    config->algo.phaseWindowLength = BGT60LTR11_WindowLength_256;
    config->algo.meanWindowLength = BGT60LTR11_WindowLength_256;

    config->digital.disableExternalClock = 0;
    config->digital.enableFasterPhaseEvaluation = 0;
    config->digital.spiMode = BGT60LTR11_SpiMode_CPOL1_CPHA0;
    config->digital.inactiveMiso = BGT60LTR11_InactiveMiso_HiZ;
    config->digital.spiDoOutput = BGT60LTR11_SpiDoOutput_Spi;

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_SetConfiguration(BGT60LTR11_Device* dev, BGT60LTR11_Configuration* config) {
    BGT60LTR11_Status status;

    uint16_t thrs =
        BGT60LTR11_SET_FIELD(BGT60LTR11_DETECT_THRESHOLDS_HPRT_FIELD, config->timing.enableHighPulseRepetitionTime) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DETECT_THRESHOLDS_APRT_FIELD, config->timing.enableAdaptivePulseRepetitionTime) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DETECT_THRESHOLDS_DIR_MODE_FIELD, config->detector.directionInactiveLevel) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DETECT_THRESHOLDS_THRS_FIELD, config->detector.threshold);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG2_DETECT_THRESHOLDS, thrs);
    if (status) {
        return status;
    }

    uint16_t pll1 =
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_DFT_DMUX_FIELD, config->digital.spiDoOutput) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_BIAS_DIS_FIELD, config->pll.disablePllBias) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_LF_ISO_FIELD, config->pll.loopfilterIsolationMode) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_LF_R4_SEL_FIELD, config->pll.loopfilterR4) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_CL_LOOP_PMODE_FIELD, config->pll.closedLoopInPulsedMode) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_LF_R2_SEL_FIELD, config->pll.loopfilterR2) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_XOSC_MODE_FIELD, config->pll.xtalOscilatorMode) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_FBDIV_CNT_FIELD, config->pll.feedbackDivderCounter) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_CP_ICP_SEL_FIELD, config->pll.chargePumpCurrent) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_CP_MODE_FIELD, config->pll.chargePumpBiasMode) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL1_PLL_PFD_RDT_SEL_FIELD, config->pll.pfdResetDelay);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG4_PLL1, pll1);
    if (status) {
        return status;
    }

    uint16_t pll2 =
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL2_PLL_FCW_FIELD, config->pll.frequency);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG5_PLL2, pll2);
    if (status) {
        return status;
    }

    uint16_t pll3 =
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL3_PLL_LD_TW_SEL_FIELD, config->pll.lockDetectionTimeWindow) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL3_PLL_LD_LEN_FIELD, config->pll.lockDetectionTiming) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_PLL3_PLL_LD_EN_FIELD, config->pll.enableLockDetection);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG6_PLL3, pll3);
    if (status) {
        return status;
    }

    uint16_t timing =
        BGT60LTR11_SET_FIELD(BGT60LTR11_TIMING_DC_REP_RATE_FIELD, config->timing.dutyCycleRepetitionRate) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_TIMING_DC_ON_PULSE_LEN_FIELD, config->timing.dutyCyclePulseOnLength) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_TIMING_VCO2PLL_DLY_FIELD, config->timing.vcoToPllDelay) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_TIMING_MPA2SH_DLY_FIELD, config->timing.mpaSampleHoldDelay) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_TIMING_PD_EN_FIELD, config->timing.enablePd) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_TIMING_MPA_CTRL_FIELD, config->timing.mediumPowerAmplifierGainControl);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG7_TIMING, timing);
    if (status) {
        return status;
    }

    uint16_t divider =
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIVIDER_DIV_SEL_FIELD, config->externalClock.divider) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIVIDER_DIV_OUT_EN_FIELD, config->externalClock.enableDividerOutput) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIVIDER_DIV_TESTMODE_EN_FIELD, config->externalClock.enableDividerTestMode);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG8_DIVIDER, divider);
    if (status) {
        return status;
    }

    uint16_t baseband =
        BGT60LTR11_SET_FIELD(BGT60LTR11_BASEBAND_BB_HP_RES_FIELD, config->baseband.highPassFilterResistor) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_BASEBAND_BB_CLK_CHOP_SEL_FIELD, config->baseband.clockChopFrequency) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_BASEBAND_BB_LPF_BW_FIELD, config->baseband.lowPassFilter) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_BASEBAND_BB_CTRL_GAIN_FIELD, config->baseband.pga);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG9_BASEBAND, baseband);
    if (status) {
        return status;
    }

    uint16_t holdtime =
        BGT60LTR11_SET_FIELD(BGT60LTR11_DETETCTOR_HOLDTIME_HOLD_FIELD, config->detector.holdTime);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG10_DETETCTOR_HOLDTIME, holdtime);
    if (status) {
        return status;
    }

    uint16_t bite =
        BGT60LTR11_SET_FIELD(BGT60LTR11_BITE_AMUX_BB_AMUX_CTRL_FIELD, config->baseband.qs4Output) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_BITE_AMUX_BB_AMUX_EN_FIELD, config->baseband.enableQs4Output) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_BITE_AMUX_BITE_PD_EN_FIELD, config->bite.enablePowerDetector) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_BITE_AMUX_BITE_CTRL_FIELD, config->bite.phase) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_BITE_AMUX_BITE_EN_FIELD, config->bite.enable);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG12_BITE_AMUX, bite);
    if (status) {
        return status;
    }

    uint16_t algo1 =
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO1_PHASE_WIN_LEN_FIELD, config->algo.phaseWindowLength) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO1_MEAN_WIN_LEN_FIELD, config->algo.meanWindowLength) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO1_PRT_MULT_FIELD, config->timing.adaptivePulseRepetitionTimeMultiplier);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG13_ALGO1, algo1);
    if (status) {
        return status;
    }

    uint16_t algo2 =
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_THRS_OFFSET_FIELD, config->detector.thresholdOffset) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_DIR_HYS_DIS_FIELD, config->detector.disableDirectionHystheresis) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_DIR_KEEP_FIELD, config->detector.dectedDirectionOnlyOnMotion) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_HOLD_X32_FIELD, config->detector.holdMultipliedBy32) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_SWAP_IQ_FIELD, config->detector.swapIq) |
        // BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_AUTOBLIND_DIS_FIELD, config->detector.disableBlanking) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_PULSE_MON_FIELD, config->detector.monitorRadarPulse) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_ALGO2_PHASE_THRS_FIELD, config->detector.directionPhaseThreshold);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG14_ALGO2, algo2);
    if (status) {
        return status;
    }

    uint16_t digitalControl =
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_CLK_EXT_DIS_FIELD, config->digital.disableExternalClock) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_FAST_PHASE_FIELD, config->digital.enableFasterPhaseEvaluation) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_FASTMODE_FIELD, config->digital.spiMode) |
        // BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_ADC_MON_FIELD, config->digital.) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_MISO_DRV_FIELD, config->digital.inactiveMiso) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_MOT_POL_FIELD, config->detector.motionPolarity) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_DIR_POL_FIELD, config->detector.directionPolarity);
    // BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_STAT_MUX_FIELD, config->detector.directionPolarity);
    status = BGT60LTR11_WriteReg(dev, BGT60LTR11_REG15_DIGITAL_CTRL, digitalControl);
    if (status) {
        return status;
    }

    dev->actualDetectorConfig.threshold = config->detector.threshold;
    dev->actualDetectorConfig.thresholdOffset = config->detector.thresholdOffset;
    dev->actualDetectorConfig.holdTime = config->detector.holdTime;
    dev->actualDetectorConfig.holdMultipliedBy32 = config->detector.holdMultipliedBy32;
    dev->actualDetectorConfig.directionInactiveLevel = config->detector.directionInactiveLevel;
    dev->actualDetectorConfig.directionPolarity = config->detector.directionPolarity;
    dev->actualDetectorConfig.motionPolarity = config->detector.motionPolarity;
    dev->actualDetectorConfig.disableDirectionHystheresis = config->detector.disableDirectionHystheresis;
    dev->actualDetectorConfig.dectedDirectionOnlyOnMotion = config->detector.dectedDirectionOnlyOnMotion;
    dev->actualDetectorConfig.swapIq = config->detector.swapIq;
    dev->actualDetectorConfig.monitorRadarPulse = config->detector.monitorRadarPulse;
    dev->actualDetectorConfig.directionPhaseThreshold = config->detector.directionPhaseThreshold;

    dev->actualDigitalConfig.disableExternalClock = config->digital.disableExternalClock;
    dev->actualDigitalConfig.enableFasterPhaseEvaluation = config->digital.enableFasterPhaseEvaluation;
    dev->actualDigitalConfig.spiMode = config->digital.spiMode;
    dev->actualDigitalConfig.inactiveMiso = config->digital.inactiveMiso;
    dev->actualDigitalConfig.spiDoOutput = config->digital.spiDoOutput;

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_SwitchToAutonomousPulsedMode(BGT60LTR11_Device* dev) {
    uint16_t digitalControl =
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_START_PM_FIELD, 1) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_CLK_EXT_DIS_FIELD, dev->actualDigitalConfig.disableExternalClock) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_FAST_PHASE_FIELD, dev->actualDigitalConfig.enableFasterPhaseEvaluation) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_FASTMODE_FIELD, dev->actualDigitalConfig.spiMode) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_MISO_DRV_FIELD, dev->actualDigitalConfig.inactiveMiso) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_MOT_POL_FIELD, dev->actualDetectorConfig.motionPolarity) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_DIR_POL_FIELD, dev->actualDetectorConfig.directionPolarity);

    return BGT60LTR11_WriteReg(dev, BGT60LTR11_REG15_DIGITAL_CTRL, digitalControl);
}

BGT60LTR11_Status BGT60LTR11_SwitchToAutonomousCWMode(BGT60LTR11_Device* dev) {
    uint16_t digitalControl =
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_START_PM_FIELD, 1) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_START_CW_FIELD, 1) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_CLK_EXT_DIS_FIELD, dev->actualDigitalConfig.disableExternalClock) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_FAST_PHASE_FIELD, dev->actualDigitalConfig.enableFasterPhaseEvaluation) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_FASTMODE_FIELD, dev->actualDigitalConfig.spiMode) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_MISO_DRV_FIELD, dev->actualDigitalConfig.inactiveMiso) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_MOT_POL_FIELD, dev->actualDetectorConfig.motionPolarity) |
        BGT60LTR11_SET_FIELD(BGT60LTR11_DIGITAL_CTRL_DIR_POL_FIELD, dev->actualDetectorConfig.directionPolarity);

    return BGT60LTR11_WriteReg(dev, BGT60LTR11_REG15_DIGITAL_CTRL, digitalControl);

}

BGT60LTR11_Status BGT60LTR11_GetQuadStateInput(BGT60LTR11_Device* dev, int inputNumber, BGT60LTR11_QuadState* state) {
    BGT60LTR11_Status status;
    uint16_t statusReg;

    if (inputNumber < 1 || inputNumber > 4) {
        return BGT60LTR11_Status_BadArg;
    }

    status = BGT60LTR11_ReadReg(dev, BGT60LTR11_REG56_STATUS, &statusReg);
    if (status) {
        return status;
    }

    if (inputNumber == 1) {
        *state = BGT60LTR11_GET_FIELD(BGT60LTR11_STATUS_QS1_S_FIELD, statusReg);
    } else if (inputNumber == 2) {
        *state = BGT60LTR11_GET_FIELD(BGT60LTR11_STATUS_QS2_S_FIELD, statusReg);
    } else if (inputNumber == 3) {
        *state = BGT60LTR11_GET_FIELD(BGT60LTR11_STATUS_QS3_S_FIELD, statusReg);
    } else {
        *state = BGT60LTR11_GET_FIELD(BGT60LTR11_STATUS_QS4_S_FIELD, statusReg);
    }

    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_GetChipVersion(BGT60LTR11_Device* dev, uint8_t* chipVersion) {
    BGT60LTR11_Status status;
    uint16_t statusReg;

    status = BGT60LTR11_ReadReg(dev, BGT60LTR11_REG56_STATUS, &statusReg);
    if (status) {
        return status;
    }

    *chipVersion = BGT60LTR11_GET_FIELD(BGT60LTR11_STATUS_CHIP_VERSION_FIELD, statusReg);
    return BGT60LTR11_Status_Ok;
}

BGT60LTR11_Status BGT60LTR11_GetAdcData(BGT60LTR11_Device* dev, BGT60LTR11_AdcChannel adcChannel, uint16_t* data) {
	return BGT60LTR11_GetAdcDataMultiple(dev, adcChannel, 1, data);
}

BGT60LTR11_Status BGT60LTR11_GetAdcDataMultiple(BGT60LTR11_Device* dev, BGT60LTR11_AdcChannel adcChannelStart, int adcChannelsCount, uint16_t* data) {
	return BGT60LTR11_ReadRegBurst(dev, BGT60LTR11_REG38_ADC_RESULTS_MPA + adcChannelStart, data, adcChannelsCount * 2);
}
