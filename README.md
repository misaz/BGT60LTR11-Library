# BGT60LTR11 60-GHz Radar Sensor C Library

This project contains library for controlling BGT60LTR11AIP, BGT60LTR11BAIP, and BGT60LTR11SAIP 60-GHz radar motion digital sensor from Infineon. Library is written in C and is designed as multiplatform.

Library was originally developed as part of [Hackster Connect things with code! with Infineon Competition](https://www.hackster.io/contests/connectthingswithcode).

## Features
- Written in C. Can be used in both C and C++ projects.
- Minimal RAM usage. No dynamic memory allocations.
- Library core depends on stdint.h (for uint8_t, uint16_t, int16_t and uint32_t types) and stddef- (for size_t type).
- Platform specific ports depends on vendor SDK (see section Supported platfroms and porting below).
- Support for configuring sensor in SPI mode
- Support for configuring 54 sensor parameters. For example (but not limited to):
	- Radio Signal Frequency
	- Detection threshold (sensitivity)
	- Hold time
	- Polarity of output signals
	- Idle state of output signal
	- PLL, Timing, Baseband and Bite parameters
	- SPI Mode and mapping internal singlas to SPI wires for debugging
- Support for reading RAW ADC Data
- Support for switching sensor mode
- Support for soft reseting sensor
- Support for reading chip version
- Support for reading state of inputs used in autonomous mode
- Tested with Infineon BGT60LTR11 S2GO Module

Library handles SPI interface of the sensor. Library do not handle TDet and PDet singnals. You are supposed to manage them using platform specific GPIO driver directly.

## Supporter Platforms

Currently Library is ported to following platforms. Library can be easily ported to any other platform by implementing 4 simple functions in `BGT60LTR11_PlatformSpecific.c` file.

Currently is library ported to following platforms:

- Infineon (Cypress) PSoC 6 Microcontroller running PDL driver.

## Getting started
1. Go to Release page and download ZIP file targetting your platform or ZIP targeting Generic platform if your platform is not supported.
1. Unzip downloaded file and copy all files to folder of your project or IDE.
1. (If used Generic variant) implement 4 functions in `BGT60LTR11_PlatformSpecific.c` according to descriptions in comments in these functions.
1. Add `#include "BGT60LTR11.h"` at the begining of your source code file.
1. Allocate `BGT60LTR11_Device` structure. It must live for whole time when sensor is used.
1. Call `BGT60LTR11_Init()` function at the begining of your program. Pass pointer to previously allocated structure and platform-specification identification of Chip Select pin.
1. Use functions starting with `BGT60LTR11_` previx as needed.

## Functions
Library contains following functions (you can find this listing in `BGT60LTR11.h` file):

```
BGT60LTR11_Status BGT60LTR11_Init(BGT60LTR11_Device* dev, uint32_t slaveSelect);
BGT60LTR11_Status BGT60LTR11_Deinit(BGT60LTR11_Device* dev);

BGT60LTR11_Status BGT60LTR11_SoftReset(BGT60LTR11_Device* dev);

BGT60LTR11_Status BGT60LTR11_LoadDefaultConfiguration(BGT60LTR11_Configuration* config);
BGT60LTR11_Status BGT60LTR11_SetConfiguration(BGT60LTR11_Device* dev, BGT60LTR11_Configuration* config);

BGT60LTR11_Status BGT60LTR11_SwitchToAutonomousPulsedMode(BGT60LTR11_Device* dev);
BGT60LTR11_Status BGT60LTR11_SwitchToAutonomousCWMode(BGT60LTR11_Device* dev);

BGT60LTR11_Status BGT60LTR11_GetQuadStateInput(BGT60LTR11_Device* dev, int inputNumber, BGT60LTR11_QuadState* state);
BGT60LTR11_Status BGT60LTR11_GetChipVersion(BGT60LTR11_Device* dev, uint8_t* chipVersion);

BGT60LTR11_Status BGT60LTR11_GetAdcData(BGT60LTR11_Device* dev, BGT60LTR11_AdcChannel adcChannel, uint16_t* data);
BGT60LTR11_Status BGT60LTR11_GetAdcDataMultiple(BGT60LTR11_Device* dev, BGT60LTR11_AdcChannel adcChannelStart, int adcChannelsCount, uint16_t* data);
```

## Examples

Following example configure radar in default mode and turn it on. Example use default configuration. It is recommended to use `BGT60LTR11_LoadDefaultConfiguration()` as a base for configuration because configuring all 54 parameters form scratch is quite complicated. After `BGT60LTR11_LoadDefaultConfiguration()` call you can update only desired parameters while remaining other in default state.

If you want to reconfigure sensor, you should reset it first. Communication on SPI bus collides with communication of internal "main controller" and chip User Guide from Infineon explicitly recommends do not reconfiguring sensor when it is running. Recommended step reconfiguring sensor at runtime is doing exactly the same steps as initial configuration, except calling `BGT60LTR11_Init` which basically just prepares SPI bus driver.

```
#include "BGT60LTR11.h"

int main(void) {
	BGT60LTR11_Device radar;
	BGT60LTR11_Status bStatus;
	
	bStatus = BGT60LTR11_Init(&radar, CY_SCB_SPI_SLAVE_SELECT0);
	// Check bStatus for errors

	bStatus = BGT60LTR11_SoftReset(&radar);
	// Check bStatus for errors

	BGT60LTR11_Configuration config;
	BGT60LTR11_LoadDefaultConfiguration(&config);
	
	// Update configuration here.

	bStatus = BGT60LTR11_SetConfiguration(&radar, &config);
	// Check bStatus for errors
	
	bStatus = BGT60LTR11_SwitchToAutonomousPulsedMode(&radar);
	// Check bStatus for errors

	while (1) {
		// process TDet and PDet signals
	}
}
```

