# STM32 Based Control
This folder contains the necessary information to utilize and build on the DAMIAO development board to control OpenArm with CANFD. 

**NOTE:** Currently, all development of the STM32 based motor control is done using the Keil MDK. Support for STM32Cube is not directly supported however it should be possible by copy over application code and generate a STM32CubeIDE project.

## Prerequisites
- DAMIAO development board - [Purchase Link](https://item.taobao.com/item.htm?id=814954787248&spm=a21xtw.29178619.0.0&pisk=gUXZcrMRiRewEYT9s99qat0Xnwv9CKz5StTXmijDfFYM5t_Vmn-woEAMmX0HVeg65I_X06SRuFZY6d1euwbeSZSYXnYcoZ8f1Ww5XGpvnz_4F8stKACrLN-MsBqDVnxgACVX_C-ynza7FXiH6P9c54T5JDcH2exMmIcgYBxkcKxMsAqe-n--iVbcokuH03xMmAYgKHYpjhDmshxn-hKJmxDcoMqeDeYDnCbcxbWcoNHe8Cqftsomp7TH_UjM8Y7RLhYZ6G8EgjXF2C8kFekmn9-NA4bgLYPXr_L6NUQ3dAJVxn7eN1zrQZSc2sAF3Vq1ribOLdpzGldPt61W_6zi3e660QvDTmDcYKfMitJuE2-dtG1cBNoqnhB1FI8JToDvGpXWZ_bZDuIHInbvwta-BFjc2TCWURuJb6jyLgWokHAJ-tCZojRMvH87YkSOM71-TgZMRjhvTWKePuE-ij0f9xGNnecxMBWwYUZg4)
- [Keil MDK](https://store.arm.com/mdk-6/) - set of tools that are required to compile and debug the DAMIAO board
- OpenArm tested and running (with basic CAN control)

## Getting Started

### Clone OpenArm Repository
```bash
git clone --recurse-submodules https://github.com/reazon-research/OpenArm.git
```

To build, you need to initialize and update submodules. By cloning the repository with `--recurse-submodules` option as shown above, all submodules will be automatically initialized and updated. 

Alternatively, you can initialize and update submodules after cloning, as shown below.

```bash
git submodule update --init --recursive
```

### Keil
After installing Keil, follow the [Getting Started Guide](https://www.keil.com/support/man/docs/mdk_gs/) for the Keil MDK.

#### **Install STM32H723 Software Packs**
During the Keil installation, the pack installer will open up. Inside, you will be able to install packages for different devices and boards. Filter the devices on the left hand side to narrow the packages down to STM32H723. Ensure that all the ARM::CMSIS packages are installed.

#### **First Build**

- **Opening Project:** Keil uVision should be installed on the PC. Open it and select Open under the File header in the Menu bar. Navigate to the MDK-ARM folder in the STM32 Motor Control repository and select the STM32H7.uvprojx file.
- **Running and Debugging Program:** Select Rebuild (rebuilds all target files) and Load (Downloads Code to Flash Memory). Similarly, debugging can be done after Rebuild, by pressing Start/Stop Debug Session in the top menu bar.

## Keil Specific Tips

### Event Recorder
Currently I am using the Event Recorder to debug STM32 code through Keil uVision for debugging purposes. Event Recorder is part of the Keil::ARM_Compiler pack and is part of the installation. It provides an API for event annotations in the application code. To setup the Event Recorder, follow the Event Recorder User Guide.

You can visualize event logs by [plotter.py](plotter.py).

#### Common Issues with Event Recorder Setup
- If you have an issue with `#include CMSIS_device_header`, it is most likely because `EventRecorder.c` does not have a path to pull for CMSIS_device_header. As a workaround, replace `#include CMSIS_device_header` with `#include "stm32h723xx.h"`. You may have to disable read-only permissions for `EventRecorder.c`

## General Resources
- [General Keil Debugging Guide](https://developer.arm.com/documentation/101407/0542/Debugging)
- [Event Recorder User Guide](https://developer.arm.com/documentation/101407/latest/Debugging/Debug-Windows-and-Dialogs/Event-Recorder)