# STM32 Based Control
This folder contains the necessary information to utilize the DAMIAO development board to control OpenArm with CANFD 

## Prerequisites
- DAMIAO development board - [Purchase Link](https://item.taobao.com/item.htm?id=814954787248&spm=a21xtw.29178619.0.0&pisk=gUXZcrMRiRewEYT9s99qat0Xnwv9CKz5StTXmijDfFYM5t_Vmn-woEAMmX0HVeg65I_X06SRuFZY6d1euwbeSZSYXnYcoZ8f1Ww5XGpvnz_4F8stKACrLN-MsBqDVnxgACVX_C-ynza7FXiH6P9c54T5JDcH2exMmIcgYBxkcKxMsAqe-n--iVbcokuH03xMmAYgKHYpjhDmshxn-hKJmxDcoMqeDeYDnCbcxbWcoNHe8Cqftsomp7TH_UjM8Y7RLhYZ6G8EgjXF2C8kFekmn9-NA4bgLYPXr_L6NUQ3dAJVxn7eN1zrQZSc2sAF3Vq1ribOLdpzGldPt61W_6zi3e660QvDTmDcYKfMitJuE2-dtG1cBNoqnhB1FI8JToDvGpXWZ_bZDuIHInbvwta-BFjc2TCWURuJb6jyLgWokHAJ-tCZojRMvH87YkSOM71-TgZMRjhvTWKePuE-ij0f9xGNnecxMBWwYUZg4)
- [Keil MDK](https://store.arm.com/mdk-6/) - set of tools that are required to compile and debug the DAMIAO board
- OpenArm tested and running (with basic CAN control)

## Getting Started

### Clone OpenArm Repository
```bash
git clone git@github.com:reazon-research/OpenArm.git
```
### Keil
After installing Keil, follow the [Getting Started Guide](https://www.keil.com/support/man/docs/mdk_gs/) for the Keil MDK.

#### Install STM32H723 Software Packs
During the Keil installation, the pack installer will open up. Inside, you will be able to install packages for different devices and boards. Filter the devices on the left hand side to narrow the packages down to STM32H723. Ensure that all the ARM::CMSIS packages are installed.

### Using uVision IDE

**Opening Project:** Open the STM32H7 uVision project file located inside MDK-ARM

**Running and Debugging Program:** Select Rebuild (rebuilds all target files) and Load (Downloads Code to Flash Memory). Similarly, debugging can be done after Rebuild, by pressing Start/Stop Debug Session in the top menu bar.

## Motor Control Architecture Overview
TODO

## Resources
- [General Keil Debugging Guide](https://developer.arm.com/documentation/101407/0542/Debugging)
- [Event Recorder User Guide](https://developer.arm.com/documentation/101407/latest/Debugging/Debug-Windows-and-Dialogs/Event-Recorder)


