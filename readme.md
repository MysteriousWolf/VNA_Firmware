# VNA_Firmware

This repository contains the STM32 firmware for the open-source 8-16 GHz VNA (Vector Network Analyzer) project. Designed for the STM32 H563VITx microcontroller, the firmware manages communication and configuration for key components in the system. It handles USB/serial + FTDI communication and the SPI configuration of the ADS4222 ADC, which is mounted on the same board as the Lattice iCE40UP5K-SG48I FPGA. Additionally, the firmware configures the FPGA's filters and data acquisition settings to ensure proper synchronization with the ADC, which is clocked by the FPGA.

The firmware also manages the SPI control of an external ST's STUW81300 PLL, which is mounted on a separate board connected to an analog 60 MHz oscillator based on a quartz crystal.

___

## The open-source 8-16 GHz VNA project  
This firmware is part of the open-source 8-16 GHz VNA project (my master's thesis). You can find all associated resources in the following repositories:
- Hardware repository - TBD
- [Firmware repository](https://github.com/MysteriousWolf/VNA_Firmware) (this one)
- [FPGA Config repository](https://github.com/MysteriousWolf/VNA_FPGA_DSP)
