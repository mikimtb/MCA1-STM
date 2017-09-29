# MCA1-STM

MCA1-STM is advanced DC brushed motor controller. The controller has been made using
STM32F401RC microcontroller from ST Microelectronics. Controller supports closed loop
control of DC motor position, velocity and current. WinForm HMI interfase has been made
to simplify MCA1-STM setting up. All paramaters related to the DC motor that is connected
and controller are stored in MCU flash memory and can be configured.

## Getting Started

Operating voltage: 				up to 60VDC
Operating current: 				up to 15A 
Communication interface: 		RS485 Full duplex, SPI
Motion feedback sensor:			Optical incremental encoder, Magnetic encoder
Current feedback:				LEM current transducer
Digital inputs:					5 optoisolated configurable inputs
Digital outputs:				5 optoisolated configurable outputs
PID loops:						Position, Velocity, Current
Configuration:					Via Windows Form application

### Prerequisites

To build and download the firmware STM System Workbench is needed. Firmware image can be
downloaded to target MCU using STLink v2 programmer debuger.


### Installing

Installation of STM Workbench Studio can be found on: 
http://www.openstm32.org/Downloading%2Bthe%2BSystem%2BWorkbench%2Bfor%2BSTM32%2Binstaller

IDE realted information can be found on:
http://www.openstm32.org/HomePage

## Running the tests

TBD

## Deployment

TBD

## Built With

Use [STM Standard Peripheral Library SDK] (http://www.st.com/content/st_com/en/products/embedded-software/mcus-embedded-software/stm32-embedded-software/stm32-standard-peripheral-libraries/stsw-stm32065.html) to build firmware

## Versioning

TBD

## Authors

* **Miroslav Bozic** - *Initial work* - [mikimtb](https://github.com/mikimtb/)

## License

This project is licensed under the GNU GPL License - see the [licence.md](licence.md) file for details

