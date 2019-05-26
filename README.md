
## Plant monitor with STM32F4Discovery using FreeRTOS

This project is for measuring the soil moisture of my balcony plants and water them as necessary.
The STM32F4 Discovery board is used. Starter code with FreeRTOS port is from istarc https://github.com/istarc/stm32.git (thanks!)

Other starter code is also used for debugging with UART printouts and communication with the DS3231 RTC chip.
As this is still a work in progress, proper references and updated README will be added later.

---

##### Original README from https://github.com/istarc/stm32.git

Prerequisites:
* Ubuntu 14.04 LTS
* STM32F4 Discovery Board

Install software dependencies:
* `cd ~`
* `sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded`
* `sudo apt-get update`
* `sudo apt-cache policy gcc-arm-none-eabi`
* `sudo apt-get install gcc-arm-none-eabi=4-8-2014q2-0trusty10` The latter packages contains GCC, GDB, binutils, newlib and newlib-nano libraries.
* `sudo apt-get install build-essential git openocd`
* `git clone https://github.com/istarc/stm32.git`
* `cd ~/stm32`
* `git submodule update --init` This command is required to initialize submodules.

Build:
* `make clean`
* `make -j4 # Non-optimized build`
* Other build options:
	* `make -j4 release # Non-optimized build`
	* `make -j4 release-memopt # Size-optimized build`
	* `make debug # Debug build`

Deploy to target via OpenOCD:
* `make deploy`

More info:
* [Deploy FreeRTOS Embedded OS in under 10 seconds!](http://istarc.wordpress.com/2014/07/10/stm32f4-deploy-an-embedded-os-under-10-seconds/)
