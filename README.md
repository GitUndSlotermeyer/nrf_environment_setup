## Introduction

This guide will provide instructions on how to build and debug firmware for nRF52xx family of SoC-s on windows, using VS Code and Ubuntu in WSL as a development environment.

Two boards are described:
* nRF52840-DK (PCA10056)
  * Has an on board Segger J-Link debug probe
  * Firmware loading and debugging via JTAG is described in the guide
  * Log output via RTT (JTAG) and UART is described in the guide
* nRF52840-DONGLE (PCA10059)
  * No on board debug probe
  * Firmware loading via serial DFU bootloader is described in the guide
  * Log output via UART is described in the guide


## Enable WSL and install Ubuntu

WSL (Windows Subsystem for Linux) first needs to be enabled in Windows.
Open PowerShell as an admin and run:

	dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all

Then, install Ubuntu 20.04 LTS (or whatever is current) from the Windows store.
Run Ubuntu 20.04 LTS and enter root user name and password for the first time.

After that run (in WSL terminal):

	sudo apt update
	sudo apt upgrade


## Get GNU make and C compiler

Then install gnu make and C compiler (run in WSL terminal): 

	sudo apt install make
	sudo apt install gcc-arm-none-eabi
	sudo apt install gdb-multiarch
	sudo apt install unzip

Since current GCC comes without GDB and the debug extension in VS code expects to find it there, let's create a symbolic link to gdb-multiarch:
	sudo ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb 


## Get the Nordic SDK

Prepare a directory to put everything in (run in WSL terminal):

	cd ~
	mkdir src
	cd src

Next, go to the SDK download page (in Windows):

	https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK/Download#infotabs

Select latest v17 SDK (this is written using v17.0.2) and download it.
Copy *DeviceDownload.zip* to e.g. *C:\Projects*
Unpack.

Copy from Windows to ~/src in WSL: 

	cp /mnt/c/Projects/DeviceDownload/nRF5SDK1702d674dde.zip ~/src
	unzip nRF5SDK1702d674dde.zip
	rm nRF5SDK1702d674dde.zip


## Install VS code

Install VS code in Windows.

Install C/C++ extension.

Install Remote - WSL extension.

Install Cortex debug extension.

Restart the computer.


## Configure the SDK

Now the SDK needs to know where the compiler is. So, run this from WSL terminal:

	which arm-none-eabi-gcc

You will get something like this:

	/usr/bin/arm-none-eabi-gcc

Then open the SDK config file by running this in the WSL terminal: 

	code ~/src/nRF5_SDK_17.0.2_d674dde/components/toolchain/gcc/Makefile.posix

File will open in VS code. Change lines according to output from *which*:

	GNU_INSTALL_ROOT ?= /usr/bin/
	GNU_VERSION ?= 9.3.1
	GNU_PREFIX ?= arm-none-eabi


## Copy an example from the SDK

Make a sub directory for nRF examples:

	cd ~/src
	mkdir nrf_ble
	cd nrf_ble

Copy the BLE beacon example from the SDK (run in WSL terminal):

	cp ../nRF5_SDK_17.0.2_d674dde/examples/ble_peripheral/ble_app_beacon ./ -R

Open the example (run in WSL terminal):

	cd ble_app_beacon
	code .

VS code server will get installed into WSL when doing this for the first time.

Go to file *pca10056/s140/armgcc/Makefile* in VS code and change line:
	
	SDK_ROOT := ../../../../../..

to:
	
	SDK_ROOT := ../../../../../nRF5_SDK_17.0.2_d674dde


## Building the example 

Press Ctrl+` to open VS code internal terminal‚ and input the following:

	cd pca10056/s140/armgcc
	make


## Seting up the debugger

Download and install Segger JLink  from:
https://www.segger.com/downloads/jlink/JLink_Windows_V686f.exe

Now create the VS code config file for debugging:

	cd ~/src_nrf_ble_app_beacon
	mkdir .vscode
	cd .vscode
	touch launch.json
	code launch.json


An empty config file will open. Now paste this:

	{
		// Use IntelliSense to learn about possible attributes.
		// Hover to view descriptions of existing attributes.
		// For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
		"version": "0.2.0",
		"configurations": [
		{
			"type": "cortex-debug",
			"request": "launch",
			"name": "Debug J-Link",
			"cwd": "${workspaceRoot}",
			"executable": "${workspaceRoot}/pca10056/s140/armgcc/_build/nrf52840_xxaa.out",
			"serverpath": "/mnt/c/Program Files (x86)/SEGGER/JLink/JLinkGDBServerCL.exe", 
			"servertype": "jlink",
			"device": "nRF52840_xxAA",
			"interface": "swd",
			"runToMain": true,
			"armToolchainPath": "/usr/bin/",
			"svdFile": "${workspaceRoot}/../nRF5_SDK_17.0.0_9d13099/modules/nrfx/mdk/nrf52840.svd" 
	}
		]
		}


## Loading using serial DFU bootloader


CFLAGS += -DBOARD_PCA10059
+i~/src/nrf_ble/ble_app_beacon$ cp pca10056 pca10059 -r
from:
CFLAGS += -DBOARD_PCA10056
to:
CFLAGS += -DBOARD_PCA10059

For this, we will need the *nrfutil* package that is provided by Nordic. 
It's installed via Python pip:

	sudo apt install python3-pip
	pip3 install nrfutil
	source ~/.profile


Now we can add a target into the Makefile, to simplify DFU package generation.
Add target to Makefile:

	dfu: default
		nrfutil pkg generate --hw-version 52 --sd-req 0x00 --application-version-string 1.0.0 --application _build/nrf52840_xxaa.hex --softdevice $(SDK_ROOT)/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --sd-id 0x0100 app_dfu_package.zip


Now invoke make dfu:

	make dfu

Find USB serial device in Windows device manager and mark its number.
Then run the following command (e.g. */dev/ttyS5* is for COM5):
	
	nrfutil dfu serial -pkg app_dfu_package.zip -p /dev/ttyS5 -b 115200


## Enable logging


If you want to change SDK config it's best to use app config.
Add the following define to the makefile 

	CFLAGS += -DUSE_APP_CONFIG

Then create an ./pca10056/s140/config/app_config.h file:

	#ifndef APP_CONFIG_H
	#define APP_CONFIG_H


	/*******************************************************************/
	// UART
	/*******************************************************************/
	// <e> NRFX_UARTE_ENABLED - nrfx_uarte - UARTE peripheral driver
	#define NRFX_UARTE_ENABLED 1

	// <e> NRFX_UART_ENABLED - nrfx_uart - UART peripheral driver
	// For the nRF52 series there's no reason to use UART over UARTE, 
	// as UARTE is both faster, has lower CPU usage, and consumes less power. 
	#define NRFX_UART_ENABLED 1

	// <e> UART_ENABLED - nrf_drv_uart - UART/UARTE peripheral driver - legacy layer
	// It seems that nrf log uart backend needs this
	#define UART_ENABLED 1

	// <e> UART0_ENABLED - Enable UART0 instance
	#define UART0_ENABLED 1

	// <e> APP_UART_ENABLED - app_uart - UART driver
	#define APP_UART_ENABLED 1

	// <o> APP_UART_DRIVER_INSTANCE  - UART instance used
	// <0=> 0 
	#define APP_UART_DRIVER_INSTANCE 0


	/*******************************************************************/
	// LOG
	/*******************************************************************/
	// <e> NRF_LOG_ENABLED - nrf_log - Logger
	#define NRF_LOG_ENABLED 1

	// <e> NRF_LOG_BACKEND_RTT_ENABLED - nrf_log_backend_rtt - Log RTT backend
	#define NRF_LOG_BACKEND_RTT_ENABLED 1

	// <o> NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE - Size of buffer for partially processed strings. 
	// <i> Size of the buffer is a trade-off between RAM usage and processing.
	// <i> if buffer is smaller then strings will often be fragmented.
	// <i> It is recommended to use size which will fit typical log and only the
	// <i> longer one will be fragmented.
	#define NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE 64

	// <o> NRF_LOG_BACKEND_RTT_TX_RETRY_DELAY_MS - Period before retrying writing to RTT 
	#define NRF_LOG_BACKEND_RTT_TX_RETRY_DELAY_MS 1

	// <o> NRF_LOG_BACKEND_RTT_TX_RETRY_CNT - Writing to RTT retries. 
	// <i> If RTT fails to accept any new data after retries
	// <i> module assumes that host is not active and on next
	// <i> request it will perform only one write attempt.
	// <i> On successful writing, module assumes that host is active
	// <i> and scheme with retry is applied again.
	#define NRF_LOG_BACKEND_RTT_TX_RETRY_CNT 3


	// <e> NRF_LOG_BACKEND_UART_ENABLED - nrf_log_backend_uart - Log UART backend
	#define NRF_LOG_BACKEND_UART_ENABLED 1

	// <o> NRF_LOG_BACKEND_UART_TX_PIN - UART TX pin 
	#define NRF_LOG_BACKEND_UART_TX_PIN 6

	// <o> NRF_LOG_BACKEND_UART_BAUDRATE  - Default Baudrate
	// <323584=> 1200 baud 
	// <643072=> 2400 baud 
	// <1290240=> 4800 baud 
	// <2576384=> 9600 baud 
	// <3862528=> 14400 baud 
	// <5152768=> 19200 baud 
	// <7716864=> 28800 baud 
	// <10289152=> 38400 baud 
	// <15400960=> 57600 baud 
	// <20615168=> 76800 baud 
	// <30801920=> 115200 baud 
	// <61865984=> 230400 baud 
	// <67108864=> 250000 baud 
	// <121634816=> 460800 baud 
	// <251658240=> 921600 baud 
	// <268435456=> 1000000 baud 
	#define NRF_LOG_BACKEND_UART_BAUDRATE 30801920

	// <o> NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE - Size of buffer for partially processed strings. 
	// <i> Size of the buffer is a trade-off between RAM usage and processing.
	// <i> if buffer is smaller then strings will often be fragmented.
	// <i> It is recommended to use size which will fit typical log and only the
	// <i> longer one will be fragmented.

	#define NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE 64


	// <h> segger_rtt - SEGGER RTT
	// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_UP - Size of upstream buffer. 
	// <i> Note that either @ref NRF_LOG_BACKEND_RTT_OUTPUT_BUFFER_SIZE
	// <i> or this value is actually used. It depends on which one is bigger.
	#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP 512

	// <o> SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS - Maximum number of upstream buffers. 
	#define SEGGER_RTT_CONFIG_MAX_NUM_UP_BUFFERS 2

	// <o> SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN - Size of downstream buffer. 
	#define SEGGER_RTT_CONFIG_BUFFER_SIZE_DOWN 16

	// <o> SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS - Maximum number of downstream buffers. 
	#define SEGGER_RTT_CONFIG_MAX_NUM_DOWN_BUFFERS 2

	// <o> SEGGER_RTT_CONFIG_DEFAULT_MODE  - RTT behavior if the buffer is full.
	// <i> The following modes are supported:
	// <i> - SKIP  - Do not block, output nothing.
	// <i> - TRIM  - Do not block, output as much as fits.
	// <i> - BLOCK - Wait until there is space in the buffer.
	// <0=> SKIP 
	// <1=> TRIM 
	// <2=> BLOCK_IF_FIFO_FULL 
	#define SEGGER_RTT_CONFIG_DEFAULT_MODE 0


	#endif


By adding this app_config, log messages will be visible in Jink RTT viewer 
and any serial terminal (JLink CDC Uart Port)


