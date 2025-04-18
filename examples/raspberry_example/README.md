# Raspberry Pi Example

## Overview

This example demonstrates how to flash an Espressif SoC (target) from a Raspberry Pi 4 (Model B) using `esp_serial_flasher`. AT command firmware to be flashed from a Raspberry Pi to ESP32 can be found in [binaries](../binaries/). USART0 is dedicated for the communication with the Espressif SoC.

The following steps are performed in order to re-program targets memory:

1. Peripherals are initialized.
2. The host puts slave device into the boot mode and tries to connect by calling `esp_loader_connect()`.
3. Then `esp_loader_flash_start()` is called to enter the flashing mode and erase amount of the memory to be flashed.
4. `esp_loader_flash_write()` function is called repeatedly until the whole binary image is transfered.
5. At the end, `loader_port_reset_target()` is called to restart the target and execute the updated firmware.

**Note:** In addition to the steps mentioned above, `esp_loader_change_transmission_rate()`  is called after connection is established in order to increase the flashing speed. Bootloader is also capable of detecting baud rate during connection phase and can be changed before calling `esp_loader_connect()`. However, it is recommended to start at lower speed and then use dedicated command to increase baud rate. This does not apply for ESP8266, as its bootloader does not support this command, therefore, baud rate can only be changed before connection phase in this case.

## Hardware Required

* Raspberry Pi 4 Model B.
* A development board with the ESP32 SoC (e.g. ESP-WROVER-KIT, ESP32-DevKitC, etc.).
* USB cable in case ESP32 board is powered from USB. ESP32 can be powered by Raspberry Pi as well.

## Hardware Connection

Table below shows connection between Raspberry Pi and ESP32.

| Raspberry Pi (host) |    ESP32 (target)   |
|:-------------------:|:-------------------:|
|        GPIO3        |         IO0         |
|        GPIO2        |         RST         |
|        GPIO14       |         RX0         |
|        GPIO15       |         TX0         |
|         GND         |         GND         |

Optionally, UART-to-USB bridge can be connected to PD5(RX) and PD6(TX) for debug purposes.

## Installation

### GPIO Library

Raspberry Pi makes use of [pigpio](http://abyz.me.uk/rpi/pigpio/) library in order to simplify controlling GPIO pins. Some distributions of 'Raspberry Pi OS' may come with `pigpio` already installed. Presence of the library in the system can checked by running command:

```bash
pigpiod -v
```

If not present, run following commands to install the library.

```bash
sudo apt-get update
sudo apt-get install pigpio
```

### Enable UART

On Raspberry Pi 4, primary UART (UART0) is connected to the On-board Bluetooth module by default.
In order to enable serial communication on this UART, run following command in terminal:

```bash
sudo raspi-config
```

* Navigate to **Interfacing Options -> Serial**.
* Then it will ask for login shell to be accessible over Serial, select **No**.
* After that, it will ask for enabling Hardware Serial port, select **Yes**.
* Reboot the Raspberry Pi.

## Build and Run

To compile the example:

Create and navigate to `build` directory:

```bash
mkdir build && cd build
```

Run cmake, build example and run example:

```bash
cmake .. && cmake --build . && ./raspberry_flasher
```

For more details regarding to esp_serial_flasher configuration, please refer to top level [README.md](../../README.md).
