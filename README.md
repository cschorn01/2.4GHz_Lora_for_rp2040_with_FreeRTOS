# RPi Pico LoRa Template

<!-- Emoji Cheat Sheet: https://github.com/ikatyang/emoji-cheat-sheet/blob/master/README.md -->

[![By Chris Schorn](https://img.shields.io/badge/Author-Chris_Schorn-FFFFFF?style=for-the-badge)](https://github.com/cschorn01)
![Status](https://img.shields.io/badge/Status-usb_not_working-FFFFFF?style=for-the-badge)
[![Clone Repository Template](https://img.shields.io/badge/Clone_Repository_Template-FFFFFF?style=for-the-badge)](https://github.com/new?template_name=Lora_Pico_Driver&template_owner=cschorn01)

[![MIT License](https://img.shields.io/badge/License-MIT-A31B34?style=for-the-badge)](https://mit-license.org/)
[![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)](https://cmake.org/)
[![Raspberry Pi](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Semtech LoRa](https://img.shields.io/badge/LoRa-1CAEED?style=for-the-badge)](https://www.semtech.com/lora)
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-5CBA5B?style=for-the-badge)](https://www.freertos.org/)

[![Description](https://img.shields.io/badge/Description-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/rpi_pico_lora_template/tree/main#description)
[![Functionality](https://img.shields.io/badge/Functionality-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/rpi_pico_lora_template/tree/main#functionality)
[![File Structure](https://img.shields.io/badge/file_structure-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/rpi_pico_lora_template/tree/main#file-structure)
[![How to Use](https://img.shields.io/badge/how_to_use-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/rpi_pico_lora_template/tree/main#how-to-use)
[![Issues](https://img.shields.io/badge/issues-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/rpi_pico_lora_template/blob/main/README.md#issues)

[![Stargazers repo roster for @cschorn01/rpi_pico_lora_template](https://reporoster.com/stars/cschorn01/rpi_pico_lora_template)](https://github.com/cschorn01/rpi_pico_lora_template/stargazers)

<!-- ![GitHub Contributors Image](https://contrib.rocks/image?repo=cschorn01/rpi_pico_lora_template) -->

<!-- [![Top Langs](https://github-readme-stats.vercel.app/api/top-langs/?username=cschorn01&layout=compact&theme=dark)](https://github.com/cschorn01/Lora_Pico_Driver) -->

## Description

The **rpi_pico_lora_template** project is an open source project based on the [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/), 
[Lora Radio](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280), and [FreeRTOS](https://www.freertos.org/). It's goal is to give hobbyists and developers a strong starting point for their projects involving Lora. This project uses the [PHY layer](https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lora-and-lorawan) of the LoRa radio, with [LoRaWAN](https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lora-and-lorawan) functionality coming.

## Functionality

This driver uses a Raspberry Pi Pico's SPI bus to communicate with a 2.4GHz LoRa radio. With FreeRTOS we can create a new task, or individually addressable superloop, that interacts with the [`vSx1280Task`](https://github.com/cschorn01/rpi_pico_lora_template/blob/44e7e5acd0a1cb4129e875321e36d574b70024c7/src/main.c#L970C6-L970C6) task independently. 

In this project there are three tasks, and main():
1. `vSimpleLEDTask` is to show the structure of a task with *setup* above an infinite loop, and the blinking of the onboard pico LED to action in the infinite loop.
2. `vUsbIOTask` takes input from a serial monitor over usb. An unsigned 8 bit pointer array is used as a dynamic ascii character array for a 'string', which is NULL terminated. The address of the pointer array is then sent to `vSx1280Task` to send over LoRa.
3. `vSx1280Task` receives the address of the pointer array from `vUsbIOTask` through a *Task Notification* and reassigns the address to a task local pointer. This is done because there are functions called which delay `vSx1280Task` and would allow `vUsbIOTask` to overwrite the address passed by *Task Notifications*. Once the data is task local `vSx1280Task` will perform a Tx operation, followed by an Rx operation. Both operations are done by using the `sx1280Setup`, `sx1280Tx`, and `sx1280Rx` functions, also in `main.c`.

## File Structure

- :file_folder: pico_projects
  - :file_folder: [RPi Pico C SDK](https://github.com/raspberrypi/pico-sdk)
  - :file_folder: [rpi_pico_lora_template](https://github.com/cschorn01/rpi_pico_lora_template/) 
    - :file_folder: [Docs](https://github.com/cschorn01/Lora_Pico_Driver/tree/main/docs)  
    - :file_folder: [FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
      - :file_folder: [include](https://github.com/cschorn01/Lora_Pico_Driver/tree/main/FreeRTOS-Kernel/include)  
        - :page_facing_up: [FreeRTOSConfig.h](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/FreeRTOS-Kernel/include/FreeRTOSConfig.h) 
    - :file_folder: [src](https://github.com/cschorn01/Lora_Pico_Driver/tree/main/src)  
      - :page_facing_up: [main.c](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/src/main.c)  
    - :page_facing_up: [CMakeLists.txt](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/CMakeLists.txt)  
    - :page_facing_up: [pico_sdk_import.cmake](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/pico_sdk_import.cmake)
    - :page_facing_up: [FreeRTOS_Kernel_import.cmake](https://github.com/cschorn01/Lora_Pico_Driver/blob/main/FreeRTOS_Kernel_import.cmake)

## How To Use

This project is not meant to be used as a library, instead it's a template to begin a given project involving a Raspberry Pi Pico, and Lora Modem. I encourage you to add sensors, or displays and create your own long range wireless [Internet of Things](https://en.wikipedia.org/wiki/Internet_of_things) network.  

There are two ways of using this project:  
1. Creating functions to handle new hardware, and using them in `vSx1280Task`
2. Creating a task to handle new hardware with functions used in the newly written task
  
The second method is recomended, as it is more dynamic and easier to debug. However, it is FOSS code with which you may do as you please.  

Given a new task has been created to handle new hardware there must be communication between tasks. Thats where [FreeRTOS Task Notifications](https://www.freertos.org/RTOS-task-notifications.html) come in. They are fast and easy to use for sending data between task in a memory efficient manner. 

In your new task you should create an 8 bit pointer array to store data in an 8 bit format to send through SPI:  
  
>```c
> uint8_t *dataBuffer = 0;
> ```  
  
Allocate the appropriate amount of memory, in 8 bit chunks, that you'll need to send your data:  
  
> ```c
> dataBuffer = ( uint8_t * ) malloc( 255 * sizeof( uint8_t ) );
> ```  
  
Here 255 is used because it's the maximum LoRa packet size on the sx1280.  
Assign your data to the newly allocated data buffer:  
  
>```c
>*( dataBuffer + 0 ) = 0x48; // 0x48 is ASCII Hexadecimal 'H' 
>*( dataBuffer + 1 ) = 0x49; // 0x49 is ASCII Hexadecimal 'I'
>```

Use the [`xTaskNotify()`](https://www.freertos.org/xTaskNotify.html) function to send a task notification from your new task to `vSx1280Task`:

> ```c
> xTaskNotify(  
>             xSx1280TaskHandle,                  /* TaskHandle_t xTaskToNotify */  
>             ( uint32_t ) &*( dataBuffer ),      /* uint32_t ulValue (int)&buffer[0] */  
>             eSetValueWithoutOverwrite );        /* eNotifyAction eAction */  
>            )
> ```
  
In [`vSx1280Task`](https://github.com/cschorn01/rpi_pico_lora_template/blob/44e7e5acd0a1cb4129e875321e36d574b70024c7/src/main.c#L970C6-L970C6) the [`xTaskNotifyWait()`](https://www.freertos.org/xTaskNotifyWait.html) will accept *Task Notifications* from all tasks that are sending them. You must process the current *Task Notification* before allowing another task to run or the current *Task Notification* may be overwritten by an incoming *Task Notification* from another task.

<!-- Forkers

[![Forkers repo roster for @cschorn01/rpi_pico_lora_template](https://reporoster.com/forks/cschorn01/rpi_pico_lora_template)](https://github.com/cschorn01/rpi_pico_lora_template/network/members) -->

## Issues

![Error](https://img.shields.io/badge/Error-A31B34?style=for-the-badge) `arm-none-eabi-gcc: fatal error: cannot read spec file 'nosys.specs': No such file or directory`

<!-- ![Solution](https://img.shields.io/badge/Solution-5CBA5B?style=for-the-badge) Go to `/usr/bin/local/` delete all files beginning in `arm-none-eabi-` then reinstall the arm toolchain.

**Mac:** `brew install --cask gcc-arm-embedded`

**Linux:** `sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential` -->

<div align="center" dir="auto">
  <a href="https://github.com/cschorn01/rpi_pico_lora_template">
    <img src="https://img.shields.io/badge/Back_To_Top-FFFFFF?style=for-the-badge" alt="Back To Top">
  </a>
</div>
