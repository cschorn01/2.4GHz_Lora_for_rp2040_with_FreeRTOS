# Raspberry Pi Pico LoRa Template

<!-- Emoji Cheat Sheet: https://github.com/ikatyang/emoji-cheat-sheet/blob/master/README.md -->

[![By Chris Schorn](https://img.shields.io/badge/Author-Chris_Schorn-FFFFFF?style=for-the-badge)](https://github.com/cschorn01)
[![Status](https://img.shields.io/badge/Status-working-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/blob/9b8eda27daef9f655651284ecc0680a135ffd662/src/main.c#L1118C1-L1118C39)
[![Clone Repository Template](https://img.shields.io/badge/Clone_Repository_Template-FFFFFF?style=for-the-badge)](https://github.com/new?template_name=2.4GHz_Lora_for_rp2040_with_FreeRTOS&template_owner=cschorn01)

[![MIT License](https://img.shields.io/badge/License-MIT-A31B34?style=for-the-badge)](https://mit-license.org/)
[![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)](https://cmake.org/)
[![Raspberry Pi](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Semtech LoRa](https://img.shields.io/badge/LoRa-1CAEED?style=for-the-badge)](https://www.semtech.com/lora)
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-5CBA5B?style=for-the-badge)](https://www.freertos.org/)

[![Description](https://img.shields.io/badge/Description-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main#)
[![Functionality](https://img.shields.io/badge/Functionality-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main#-1)
[![File Structure](https://img.shields.io/badge/file_structure-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main#-2)
[![How to Use](https://img.shields.io/badge/how_to_use-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main#-3)
[![Issues](https://img.shields.io/badge/issues-FFFFFF?style=for-the-badge)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main#-4)

<!-- ![GitHub Contributors Image](https://contrib.rocks/image?repo=cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS) -->

<!-- [![Top Langs](https://github-readme-stats.vercel.app/api/top-langs/?username=cschorn01&layout=compact&theme=dark)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS) -->

[![Stargazers repo roster for @cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS](https://reporoster.com/stars/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/stargazers)


## ![Description](https://img.shields.io/badge/Description-FFFFFF?style=for-the-badge)

The **2.4GHz_Lora_for_rp2040_with_FreeRTOS** is an open source project based on the [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/), [Lora Radio](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280), and [FreeRTOS](https://www.freertos.org/). It's goal is to give hobbyists and developers a strong starting point for their projects involving Lora. This project uses the [PHY layer](https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lora-and-lorawan) of the LoRa radio, with [LoRaWAN](https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lora-and-lorawan) functionality coming.

## ![Functionality](https://img.shields.io/badge/Functionality-FFFFFF?style=for-the-badge)

This project uses a Raspberry Pi Pico's SPI bus to communicate with a 2.4GHz LoRa radio. With FreeRTOS we can create a new task, or individually addressable superloop, that interacts with the [`vSx1280Task`](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/blob/44e7e5acd0a1cb4129e875321e36d574b70024c7/src/main.c#L970C6-L970C6) independently. 

In this project there are three tasks, and main():
1. `vSimpleLEDTask` is to show the structure of a task with *setup* above an infinite loop, and the blinking of the onboard pico LED in the infinite loop.
2. `vUsbIOTask` takes typed input from a serial monitor over usb. An unsigned 8 bit pointer array is used as a dynamic ascii character array for a 'string', which is NULL terminated. The address of the pointer array is then sent to `vSx1280Task` to send over LoRa.
3. `vSx1280Task` receives the address of the pointer array from `vUsbIOTask` through a *Task Notification* and reassigns the address to a task local pointer. This is done because there are functions called which delay `vSx1280Task` and would allow `vUsbIOTask` to overwrite the address passed by *Task Notifications*. Once the data is task local `vSx1280Task` will perform a Tx operation, followed by an Rx operation. Both operations are done by using the `sx1280Setup`, `sx1280Tx`, and `sx1280Rx` functions, also in `main.c`.

## ![File Structure](https://img.shields.io/badge/file_structure-FFFFFF?style=for-the-badge)

- :file_folder: pico_projects
  - :file_folder: [Raspberry Pi Pico C SDK](https://github.com/raspberrypi/pico-sdk)
  - :file_folder: [2.4GHz_Lora_for_rp2040_with_FreeRTOS](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/) 
    - :file_folder: [Docs](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main/docs)  
    - :file_folder: [FreeRTOS-Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel)
    - :file_folder: [src](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main/src)  
      - :page_facing_up: [main.c](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/blob/main/src/main.c)
    - :file_folder: [include](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main/include)
      - :page_facing_up: [FreeRTOSConfig.h](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/tree/main/include/FreeRTOSConfig.h)
    - :page_facing_up: [CMakeLists.txt](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/blob/main/CMakeLists.txt)  
    - :page_facing_up: [pico_sdk_import.cmake](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/blob/main/pico_sdk_import.cmake)
    - :page_facing_up: [FreeRTOS_Kernel_import.cmake](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/blob/main/FreeRTOS_Kernel_import.cmake)

## ![How to Use](https://img.shields.io/badge/how_to_use-FFFFFF?style=for-the-badge)

This project is a template to begin a project with a Raspberry Pi Pico, and Lora Modem, for easily adding sensors, or displays.  Then you can create your own long range wireless [Internet of Things](https://en.wikipedia.org/wiki/Internet_of_things) network.  

When adding sensors, create a new task for handling each one. With a new task handling new hardware there must be communication between tasks, which is where [FreeRTOS Task Notifications](https://www.freertos.org/RTOS-task-notifications.html) come in.

In the new task instantiate an 8 bit pointer to store data for sending over SPI:  

>```c
> uint8_t *dataBuffer = NULL;
> ```  
  
Allocate the appropriate amount of memory, in 8 bit chunks, that is needed to send your data:  
  
> ```c
> dataBuffer = ( uint8_t * ) malloc( 255 * sizeof( uint8_t ) );
> ```  
  
  
Assign your data to the newly allocated data buffer. Here, 255 is used because it's the maximum LoRa packet size on the sx1280:  
  
>```c
>*( dataBuffer + 0 ) = 0x48; // 0x48 is ASCII Hexadecimal 'H' 
>*( dataBuffer + 1 ) = 0x49; // 0x49 is ASCII Hexadecimal 'I'
>```

Use the [`xTaskNotify()`](https://www.freertos.org/xTaskNotify.html) function to send a *Task Notification* from your new task to `vSx1280Task`. This is where a LoRa message will be sent:

> ```c
> xTaskNotify(  
>             xSx1280TaskHandle,                  /* TaskHandle_t xTaskToNotify */  
>             ( uint32_t ) dataBuffer,            /* uint32_t ulValue (int)&buffer[0] */  
>             eSetValueWithoutOverwrite );        /* eNotifyAction eAction */  
>            )
> ```

To use the incoming *Tast Notification*, it must be reassigned to a task local pointer. This template uses a struct containing an 8 bit pointer:

```c
struct sx1280MessageStorageTillUse{
  uint8_t *message;
};
struct sx1280MessageStorageTillUse messageStorageTillUse = { NULL };
```

<!-- Forkers

[![Forkers repo roster for @cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS](https://reporoster.com/forks/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS)](https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS/network/members) -->

## ![Issues](https://img.shields.io/badge/issues-FFFFFF?style=for-the-badge)

- ![Error](https://img.shields.io/badge/Error-A31B34?style=for-the-badge) `arm-none-eabi-gcc: fatal error: cannot read spec file 'nosys.specs': No such file or directory`
  - ![Solution](https://img.shields.io/badge/Solution-5CBA5B?style=for-the-badge) Go to `/usr/bin/local/` delete all files beginning in `arm-none-eabi-` then reinstall the arm toolchain.
    - **Linux:** `sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential`
    - **Mac:** `brew install --cask gcc-arm-embedded`
    - **Windows:** [Pico Setup for Windows](https://github.com/raspberrypi/pico-setup-windows)

<div align="center" dir="auto">
  <a href="https://github.com/cschorn01/2.4GHz_Lora_for_rp2040_with_FreeRTOS">
    <img src="https://img.shields.io/badge/Back_To_Top-FFFFFF?style=for-the-badge" alt="Back To Top">
  </a>
</div>
