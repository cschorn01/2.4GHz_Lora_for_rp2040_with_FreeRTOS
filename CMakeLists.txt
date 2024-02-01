cmake_Minimum_required(VERSION 3.5)

include(pico_sdk_import.cmake) # initialize the SDK based on PICO_SDK_PATH
include(FreeRTOS_Kernel_import.cmake) # Pull in FreeRTOS

project(blink C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_PLATFORM=rp2040)

pico_sdk_init() # initialize the Pico SDK

# FILE(GLOB FreeRTOS_src FREERTOS-Kernel/*.c)

add_executable(blink
        src/main.c
        myLibraries/sx1280ForRp2040.c )

add_library( FreeRTOS STATIC
        ${FreeRTOS_src}
        #FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/port.c
        FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
        FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

# add_library( sdCardLibrary STATIC src/sdCardLibrary.c )
# add_library( sx1280ForRp2040 STATIC src/sx1280ForRp2040.c )

target_include_directories( FreeRTOS PUBLIC
        include/
        FreeRTOS-Kernel/include
        #FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040
        FreeRTOS-Kernel/portable/GCC/ARM_CM0
)


target_link_libraries( blink PUBLIC
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        FreeRTOS
        # pico_stdio
        pico_stdlib
        hardware_gpio
        hardware_spi
        hardware_irq
        hardware_adc
)

# enable usb output, disable uart output
pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 0)

pico_add_extra_outputs(blink)
