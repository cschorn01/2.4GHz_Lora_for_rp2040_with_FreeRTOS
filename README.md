# Open_rp2040_Lora_driver


## Description

The **Open_rp2040_Lora_Driver** project is an open source project based on the rp2040, 
sx1280(2.4GHz Lora radio modem), and freeRTOS. It's goal is to give hobbyists and developers a strong starting point
for their projects involving Lora radio.

#Functionality
Just an open driver for Lora on an rp2040 would not allow for the expansion of an application.
Adding freeRTOS to the rp2040 allows the application developer to easily create new functionalities.
For example, it would be challenging to add a new sensor to a sensor array with Lora in the same 
superloop, but with freeRTOS we can create a new task(individually addressable superloop) that interacts
with the Lora task independently of the original sensor array. In this driver there are three tasks, 
and main():
1. `vSimpleLEDTask` is a few lines to show 
the structure of a task with the setup being above the infinite loop, and the blinking of the onboard pico
LED to show the action of the infinite loop.
2. `vUsbIOTask` which takes input from a serial monitor over usb using getchar(). A 0x00 or NULL is 
appended to the end of the unsigned 8 bit pointer array used as a dynamic ascii character array for a NULL terminated 'string'. 
The address of the pointer array is then sent to `vSx1280Task` for sending through the sx1280.
3. `vSx1280Task` will take the pointer array from `vUsbIOTask` through a *Task Notification* and reassign the pointer to a 
task local pointer array. This is done
because there are functions called which delays `vSx1280Task` and would allow `vUsbIOTask` to overwrite the *Task Notification* pointer array.
Once the data is task local `vSx1280Task` will perform a Tx operation, followed by an RX operation. Both operations are done by using the 
sx1280Setup, sx1280Tx, and sx1280Rx functions.

#To Add
1. How to use
2. File Structure with pico_sdk
3. 
