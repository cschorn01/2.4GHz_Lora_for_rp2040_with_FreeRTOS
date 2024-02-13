/*
 * Author: Chris Schorn
 * Open Lora Mesh Network
 * Version: 
 * Date: 
 * Versioning Reason: 
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* 
                General Structure Notes 

    In main, setup tasks and run vTaskStartScheduler.
    In tasks, setup hardware before while(true), and
        run data transfer with hardware in while(true).
    Mesh network processing will move to the second core
        to continue in the background if needed
    Use "back" input from touchscreen to put current
        task/app in the blocked state and go home screen
    Using task notifications, I can take the 'notification
        value' and convert it to an 32 bit pointer address
*/

/* FreeRTOS Includes */
#include "../FreeRTOS-Kernel/include/FreeRTOS.h" /* MUST COME FIRST */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

/* Raspberry Pi Pico Inlcudes */
#include <stdio.h> 
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

/* Including my libraries */
#include "../myLibraries/sx1280ForRp2040.h"

static TaskHandle_t xSimpleLEDTaskHandle = NULL;
static TaskHandle_t xUsbIOTaskHandle = NULL;
static TaskHandle_t xSx1280TaskHandle = NULL;


/* --------------------------- sx1280 2.4GHz Lora Operation -------------------------------- */



/*  Task setting up and running sx1280
    Multi-packet messaging commented out, putting in single packet */
void vSx1280Task( void *pvParameters ){

    /* 8 bit pointer variables for dynamic arrays */
    uint8_t writeData[ 256 ] = { 0 };
    uint8_t readData[ 256 ] = { 0 };

    /* 32 bit pointer for address received from vUsbIOTask task notification */
    uint32_t *taskNotificationFromUSB = ( uint32_t * ) pvPortMalloc( 1*sizeof( uint32_t ) );

    /* 8 bit integer for a boolean value, about if messageStorageTillUse is empty */
    uint8_t isMessageStorageTillUseEmpty = 1;
    /* 32 bits for message currently being operated on in messageStorageTillUse */
    uint32_t messageStorageIndex = 0;

    /* Lora Structs */
    struct sx1280MessageStorageTillUse{

        uint8_t message[ 256 ];
    };

    /* Instantiating struct, containing an 8 bit pointer to temporarily store a message */
    struct sx1280MessageStorageTillUse messageStorageTillUse = { { 0 } };

    /* Initalizing unsigned 8 bit integers to store the pin assignments for the sx1280 */
    uint8_t sx1280BusyPin = 22;
    uint8_t sx1280ResetPin = 21;
    uint8_t sx1280SpiNumber = 1;
    uint8_t sx1280SpiSckPin = 10;
    uint8_t sx1280SpiTxPin = 11;
    uint8_t sx1280SpiRxPin = 12;
    uint8_t sx1280ChipSelectPin = 13;

    sx1280Rp2040Setup( sx1280BusyPin,           /* uint8_t sx1280Rp2040BusyPin          */
                       sx1280ResetPin,          /* uint8_t sx1280Rp2040ResetPin         */
                       sx1280SpiNumber,         /* uint8_t sx1280Rp2040SpiNumber        */
                       sx1280SpiSckPin,         /* uint8_t sx1280Rp2040SpiSckPin        */
                       sx1280SpiTxPin,          /* uint8_t sx1280Rp2040SpiTxPin         */
                       sx1280SpiRxPin,          /* uint8_t sx1280Rp2040SpiRxPin         */
                       sx1280ChipSelectPin );   /* uint8_t sx1280Rp2040ChipSelectPin    */


    uint8_t antselPin = 28; /* Antenna Select pin for DLP-RFS1280 module */
    gpio_set_dir( antselPin, 1 );/* Setting GPIO direction to output, True is out False is in */
    /* static void gpio_put (uint gpio, bool value) */
    gpio_put( antselPin, 0 );/* GPIO Low, using DLP-RFS1280 module onboard antenna */

    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;

    /* Resetting sx1280 during startup, says in documentation FIND PAGE */
    gpio_put( 21, 0 ); 
    asm volatile ("nop \n nop \n nop");
    gpio_put( 21, 1 );

    while( true ){

        /* Asserting messageStorageTillUse array of structs is empty beginning the super loop */
        isMessageStorageTillUseEmpty = 1;
        *( taskNotificationFromUSB ) == 0;


        xTaskNotifyWait(
                0xffffffff,               /* uint32_t ulBitsToClearOnEntry */
                0,                        /* uint32_t ulBitsToClearOnExit */
                taskNotificationFromUSB,  /* uint32_t *pulNotificationValue */
                100 );                    /* TickType_t xTicksToWait */
 
        if( *( taskNotificationFromUSB ) != 0 ){
            for( j = 0; *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ) != 0x00; j++){
                printf("sx1280 task notification = 0x%X %c\n", *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ), *( ( uint8_t * ) *( taskNotificationFromUSB ) + j ) );
            }
            printf("sx1280 task notification held address: 0x%X\n", *(taskNotificationFromUSB));
            memcpy( messageStorageTillUse.message, ( uint8_t * ) *( taskNotificationFromUSB ), 256 );
        }

        vTaskDelay( 10 );

        if( messageStorageTillUse.message == NULL ){
            /* Arbitrarily allocating 255 bytes to writeData for payloadLength in sx1280Setup
               Payload length does not matter for messages with headers */
            for( i = 0; i <= 255; i++){
                *( writeData + i ) = 0xFF;
                if( i == 255 ){
                    *( writeData + i ) = 0x00;
                }
            }

            sx1280Setup( 0x00,                  /* uint8_t standbyMode              */
                         0x01,                  /* uint8_t packetType               */
                         0xB8,                  /* uint8_t rfFrequency2316          */
                         0x9D,                  /* uint8_t rfFrequency158           */
                         0x89,                  /* uint8_t rfFrequency70            */
                         0x70,                  /* uint8_t spreadingFactor          */
                         0x0A,                  /* uint8_t bandwidth                */
                         0x01,                  /* uint8_t codingRate               */
                         0x0C,                  /* uint8_t preambleLength           */
                         0x00,                  /* uint8_t headerType               */
                         0x20,                  /* uint8_t cyclicalRedundancyCheck  */
                         0x40,                  /* uint8_t chirpInvert              */
                         sx1280ChipSelectPin,   /* uint8_t setupChipSelectPin       */
                         sx1280ResetPin,        /* uint8_t setupResetPin            */
                         sx1280BusyPin,         /* uint8_t setupBusyPin             */
                         sx1280SpiNumber,       /* uint8_t setupSpiNumber           */
                         writeData );           /* uint8_t outboundMessage[ ]       */


            /* Assigning 0 to 0th element of readData, no incoming message data if still 0 */
            memset( readData, 0, 256 );

            sx1280Rx( 0x40,                 /* uint8_t rxIrq158                 */
                      0x7E,                 /* uint8_t rxIrq70                  */
                      0x02,                 /* uint8_t rxPeriodBase             */
                      0xFF,                 /* uint8_t rxPeriodBaseCount158     */
                      0xFF,                 /* uint8_t rxPeriodBaseCount70      */
                      sx1280ChipSelectPin,  /* uint8_t rxChipSelectPin          */
                      sx1280BusyPin,        /* uint8_t rxBusyPin                */
                      sx1280SpiNumber,      /* uint8_t rxSpiNumber              */
                      readData );           /* uint8_t inboundMessage[ ]        */

            /* If there are not 0's in the first two spots of the readData array */
            if( *( readData ) != 0x00 && *( readData + 1 ) != 0x00 ){

                for( i = 3; *( readData + i ) != 0x00; i++){ /* Looping and printing readData */
                    printf("Inbound Message: 0x%X %c %i\n", *(readData + 3), *(readData +3), i);
                }
                memset( writeData, 0, 256 );
            }
            /* Otherwise no inbound message */
            else{

                printf("No inbound message\n");
            }
        }
        else if( messageStorageTillUse.message[ 0 ] != 0 ){ 

            *( writeData ) = 0x52;      /* R */
            *( writeData + 1 ) = 0x54;  /* T */
            *( writeData + 2 ) = 0x52;  /* R */
            *( writeData + 3 ) = 0x47;  /* G */
            *( writeData + 4 ) = 0x00;  /* NULL */

            /* Setting up and Sending RTRG message on the 2.4Ghz Frequency */
            sx1280Setup( 0x00,                  /* uint8_t standbyMode                  */ 
                         0x01,                  /* uint8_t packetType                   */ 
                         0xB8,                  /* uint8_t rfFrequency2316              */ 
                         0x9D,                  /* uint8_t rfFrequency158               */ 
                         0x89,                  /* uint8_t rfFrequency70                */ 
                         0x70,                  /* uint8_t spreadingFactor              */ 
                         0x0A,                  /* uint8_t bandwidth                    */ 
                         0x01,                  /* uint8_t codingRate                   */ 
                         0x0C,                  /* uint8_t preambleLength               */ 
                         0x00,                  /* uint8_t headerType                   */ 
                         0x20,                  /* uint8_t cyclicalRedundancyCheck      */ 
                         0x40,                  /* uint8_t chirpInvert                  */ 
                         sx1280ChipSelectPin,   /* uint8_t setupChipSelectPin           */
                         sx1280ResetPin,        /* uint8_t setupResetPin                */
                         sx1280BusyPin,         /* uint8_t setupBusyPin                 */
                         sx1280SpiNumber,       /* uint8_t setupSpiNumber               */
                         writeData );           /* uint8_t outboundMessage[ ]           */ 

            sx1280Tx( 0x1F,                 /* uint8_t power                    */
                      0xE0,                 /* uint8_t rampTime                 */
                      writeData,            /* uint8_t outboundMessage[ ]       */
                      0x40,                 /* uint8_t txIrq158                 */
                      0x01,                 /* uint8_t txIrq70                  */
                      0x02,                 /* uint8_t txPeriodBase             */
                      0x01,                 /* uint8_t txPeriodBaseCount158     */
                      0xF4,                 /* uint8_t txPeriodBaseCount70      */
                      sx1280ChipSelectPin,  /* uint8_t txChipSelectPin          */
                      sx1280BusyPin,        /* uint8_t txBusyPin                */
                      sx1280SpiNumber );    /* uint8_t txSpiNumber              */

            memset( writeData, 0, 256 );
        }

        vTaskDelay( 10 );
    }
}


/* ----------------------------- Pi Pico Onboard LED Task ------------------------------- */

void vSimpleLEDTask( void *pvParameters ){

    gpio_init( PICO_DEFAULT_LED_PIN );
    gpio_set_dir( PICO_DEFAULT_LED_PIN, GPIO_OUT );

    while( true ){

        gpio_put( PICO_DEFAULT_LED_PIN, 1 );
        vTaskDelay( 100 );
        gpio_put( PICO_DEFAULT_LED_PIN, 0 );
        vTaskDelay( 100 );
    }
}


/* --------------------------- Serial Monitor USB IO Task -------------------------------- */

/*  Task running usb serial input
    Sends task notification to vSx1280Task with pointer to 
        buffer holding the message to be sent from the sx1280 */
void vUsbIOTask( void *pvParameters ){

    uint8_t currentChar = 0x00; /* 8 bit integer to hold hex value from getchar() */

    uint8_t messageBuffer[ 256 ] = { 0 }; /* 8 bit pointer to hold outgoing message */

    /* 32 bit integer for placing input characters in the correct places in messageBuffer */
    uint32_t messageCounter = 0;

    /* Iterators */
    uint32_t i = 0;

    while( true ){

        vTaskDelay( 10 );

        currentChar = 0x00; /* Resetting the character read in from usb serial */

        /* Setting currentChar to the character being read in by getchar() */
        currentChar = getchar_timeout_us( 1000 );

        /* Checking currentChar for the error code 0xFF, isn't on the ascii keyboard, < 0x20,
           and isn't a new line character */
        if( currentChar == 0xFF || ( currentChar < 0x20 && currentChar != 0x0A ) ){
            currentChar = 0x00;
        }

        /* if character read isn't "\n", and 0x00, and less than 255 characters, index 1,
                keeping the 256th character, index 1, open for NULL, or 0x00, terminated string
           messageCounter is indexed from 0 to work with C arrays */
        if( currentChar != 0x0A && currentChar != 0x00 && messageCounter <= 254 ){ 
            /* Adding currentChar to messageBuffer at messageCounter */
            *( messageBuffer + messageCounter ) = currentChar;
            messageCounter = messageCounter + 1; /* Incrementing the value of messageCounter */
        }
        /* Checking if the character being read in is "\n" */
        else if( currentChar == 0x0A && messageCounter <= 254 ){ 

            /* Adding currentChar to the last cell in the pointer array */
            *( messageBuffer + 255 ) = 0x00; /* Adding 0x00 to NULL terminate input string */

            /* for( i = 0; i <= 255; i++ ){
                printf( "Typed Message: 0x%X %c %i\n", *( messageBuffer + i ), *( messageBuffer + i ), i );
            } */


            /* Checking messageBuffer to see if it begins with "SD:"
               Messages begining with "SD:" are SD commands to be sent to vSdCardTask */

            /* FreeRTOS function updating a receiving task’s notification value */
            xTaskNotify(
                xSx1280TaskHandle,                /* TaskHandle_t xTaskToNotify */ 
                ( uint32_t ) messageBuffer,       /* (int)&buffer[0] */
                eSetValueWithoutOverwrite );      /* eNotifyAction eAction */

            vTaskDelay( 2000 ); /* To allow other tasks time to grab the notification values */
            memset( messageBuffer, 0, 256 ); /* Reassign all values in messageBuffer to 0 */
            messageCounter = 0; /* Reassign messageCounter to 0 to restart input cycle */
        }
        else if( messageCounter > 254 ){ /* if the message is more than 255 characters */

            printf( "Message is too large, must be 255 characters or less! Try Again\n" );
            messageCounter = 0;
        }
    }
}


/* ------------------------------------- MAIN ------------------------------------------- */

int main( void ){

    stdio_init_all( );

    /* TaskFunction_t pvTaskCode */
    /* const char * const pcName */
    /* uint16_t usStackDepth in words not bytes, 32 bit stack width bytes = usStackDepth * 4 */
    /* void *pvParameters */
    /* UBaseType_t uxPriority */
    /*TaskHandle_t *pxCreatedTask */

    uint32_t status = xTaskCreate(
                    vSimpleLEDTask,  
                    "Green Led",    
                    1024,           
                    NULL,           
                    1,              
                    &xSimpleLEDTaskHandle ); 

    uint32_t ioStatus = xTaskCreate(
                    vUsbIOTask,             /* TaskFunction_t pvTaskCode    */
                    "Simple IO",            /* const char * const pcName    */
                    1024,                   /* uint16_t usStackDepth        */
                    NULL,                   /* void *pvParameters           */
                    1,                      /* UBaseType_t uxPriority       */
                    &xUsbIOTaskHandle );    /* TaskHandle_t *pxCreatedTask  */

    uint32_t sx1280Status = xTaskCreate(
                    vSx1280Task,
                    "sx1280",
                    1024, /* usStackDepth * 4 = stack in bytes, because pico is 32 bits wide */
                    NULL,
                    1,
                    &xSx1280TaskHandle );


    vTaskStartScheduler();

    while(true){

    }

    return 0;
}
