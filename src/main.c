
/*
 * Author: Chris Schorn
 * Open Lora Mesh Network
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




#include "../FreeRTOS-Kernel/include/FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "stdio.h"
#include "stdlib.h"
#include "hardware/gpio.h"
#include "limits.h"

static TaskHandle_t xSimpleLEDTaskHandle = NULL;
static TaskHandle_t xUsbIOTaskHandle = NULL;
static TaskHandle_t xSx1280TaskHandle = NULL;

/*      Defining SX1280 Hexadecimel Commands with Opcodes       */

/*  Retrieve the transceiver status
    Cannot be the first command sent over the interface
    Is not strictly necessary for SPI b/c device returns status info
        also on cammand bytes
    params( void ) return( status ) */
#define GETSTATUS 0xC0

/*  Writes a block of bytes in a data memory space starting 
        at a specific address.
    params( address[15:8], address[7:0], data[0:n] ) return( void ) */
#define WRITEREGISTER 0x18 

/*  Reads a block of data starting at a given address
    The host must send a NOP after the address to receive data!!!!!!!!!!!!!!!
    params( address[15:8], address[7:0] ) return( data[0:n-1] )  */
#define READREGISTER 0x19

/*  Write the data payload to be transmitted
    Data sent in hex, most likely translated using ascii for text
    Audio data tbd
    params( offset, data[0:n] ) return( void ) */
#define WRITEBUFFER 0x1A

/*  Function allows reading (n-3) bytes of payload received 
        starting at offset.
    Data received in hex, most likely translated using ascii for text
    params( offset ) return( data[0:n-1] ) */
#define READBUFFER 0x1B

/*  Set transceiver to Sleep mode
    Lowest current consumption
    params( sleepConfig ) return( void )
    sleepConfig[7:4] unused 
    sleepConfig[1] 1: Data buffer in retention mode
    sleepConfig[0] 0: Data Ram is flushed 1: Data Ram in retention  */
#define SETSLEEP 0x84

/*  Set the device in either STDBY_RC or STDBY_XOSC mode
    Used to configure the transceiver 
    Intermediate levels of power consumption
    params( standbyConfig )
    standbyConfig 0: STDBY_RC mode 1: STDBY_XOSC mode
    return( void ) */
#define SETSTANDBY 0x80

/*  Set the device in Frequency Synthesizer mode
    PLL(Phase Locked Loop) is locked to the carrier frequency
    For test purposes of PLL
    params( void ) return( void ) */
#define SETFS 0xC1

/*  Set the device in Transmit mode
    Clear IRQ status before using this command
    Timeout = periodBase * periodBaseCount
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out Other: Time out active
    return( void ) */
#define SETTX 0x83

/*  Set the device in Receiver mode
    Timeout = periodBase * periodBaseCount 
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out 
                          0xFFFF: Rx Continuous mode, multi-packet Rx
                          Other: Time out active
    return( void ) */
#define SETRX 0x82

/*  Set transceiver in sniff mode
    setLongPreamble must be issued prior to setRxDutyCycle
    RxPeriod = periodBase * rxPeriodBaseCount
    SleepPeriod = periodBase * sleepPeriodBaseCount
    params( rxPeriodBase, rxPeriodBaseCount[15:8], 
        rxPeriodBaseCount[7:0], sleepPeriodBase,
        sleepPeriodBaseCount[15:8], sleepPeriodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out 
                          Other: Device will stay in Rx Mode for 
                                 RxPeriod and return 
                                 to Sleep Mode for SleepPeriod
    return( void ) */
#define SETRXDUTYCYCLE 0x94

/*  Set transceiver to Channel Activity Detection mode
    Device searches for a Lora signal
    Returns to STDBY_RC mode when finished
    Always sends CadDone IRQ, sends CadDetected IRQ if signal found
    Useful in Listen before Talk Applications
    params( void ) return( void ) */
#define SETCAD 0xC5

/*  Test command to generate a Continuous Wave (RF tone)
    Frequency and power settings from setRfFrequency, and setTxParams
    params( void ) return( void ) */
#define SETTXCONTINUOUSWAVE 0xD1

/*  Test command to generate infinite sequence pf symbol 0 in Lora
    params( void ) return( void ) */
#define SETTXCONTNIOUSPREAMBLE 0xD2

/*  Sets the transceiver radio frame
    MUST BE THE FIRST IN A RADIO CONFIGURATION SEQUENCE!!!!!!!
    params( packetType )
    packetType[8:0] 0x00: GFSK
                    0x01: Lora 
                    0x02: Ranging 
                    0x03: FLRC
                    0x04: BLE
    return( void ) */
#define SETPACKETTYPE 0x8A

/*  Returns the current operation packet type of the radio
    packetType probly comes in same format as setPacketType
    params( void ) return( packetType ) */
#define GETPACKETTYPE 0x03

/*  Set the frequency of the RF frequency mode
    rfFrequency sets the number of PLL steps
    Frf = ( Fxosc/2^18 ) * rfFrequency
        Gives frequency in kilohertz
    params( rfFrequency[23:16], rfFrequency[15:8], rfFrequency[7:0] )
    return( void ) */
#define SETRFFREQUENCY 0x86

/*  Sets the Tx output power and the Tx ramp time
    params( power, rampTime )
    power  Pout[dB] = -18 + power i.e. -18 + 0x1F(31) = 13dbm
    rampTime 0x00: 2um 0x20: 4us 0x40: 5us 0x60: 8us 
            0x80: 10us 0xA0: 12us 0xC0: 16us 0xE0: 20us
    return( void ) */
#define SETTXPARAMS 0x8E

/*  Sets number of symbols which Channel Activity Detected operates
    For symbols 1 & 2, there are higher risks of false detection.
    params( cadSymbolNum )
    cadSymbolNum 0x00: 1 symbol
                 0x20: 2 symbols
                 0x40: 4 symbols
                 0x60: 8 symbols
                 0x80: 16 symbols
    return( void ) */
#define SETCADPARAMS 0x88

/*  Fixes the base address for the packet handing operation
        in Tx and Rx mode for all packet types
    params( txBaseAddress, rxBaseAddress ) return( void ) */
#define SETBUFFERBASEADDRESS 0x8F

/*  Configure the modulation parameters of the radio
    Params passed will be interpreted depending on the frame type
    Frame Type 
    params( modParam1, modParam2, modParam3 )
    modParam1 BLE: BitrateBandwidth   Lora/Ranging: Spreading Factor
    modParam2 BLE: ModulationIndex    Lora/Ranging: Bandwith
    modParam3 BLE: ModulationShaping  Lora & Ranging: Coding Rate
    return( void ) */
#define SETMODULATIONPARAMS 0x8B

/*  Set the parameters of the packet handling block
    params( packetParam1, packetParam2, packetParam3, packetParam4,
        packetParam5, packetParam6, packetParam7 )
    packetParam1 BLE: ConnectionState Lora/Ranging: Preambl Length
    packetParam2 BLE: CrcLength       Lora/Ranging: Header Type
    packetParam3 BLE: BleTestPayload  Lora/Ranging: PayloadLength
    packetParam4 BLE: Whitening       Lora/Ranging: CRC
    packetParam5 BLE: Not Used     Lora/Ranging: InvertIQ/chirp invert
    packetParam6 BLE: Not Used        Lora/Ranging: Not Used
    packetParam7 BLE: Not Used        Lora/Ranging: not Used
    return( void ) */ 
#define SETPACKETPARAMS 0x8C

/*  Returns the length of the last received packet 
        and the address of the first byte received
    In Lora packet type, 0x00 always returned for rxPayloadLength.
        Instead read register 0x901, for Lora payload length
    params( void ) return( payloadLength, rxBufferOffset ) */
#define GETRXBUFFERSTATUS 0x17

/*  Retrieve information about the last received packet
    rssiSync: RSSI value latched upon  detection of sync address.
        Actual signal power is –(rssiSync)/2dBm
    snr: Estimation of Signal to Noise Ratio on last packet received. 
        In two’s compliment format multiplied by 4. 
        Actual Signal to Noise Ratio(SNR) is (snr)/4dB. If SNR ≤ 0, 
        RSSI_{packet, real} = RSSI_{packet,measured} – SNR_{measured}
    params( void ) 
    return( packetStatus[39:32], packetStatus[31:24],
        packetStatus[23:16], packetStatus[15:8], packetStatus[7:0] ) 
    packetStatus[7:0]   BLE: RFU        Lora/Ranging: rssiSync
    packetStatus[15:8]  BLE: rssiSync   Lora/Ranging: snr
    packetStatus[16:23] BLE: errors     Lora/Ranging: -
    packetStatus[24:31] BLE: status     Lora/Ranging: -
    packetStatus[32:39] BLE: sync       Lora/Ranging: - */
#define GETPACKETSTATUS 0x1D

/*  Returns instantaneous RSSI value during reception of a  packet
    rssilnst: Signal power is (–rssiInst)/2dBm
    params( void ) return( rssilnst ) */
#define GETRSSILNST 0x1F

/*  Enable IRQs and to route IRQs to DIO pins
    An interrupt is flagged in IRQ register if the corresponding 
        bit in flag register is set
    irqMask[15:0] set which IRQ's are active, 
        pg 95 in sx1280 manual has IRQ table
    dioMasks active bits correspond to the active bits irqMasks
        If coresponding bits are both on, IRQ is sent through that DIO
    params( irqMask[15:8], irqMask[7:0], dio1Mask[15:8],dio1Mask[7:0],
    dio2Mask[15:8], dio2Mask[7:0], dio3Mask[15:8], dio3Mask[7:0] )
    return( void ) */
#define SETDIOIRQPARAMS 0x8D

/*  Returns the value of the IRQ register
    IRQ register is only interacatable through these commands
    params( void ) return( irqStatus[15:8], irqStatus[7:0] ) */
#define GETIRQSTATUS 0x15

/*  Clears an IRQ flag in IRQ register
    Corresponding bits in irqMask will clear flag of that IRQ
    params( irqMask[15:8], irqMask[7:0] ) return( void ) */
#define CLRIRQSTATUS 0x97

/*  Why Kansas but no arkansas
    Havent found in book
    params( regulatorMode ) return( void ) */
#define SETREGULATORMODE 0x96

/*  Havent found in book
    params( void ) return( void ) */
#define SETSAVECONTEXT 0xD5

/*  Set the state following a Rx or Tx operation is FS, not STDBY
    Reduces switching time between consecutive Rx and/or Tx operations
    params( 0x00=disable or 0x01=enable ) return( void ) */
#define SETAUTOFS 0x9E

/*  Allows transceiver to send a packet at a user programmable time 
        after the end of a packet reception
    Must be issued in STDBY_RC mode
    TxDelay = time + 33us(time needed for transceiver to switch modes)
    params( time[15:8], time[7:0] ) return( void ) */
#define SETAUTOTX 0x98

/*  Set the transceiver into Long Preamble mode
    RxDutyCycle is modified so that if a preamble is detected,
         the Rx window is extended by SleepPeriod + 2 * RxPeriod
    params( enable )
    enable 0x00: disable 0x01: enable
    return( void ) */
#define SETLONGPREAMBLE 0x9B

/* #define SETUARTSPEED 0x9D, using spi not uart interface */

/*  params( 0x00=slave or 0x01=master ) return( void ) */
#define SETRANGINGROLE 0xA3

/* params( 0x00=slave or 0x01=master ) return( void ) */
#define SETADVANCEDRANGING 0x9A


/* 
                General Structure Notes 

    In main, setup tasks and run vTaskStartScheduler.
    In tasks, setup hardware before while(true), and
        run data transfer with hardware in while(true).
    Might have to figure out how to pass parameters using
        pvParameter.
    Contemplating on invoking second core if first core
        is busy when multiple tasks are ready
    Using task notifications, I am taking the uint32_t ulValue
        and convert it to a 32 bit pointer address
*/


/*  Driving the chip select pin low 
    Transactions with sx1280 start with chip select low */
static inline void sx1280Select(){

    asm volatile ("nop \n nop \n nop");/* Find out what it does */
    gpio_put( 13, 0 );
    asm volatile ("nop \n nop \n nop");

}


/*  Driving the chip select pin high 
    Transactions with sx1280 end with chip select high */
static inline void sx1280Deselect(){

     asm volatile ("nop \n nop \n nop");
     gpio_put( 13, 1 );
     asm volatile ("nop \n nop \n nop");

}


/*  Task setting up and running sx1280 
    Initializing SPI interface on rp2040
    Using GP10-GP13
        GP10: SPI1 SCK
        GP11: SPI1 Tx
        GP12: SPI1 Rx
        GP13: SPI1 CSn */
void vSx1280Task( void *pvParameters ){

    /* Instantiating pointer variables for dynamic arrays */
    uint8_t *writeData;
    uint8_t *readData;

    /* Instantiating pointer for address received from task notification
       Used to get data about the messages sent from usb to add to 
            2D array for storing until sent, then freed here */
    uint32_t *taskNotificationFromUSB;

    /* Instantiating pointer to hold the addresses of messages
            waiting to be sent out by sx1280 */
    uint32_t *messageStorageTillUse;
    messageStorageTillUse = ( uint32_t * ) malloc( 10*sizeof( uint32_t ) );
    /* Looping through the messageStorageTillUse to instantiate each memory block as 0
       0 will be placed in a memory block when it is useable */
    for( uint32_t i = 0; i <= 9; i++ ){
        *( messageStorageTillUse + i ) = 0;
    }
    /* 8 bit integer holding a boolean value
       If value is 1 this asserts that messageStorageTillUse is empty
       If value is 0 this asserts that messageStorageTillUse is not empty, signifying 
            there is at least one message in messageStorageTillUse and a message is ready to send  */
    uint8_t isMessageStorageTillUseEmpty = 1;
    /*uint32_t messageStorageCounter = 0;*/

    /* Holder variable used in rx operation
       Used to hold the size of any newly received message on the sx1280's buffer
       Needed because of the inability to see a pointer array's size while it is empty
            A newline character is usually used in this code as an end of array signifier */
    uint8_t sizeOfMessageInBuffer = 0;
    
    /* Setting up Busy Pin */

    /* void gpio_init (uint gpio)
       Initializing GP22 to input pin for Busy */
    gpio_init( 22 );
    /* static void gpio_set_dir (uint gpio, bool out)
       Setting GP22 direction to output, True is out False is in
       Use gpio_get( uint gpio ) to get the state of a gpio */
    gpio_set_dir( 22, 0 );

    /* Setting up Reset Pin */

    /* Initializing GP9 to output pin for Reset */
    gpio_init( 21 );
    /* Setting GP9 direction to output, True is out False is in */
    gpio_set_dir( 21, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving GP9 High
       A resit is initiated by driving Reset Low */
    gpio_put( 21, 1 );

    /* Setting up SPI1 Interface */

    /* Inializing spi0 at 1MHz */
    spi_init( spi1, 1000000 );

    /* void gpio_set_function (uint gpio, enum gpio_function fn)
       Setting  GP10-GP12 as SCK, TX, and RX respectively */
    gpio_set_function( 10, GPIO_FUNC_SPI );
    gpio_set_function( 11, GPIO_FUNC_SPI );
    gpio_set_function( 12, GPIO_FUNC_SPI );

    /* Initializing GP13 to output pin for Chip Select */
    gpio_init( 13 );
    /* Setting GP13 direction to output, True is out False is in */
    gpio_set_dir( 13, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving GP13 High
       A data transfer is started by driving Chip Select low */
    gpio_put( 13, 1 );

    while( true ){

        /* Format For SPI Communication with sx1280 */

        /* Allocate memory for a dynamic array to 
                send data to sx1280 (writeData) */
        /* Allocate memory for a dynamic array to 
                receive data from sx1280 (readData) */
        /* Add Opcode to first memory byte in writeData */
        /* Add data in hexadecimal to second, third, etc. 
                memory byte in writeData for sets, NOP for gets  */
        /* int spi_write_blocking( spi_inst_t *spi, 
                                   const uint8_t *src, 
                                   size_t len ) */
        /* Deallocate memory for writeData and/or readData */

        /* Putting this so other tasks run */
        /*vTaskDelay( 100 );*/

        /* Allocating 4 bytes (32 bits) to take in the address
           containing data from vUsbIOTask */
        taskNotificationFromUSB = ( uint32_t * ) malloc( 1*sizeof( uint32_t ) );

        xTaskNotifyWait(
                0xffffffff,               /* uint32_t ulBitsToClearOnEntry */
                0,                        /* uint32_t ulBitsToClearOnExit */
                taskNotificationFromUSB,  /* uint32_t *pulNotificationValue */
                100 );                    /* TickType_t xTicksToWait */
 
        /*if( *( taskNotificationFromUSB ) != 0 ){
            printf("taskNotificationFromUSB = %X\n", *( taskNotificationFromUSB ));
        }*/

        /* if( *( taskNotificationFromUSB ) != 0 ){
            *( messageStorageTillUse + messageStorageCounter ) = *( taskNotificationFromUSB );*/
            /*printf("messageStorage = %X\n", *( messageStorageTillUse + messageStorageCounter ) );*/
            /*messageStorageCounter++;
            messageStorageTillUse = ( uint8_t * ) realloc( messageBuffer, ( messageCounter+1 )*sizeof( uint8_t ) );
        }*/


        /*if( messageStorageCounter == 4 ){
            for(uint32_t i = 0; i<messageStorageCounter; i++){
                printf( "%X\n",  *( messageStorageTillUse + i ) );
                uint8_t j = 0;
                uint32_t k = 0;
                while( j != 0x0A ){
                    j = *( ( uint8_t * ) *( messageStorageTillUse + i ) + k );
                    printf( "%c", j );
                    k++;
                }
                printf("\n");
            }
            messageStorageCounter = 0;
            free( messageStorageTillUse );
            messageStorageTillUse = ( uint8_t * ) malloc( 1 * sizeof( uint8_t ) );
        }*/

        /* Loop checking to see if messageStorageTillUse is empty
           If there is something in messageStorageTillUse then isMessageStorageTillUseEmpty
                will be changed to 0 (False) */
        for( uint32_t i = 0; i <= 9 ; i++ ){

            if( *( messageStorageTillUse + i ) == 0 && *( taskNotificationFromUSB ) != 0 ){
                *( messageStorageTillUse + i ) = *( taskNotificationFromUSB );
                isMessageStorageTillUseEmpty = 0; /* asserting that messageStorageTillUse is no longer empty */ 
            }
            /* May need an else if later for if the messageStorageTillUse gets full
               If the array gets full there is a problem
                    If messageStorageTillUse becomes full it would most likely mean that
                        the owner of the radio is receiveing more messages than can be handled
                        or the mesh is requesting more data throughput than can be handled
                        and is causing a backup in radio processor memory */
        }

        free( taskNotificationFromUSB );

        /* Driving pin 21 low to reset the sx1280 */
        gpio_put( 21, 0 );
        asm volatile ("nop \n nop \n nop");
        gpio_put( 21, 1 );

        /* Waiting till the busy pin is driven low 
           So the sx1280 is not sent a command while busy
                Because it wont receive the  */
        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after reset\n");
        }

        /* Setting sx1280 mode to STDBY_RC */
        writeData = ( uint8_t * ) malloc( 2*sizeof( uint8_t ) );
        *( writeData ) = SETSTANDBY;
        *( writeData + 1 ) = 0x00; /* Setting to STDBY_RC Mode 0x01 is STDBY_XOSC Mode */
        sx1280Select();
        /* int spi_write_blocking( spi_inst_t *spi, const uint8_t *src, size_t len ) */
        spi_write_blocking( spi1, writeData, 2*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );

        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after SETSTANDBY\n");
        }

        /* Setting sx1280 Packet Type to Lora */
        writeData = ( uint8_t * ) malloc( 2*sizeof( uint8_t ) );
        *( writeData ) = SETPACKETTYPE;
        *( writeData + 1 ) = 0x01;
        sx1280Select();
        spi_write_blocking( spi1, writeData, 2*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );

        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after SETPACKETTYPE\n");
        }

        /* Setting RF Frequency
           Following sx1280 Guide SPI Example here to 2.4GHz */
        writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
        *( writeData ) = SETRFFREQUENCY;
        *( writeData + 1 ) = 0xB8; /* rfFrequency[23:16] */
        *( writeData + 2 ) = 0x9D; /* rfFrequency[15:8] */
        *( writeData + 3 ) = 0x89; /* rfFrequency[7:0] */
        sx1280Select();
        spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );

        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after SETRFFREQUENCY\n");
        }

        /* Setting Tx and Rx Buffer Base Addresses
           Putting both at 0 since messages can be size of buffer */
        writeData = ( uint8_t * ) malloc( 3*sizeof( uint8_t ) );
        *( writeData ) = SETBUFFERBASEADDRESS;
        *( writeData + 1 ) = 0x00;
        *( writeData + 2 ) = 0x00;
        sx1280Select();
        spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );

        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after SETBUFFERBASEADDRESS\n");
        }

        /* Setting the Modulation Params
           Following sx1280 Guide SPI Exampes here */
        writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
        *( writeData ) = SETMODULATIONPARAMS;
        *( writeData + 1 ) = 0x70; /* Spreading Factor 7 (SF7) */
        *( writeData + 2 ) = 0x0A; /* Bandwidth 1600 */
        *( writeData + 3 ) = 0x01; /* Coding Rate 4/5 */
        sx1280Select();
        spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );
        /* 0x37 Must be written to register 0x0925 for SF7 */
        writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
        *( writeData ) = WRITEREGISTER;
        *( writeData + 1 ) = 0x09;
        *( writeData + 2 ) = 0x25;
        *( writeData + 3 ) = 0x37;
        sx1280Select();
        spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );
        /* 0x01 must be written to register 0x093C */
        writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
        *( writeData ) = WRITEREGISTER;
        *( writeData + 1 ) = 0x09;
        *( writeData + 2 ) = 0x3C;
        *( writeData + 3 ) = 0x01;
        sx1280Select();
        spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );

        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after SETMODULATIONPARAMS\n");
        }

        /* Setting Packet Params
           Following the sx1280 Guide SPI Example */
        writeData = ( uint8_t * ) malloc( 8*sizeof( uint8_t ) );
        *( writeData ) = SETPACKETPARAMS;
        *( writeData + 1 ) = 0x0C; /* Preamble Length */
        *( writeData + 2 ) = 0x00; /* Header Type */
        *( writeData + 3 ) = 0x04; /* Payload Length */
        *( writeData + 4 ) = 0x20; /* Cyclical Redundancy Check */
        *( writeData + 5 ) = 0x40; /* Invert IQ/chirp invert */
        *( writeData + 6 ) = 0x00; /* Not Used */
        *( writeData + 7 ) = 0x00; /* Not Used */
        sx1280Select();
        spi_write_blocking( spi1, writeData, 8*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );

        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after SETPACKETPARAMS\n");
        }

        /* Testing connecting from pico to sx1280 by writing to
                and reading from buffer
           Working, output should be "status status FF" and is */

        /*writeData = ( uint8_t * ) malloc( 3*sizeof( uint8_t ) );
        *( writeData ) = WRITEBUFFER;
        *( writeData + 1 ) = 0x00;
        *( writeData + 2 ) = 0xFF;
        sx1280Select();
        spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
        sx1280Deselect();
        free( writeData );*/

        /* Must use two NOP's for reads because data is
                returned beginning on the second NOP */
        /*readData = ( uint8_t * ) malloc( 5*sizeof( uint8_t ) );
        writeData = ( uint8_t * ) malloc( 5*sizeof( uint8_t ) );
        *( writeData ) = READBUFFER;
        *( writeData + 1 ) = 0x00;
        *( writeData + 2 ) = 0x00;
        *( writeData + 3 ) = 0x00;
        *( writeData + 4 ) = 0x00;
        sx1280Select();
        spi_write_read_blocking( spi1, writeData, readData, 5*sizeof( uint8_t ) );
        sx1280Deselect(); 
        printf( "%X %X %X %X %X\n", *( readData ), *( readData + 1 ), *( readData + 2 ), *( readData + 3 ), *( readData + 4 ) );
        free( writeData );
        free( readData );*/

        /* Mesh below */

         if( isMessageStorageTillUseEmpty == 1 ){ /*|| neighbors unavailable to receive messages ){*/

            /* Setting the tx parameters necessary for sending a message */
            writeData = ( uint8_t * ) malloc( 3*sizeof( uint8_t ) );
            *( writeData ) =  SETTXPARAMS;
            *( writeData + 1 ) = 0x1F; // power, 13dbm
            *( writeData + 2 ) = 0xE0; // rampTime, 20us
            sx1280Select();
            spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
            sx1280Deselect();
            free( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after SETTXPARAMS\n");
            }

            //send message with radio's public address and general ready to receive status
            //uint8_t messageSizeCount = 0; //can't access size of pointer array, must keep count
            //while( ( uint8_t * ) *( *( messageStorageTillUse + messageStorageCounter ) + messageSizeCount ) != 0x0A ) for accessing each of the 4 messages in the current messageStorageTillUse memory scheme, and knowing the size of each message as they are being used for dynamic memory allocation

            /* writing the general ready to recieve message to the sx1280 message buffer */
            writeData = ( uint8_t * ) malloc( 6*sizeof( uint8_t ) );
            *( writeData ) = WRITEBUFFER;
            *( writeData + 1 ) = 0x00;
            *( writeData + 2 ) = 0x52; /* RTRG (Ready To Recieve General) */
            *( writeData + 3 ) = 0x54;
            *( writeData + 4 ) = 0x52;
            *( writeData + 5 ) = 0x47;
            sx1280Select();
            spi_write_blocking( spi1, writeData, 6*sizeof( uint8_t ) );
            sx1280Deselect();
            free( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after tx WRITEBUFFER\n");
            }

            /* seems possible to use GETIRQSTATUS over spi instead of DIO pins
               can loop GETIRQSTATUS with a vTaskDelay(n) until Tx done is flagged */

            /* setting IRQ parameters for the general ready to receive outgoing message */
            writeData = ( uint8_t * ) malloc( 9*sizeof( uint8_t ) );
            *( writeData ) = SETDIOIRQPARAMS;
            *( writeData + 1 ) = 0x40; /* IRQ Mask for bits 15:8 of IRQ register, allowing RxTxTimeout */
            *( writeData + 2 ) = 0x01; /* IRQ Mask for bits 7:0 of IRQ register, allowing TxDone */
            *( writeData + 3 ) = 0x00; /* setting DIO 1 Mask bits 15:8 to 0 */
            *( writeData + 4 ) = 0x00; /* setting DIO 1 Mask bits 7:0 to 0 */
            *( writeData + 5 ) = 0x00; /* setting DIO 2 Mask bits 15:8 to 0 */
            *( writeData + 6 ) = 0x00; /* setting DIO 2 Mask bits 7:0 to 0 */
            *( writeData + 7 ) = 0x00; /* setting DIO 3 Mask bits 15:8 to 0 */
            *( writeData + 8 ) = 0x00; /* setting DIO 3 Mask bits 7:0 to 0 */
            sx1280Select();
            spi_write_blocking( spi1, writeData, 9*sizeof( uint8_t ) );
            sx1280Deselect();
            free( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after tx SETDIOIRQPARAMS\n");
            }

            /* setting sx1280 to transmit mode to send the message in sx1280's message buffer */
            writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
            *( writeData ) = SETTX;
            *( writeData + 1 ) = 0x02; /* setting periodBase to 1ms, RTC step */
            *( writeData + 2 ) = 0x01; /* setting periodBaseCount[15:0] to 500 */
            *( writeData + 3 ) = 0xF4; /* timeout is periodBase * periodBaseCount */
            sx1280Select();
            spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
            sx1280Deselect();
            free( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after tx SETTX\n");
            }


            /* Checking the IRQ register through the spi connection
               Looping over GETIRQSTATUS, with vTaskDelay, till the TxDone bit is positive
               Will most likely put this loop onto a pio state machine to save power, and processing time */
            for( uint32_t i = 0; i <= 100; i++){

                vTaskDelay( 50 );

                /* writeData = ( uint8_t * ) malloc( 3*sizeof( uint8_t ) );
                readData = ( uint8_t * ) malloc( 3*sizeof( uint8_t ) );
                *( writeData ) = GETPACKETTYPE;
                *( writeData + 1 ) = 0x00;
                *( writeData + 2 ) = 0x00;
                sx1280Select();
                spi_write_read_blocking( spi1, writeData, readData, 3*sizeof( uint8_t ) );
                sx1280Deselect();
                printf("Packet Type: %X\n", *( readData + 2 ) );
                free( writeData );
                free( readData );

                while( gpio_get( 22 ) == 1 ){
                    vTaskDelay( 10 );
                  printf("Busy after tx GETPACKETTYPE\n");
                } */

                writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
                readData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
                *( writeData ) = GETIRQSTATUS;
                *( writeData + 1 ) = 0x00;
                *( writeData + 2 ) = 0x00;
                *( writeData + 3 ) = 0x00;
                sx1280Select();
                spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
                sx1280Deselect();
 
                printf("IRQ Check: %X\n", *( readData + 3 ) );

                while( gpio_get( 22 ) == 1 ){
                    vTaskDelay( 10 );
                    printf("Busy after tx GETIRQSTATUS\n");
                }

                /* Checking bits [7:0] to see if the TxDone bit in the IRQ register is high
                   Doing a bitwise 'and' operation with 0x01 to mask the rest of the bits in the IRQ register,
                        giving a clear indication that a message has been sent
                    Bits [15:8] would be in  *( readData + */
                if( ( *( readData + 3 ) & 0x01 ) == 0x01 ){ /* GETIRQSTATUS TxDone == 1 */

                    /* Once message is sent move on to rx mode to upkeep network 
                       test this to see if the message is actually being sent, the loop skip issue 
                            in rx part may not be an actual issue printf is weird on rp2040 */
                    printf("IRQ: %X\n", *( readData + 3 ) );
                    break;

                }

                free( writeData );
                free( readData );

            }

            /* clearing the IRQ register, reseting IRQ Mask bits to 0 */
            writeData = ( uint8_t * ) malloc( 3*sizeof( uint8_t ) );
            *( writeData ) = CLRIRQSTATUS;
            *( writeData + 1 ) = 0xFF; /* clearing bits 15:8 of IRQ mask */
            *( writeData + 2 ) = 0xFF; /* clearing bits 7:0 of IRQ mask */
            sx1280Select();
            spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
            sx1280Deselect();
            free( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after tx CLRIRQSTATUS\n");
            }

            /* May have to reset sx1280 to change it from tx mode to rx mode
               Don't seem to need to */

            /* set sx1280 to rx mode and listen for a message
                if a general ready to receive is received then add the public address to a 
                    list of neighbors. when a ready to receive has not been heard from a 
                    certain address for a certain amount of time the address is removed 
                    from neighbors list
                if a ready to send message is received, change to tx mode and send a specified
                    ready to receive. when message is received, add it to messageStorageTillUse */

            /* setting IRQ parameters for the general rx mode */
            writeData = ( uint8_t * ) malloc( 9*sizeof( uint8_t ) );
            *( writeData ) = SETDIOIRQPARAMS;
            *( writeData + 1 ) = 0x40; /* IRQ Mask for bits 15:8 of IRQ register, allowing RxTxTimeout */
            *( writeData + 2 ) = 0x7E; /* IRQ Mask for bits 7:0 of IRQ register, allowing RxDone, SyncWordValid, SyncWordError, HeaderValid, HeaderError, CrcError */
            *( writeData + 3 ) = 0x00; /* setting DIO 1 Mask bits 15:8 to 0 */
            *( writeData + 4 ) = 0x00; /* setting DIO 1 Mask bits 7:0 to 0 */
            *( writeData + 5 ) = 0x00; /* setting DIO 2 Mask bits 15:8 to 0 */
            *( writeData + 6 ) = 0x00; /* setting DIO 2 Mask bits 7:0 to 0 */
            *( writeData + 7 ) = 0x00; /* setting DIO 3 Mask bits 15:8 to 0 */
            *( writeData + 8 ) = 0x00; /* setting DIO 3 Mask bits 7:0 to 0 */
            sx1280Select();
            spi_write_blocking( spi1, writeData, 9*sizeof( uint8_t ) );
            sx1280Deselect();
            free( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx SETDIOIRQPARAMS\n");
            }

            /* setting sx1280 to Rx mode */
            writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
            *( writeData ) = SETRX;
            *( writeData + 1 ) = 0x02; /*  setting the RTC step to 1ms */
            /* setting Rx mode to continuous, so multiple messages can be received
               If a ready to receive general message is received it updates the list of neighbors, and continue Rx
               If a ready to send message is received update neighbors list and switch to message tansfer sequence */
            *( writeData + 2 ) = 0xFF;
            *( writeData + 3 ) = 0xFF;
            sx1280Select();
            spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
            sx1280Deselect();
            free( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after SETRX\n");
            }

            /* probly going to have to loop over getrxbufferstatus to check new messages and if 
                    the payload length is greater than 0 the message buffer will be read and the 
                    message contents will either be added to the list of neighbors
                    or used to engage in a message transfer, once a message transfer is initiated
               can also use the GETIRQSTATUS command, the RxDone flag will be set to 1 when
                    a new message has been received 
                    should probably do it this way or a message paylod will have to be checked
                        each time one is received to see if it is new or not
               Assuming that the rx buffer is cleared when each new message is received */


            /* Looping 10 times over rx mode
               Each loop has a 10 clock-tick delay allowing other tasks to run
               10 is arbitrarily picked, just needed to be able to exit the loop sometime */
            for( uint8_t i = 0; i <= 100; i++ ){ /* we want to keep listening generally */

                printf("Listening: %i\n", i );
                vTaskDelay( 50 );

                /* using GETIRQSTATUS to check if there is a new message in the rx buffer */
                writeData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
                readData = ( uint8_t * ) malloc( 4*sizeof( uint8_t ) );
                *( writeData ) = GETIRQSTATUS;
                *( writeData + 1 ) = 0x00;
                *( writeData + 2 ) = 0x00;
                *( writeData + 3 ) = 0x00;
                sx1280Select();
                spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
                sx1280Deselect();
                /* not freeing allocated memory for writeData and readData here because readData is
                        needed in the following if statement to see if a new message was received */

                /* checking to see if the RxDone bit in the IRQ register is high
                   Doing a bitwise 'and' operation with 0x02 to mask the rest of the bits in the IRQ register,
                        giving a clear indication that a message has been received */
                if( ( *( readData + 3 ) & 0x02 ) == 0x02 ){ /* GETIRQSTATUS RxDone == 1 */
 
                    free( writeData );
                    free( readData );

                    while( gpio_get( 22 ) == 1 ){
                        vTaskDelay( 10 );
                        printf("Busy after rx GETIRQSTATUS\n");
                    }

                    /* see what the message is and decide what to do with it */

                    /* using GETPACKETSTATUS which returns the rssiSync, and the snr
                       not using this at the moment but it's in the sx1280 Documentation for Rx operation
                            pretty sure it's used to see if the received message is useable or not */
                    writeData = ( uint8_t * ) malloc( 7*sizeof( uint8_t ) );
                    readData = ( uint8_t * ) malloc( 7*sizeof( uint8_t ) );
                    *( writeData ) = GETPACKETSTATUS;
                    *( writeData + 1 ) = 0x00;
                    *( writeData + 2 ) = 0x00;
                    *( writeData + 3 ) = 0x00;
                    *( writeData + 4 ) = 0x00;
                    *( writeData + 5 ) = 0x00;
                    *( writeData + 6 ) = 0x00;
                    sx1280Select();
                    spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
                    sx1280Deselect();
                    free( writeData );
                    free( readData );

                    while( gpio_get( 22 ) == 1 ){
                        vTaskDelay( 10 );
                        printf("Busy after rx GETPACKETSTATUS\n");
                    }

                    /* clearing the IRQ register on the sx1280
                       Not sure why it's done here in the rx operation in sx1280 documentation
                            but I'm going to follow it */
                    writeData = ( uint8_t * ) malloc( 3*sizeof( uint8_t ) );
                    *( writeData ) = CLRIRQSTATUS;
                    *( writeData + 1 ) = 0xFF;
                    *( writeData + 2 ) = 0xFF;
                    sx1280Select();
                    spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
                    sx1280Deselect();
                    free( writeData );

                    while( gpio_get( 22 ) == 1 ){
                        vTaskDelay( 10 );
                        printf("Busy after rx CLRIRQSTATUS\n");
                    }

                    /* Getting the length of the newly received message by reading register 0x901
                            on the sx1280
                            Using GETRXBUFFERSTATUS only works if there is no header in the Lora message */
                    writeData = ( uint8_t * ) malloc( 5*sizeof( uint8_t ) );
                    readData = ( uint8_t * ) malloc( 5*sizeof(uint8_t ) );
                    *( writeData ) = READREGISTER;
                    *( writeData + 1 ) = 0x09;
                    *( writeData + 2 ) = 0x01;
                    *( writeData + 3 ) = 0x00;
                    *( writeData + 4 ) = 0x00;
                    sx1280Select();
                    spi_write_read_blocking( spi1, writeData, readData, 5*sizeof( uint8_t ) );
                    sx1280Deselect();
                    /* using holder variable to grab the size of the message to cleanly allocate
                            the precise amount of memory needed to handle the incoming message */
                    sizeOfMessageInBuffer = *( readData + 4 );
                    free( writeData );
                    free( readData );
                    
                    while( gpio_get( 22 ) == 1 ){
                        vTaskDelay( 10 );
                        printf("Busy after rx READREGISTER\n");
                    }

                    /* Reading the message buffer of the sx1280
                       Allocating the size of the message in the sx1280 buffer plus 3 because over 
                            spi you must send an opcode, the buffer offset, and a nop to receive the
                            payload on the buffer */
                    writeData = ( uint8_t * ) malloc( ( sizeOfMessageInBuffer + 3 )*sizeof( uint8_t ) );
                    readData = ( uint8_t * ) malloc( ( sizeOfMessageInBuffer + 3 )*sizeof( uint8_t ) );
                    *( writeData ) = READBUFFER;
                    *( writeData + 1 ) = 0x00; /* sx1280 message buffer offset */
                    *( writeData + 2 ) = 0x00; /* sending first nop */
                    /* Looping through the rest of writeData to add nops for retreiving data in the buffer
                       Adding 3 because the nop assignment begins at the 4th element in the pointer array
                            or *( writeData + 3 ), indexed from 0 */
                    for( uint8_t i = 3; i < sizeOfMessageInBuffer + 3 ; i++){
                        *( writeData + i ) = 0x00;
                    }
                    sx1280Select();
                    spi_write_read_blocking( spi1, writeData, readData, ( sizeOfMessageInBuffer + 3 )*sizeof( uint8_t ) );
                    sx1280Deselect();
                    free( writeData );

                    /* dont free the readData yet, will have to use it in the following if statements
                       will then add to the neighbors list or messageStorageTillUse */

                    /* if the message is a ready to receive general, add the radio to the neighbors list */
                    if( *( readData + 3 ) == 0x52 && *( readData + 4 ) == 0x54 && *( readData + 5 ) == 0x52 && *( readData + 6 ) == 0x47 ){
                       printf("%c%c%c%c\n", *( readData + 3 ), *( readData + 4 ), *( readData + 5 ), *( readData + 6 ) );
                       break; 
                    }
                    /* if the message is a ready to send with this radios address,
                            switch to message transfer and recieve the incoming message */
                }
                free( writeData );
                free( readData );

                while( gpio_get( 22 ) == 1 ){
                    vTaskDelay( 10 );
                    printf("Busy after rx READBUFFER\n");
                }

            }

        }
        /* else if( messageStorageTillUse is not empty && neighbors are available to send to ) {
            neighbors list will most likely be a 2D pointer array with the 2nd dimension
                containing the neighbors public address and time general ready to receive was received
                if the general ready to receive occurred recently (10s of milliseconds probably) 
                    then it can be said that others are available (a possible way of doing it)

            set sx1280 to tx mode with appropriate spreading factor, bandwidth, and coding rate.
            send message with radio's public address, ready to send status, and size of whole message
            set sx1280 to rx mode and listen for a ready to receive specified with 
                the senders public address
                if no specified ready to receive occurred the other sx1280's are busy, probably
                    need to move on and receive a message to create an open sx1280 and they can 
                    "trade" messages to keep data moving 
            set sx1280 back to tx mode and broadcast the message
        } */
    }

}


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


/*  Task running usb serial input
    Sends task notification to vSx1280Task with pointer to 
        buffer holding the message to be sent from the sx1280 */
void vUsbIOTask( void *pvParameters ){

    /* Instantiating 8 bit integer to hold hex value from getchar()
       Needed to compare the char read in to "\n" for finished input */
    uint8_t currentChar = 0;
 
    /* Instantiating pointer array to hold outgoing message
       Will free the data after sending address to vSx1280Task
            and vSx1280Task has used the data */
    uint8_t *messageBuffer = 0;

    /* Instantiating integer used for placing each character in
            the correct place in messageBuffer and reallocing 
            new memory as the message size grows */
    uint32_t messageCounter = 0;
        
    /* Allocating 4 bytes for messageBuffer
       Must be allocated before loop so the function realloc 
            has a data array it can work from */ 
    messageBuffer = ( uint8_t * ) malloc( 1*sizeof( uint8_t ) );

    while( true ){
        if(messageCounter == 0){
            printf("What's your message?\n");
        }
        /* Setting currentChar to the character being read in
                by getChar() */
        currentChar = getchar();
        /* Checking to see if the character being read in is not "\n" */
        if( currentChar != 0x0A ){
            /* Reallocing the memory malloced for messageBuffer
               Doing this as the message grows until "/n"
               Making input completely dynamic so memory is not used up 
               Must add one to messageCounter to be at correct size for the message */
            messageBuffer = ( uint8_t * ) realloc( messageBuffer, ( messageCounter+1 )*sizeof( uint8_t ) );
            /* Adding currentChar to the correct place in the pointer 
                    array allocated for the message */
            *( messageBuffer + messageCounter ) = currentChar;
            /* Incrementing the value of messageCounter */
            messageCounter++;
        }
        /* Checking to see if the character being read in is "\n" */
        else if( currentChar == 0x0A ){

            /* Adding "\n" to messageBuffer for common end of string character */
            messageBuffer = ( uint8_t * ) realloc( messageBuffer, ( messageCounter+1 )*sizeof( uint8_t ) );
            *( messageBuffer + messageCounter ) = currentChar;
            /*printf("newline: %X messageCounter: %d\n",  *(messageBuffer + messageCounter), messageCounter);*/
            /* FreeRTOS function used to update the receiving
                    task’s notification value
               The receiving task is vSx1280Task
               Trying to send the address of messageBuffer, only sending values */
            xTaskNotify(
                xSx1280TaskHandle,                   /* TaskHandle_t xTaskToNotify */ 
                ( uint32_t ) &*( messageBuffer ),   /* uint32_t ulValue (int)&buffer[0] */
                eSetValueWithoutOverwrite );        /* eNotifyAction eAction */

            /* Resetting messageCounter to 0 so a new message can be taken in */
            messageCounter = 0;
            /* Freeing messageBuffer
               Will probably have to delete so vSx1280Task can grab the data
                    before freeing the memory */
            //free( messageBuffer );
            /* Allocating another block of memory for a new message
               Must be done after the memory is freed or the realloc won't work */
            messageBuffer = ( uint8_t * ) malloc( 1*sizeof( uint8_t ) );
        }

        vTaskDelay( 100 );
    }

}


int main( void ){

    stdio_init_all();

    /* Setting Up USB IRQ didnt work so not using */
    /* Setting exclusivity for usbInterruptHandler 
       to Interrupt Number '5', USBCTRL_IRQ */
    /*irq_set_exclusive_handler( USBCTRL_IRQ, &usbInterruptHandler );
    */
    /* Using hardware_irq api to enable usb interrupt.
       Interrupt number '5' corresponds to USBCTRL_IRQ */
    /*irq_set_enabled( USBCTRL_IRQ, true );*/
    

    uint32_t status = xTaskCreate(
                    vSimpleLEDTask,  
                    "Green Led",    
                    1024,           
                    NULL,           
                    1,              
                    &xSimpleLEDTaskHandle ); 

    uint32_t ioStatus = xTaskCreate(
                    vUsbIOTask,         /* TaskFunction_t pvTaskCode */
                    "Simple IO",        /* const char * const pcName */
                    1024,               /* uint16_t usStackDepth */
                    NULL,               /* void *pvParameters */
                    1,                  /* UBaseType_t uxPriority */
                    &xUsbIOTaskHandle );/*TaskHandle_t *pxCreatedTask*/

    uint32_t sx1280Status = xTaskCreate(
                    vSx1280Task,
                    "sx1280",
                    1024,
                    NULL,
                    1,
                    &xSx1280TaskHandle );

    vTaskStartScheduler();

    while(true){

    }
}
