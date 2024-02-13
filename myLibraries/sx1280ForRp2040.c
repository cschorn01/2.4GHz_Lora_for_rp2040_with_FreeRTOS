/* Author: Chris Schorn
   Date: 9/18/2023
   Version: 1.0.0
   License: Currently No License, but do what you want just don't sue me
            when it doesn't work */

/* FreeRTOS Includes */
#include "../FreeRTOS-Kernel/include/FreeRTOS.h" /* MUST COME FIRST */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

/* Raspberry Pi Pico Inlcudes */
#include <stdio.h> 
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "sx1280ForRp2040.h"

/* ----------------------------- sx1280 2.4GHz Lora Operation ----------------------------- */

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


/* --------------------------- Macros -------------------------------- */

/* Allows the spi instance to be variable based on a 1 or 0 input */
#define SPISELECT(spiNum)    (spiNum == 0 ? spi0 : (spiNum == 1 ? spi1 : NULL))


/* --------------------------- sx1280 on rp2040 Setup-------------------------------- */

/*  Function setting up sx1280 connection to rp2040 
    Initializing SPI interface on rp2040
    Using GP10-GP13
        GP10: SPI1 SCK
        GP11: SPI1 Tx
        GP12: SPI1 Rx
        GP13: SPI1 CSn */
void sx1280Rp2040Setup( uint8_t sx1280Rp2040BusyPin,
                        uint8_t sx1280Rp2040ResetPin,
                        uint8_t sx1280Rp2040SpiNumber,
                        uint8_t sx1280Rp2040SpiSckPin,
                        uint8_t sx1280Rp2040SpiTxPin,
                        uint8_t sx1280Rp2040SpiRxPin,
                        uint8_t sx1280Rp2040ChipSelectPin ){

    /* Setting up Busy Pin */

    /* void gpio_init (uint gpio)
       Initializing GPIO to input pin for Busy */
    gpio_init( sx1280Rp2040BusyPin );
    /* static void gpio_set_dir (uint gpio, bool out)
       Setting busyPin direction to input, True is out False is in
       Use gpio_get( uint gpio ) to get the state of a gpio */
    gpio_set_dir( sx1280Rp2040BusyPin, 0 );

    /* Setting up Reset Pin */

    /* Initializing GPIO to output pin for Reset */
    gpio_init( sx1280Rp2040ResetPin );
    /* Setting GPIO direction to output, True is out False is in */
    gpio_set_dir( sx1280Rp2040ResetPin, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving resetPin High
       A reset is initiated by driving Reset Low */
    gpio_put( sx1280Rp2040ResetPin, 1 );

    /* Setting up spi Interface */

    /* Inializing spi1 at 1MHz */
    spi_init( SPISELECT( sx1280Rp2040SpiNumber ), 1000000 );

    /* void gpio_set_function( uint gpio, enum gpio_function fn )
       Setting SCK, TX, and RX */
    gpio_set_function( sx1280Rp2040SpiSckPin, GPIO_FUNC_SPI );
    gpio_set_function( sx1280Rp2040SpiTxPin, GPIO_FUNC_SPI );
    gpio_set_function( sx1280Rp2040SpiRxPin, GPIO_FUNC_SPI );

    /* Initializing GPIO to output pin for Chip Select */
    gpio_init( sx1280Rp2040ChipSelectPin );
    /* Setting GPIO direction to output, True is out False is in */
    gpio_set_dir( sx1280Rp2040ChipSelectPin, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving chipSelectPin High
       A data transfer is started by driving Chip Select low */
    gpio_put( sx1280Rp2040ChipSelectPin, 1 );
}



/* --------------------------- sx1280 2.4GHz Lora Operation -------------------------------- */

/*  Driving the chip select pin low 
    Transactions with sx1280 start with chip select low */
static inline void sx1280Select( uint8_t sx1280ChipSelectPin ){

    asm volatile ("nop \n nop \n nop");/* Find out what it does */
    gpio_put( sx1280ChipSelectPin, 0 );
    asm volatile ("nop \n nop \n nop");
}

/*  Driving the chip select pin high 
    Transactions with sx1280 end with chip select high */
static inline void sx1280Deselect( uint8_t sx1280ChipDeselectPin ){

     asm volatile ("nop \n nop \n nop");
     gpio_put( sx1280ChipDeselectPin, 1 );
     asm volatile ("nop \n nop \n nop");
}

void getStatus( uint8_t statusChipSelectPin,        /* Passing chosen chip select pin       */
                uint8_t statusBusyPin,             /* Passing chosen busy pin for sx1280    */
                uint8_t statusSpiNumber){

    /* 8 bit pointers */
    uint8_t getStatusWriteData[ 1 ] = { 0 };
    uint8_t getStatusReadData[ 1 ] = { 0 };

    /* Setting sx1280 Standby mode */
    *( getStatusWriteData ) = GETSTATUS;
    sx1280Select( statusChipSelectPin );
    spi_write_read_blocking( SPISELECT( statusSpiNumber ), getStatusWriteData, getStatusReadData, 1*sizeof( uint8_t ) );
    sx1280Deselect( statusChipSelectPin );

    while( gpio_get( statusBusyPin ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after GETSTATUS\n");
    }

    printf("sx1280 Status: 0b%b\n", *( getStatusReadData ) );

}

/* Function sending common transciever settings to sx1280 */ 
void sx1280Setup( uint8_t standbyMode,
                  uint8_t packetType,               /* Setting sx1280 Packet Type, lora=0x01 */ 
                  uint8_t rfFrequency2316,          /* Setting RF Frequency bits 23 to 16    */
                  uint8_t rfFrequency158,           /* Setting RF Frequency bits 15 to 8     */
                  uint8_t rfFrequency70,            /* Setting RF Frequency bits 7 to 0      */
                  uint8_t spreadingFactor,          /* Setting the Spreading Factor          */
                  uint8_t bandwidth,                /* Setting the Bandwidth                 */
                  uint8_t codingRate,               /* Setting the Coding Rate               */
                  uint8_t preambleLength, 
                  uint8_t headerType, 
                  uint8_t cyclicalRedundancyCheck,   
                  uint8_t chirpInvert, 
                  uint8_t setupChipSelectPin,       /* Passing chosen chip select pin       */
                  uint8_t setupResetPin,            /* Passing chosen reset pin for sx1280   */
                  uint8_t setupBusyPin,             /* Passing chosen busy pin for sx1280    */
                  uint8_t setupSpiNumber,           /* Passing chosen SPI bus number         */
                  uint8_t outboundMessage[ ] ){     /* Used to check length of message       */

    uint8_t setupWriteData[ 10 ] = { 0 };
    uint8_t setupReadData[ 10 ] = { 0 };
    uint32_t payloadLength = 0;

    /* Driving setupResetPin low to reset the sx1280 SPISELECT( setupSpiNumber )*/
    gpio_put( setupResetPin, 0 );
    asm volatile ("nop \n nop \n nop");
    gpio_put( setupResetPin, 1 );

    /* Waiting till the busy pin is driven low 
       So the sx1280 is not sent a command while busy
            Because it wont receive the command */
    while( gpio_get( setupBusyPin ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after reset\n");
    }

    /* Setting sx1280 Standby mode */
    *( setupWriteData ) = SETSTANDBY;
    *( setupWriteData + 1 ) = standbyMode; /* Setting STDBY_RC Mode to 0x01 or STDBY_XOSC */
    sx1280Select( setupChipSelectPin );
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect( setupChipSelectPin );

    while( gpio_get( setupBusyPin ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETSTANDBY\n");
    }

    /* Setting sx1280 Packet Type */
    *( setupWriteData ) = SETPACKETTYPE;
    *( setupWriteData + 1 ) = packetType;
    sx1280Select( setupChipSelectPin );
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect( setupChipSelectPin );

    while( gpio_get( setupBusyPin ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETPACKETTYPE\n");
    }

    /* Setting RF Frequency */
    *( setupWriteData ) = SETRFFREQUENCY;
    *( setupWriteData + 1 ) = rfFrequency2316; /* rfFrequency[23:16] */
    *( setupWriteData + 2 ) = rfFrequency158;  /* rfFrequency[15:8] */
    *( setupWriteData + 3 ) = rfFrequency70;   /* rfFrequency[7:0] */
    sx1280Select( setupChipSelectPin );
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect( setupChipSelectPin );

    while( gpio_get( setupBusyPin ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETRFFREQUENCY\n");
    }

    /* Setting Tx and Rx Buffer Base Addresses
       Putting both at 0 since messages can be size of buffer */
    *( setupWriteData ) = SETBUFFERBASEADDRESS;
    *( setupWriteData + 1 ) = 0x00;
    *( setupWriteData + 2 ) = 0x00;
    sx1280Select( setupChipSelectPin );
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect( setupChipSelectPin );

    while( gpio_get( setupBusyPin ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETBUFFERBASEADDRESS\n");
    }

    /* Setting the Modulation Params */
    *( setupWriteData ) = SETMODULATIONPARAMS;
    *( setupWriteData + 1 ) = spreadingFactor; /* Spreading Factor */
    *( setupWriteData + 2 ) = bandwidth; /* Bandwidth */
    *( setupWriteData + 3 ) = codingRate; /* Coding Rate */
    sx1280Select( setupChipSelectPin );
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect( setupChipSelectPin );
    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if( spreadingFactor == 0x50 || spreadingFactor == 0x60 ){

        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x1E;
        sx1280Select( setupChipSelectPin );
        spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect( setupChipSelectPin );
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if( spreadingFactor == 0x70 || spreadingFactor == 0x80 ){

        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x37;
        sx1280Select( setupChipSelectPin );
        spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect( setupChipSelectPin );

    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if( spreadingFactor == 0x90 || spreadingFactor == 0xA0 || spreadingFactor == 0xB0 || spreadingFactor == 0xC0 ){
        
        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x32;
        sx1280Select( setupChipSelectPin );
        spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect( setupChipSelectPin );
    }
    /* 0x01 must be written to register 0x093C */
    *( setupWriteData ) = WRITEREGISTER;
    *( setupWriteData + 1 ) = 0x09;
    *( setupWriteData + 2 ) = 0x3C;
    *( setupWriteData + 3 ) = 0x01;
    sx1280Select( setupChipSelectPin );
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect( setupChipSelectPin );

    while( gpio_get( setupBusyPin ) == 1 ){
        vTaskDelay( 10 );
        // printf("Busy after SETMODULATIONPARAMS\n");
    }

    /* Setting Packet Params */
    while( *( outboundMessage + payloadLength ) != 0x00 ){
        /* Maximum payloadLength on sx1280 is 255, PRESET TO MAXIMUM PAYLOAD ALWAYS */
        if( payloadLength > 255 ){
            payloadLength = 255;
            break;
        }
        payloadLength = payloadLength + 1;
    }
    *( setupWriteData ) = SETPACKETPARAMS;
    *( setupWriteData + 1 ) = preambleLength; /* Preamble Length */
    *( setupWriteData + 2 ) = headerType; /* Header Type */
    *( setupWriteData + 3 ) = payloadLength; /* Payload Length */
    *( setupWriteData + 4 ) = cyclicalRedundancyCheck; /* Cyclical Redundancy Check */
    *( setupWriteData + 5 ) = chirpInvert; /* Invert IQ/chirp invert */
    *( setupWriteData + 6 ) = 0x00; /* Not Used */
    *( setupWriteData + 7 ) = 0x00; /* Not Used */
    sx1280Select( setupChipSelectPin );
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData, 8*sizeof( uint8_t ) );
    sx1280Deselect( setupChipSelectPin );

    while( gpio_get( setupBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETPACKETPARAMS\n");
    }

    /* Testing connecting from pico to sx1280 by writing to and reading from buffer
       Working output should be "0xStatus 0xStatus 0xFF" */

    /* 
    *( setupWriteData ) = WRITEBUFFER;
    *( setupWriteData + 1 ) = 0x00;
    *( setupWriteData + 2 ) = 0xFF;
    sx1280Select();
    spi_write_blocking( SPISELECT( setupSpiNumber ), setupWriteData writeData, 3*sizeof( uint8_t ) );
    sx1280Deselect(); */

    /* Must use two NOP's for reads because data is
            returned beginning on the second NOP */
    /* 
    *( setupWriteData ) = READBUFFER;
    *( setupWriteData + 1 ) = 0x00;
    *( setupWriteData + 2 ) = 0x00;
    *( setupWriteData + 3 ) = 0x00;
    *( setupWriteData + 4 ) = 0x00;
    sx1280Select();
    spi_write_read_blocking( SPISELECT( setupSpiNumber ), setupWriteData, setupReadData readData, 5*sizeof( uint8_t ) );
    sx1280Deselect(); 
    printf( "%X %X %X %X %X\n", *( setupReadData ), *( setupReadData + 1 ), *( setupReadData + 2 ), *( setupReadData + 3 ), *( setupReadData + 4 ) ); */

}


/* Function setting up and running tx operation on an sx1280, taking 255 byte message packets */
void sx1280Tx( uint8_t power, 
               uint8_t rampTime,
               uint8_t outboundMessage[ ],
               uint8_t txIrq158,                /* IRQ Mask for bits 15:8 of IRQ register */
               uint8_t txIrq70,                 /* IRQ Mask for bits 7:0 of IRQ register */
               uint8_t txPeriodBase,
               uint8_t txPeriodBaseCount158,    /* setting periodBaseCount bits 15 to 8 */
               uint8_t txPeriodBaseCount70,     /* setting periodBaseCount bits 8 to 0 */
               uint8_t txChipSelectPin,
               uint8_t txBusyPin,
               uint8_t txSpiNumber ){

    uint8_t txWriteData[ 259 ] = { 0 };
    uint8_t txReadData[ 4 ] = { 0 };
    uint32_t txPayloadLength = 0;

    /* Iterators */
    uint32_t i = 0;

    /* Setting the tx parameters necessary for sending a message */
    *( txWriteData ) = SETTXPARAMS;
    *( txWriteData + 1 ) = power;    /* power       */
    *( txWriteData + 2 ) = rampTime; /* rampTime    */
    sx1280Select( txChipSelectPin );
    spi_write_blocking( SPISELECT( txSpiNumber ), txWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect( txChipSelectPin );

    while( gpio_get( txBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETTXPARAMS\n");
    }

    /* Writing a message to the sx1280 Tx message buffer */
    while( *( outboundMessage + txPayloadLength ) != 0x00  ){

        /* Getting size of a single outbound message, storing it in a holder variable */
        txPayloadLength = txPayloadLength + 1;

        if( txPayloadLength > 255 ){

            txPayloadLength = 255;
            *( outboundMessage + txPayloadLength ) = 0x00;
        }
    }
    /* Allocating txPayloadLength+3 bytes to writeData, payloadLength is indexed from zero
            and space is needed for the WRITEBUFFER command and nop
    txWriteData = ( uint8_t * )pvPortMalloc( ( txPayloadLength+3 )*sizeof( uint8_t ) ); */
    *( txWriteData ) = WRITEBUFFER;
    *( txWriteData + 1 ) = 0x00;
    /* Looping payloadLength times, writing outboundMessage data to WRITEBUFFER command */
    for( i = 0; i <= txPayloadLength; i++ ){
        *( txWriteData + i + 2 ) = *( outboundMessage + i );
        printf("Outbound Message: 0x%X\n", *( outboundMessage + i ) );
    }
    sx1280Select( txChipSelectPin );
    spi_write_blocking( SPISELECT( txSpiNumber ), txWriteData, ( txPayloadLength+3 )*sizeof( uint8_t ) );
    sx1280Deselect( txChipSelectPin );

    while( gpio_get( txBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx WRITEBUFFER\n");
    }

    /* setting IRQ parameters for the outgoing message, looping SPI not DIO pins to check*/
    *( txWriteData ) = SETDIOIRQPARAMS;
    *( txWriteData + 1 ) = txIrq158; /* IRQ Mask for bits 15:8 of IRQ register  */
    *( txWriteData + 2 ) = txIrq70;  /* IRQ Mask for bits 7:0 of IRQ register   */
    *( txWriteData + 3 ) = 0x00;     /* setting DIO 1 Mask bits 15:8 to 0       */
    *( txWriteData + 4 ) = 0x00;     /* setting DIO 1 Mask bits 7:0 to 0        */
    *( txWriteData + 5 ) = 0x00;     /* setting DIO 2 Mask bits 15:8 to 0       */
    *( txWriteData + 6 ) = 0x00;     /* setting DIO 2 Mask bits 7:0 to 0        */
    *( txWriteData + 7 ) = 0x00;     /* setting DIO 3 Mask bits 15:8 to 0       */
    *( txWriteData + 8 ) = 0x00;     /* setting DIO 3 Mask bits 7:0 to 0        */
    sx1280Select( txChipSelectPin );
    spi_write_blocking( SPISELECT( txSpiNumber ), txWriteData, 9*sizeof( uint8_t ) );
    sx1280Deselect( txChipSelectPin );

    while( gpio_get( txBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx SETDIOIRQPARAMS\n");
    }

    /* Putting sx1280 in transmit mode to send the message in sx1280's message buffer 
       Timeout is periodBase * periodBaseCount */
    *( txWriteData ) = SETTX;
    *( txWriteData + 1 ) = txPeriodBase; /* setting periodBase, RTC step */
    *( txWriteData + 2 ) = txPeriodBaseCount158; /* setting periodBaseCount[15:8] */
    *( txWriteData + 3 ) = txPeriodBaseCount70; /* setting periodBaseCount[8:0] */
    sx1280Select( txChipSelectPin );
    spi_write_blocking( SPISELECT( txSpiNumber ), txWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect( txChipSelectPin );

    while( gpio_get( txBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx SETTX\n");
    }

    /* Looping over GETIRQSTATUS using SPI, till TxDone bit is high */
    for( i = 0; i <= 100; i++){

        // vPortFree( txReadData );
        vTaskDelay( 50 );

        *( txWriteData ) = GETIRQSTATUS;
        *( txWriteData + 1 ) = 0x00;
        *( txWriteData + 2 ) = 0x00;
        *( txWriteData + 3 ) = 0x00;
        sx1280Select( txChipSelectPin );
        spi_write_read_blocking( SPISELECT( txSpiNumber ), txWriteData, txReadData, 4*sizeof( uint8_t ) );
        sx1280Deselect( txChipSelectPin );
 
        while( gpio_get( txBusyPin ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after tx GETIRQSTATUS\n");
        }

        printf("IRQ Check: 0x%X %i\n", *( txReadData + 3 ), i );

        /* Checking bits [7:0] to see if the TxDone bit in the IRQ register is high
           Doing bitwise 'and' operation with 0x01 to mask the rest of the bits in 
                the IRQ register, giving a clear indication that a message has been sent
            Bits [15:8] would be in  *( readData + 4 ) */
        if( *( txReadData + 3 ) != 0x00 ){ /* GETIRQSTATUS TxDone == 1 */

            printf("IRQ: 0x%X %i \n", *( txReadData + 3 ), i );
            break;
        }

    }

    /* Clearing the IRQ register, reseting IRQ Mask bits to 0 */
    *( txWriteData ) = CLRIRQSTATUS;
    *( txWriteData + 1 ) = 0xFF; /* clearing bits 15:8 of IRQ mask */
    *( txWriteData + 2 ) = 0xFF; /* clearing bits 7:0 of IRQ mask */
    sx1280Select( txChipSelectPin );
    spi_write_blocking( SPISELECT( txSpiNumber ), txWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect( txChipSelectPin );

    while( gpio_get( txBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx CLRIRQSTATUS\n");
    }

    /* Tx SETSANDBY */
    *( txWriteData ) = SETSTANDBY;
    *( txWriteData + 1 ) = 0x00;
    sx1280Select( txChipSelectPin );
    spi_write_blocking( SPISELECT( txSpiNumber ), txWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect( txChipSelectPin );

    while( gpio_get( txBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx SETSTANDBY\n");
    }
}


/* Function setting up and running rx operation on an sx1280, 2.4Ghz LORA Modem*/
void sx1280Rx( uint8_t rxIrq158,                /* IRQ Mask for bits 15 to 8 of IRQ register */
               uint8_t rxIrq70,                 /* IRQ Mask for bits 7 to 0 of IRQ register */
               uint8_t rxPeriodBase,
               uint8_t rxPeriodBaseCount158,    /* perdiodBase bits 15 to 8 for rx */
               uint8_t rxPeriodBaseCount70,     /* perdiodBase bits 7 to 0 for rx */
               uint8_t rxChipSelectPin,
               uint8_t rxBusyPin,
               uint8_t rxSpiNumber,
               uint8_t inboundMessage[ ] ){       /* Pointer to return received message */
               

    uint8_t rxWriteData[ 259 ] = { 0 };
    uint8_t rxReadData[ 259 ] = { 0 };
    uint32_t totalSizeOfMessage = 0;
    uint32_t sizeOfMessageInBuffer = 0;

    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;

    /* setting IRQ parameters for Rx mode */
    *( rxWriteData ) = SETDIOIRQPARAMS;
    *( rxWriteData + 1 ) = rxIrq158; /* IRQ Mask for bits 15:8 of IRQ register */
    *( rxWriteData + 2 ) = rxIrq70; /* IRQ Mask for bits 7:0 of IRQ register */ 
    *( rxWriteData + 3 ) = 0x00; /* setting DIO 1 Mask bits 15:8 to 0 */
    *( rxWriteData + 4 ) = 0x00; /* setting DIO 1 Mask bits 7:0 to 0 */
    *( rxWriteData + 5 ) = 0x00; /* setting DIO 2 Mask bits 15:8 to 0 */
    *( rxWriteData + 6 ) = 0x00; /* setting DIO 2 Mask bits 7:0 to 0 */
    *( rxWriteData + 7 ) = 0x00; /* setting DIO 3 Mask bits 15:8 to 0 */
    *( rxWriteData + 8 ) = 0x00; /* setting DIO 3 Mask bits 7:0 to 0 */
    sx1280Select( rxChipSelectPin );
    spi_write_blocking( SPISELECT( rxSpiNumber ), rxWriteData, 9*sizeof( uint8_t ) );

    while( gpio_get( rxBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after rx SETDIOIRQPARAMS\n");
    }

    /* setting sx1280 to Rx mode
       Setting Rx mode to continuous, so multiple messages can be received */
    *( rxWriteData ) = SETRX;
    *( rxWriteData + 1 ) = rxPeriodBase; /* Setting the RTC step */
    *( rxWriteData + 2 ) = rxPeriodBaseCount158; /* perdiodBase[15:8] for rx */
    *( rxWriteData + 3 ) = rxPeriodBaseCount70; /* perdiodBase[7:0] for rx */
    sx1280Select( rxChipSelectPin );
    spi_write_blocking( SPISELECT( rxSpiNumber ), rxWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect( rxChipSelectPin );

    while( gpio_get( rxBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETRX\n");
    }

    /* Loop polling each time it revolves over rx mode
       Each loop has a 50 clock-tick delay allowing other tasks to run */
    for( i = 0; i <= 100; i++ ){ 

        vTaskDelay( 50 ); /* 50 clock-tick delay */

        printf("Listening: %i\n", i );
        sizeOfMessageInBuffer = 0;

        /* Using GETIRQSTATUS to check if there is a new message in the rx buffer */
        *( rxWriteData ) = GETIRQSTATUS;
        *( rxWriteData + 1 ) = 0x00;
        *( rxWriteData + 2 ) = 0x00;
        *( rxWriteData + 3 ) = 0x00;
        sx1280Select( rxChipSelectPin );
        spi_write_read_blocking( SPISELECT( rxSpiNumber ), rxWriteData, rxReadData, 4*sizeof( uint8_t ) );
        sx1280Deselect( rxChipSelectPin );

        /* Checking to see if the RxDone bit in the IRQ register is high, with 0x02 bitmask */
        if( ( *( rxReadData + 3 ) & 0x02 ) == 0x02 ){ /* GETIRQSTATUS RxDone == 1 */

            // maybe 0 the array here I do it in the arduino library, not sure why tho 

            while( gpio_get( rxBusyPin ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx GETIRQSTATUS\n");
            }

            /* using GETPACKETSTATUS which returns rssiSync, and Signal to Noise Ratio ( SNR )
               Not currently using but it's in sx1280 Documentation for Rx operation
                    pretty sure it's used to see if the received message is useable or not */
            *( rxWriteData ) = GETPACKETSTATUS;
            *( rxWriteData + 1 ) = 0x00;
            *( rxWriteData + 2 ) = 0x00;
            *( rxWriteData + 3 ) = 0x00;
            *( rxWriteData + 4 ) = 0x00;
            *( rxWriteData + 5 ) = 0x00;
            *( rxWriteData + 6 ) = 0x00;
            sx1280Select( rxChipSelectPin );
            spi_write_read_blocking( SPISELECT( rxSpiNumber ), rxWriteData, rxReadData, 7*sizeof( uint8_t ) );
            sx1280Deselect( rxChipSelectPin );

            while( gpio_get( rxBusyPin ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx GETPACKETSTATUS\n");
            }

            /* Clearing the IRQ register on the sx1280
               Not sure why it's done here in the rx operation in sx1280 documentation */
            *( rxWriteData ) = CLRIRQSTATUS;
            *( rxWriteData + 1 ) = 0xFF;
            *( rxWriteData + 2 ) = 0xFF;
            sx1280Select( rxChipSelectPin );
            spi_write_blocking( SPISELECT( rxSpiNumber ), rxWriteData, 3*sizeof( uint8_t ) );
            sx1280Deselect( rxChipSelectPin );

            while( gpio_get( rxBusyPin ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx CLRIRQSTATUS\n");
            }

            /* Getting the length of the newly received message
               GETRXBUFFERSTATUS only works for LORA messages with headers, 
                    otherwise read register 0x0901 */
            *( rxWriteData ) = GETRXBUFFERSTATUS; 
            *( rxWriteData + 1 ) = 0x00;
            *( rxWriteData + 2 ) = 0x00;
            *( rxWriteData + 3 ) = 0x00;
            sx1280Select( rxChipSelectPin );
            spi_write_read_blocking( SPISELECT( rxSpiNumber ), rxWriteData, rxReadData, 4*sizeof( uint8_t ) );
            sx1280Deselect( rxChipSelectPin );
            /* Grabbing message size for correct memory allocation for incoming message */
            sizeOfMessageInBuffer = *( rxReadData + 2 );

            while( gpio_get( rxBusyPin ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx READREGISTER\n");
            }

            /* Reading message buffer of sx1280
               Allocating the size of the message in the sx1280 buffer plus 3 because over 
                    spi you must send an opcode, the buffer offset, and a nop to receive the
                    payload on the buffer */
            *( rxWriteData ) = READBUFFER;
            *( rxWriteData + 1 ) = 0x00; /* sx1280 message buffer offset */
            *( rxWriteData + 2 ) = 0x00; /* sending first nop */
            /* Looping through rest of rxWriteData to add nops, i begins at *( rxWriteData + 3 ) */
            printf("Final Address = 0x%X\n", ( rxWriteData + sizeOfMessageInBuffer + 3 ));
            for( j = 3; j <= sizeOfMessageInBuffer; j++){
                *( rxWriteData + j ) = 0x00;
                /* printf("rxWriteData + j = 0x%X j = %i \n", ( rxWriteData + j ), j ); */
            }
            sx1280Select( rxChipSelectPin );
            spi_write_read_blocking( SPISELECT( rxSpiNumber ), rxWriteData, rxReadData, ( sizeOfMessageInBuffer + 3 )*sizeof( uint8_t ) );
            sx1280Deselect( rxChipSelectPin );
            /* Passing newly received message pointer to vSx1280Task */
            for( j = 0; j <= sizeOfMessageInBuffer; j++ ){
                inboundMessage[ j ] = *( rxWriteData + j );
            }

            while( gpio_get( rxBusyPin ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx READBUFFER\n");
            }
        }
    }

    /* Rx SETSANDBY */
    *( rxWriteData ) = SETSTANDBY;
    *( rxWriteData + 1 ) = 0x00;
    sx1280Select( rxChipSelectPin );
    spi_write_blocking( SPISELECT( rxSpiNumber ), rxWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect( rxChipSelectPin );

    while( gpio_get( rxBusyPin ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after rx SETSTANDBY\n");
    }
}

