/**
 *  @file
 *  @author  Matthew Paulishen <ticiane1@uga.edu>
 *  @version v2012.02.15.1600
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  @section LICENSE
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Just to be safe:
 *   'Dynamixel' is property of Robotis, Inc.
 *      <http://www.robotis.com>
 *
 *
 *  Copyright (c) 2011, 2012 Matthew Paulishen. All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  @section DESCRIPTION
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  -A header file of questionable quality for use with the CM-530 from Robotis.
 *    A bit of a smorgasbord of code intended to keep user's code cleaner.  It
 *    initializes all pins (ADC Input, Digital Output, and U(S)ARTs) and their
 *    clocks.  It also initializes the SysTick counter to allow timeout counters
 *    for the PC UART and Dynamixel bus, as well as the user-accessible delay
 *    function and an independent countdown timer.  By default, the minimum
 *    time interval of the uDelay() and timeout counters is 10 [us], but can be 
 *    reduced to 1 [us] via the USING_SYSTICK_*** preprocessor definitions (may
 *    lead to unstable code, but might not).
 *
 *  -The Dynamixel and Zigbee libraries are based primarily on the examples
 *    from Robotis, but have been modified a bit (renamed/removed variables,
 *    switched to ring buffers, added more descriptive error output for
 *    dynamixel packets (16-bit maskable error variable), etc.).
 *
 *  -The PC UART library is partly based on the Robotis library, but variables
 *    were changed to achieve uniformity with Dxl and Zig libraries as well
 *    as switching to a ring buffer.  Other print/get functions were created
 *    to work around a problem with WinARM's eabi-arm libraries not linking (no
 *    stdio.h accessible).
 *
 *  -The CM-530 helper functions were refined from a header file previously
 *    created for the CM-510 and CM-700 (may backport cm530.h/c to the CM-510
 *    and CM-700 in the future).
 *
 *  FUNCTIONS:
 *    SetLED(LED_t led, uint8_t state);
 *        // Control LEDs of CM-530
 *        //   {MANAGE, PROGRAM, PLAY, TXD, RXD, AUX, POWER}
 *    ReadButton(Button_t button);
 *        // Read User Buttons and Microphone input of CM-530
 *        //   {UP, DOWN, LEFT, RIGHT, START, MIC}
 *    Buzzed(uint32_t mlength, uint32_t tone);
 *        // Control Buzzer of CM-530
 *        //   mlength is the length in [ms] to play the tone
 *        //   tone is the delay in [us] to produce a 50% duty cycle
 *        //     (Buzzer_ON -> uDelay(tone) -> Buzzer_OFF -> uDelay(tone)
 *    PlayNote(uint32_t mlength, buzzed_note_t note, uint8_t octave);
 *        // Play a note (12-TET. 12 notes from A flat to G sharp.)
 *        //   mlength is the length in [ms] to play the note
 *        //   note is the note to play
 *        //     (C, C#, Db, D, D#, Eb, E, F, F#, Gb, G, G#, Ab, A, A#, Bb, B)
 *        //   octave indicates the octave where the note is to be played
 *        //     To play Middle C (C_4) for 10 seconds:
 *        //       PlayNote(10000, NOTE_C, 4);
 *    ReadIR(EPortA_t port);
 *        // Control a Bioloid IR module easily (not the SHARP module used in chest)
 *    SetEPort(EPortD_t pin, uint8_t state);
 *        // Control Pins 1 and 5 of the 6 External Ports
 *    ReadAnalog(EPortA_t port);
 *        // Read Pin 3 (analog) of the 6 External Ports or Battery Voltage (VBUS)
 *    StartCountdown(uint32_t nTime);
 *        // Start a variable (glCountdownCounter) to countdown in 1 [ms]
 *        //   intervals without stopping code execution
 *    mDelay(uint32_t nTime);
 *        // Delay code execution for nTime in [ms]
 *    uDelay(uint32_t nTime);
 *        // Delay code execution for nTime in [us]
 *        //   The default interval is 10 [us] between SysTick() decrements, so
 *        //   will actually round to nearest 10 [us] (can be changed, but may
 *        //   introduce bugs)
 *
 *  KNOWN ISSUES:
 *    -As has been mentioned several times in the preceding comments, the
 *      actual resolution of the uDelay() function varies with the
 *      "USING_SYSTICK_XUS" preprocessor definition in "cm530.c".
 *      "USING_SYSTICK_10US" is the default, and "USING_SYSTICK_1US" has
 *      not been tested since very, very early in development.
 *    -There may still be a bug somewhere in the revised dynamixel library that
 *      had been causing a few problems with servos not responding immediately
 *      or not being detected during initial startup (simply not found in the 
 *      bus scan or having an improper Model number).
 *      It appears to be fixed since adding a function to clear the received
 *      packet array, adding a small delay between calls to dxl_rxpacket(),
 *      and adding a check for the DXL_RXSUCCESS status bit for all read based
 *      high-level functions.
 *  
 *  TODO:
 *    -Test the ZigBee library.
 *    -Verify the reboot to bootloader commands.
 *    -Add documentation using Doxygen?
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef __CM_530_H_
#define __CM_530_H_

#ifdef __cplusplus
"C" {
#endif

#define CM530_FIRMWARE_VERSION          "v2012.02.15.1600\n"

#define USING_DYNAMIXEL
#define USING_PC_UART
//#define USING_ZIGBEE

#include "stm32f10x_it.h"

//#include <stdint.h>
#define uint8_t                         u8  // unsigned char
#define uint16_t                        u16 // unsigned short
#define uint32_t                        u32 // unsigned long
#define int8_t                          s8  // signed char
#define int16_t                         s16 // signed short
#define int32_t                         s32 // signed long
//#define NULL                            0

#ifdef USING_DYNAMIXEL
extern uint32_t Baudrate_DXL;
#endif
#ifdef USING_ZIGBEE
extern uint32_t Baudrate_ZIG;
#endif
#ifdef USING_PC_UART
extern uint32_t Baudrate_PCU;
#endif


#ifdef USING_DYNAMIXEL
//##############################################################################
//##############################################################################
// Dynamixel SDK platform independent header
//##############################################################################
//##############################################################################
#include "dynamixel_address_tables.h"

/**
 * High-level function to initialize the Dynamixel Library.
 * @param baudrate The baudrate [bps] to be used.
 * @see dxl_terminate()
 * @return Returns 1 if successful.
 */
uint8_t dxl_initialize(uint32_t baudrate);
/**
 * High-level function to terminate the Dynamixel Library.
 * @see dxl_initialize()
 */
void dxl_terminate(void);

/**
 * Set the outgoing packet ID byte.
 * @param id The target device's ID.
 */
void dxl_set_txpacket_id(uint8_t id);
/**
 * Set the outgoing packet Instruction byte.
 * @param instruction The Instruction byte (read, write, etc.).
 */
void dxl_set_txpacket_instruction(uint8_t instruction);
/**
 * Set one parameter byte of the outgoing packet.
 * @param index The position of the parameter byte in the outgoing packet.
 * @param value The value to be stored at 'index' of the outgoing packet.
 */
void dxl_set_txpacket_parameter(uint8_t index, uint8_t value);
/**
 * Set the length of the outgoing packet.
 * @param length The number of parameter bytes + 2.
 */
void dxl_set_txpacket_length(uint8_t length);

/**
 * Check the incoming packet's error byte for an active flag.
 * @param errbit The bit mask of the error flag you seek to check.
 * @see dxl_get_result()
 * @return Returns 1 if the flag is set.
 */
uint8_t dxl_get_rxpacket_error(uint8_t errbit);
/**
 * Get the value stored at a location in the incoming packet.
 * @param index The position of the parameter byte in the incoming packet.
 * @see dxl_get_rxpacket_length()
 * @return Returns the value of the parameter byte.
 */
uint8_t dxl_get_rxpacket_parameter(uint8_t index);
/**
 * Get the length of the incoming packet.
 * @see dxl_get_rxpacket_parameter()
 * @return Returns the number of paramter bytes + 2.
 */
uint8_t dxl_get_rxpacket_length(void);

/**
 * Makes a 16-bit word from two input bytes.
 * @param lowbyte The low byte of the word.
 * @param highbyte The high byte of the word.
 * @return Returns a 16-bit word.
 */
uint16_t dxl_makeword(uint8_t lowbyte, uint8_t highbyte);
/**
 * Gets the low byte of a 16-bit word.
 * @param word The word of which to find the low byte.
 * @see dxl_get_highbyte()
 * @return The low byte of the word.
 */
uint8_t dxl_get_lowbyte(uint16_t word);
/**
 * Gets the low byte of a 16-bit word.
 * @param word The word of which to find the high byte.
 * @see dxl_get_lowbyte()
 * @return The high byte of the word.
 */
uint8_t dxl_get_highbyte(uint16_t word);

/**
 * Send and receive a dynamixel packet.
 * Must be called after constructing a custom packet.
 * @see dxl_get_result()
 * @see dxl_get_rxpacket_error()
 */
void dxl_txrx_packet(void);

/**
 * Retrieves the 16-bit error word produced by dxl_txrx_packet().
 * Each bit indicates a different error or success flag from
 * the attempt to send a dynamixel packet and receive the response
 * from the device (if any).
 * @see dxl_txrx_packet()
 * @see dxl_get_rxpacket_error()
 * @return Returns the error word.
 */
uint16_t dxl_get_result(void);

/**
 * High-level function to send a ping packet to a device.
 * @param id The ID of the device to ping.
 */
void dxl_ping(uint8_t id);
/**
 * High-level function to read a single byte from a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the byte to be read.
 * @see dxl_read_word()
 * @return The value of the byte read from the device.
 */
uint8_t dxl_read_byte(uint8_t id, uint8_t address);
/**
 * High-level function to write a single byte to a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the byte to be written.
 * @param value The value to be written to that address.
 * @see dxl_write_word()
 */
void dxl_write_byte(uint8_t id, uint8_t address, uint8_t value);
/**
 * High-level function to read a 16-bit word from a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the word to be read.
 * @see dxl_read_byte()
 * @return The value of the two consecutive bytes read from the device.
 */
uint16_t dxl_read_word(uint8_t id, uint8_t address);
/**
 * High-level function to write a 16-bit word to a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the word to be written.
 * @param value The value to be written to that address.
 * @see dxl_write_byte()
 */
void dxl_write_word(uint8_t id, uint8_t address, uint16_t value);

/**
 * High-level function to begin an image capture with the HaViMo2 camera module.
 * @param id HaViMo2 camera ID (fixed as 100 in HaViMo2 firmware).
 * @see dxl_recover()
 */
void dxl_capture(uint8_t id);
/**
 * High-level function to retrieve an image buffer from a HaViMo2 camera module.
 * @param id HaViMo2 camera ID (fixed as 100 in HaViMo2 firmware).
 * @param hvm2rb Pointer to a user region buffer data type.
 * @see dxl_capture()
 * @return The number of valid regions found in the image.
 */
uint8_t dxl_recover(uint8_t id, HaViMo2_Region_Buffer_t* hvm2rb);

#endif



#ifdef USING_PC_UART
//##############################################################################
//##############################################################################
// Serial/PC_UART platform independent header
//##############################################################################
//##############################################################################
/**
 * High-level function to initialize the PC UART Library.
 * @param baudrate The baudrate [bps] to be used.
 * @see pcu_terminate()
 * @return Returns 1 if successful.
 */
uint8_t pcu_initialize(uint32_t baudrate);
/**
 * High-level function to terminate the PC UART Library.
 * @see pcu_initialize()
 */
void pcu_terminate(void);
/**
 * Get the number of available characters in the receive buffer/queue.
 * @return The number of available characters.
 */
uint8_t pcu_get_qstate(void);

// stdio.h compatibility
/**
 * Implementation of the C-Standard putchar() function.
 * @param c The char to be printed to stdout (PC UART).
 * @see std_puts()
 * @return The character printed to stdout (PC UART).
 */
int std_putchar(char);
/**
 * Implementation of the C-Standard puts() function.
 * @param str Pointer to the char array to be printed to stdout (PC UART).
 * @see std_putchar()
 * @return The number of characters printed to stdout (PC UART).
 */
int std_puts(const char*);
/**
 * Implementation of the C-Standard getchar() function.
 * @see std_gets()
 * @return The next character from stdin (PC UART receive buffer/queue).
 */
int std_getchar(void);
/**
 * Implementation of the C-Standard gets() function.
 * @param str Pointer to a char array buffer to receive from stdin (PC UART).
 * @see std_getchar()
 * @return If successful, the input pointer.  If error, a null pointer.
 */
char* std_gets(char*);

/**
 * Print message for any and all errors encountered during dxl_txrx_packet().
 * @param Status The 16-bit error variable returned by dxl_get_result().
 * @see PrintErrorCode()
 */
void PrintCommStatus(uint16_t);
/**
 * Print message for any and all error flags of the incoming dynamixel packet.
 * @see PrintCommStatus()
 */
void PrintErrorCode(void);

/**
 * Wrapper function for putchar().
 * @param c The char to be printed to stdout (PC UART).
 * @see PrintString()
 * @return The character printed to stdout (PC UART).
 */
int PrintChar(char c);
/**
 * Wrapper function for puts().
 * @param str Pointer to the char array to be printed to stdout (PC UART).
 * @see PrintChar()
 * @return The number of characters printed to stdout (PC UART).
 */
int PrintString(const char* s);
/**
 * Wrapper function for getchar().
 * @param str Pointer to a char array buffer to receive from stdin (PC UART).
 * @see GetString()
 * @return If successful, the input pointer.  If error, a null pointer.
 */
int GetChar(void);
/**
 * Wrapper function for gets().
 * @param str Pointer to a char array buffer to receive from stdin (PC UART).
 * @see GetChar()
 * @return If successful, the input pointer.  If error, a null pointer.
 */
char* GetString(char* s);

/**
 * Print an unsigned 32-bit integer to stdout (PC UART).
 * Prints in decimal format without leading zeroes. Partly replicates printf()
 * because of problems getting WinARM to actually link to libc (stdio.h).
 * @param lNum The number to be printed to stdout.
 * @see Prints32d()
 * @see Printu16h()
 * @see Printu8h()
 */
void Printu32d(uint32_t);
/**
 * Print a signed 32-bit integer to stdout (PC UART).
 * Prints in decimal format with a '+' or '-' but without any leading zeroes.
 * Partly replicates printf() because of problems getting WinARM to actually
 * link to libc (stdio.h).
 * @param lNumS The number to be printed to stdout.
 * @see Printu32d()
 * @see Printu16h()
 * @see Printu8h()
 */
void Prints32d(int32_t);
/**
 * Print an unsigned 16-bit integer to stdout (PC UART).
 * Prints in hexadecimal format with preceding '0x' but without any leading
 * zeroes. Partly replicates printf() because of problems getting WinARM to
 * actually link to libc (stdio.h).
 * @param wNum The number to be printed to stdout.
 * @see Printu8h()
 * @see Printu32d()
 * @see Prints32d()
 */
void Printu16h(uint16_t);
/**
 * Print an unsigned 8-bit integer to stdout (PC UART).
 * Prints in hexadecimal format with preceding '0x' but without any leading
 * zeroes. Partly replicates printf() because of problems getting WinARM to
 * actually link to libc (stdio.h).
 * @param bNum The number to be printed to stdout.
 * @see Printu16h()
 * @see Printu32d()
 * @see Prints32d()
 */
void Printu8h(uint8_t);

#endif



#ifdef USING_ZIGBEE
//##############################################################################
//##############################################################################
// Zigbee SDK platform independent header
//##############################################################################
//##############################################################################
// RC-100 Button values
#define RC100_BTN_U                     0x0001
#define RC100_BTN_D                     0x0002
#define RC100_BTN_L                     0x0004
#define RC100_BTN_R                     0x0008
#define RC100_BTN_1                     0x0010
#define RC100_BTN_2                     0x0020
#define RC100_BTN_3                     0x0040
#define RC100_BTN_4                     0x0080
#define RC100_BTN_5                     0x0100
#define RC100_BTN_6                     0x0200

/**
 * High-level function to initialize the ZigBee Library.
 * @param baudrate The baudrate [bps] to be used.
 * @see zgb_terminate()
 * @return Returns 1 if successful.
 */
uint8_t zgb_initialize(uint32_t baudrate);
/**
 * High-level function to terminate the ZigBee Library.
 * @see zgb_initialize()
 */
void zgb_terminate(void);

/**
 * High-level function to send a 16-bit data payload in ZigBee packet.
 * @param data The 16-bit value to be sent.
 * @see zgb_rx_check()
 * @see zgb_rx_data()
 * @return Returns 1 if successful.
 */
uint8_t zgb_tx_data(uint16_t data);
/**
 * High-level function to check for a valid received ZigBee packet.
 * @see zgb_rx_data()
 * @return Returns 1 if valid packet available.
 */
uint8_t zgb_rx_check(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
uint16_t zgb_rx_data(void);

#endif



//##############################################################################
//##############################################################################
// All CM-530 PORT and PIN definitions
//##############################################################################
//##############################################################################
// Button/Mic PORT and PIN definitions
#define PORT_SW_UP                      GPIOC
#define PORT_SW_DOWN                    GPIOC
#define PORT_SW_RIGHT                   GPIOA
#define PORT_SW_LEFT                    GPIOA
#define PORT_SW_START                   GPIOB
#define PORT_MIC                        GPIOC
#define PIN_SW_UP                       GPIO_Pin_11
#define PIN_SW_DOWN                     GPIO_Pin_10
#define PIN_SW_RIGHT                    GPIO_Pin_14
#define PIN_SW_LEFT                     GPIO_Pin_15
#define PIN_SW_START                    GPIO_Pin_3
#define PIN_MIC                         GPIO_Pin_4

// LED PORT and PIN definitions
#define PORT_LED_POWER                  GPIOC
#define PORT_LED_MANAGE                 GPIOB
#define PORT_LED_PROGRAM                GPIOB
#define PORT_LED_PLAY                   GPIOB
#define PORT_LED_TXD                    GPIOC
#define PORT_LED_RXD                    GPIOC
#define PORT_LED_AUX                    GPIOB
#define PIN_LED_POWER                   GPIO_Pin_13
#define PIN_LED_MANAGE                  GPIO_Pin_13
#define PIN_LED_PROGRAM                 GPIO_Pin_14
#define PIN_LED_PLAY                    GPIO_Pin_15
#define PIN_LED_TXD                     GPIO_Pin_14
#define PIN_LED_RXD                     GPIO_Pin_15
#define PIN_LED_AUX                     GPIO_Pin_12

// Buzzer PORT and PIN definitions
#define PORT_BUZZER                     GPIOA
#define PIN_BUZZER                      GPIO_Pin_6

// ADC select PORT and PIN definitions
#define PORT_ADC_SELECT0                GPIOC
#define PORT_ADC_SELECT1                GPIOC
#define PIN_ADC_SELECT0                 GPIO_Pin_1
#define PIN_ADC_SELECT1                 GPIO_Pin_2
#define PIN_ADC0                        GPIO_Pin_0
#define PIN_ADC1                        GPIO_Pin_5
#define PIN_VDD_VOLT                    GPIO_Pin_3

// Ollo port PORT and PIN definitions
#define PORT_SIG_MOT1P                  GPIOA
#define PORT_SIG_MOT1M                  GPIOA
#define PORT_SIG_MOT2P                  GPIOA
#define PORT_SIG_MOT2M                  GPIOA
#define PORT_SIG_MOT3P                  GPIOC
#define PORT_SIG_MOT3M                  GPIOC
#define PORT_SIG_MOT4P                  GPIOC
#define PORT_SIG_MOT4M                  GPIOC
#define PORT_SIG_MOT5P                  GPIOA
#define PORT_SIG_MOT5M                  GPIOA
#define PORT_SIG_MOT6P                  GPIOB
#define PORT_SIG_MOT6M                  GPIOB
#define PIN_SIG_MOT1P                   GPIO_Pin_0
#define PIN_SIG_MOT1M                   GPIO_Pin_1
#define PIN_SIG_MOT2P                   GPIO_Pin_2
#define PIN_SIG_MOT2M                   GPIO_Pin_3
#define PIN_SIG_MOT3P                   GPIO_Pin_6
#define PIN_SIG_MOT3M                   GPIO_Pin_7
#define PIN_SIG_MOT4P                   GPIO_Pin_8
#define PIN_SIG_MOT4M                   GPIO_Pin_9
#define PIN_SIG_MOT5P                   GPIO_Pin_8
#define PIN_SIG_MOT5M                   GPIO_Pin_11
#define PIN_SIG_MOT6P                   GPIO_Pin_8
#define PIN_SIG_MOT6M                   GPIO_Pin_9

// Dynamixel
#define PORT_ENABLE_TXD                 GPIOB
#define PORT_ENABLE_RXD                 GPIOB
#define PORT_DXL_TXD                    GPIOB
#define PORT_DXL_RXD                    GPIOB
#define PIN_ENABLE_TXD                  GPIO_Pin_4
#define PIN_ENABLE_RXD                  GPIO_Pin_5
#define PIN_DXL_TXD                     GPIO_Pin_6
#define PIN_DXL_RXD                     GPIO_Pin_7

// Zigbee
#define PORT_ZIGBEE_TXD                 GPIOC
#define PORT_ZIGBEE_RXD                 GPIOD
#define PORT_ZIGBEE_RESET               GPIOA
#define PIN_ZIGBEE_TXD                  GPIO_Pin_12
#define PIN_ZIGBEE_RXD                  GPIO_Pin_2
#define PIN_ZIGBEE_RESET                GPIO_Pin_12

// Serial/PC_UART
#define PORT_PC_TXD                     GPIOB
#define PORT_PC_TXD                     GPIOB
#define PIN_PC_TXD                      GPIO_Pin_10
#define PIN_PC_RXD                      GPIO_Pin_11



//##############################################################################
//##############################################################################
// CM-530 Helper functions - NaN
//##############################################################################
//##############################################################################
typedef enum Button_e {
    UP                                = 0,
    DOWN                              = 1,
    LEFT                              = 2,
    RIGHT                             = 3,
    START                             = 4,
    MIC                               = 5
} Button_t;
typedef enum LED_e {
    POWER                             = 0,
    MANAGE                            = 1,
    PROGRAM                           = 2,
    PLAY                              = 3,
    TXD                               = 4,
    RXD                               = 5,
    AUX                               = 6
} LED_t;
typedef enum Motor_e {
    MOTOR1P                           = 0,
    MOTOR1M                           = 1,
    MOTOR2P                           = 2,
    MOTOR2M                           = 3,
    MOTOR3P                           = 4,
    MOTOR3M                           = 5,
    MOTOR4P                           = 6,
    MOTOR4M                           = 7,
    MOTOR5P                           = 8,
    MOTOR5M                           = 9,
    MOTOR6P                           = 10,
    MOTOR6M                           = 11
} Motor_t;
typedef enum EPortD_e {
    EPORT11                           = 0,
    EPORT15                           = 1,
    EPORT21                           = 2,
    EPORT25                           = 3,
    EPORT31                           = 4,
    EPORT35                           = 5,
    EPORT41                           = 6,
    EPORT45                           = 7,
    EPORT51                           = 8,
    EPORT55                           = 9,
    EPORT61                           = 10,
    EPORT65                           = 11
} EPortD_t;
typedef enum EPortA_e {
    EPORT1A                           = 0,
    EPORT2A                           = 1,
    EPORT3A                           = 2,
    EPORT4A                           = 3,
    EPORT5A                           = 4,
    EPORT6A                           = 5,
    VBUS                              = 6
} EPortA_t;
typedef enum buzzed_note_e {
// Twelve-Tone Equal Temperment (12-TET)
//   1 octave is a doubling of frequency and equal to 1200 cents
//   1 octave => 12 equally distributed notes (12 intervals/semitones)
//     so 100 cents per note
// Tuned to A 440 (440Hz), so 100 cents per note relative to A_5 (440Hz)
// tone_delay = 1/(2*1e-6*f) = 1/(2*1e-6*440*2^(cents_relative/1200))
//   using uDelay(), 50% duty cycle, cents relative to A_4
                                  // tone_delay // cents, ideal frequency
    NOTE_C                            = 30578,  // -5700, 16.35159783 Hz, C_0
    NOTE_Cs                           = 28862,  // -5600, 17.32391444 Hz, Cs_0
    NOTE_Db                           = 28862,  // -5600, 17.32391444 Hz, Db_0
    NOTE_D                            = 27242,  // -5500, 18.35404799 Hz, D_0
    NOTE_Ds                           = 25713,  // -5400, 19.44543648 Hz, Ds_0
    NOTE_Eb                           = 25713,  // -5400, 19.44543648 Hz, Eb_0
    NOTE_E                            = 24270,  // -5300, 20.60172231 Hz, E_0
    NOTE_F                            = 22908,  // -5200, 21.82676446 Hz, F_0
    NOTE_Fs                           = 21622,  // -5100, 23.12465142 Hz, Fs_0
    NOTE_Gb                           = 21622,  // -5100, 23.12465142 Hz, Gb_0
    NOTE_G                            = 20408,  // -5000, 24.49971475 Hz, G_0
    NOTE_Gs                           = 19263,  // -4900, 25.9566436  Hz, Gs_0
    NOTE_Ab                           = 19263,  // -4900, 25.9565436  Hz, Ab_0
    NOTE_A                            = 18182,  // -4800, 27.5        Hz, A_0
    NOTE_As                           = 17161,  // -4700, 29.13523509 Hz, As_0
    NOTE_Bb                           = 17161,  // -4700, 29.13523509 Hz, Bb_0
    NOTE_B                            = 16198,  // -4600, 30.86770633 Hz, B_0
} buzzed_note_t;

//##############################################################################
//##############################################################################
// CM-530 Helper functions - NaN
//##############################################################################
//##############################################################################
typedef struct EasyPort_s {
    GPIO_TypeDef* port;
    uint16_t pin;
} EasyPort_t;

/**
 * Easy Function to control an LED on the CM-530.
 * Possible LED's on the CM-530 are:
 *   MANAGE, PROGRAM, PLAY, TXD, RXD, AUX, POWER
 * @param led The LED to be controlled.
 * @param state The target state of the LED.
 */
void SetLED(LED_t led, uint8_t state);
/**
 * Easy Function to read a button on the CM-530.
 * Possible buttons on the CM-530 are:
 *   START, UP, DOWN, LEFT, RIGHT, MIC
 * @param button The button to be read.
 * @return The state of the button.
 */
uint8_t ReadButton(Button_t button);
/**
 * Easy Function to control the buzzer on the CM-530.
 * @param mlength The length of time [ms] to play the tone.
 * @param tone The length of time [us] of half the period of the wave.
 */
void Buzzed(uint32_t mlength, uint32_t tone);
/**
 * Easy Function to play a musical note with the buzzer on the CM-530.
 * To play Middle C (C4) for 10 seconds:
 *   PlayNote( 10000, NOTE_C, 4 );
 * @param mlength The length of time [ms] to play the note.
 * @param note The musical note to be played.
 * @param octave The octave of the note to be played.
 */
void PlayNote(uint32_t mlength, buzzed_note_t note, uint8_t octave);
/**
 * Easy Function to control a pin of an external port of the CM-530.
 * Using CM-510/530 pin numbering convention:
 *   Pin 5 is gray (beside GND) and Pin 1 is black (beside VCC).
 *     It is very important to note that the numbering convention is reversed
 *     between the IR sensor/Servo motor pages and the CM-510/530 pages on
 *     the Robotis e-Support site.
 * @param pin The pin to be controlled.
 * @param state The target state of the pin.
 */
void SetEPort(EPortD_t pin, uint8_t state);
/**
 * Easy Function to read analog pin (Pin 3) of an external port of the CM-530.
 * @param port The external port to retrieve the analog value.
 * @return The 12-bit analog value from the analog pin.
 */
uint16_t ReadAnalog(EPortA_t port);
/**
 * Easy Function to get an analog value from an IR Module connected to CM-530.
 * Will automatically turn on the IR Module, wait a fixed time period, read 
 * the ADC value, turn off the IR Module, and return the ADC value representing
 * a distance.
 * @param port The external port to retrieve the analog value.
 * @return The 12-bit analog value from the analog pin.
 */
uint16_t ReadIR(EPortA_t port);



//##############################################################################
//##############################################################################
// CM-530 specific configuration/utility functions
//##############################################################################
//##############################################################################
/**
 * High-level function to configure all system settings.
 * Absolutely essential to include at the very beginning of any program using
 * this library. It configures all pins, the ADC's, SysTick counter, U(S)ART's
 * and related clocks. It also initializes the PC UART, Dynamixel, and ZigBee
 * libraries to their default baudrates, and checks the dynamixel bus for any
 * present devices while printing debug info to the PC UART.
 */
void SysInit(void);

/**
 * Function to initialize an independent countdown timer
 * @param nTime Time in milliseconds for the timer to countdown.
 */
void StartCountdown(uint32_t);
/**
 * Independent countdown timer's variable
 * @see StartCountdown()
 */
extern volatile uint32_t glCountdownCounter;

/**
 * Microsecond Delay function (default resolution is 10 [us])
 * @param nTime Time in microseconds to delay (unsigned 32-bit integer).
 * @see mDelay()
 */
void uDelay(uint32_t);    // Actually uses a 10 [us] delay interval
/**
 * Millisecond Delay function
 * @param nTime Time in milliseconds to delay (unsigned 32-bit integer).
 * @see uDelay()
 */
void mDelay(uint32_t);    // Correctly uses 1 [ms] delay interval

//#define BATTERY_MONITOR_INTERVAL        10    // Check once every 10 seconds
//void start_timeout_bat(uint32_t);
#define VBUS_LOW_LIMIT                  115    // 11.5 Volts
/**
 * Function to check battery voltage and sound an alarm if below threshold.
 */
void Battery_Monitor_Alarm(void);



#ifdef __cplusplus
}
#endif

#endif

