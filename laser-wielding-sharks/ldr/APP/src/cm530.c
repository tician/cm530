/*
 *******************************************************************************
 *  cm530.c
 *******************************************************************************
 *  -A source file of questionable quality for use with the CM-530 from Robotis.
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
 *******************************************************************************
 *  LEGAL STUFF
 *******************************************************************************
 *
 *  Just to be safe:
 *   'Dynamixel' is property of Robotis, Inc.
 *      http://www.robotis.com
 *
 *
 *  Copyright (c) 2011, 2012, 2013 Matthew Paulishen. All rights reserved.
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
 *******************************************************************************
 */


#include "cm530.h"

#ifdef USING_DYNAMIXEL
uint32_t Baudrate_DXL = 1000000;
#endif
#ifdef USING_ZIGBEE
uint32_t Baudrate_ZIG = 57600;
#endif
#ifdef USING_PC_UART
uint32_t Baudrate_PCU = 57600;
#endif

// If active, will cause STM32 to reboot into the bootloader after receiving
//   'n' sequential '#' characters over either the Pcu or Zig UART (n==15).
// Have not yet identified correct commands to cause ARM to reboot. Probably
//   achieved by setting WatchDog Timer and letting it run out, but do not 
//   want to attempt without confirmation from Robotis (if bootloader does
//   not clear the WDT timer, may get stuck in endless reset loop).
//#define USING_BREAK_TO_BOOTLOADER


// Select which time interval to call SysTick() interrupt
//   Using "USING_SYSTICK_1US" calls interrupt every 72 cycles, so increased
//   accuracy/resolution for Dxl/Pcu timeouts might not be worth the lost cycles
//   due to handling the interrupt.  i.e. 10 [us] interval works fine and will
//   round to nearest 10 [us].
//#define USING_SYSTICK_100US
//#define USING_SYSTICK_10US
#define USING_SYSTICK_1US

#define DEBUG_PRINT_VOLTAGE

//##############################################################################
//##############################################################################
// CM-530 Helper functions - NaN
//##############################################################################
//##############################################################################
// Mic - input - (read(mic, mic) != SET) => heard
    // Pull-up?
    // Differential opamp?
// LED - out - reset(led, led) => ON
    // 5V -> LED -> Resistor -> Pin
// SW  - input - (read(sw, sw) != SET) => pressed
    // Pull-ups
    // 3.3V -> Resistor -> Pin -> GND

//##############################################################################

EasyPort_t
    EasyButton[6] = {
        {PORT_SW_UP, PIN_SW_UP},
        {PORT_SW_DOWN, PIN_SW_DOWN},
        {PORT_SW_LEFT, PIN_SW_LEFT},
        {PORT_SW_RIGHT, PIN_SW_RIGHT},
        {PORT_SW_START, PIN_SW_START},
        {PORT_MIC, PIN_MIC}
    },
    EasyLED[7] = {
        {PORT_LED_POWER, PIN_LED_POWER},
        {PORT_LED_MANAGE, PIN_LED_MANAGE},
        {PORT_LED_PROGRAM, PIN_LED_PROGRAM},
        {PORT_LED_PLAY, PIN_LED_PLAY},
        {PORT_LED_TXD, PIN_LED_TXD},
        {PORT_LED_RXD, PIN_LED_RXD},
        {PORT_LED_AUX, PIN_LED_AUX}
    },
    EasyEPort[12] = {
        {PORT_SIG_MOT1P, PIN_SIG_MOT1P},
        {PORT_SIG_MOT1M, PIN_SIG_MOT1M},
        {PORT_SIG_MOT2P, PIN_SIG_MOT2P},
        {PORT_SIG_MOT2M, PIN_SIG_MOT2M},
        {PORT_SIG_MOT3P, PIN_SIG_MOT3P},
        {PORT_SIG_MOT3M, PIN_SIG_MOT3M},
        {PORT_SIG_MOT4P, PIN_SIG_MOT4P},
        {PORT_SIG_MOT4M, PIN_SIG_MOT4M},
        {PORT_SIG_MOT5P, PIN_SIG_MOT5P},
        {PORT_SIG_MOT5M, PIN_SIG_MOT5M},
        {PORT_SIG_MOT6P, PIN_SIG_MOT6P},
        {PORT_SIG_MOT6M, PIN_SIG_MOT6M}
    };

//##############################################################################
void SetLED(LED_t led, uint8_t state)
{
    if (state)
        GPIO_ResetBits(EasyLED[led].port, EasyLED[led].pin);
    else
        GPIO_SetBits(EasyLED[led].port, EasyLED[led].pin);
}

//##############################################################################
uint8_t ReadButton(Button_t button)
{
    if (GPIO_ReadInputDataBit(EasyButton[button].port, EasyButton[button].pin)!=SET)
        return 1;
    return 0;
}

volatile uint32_t glBuzzerCounter;
void start_countdown_buzzer(uint32_t);
//##############################################################################
void Buzzed(uint32_t mlength, uint32_t tone)
{
    // Twelve-Tone Equal Temperment (12-TET)
    //   1 octave is a doubling of frequency and equal to 1200 cents
    //   1 octave => 12 equally distributed notes (12 intervals/semitones)
    //     so 100 cents per note
    // Tuned to A 440 (440Hz), so 100 cents per note relative to A_5 (440Hz)

    // n [cents] = 1200 log2(b/a)
    // b = a * 2^(n/1200)

    // tone = 1/(2*1e-6*f) = 1/(2*1e-6*440*2^(cents_relative/1200))
    //   using uDelay(), 50% duty cycle, cents relative to A_5

//#define FREQTOTONE(f)    (5000000/f)    // (1/(2*1e-6*f))

    start_countdown_buzzer(mlength);
    while (glBuzzerCounter>0)
    {
        GPIO_ResetBits(PORT_BUZZER, PIN_BUZZER);
        uDelay(tone);
        GPIO_SetBits(PORT_BUZZER, PIN_BUZZER);
        uDelay(tone);
    }
}

//##############################################################################
void PlayNote(uint32_t mlength, buzzed_note_t note, uint8_t octave)
{
    Buzzed(mlength, (uint32_t) (note>>octave));
}

//##############################################################################
void SetEPort(EPortD_t pin, uint8_t state)
{
    if (state)
        GPIO_SetBits(EasyEPort[pin].port, EasyEPort[pin].pin);
    else
        GPIO_ResetBits(EasyEPort[pin].port, EasyEPort[pin].pin);
}

#define ANALOG_RIGHT_BIT_SHIFT          0
//##############################################################################
uint16_t ReadAnalog(EPortA_t port)
{
    if ( (port==EPORT1A) || (port==EPORT4A) )
    {
        // Select EPORT1A and EPORT4A via multiplexer
        GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
        GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);

//        uDelay(5);

        if (port==EPORT1A)
        {
            // Start ADC1 Software Conversion
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            uDelay(4);
            return (uint16_t) (ADC_GetConversionValue(ADC1))>>ANALOG_RIGHT_BIT_SHIFT;
        }
        else
        {
            // Start ADC2 Software Conversion
            ADC_SoftwareStartConvCmd(ADC2, ENABLE);
            uDelay(4);
            return (uint16_t) (ADC_GetConversionValue(ADC2))>>ANALOG_RIGHT_BIT_SHIFT;
        }
    }
    else if ( (port==EPORT2A) || (port==EPORT5A) )
    {
        // Select EPORT2A and EPORT5A via multiplexer
        GPIO_SetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
        GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);

//        uDelay(5);

        if (port==EPORT2A)
        {
            // Start ADC1 Software Conversion
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            uDelay(4);
            return (uint16_t) (ADC_GetConversionValue(ADC1))>>ANALOG_RIGHT_BIT_SHIFT;
        }
        else
        {
            // Start ADC2 Software Conversion
            ADC_SoftwareStartConvCmd(ADC2, ENABLE);
            uDelay(4);
            return (uint16_t) (ADC_GetConversionValue(ADC2))>>ANALOG_RIGHT_BIT_SHIFT;
        }
    }
    else if ( (port==EPORT3A) || (port==EPORT6A) )
    {
        // Select EPORT3A and EPORT6A via multiplexer
        GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
        GPIO_SetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);

//        uDelay(5);

        if (port==EPORT3A)
        {
            // Start ADC1 Software Conversion
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            uDelay(4);
            return (uint16_t) (ADC_GetConversionValue(ADC1))>>ANALOG_RIGHT_BIT_SHIFT;
        }
        else
        {
            // Start ADC2 Software Conversion
            ADC_SoftwareStartConvCmd(ADC2, ENABLE);
            uDelay(4);
            return (uint16_t) (ADC_GetConversionValue(ADC2))>>ANALOG_RIGHT_BIT_SHIFT;
        }
    }
    else if (port==VBUS)
    {
        uint16_t temp;

        // Set ADC1 to read SIG_VDD/VBUS on Channel 13
        ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1 , ADC_SampleTime_239Cycles5);
        uDelay(5);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        uDelay(5);
        temp = (ADC_GetConversionValue(ADC1))>>ANALOG_RIGHT_BIT_SHIFT;

        // Set ADC1 to read SIG_ADC0 (ADC1 multiplexer output) on Channel 10
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1 , ADC_SampleTime_239Cycles5);
        uDelay(5);

        return temp;
    }
    return 0x8000;
}

//##############################################################################
uint16_t ReadIR(EPortA_t port)
{
    uint16_t temp;

    SetEPort((port*2), 1);
    SetEPort((port*2)+1, 0);

    uDelay(25);
    temp = ReadAnalog(port);

    SetEPort((port*2), 0);
    SetEPort((port*2)+1, 0);

    return temp;
}

//##############################################################################
void Battery_Monitor_Alarm(void)
{
    uint16_t volt = ReadAnalog(VBUS)>>4;
#ifdef DEBUG_PRINT_VOLTAGE
    PrintString("\nBattery Voltage: ");
    Printu32d(volt);
    PrintString("e-1 [Volts]\n");
#endif

    // ALARM!!!
    if (volt<VBUS_LOW_LIMIT)
    {
        Buzzed(500,100);
        Buzzed(500,5000);
        Buzzed(500,100);
        Buzzed(500,5000);
        PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        PrintString("Battery Voltage Critical");
        PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    }
    return;
}




//##############################################################################
//##############################################################################
// CM-530 Utility functions
//##############################################################################
//##############################################################################
volatile uint32_t glDelayCounter;
volatile uint32_t glCountdownCounter;
volatile uint32_t glDxlTimeoutCounter;
volatile uint32_t glPcuTimeoutCounter;
//volatile uint32_t glBatTimeoutCounter;
//volatile uint32_t glBatTimeoutSet;
volatile uint8_t gbCounterCount;

void start_timeout_dxl(uint32_t);
void start_timeout_pcu(uint32_t);
#ifdef USING_BREAK_TO_BOOTLOADER
void BreakToBootLoader(void);
#endif

//##############################################################################
void mDelay(uint32_t nTime)
{
    uDelay(nTime*1000);
}

//##############################################################################
void uDelay(uint32_t nTime)
{
    if (glDelayCounter==0)
        gbCounterCount++;

    // Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_100US
    if (nTime>=100)
        glDelayCounter = (nTime/100);
    else
        glDelayCounter = 1;
#elif defined USING_SYSTICK_10US
    if (nTime>=10)
        glDelayCounter = (nTime/10);
    else
        glDelayCounter = 1;
#elif defined USING_SYSTICK_1US
    glDelayCounter = (nTime);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }

    while (glDelayCounter!=0);
}

//##############################################################################
void StartCountdown(uint32_t StartTime)
{
    if (glCountdownCounter==0)
        gbCounterCount++;

    // Want Timer counting in 1 [ms] intervals
#ifdef USING_SYSTICK_100US
    glCountdownCounter = (StartTime*10);
#elif defined USING_SYSTICK_10US
    glCountdownCounter = (StartTime*100);
#elif defined USING_SYSTICK_1US
    glCountdownCounter = (StartTime*1000);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
}

//##############################################################################
void StartMicroCountdown(uint32_t StartTime)
{
    if (glCountdownCounter==0)
        gbCounterCount++;

    // Want Timer counting in 1 [ms] intervals
    glCountdownCounter = (StartTime);

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
}

//##############################################################################
void start_countdown_buzzer(uint32_t nTime)
{
    if (glBuzzerCounter==0)
        gbCounterCount++;

    // Want Timer counting in 1 [ms] intervals
#ifdef USING_SYSTICK_100US
    glBuzzerCounter = (nTime*10);
#elif defined USING_SYSTICK_10US
    glBuzzerCounter = (nTime*100);
#elif defined USING_SYSTICK_1US
    glBuzzerCounter = (nTime*1000);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
}

//##############################################################################
void start_timeout_dxl(uint32_t nTime)
{
    if (glDxlTimeoutCounter==0)
        gbCounterCount++;

    // Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_100US
    if (nTime>=100)
        glDxlTimeoutCounter = (nTime/100);
    else
        glDxlTimeoutCounter = 1;
#elif defined USING_SYSTICK_10US
    if (nTime>=10)
        glDxlTimeoutCounter = (nTime/10);
    else
        glDxlTimeoutCounter = 1;
#elif defined USING_SYSTICK_1US
    glDxlTimeoutCounter = (nTime);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
}

//##############################################################################
void start_timeout_pcu(uint32_t nTime)
{
    if (glPcuTimeoutCounter==0)
        gbCounterCount++;

    // Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_100US
    if (nTime>=100)
        glPcuTimeoutCounter = (nTime/100);
    else
        glPcuTimeoutCounter = 1;
#elif defined USING_SYSTICK_10US
    if (nTime>=10)
        glPcuTimeoutCounter = (nTime/10);
    else
        glPcuTimeoutCounter = 1;
#elif defined USING_SYSTICK_1US
    glPcuTimeoutCounter = (nTime);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
}

//void start_timeout_bat(uint32_t nTime)
//{
//    if (glBatTimeoutCounter==0)
//        gbCounterCount++;
//
//    // Want Timer counting in 1 [s] intervals
//#ifdef USING_SYSTICK_100US
//    glBatTimeoutSet = (nTime*10000);
//#elif defined USING_SYSTICK_10US
//    glBatTimeoutSet = (nTime*100000);
//#elif defined USING_SYSTICK_1US
//    glBatTimeoutSet = (nTime*1000000);
//#endif
//
//    glBatTimeoutCounter = glBatTimeoutSet;
//
//    if (gbCounterCount==1)
//    {
//        // Enable the SysTick Counter
//        SysTick_CounterCmd(SysTick_Counter_Enable);
//    }
//}


#ifdef USING_BREAK_TO_BOOTLOADER
//##############################################################################
void BreakToBootLoader(void)
{
//    WWDG_DeInit();
//    WWDG_SetPrescaler(WWDG_PRESCALER_1);
//    WWDG_SetWindowValue(0x00);
//    WWDG_SetCounter(0x40);
}
#endif



#ifdef USING_DYNAMIXEL
//##############################################################################
//##############################################################################
// Dynamixel SDK platform dependent source
//##############################################################################
//##############################################################################
#define DXL_BUFFER_LENGTH               256

static volatile uint16_t gbDxlWrite=0, gbDxlRead=0;
static volatile uint8_t gbpDxlBuffer[DXL_BUFFER_LENGTH];

uint8_t dxl_hal_open(uint32_t);
void dxl_hal_close(void);
void dxl_hal_clear(void);
uint8_t dxl_hal_tx(uint8_t*, uint8_t);
uint8_t dxl_hal_rx(uint8_t*, uint8_t);
void dxl_hal_set_timeout(uint8_t);
uint8_t dxl_hal_timeout(void);
void RxD_DXL_Interrupt(void);

//##############################################################################
uint8_t dxl_hal_open(uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_DeInit(USART1);
    mDelay(10);
    // Configure USART1 (dynamixel)
    USART_Init(USART1, &USART_InitStructure);

    // Enable USART1 (dynamixel) Receive interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // Enable USART1 (dynamixel)
    USART_Cmd(USART1, ENABLE);

    return 1;
}

//##############################################################################
void dxl_hal_close(void)
{
    // Disable USART1 (dynamixel)
    USART_Cmd(USART1, DISABLE);
}

//##############################################################################
void dxl_hal_clear(void)
{
    // Clear communication buffer
    uint16_t i;
    for (i=0; i<DXL_BUFFER_LENGTH; i++)
        gbpDxlBuffer[i] = 0;
    gbDxlRead = 0;
    gbDxlWrite = 0;
}

//##############################################################################
uint8_t dxl_hal_tx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        // RX Disable
        GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);
        // TX Enable
        GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);

        USART_SendData(USART1,pPacket[i]);
        while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

        // TX Disable
        GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);
        // RX Enable
        GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);
    }

    return numPacket;
}

//##############################################################################
uint8_t dxl_hal_rx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        if (gbDxlRead!=gbDxlWrite)
        {
            pPacket[i] = gbpDxlBuffer[gbDxlRead++];
            if (gbDxlRead>(DXL_BUFFER_LENGTH-1))
                gbDxlRead = 0;
        }
        else
            return i;
    }

    return numPacket;
}

//##############################################################################
void dxl_hal_set_timeout(uint8_t NumRcvByte)
{
    start_timeout_dxl(NumRcvByte*30);
}

//##############################################################################
uint8_t dxl_hal_timeout(void)
{
    if (glDxlTimeoutCounter==0)
        return 1;
    else
        return 0;
}

//##############################################################################
void RxD_DXL_Interrupt(void)
{
    uint8_t temp;
    if (USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)
    {
        temp = USART_ReceiveData(USART1);
    }
    else
        return;

    if (gbDxlWrite<(DXL_BUFFER_LENGTH-1))
    {
        gbpDxlBuffer[gbDxlWrite++] = temp;
    }
    else
    {
        gbpDxlBuffer[gbDxlWrite] = temp;
        gbDxlWrite = 0;
    }

    if (gbDxlRead==gbDxlWrite)
        gbDxlRead++;
    if (gbDxlRead>(DXL_BUFFER_LENGTH-1))
        gbDxlRead=0;
}


//##############################################################################
//##############################################################################
// Dynamixel SDK platform independent source
//##############################################################################
//##############################################################################
#define DXL_MAXNUM_TXPARAM                  160
#define DXL_MAXNUM_RXPARAM                  80

static uint8_t gbInstructionPacket[DXL_MAXNUM_TXPARAM] = {0};
static uint8_t gbStatusPacket[DXL_MAXNUM_RXPARAM] = {0};
static uint8_t gbRxPacketLength = 0;
static uint8_t gbRxGetLength = 0;
static volatile uint16_t gbCommStatus = DXL_RXSUCCESS;
static volatile uint8_t giBusUsing = 0;

void dxl_tx_packet(void);
void dxl_rx_packet(void);
void dxl_clear_statpkt(void);

//##############################################################################
uint8_t dxl_initialize(uint32_t baudrate)
{
    if (dxl_hal_open(baudrate)==0)
        return 0;

    gbCommStatus = DXL_RXSUCCESS;
    giBusUsing = 0;

    return 1;
}

//##############################################################################
void dxl_terminate(void)
{
    dxl_hal_close();
}

//##############################################################################
void dxl_tx_packet(void)
{
    uint8_t i;
    uint8_t TxNumByte, RealTxNumByte;
    uint8_t checksum = 0;

    if (giBusUsing==1)
        return;

    giBusUsing = 1;

    gbCommStatus = 0;

    if (gbInstructionPacket[DXL_PKT_LEN]>(DXL_MAXNUM_TXPARAM+2))
    {
        gbCommStatus |= DXL_TXERROR;
        giBusUsing = 0;
        return;
    }

    if (   (gbInstructionPacket[DXL_PKT_INST] != INST_PING)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_READ_DATA)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_WRITE_DATA)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_REG_WRITE)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_ACTION)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_RESET)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_SYNC_WRITE)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_CAP_REGION) )
    {
        gbCommStatus |= DXL_BAD_INST;
        giBusUsing = 0;
        return;
    }

    gbInstructionPacket[0] = 0xFF;
    gbInstructionPacket[1] = 0xFF;
    for (i=0; i<(gbInstructionPacket[DXL_PKT_LEN]+1); i++)
        checksum += gbInstructionPacket[i+2];
    gbInstructionPacket[gbInstructionPacket[DXL_PKT_LEN]+3] = ~checksum;

    if (gbCommStatus&(DXL_RXFAIL | DXL_RXTIMEOUT | DXL_RXCHECKSUM | DXL_RXLENGTH | DXL_BAD_INST | DXL_BAD_ID))
    {
        dxl_hal_clear();
    }

    TxNumByte = gbInstructionPacket[DXL_PKT_LEN] + 4;
    RealTxNumByte = dxl_hal_tx((uint8_t*)gbInstructionPacket, TxNumByte);

    if (TxNumByte!=RealTxNumByte)
    {
        gbCommStatus |= DXL_TXFAIL;
        giBusUsing = 0;
        return;
    }

    if (gbInstructionPacket[DXL_PKT_INST]==INST_READ_DATA)
        dxl_hal_set_timeout(gbInstructionPacket[DXL_PKT_PARA+1]+6);
    else
        dxl_hal_set_timeout(6);

    gbCommStatus = DXL_TXSUCCESS;
}

//##############################################################################
void dxl_rx_packet(void)
{
    uint8_t i, j, nRead;
    uint8_t checksum = 0;

    if (giBusUsing==0)
        return;

    giBusUsing = 1;

    if (gbInstructionPacket[DXL_PKT_ID]==BROADCAST_ID)
    {
        gbCommStatus = DXL_RXSUCCESS;
        giBusUsing = 0;
        return;
    }

    if (gbCommStatus&DXL_TXSUCCESS)
    {
        gbRxGetLength = 0;
        gbRxPacketLength = 6;
    }

    nRead = dxl_hal_rx((uint8_t*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength-gbRxGetLength);

    gbRxGetLength += nRead;
    if (gbRxGetLength<gbRxPacketLength)
    {
        if (dxl_hal_timeout()==1)
        {
            if (gbRxGetLength==0)
                gbCommStatus = DXL_RXTIMEOUT;
            else
                gbCommStatus = DXL_RXLENGTH;
            giBusUsing = 0;
            return;
        }
    }

    // Find packet header
    for (i=0; i<(gbRxGetLength-1); i++)
    {
        if ( (gbStatusPacket[i]==0xFF) && (gbStatusPacket[i+1]==0xFF) )
        {
            break;
        }
        else if ( (i==gbRxGetLength-2) && (gbStatusPacket[gbRxGetLength-1]==0xFF) )
        {
            break;
        }
    }
    if (i>0)
    {
        for (j=0; j<(gbRxGetLength-i); j++)
            gbStatusPacket[j] = gbStatusPacket[j + i];

        gbRxGetLength -= i;
    }

    // Check if received full packet
    if (gbRxGetLength<gbRxPacketLength)
    {
        gbCommStatus = DXL_RXWAITING;
        return;
    }

    // Check id pairing
    if (gbInstructionPacket[DXL_PKT_ID]!=gbStatusPacket[DXL_PKT_ID])
    {
        gbCommStatus = DXL_BAD_ID | DXL_RXFAIL;
        giBusUsing = 0;
        return;
    }

    gbRxPacketLength = gbStatusPacket[DXL_PKT_LEN] + 4;
    if (gbRxGetLength<gbRxPacketLength)
    {
        nRead = dxl_hal_rx((uint8_t*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength-gbRxGetLength);
        gbRxGetLength += nRead;
        if (gbRxGetLength<gbRxPacketLength)
        {
            gbCommStatus = DXL_RXWAITING;
            return;
        }
    }

    // Check checksum
    for (i=0; i<(gbStatusPacket[DXL_PKT_LEN]+1); i++)
        checksum += gbStatusPacket[i+2];
    checksum = ~checksum;

    if (gbStatusPacket[gbStatusPacket[DXL_PKT_LEN]+3]!=checksum)
    {
        gbCommStatus = DXL_RXCHECKSUM | DXL_RXFAIL;
        giBusUsing = 0;
        return;
    }

    gbCommStatus = DXL_RXSUCCESS;
    giBusUsing = 0;
}

//##############################################################################
void dxl_txrx_packet(void)
{
    dxl_tx_packet();

    if (!(gbCommStatus&DXL_TXSUCCESS))
        return;

    dxl_clear_statpkt();
    do {
        dxl_rx_packet();
        uDelay(50);
    } while (gbCommStatus&DXL_RXWAITING);
}

//##############################################################################
uint16_t dxl_get_result(void)
{
    return gbCommStatus;
}

//##############################################################################
void dxl_set_txpacket_id(uint8_t id)
{
    gbInstructionPacket[DXL_PKT_ID] = id;
}

//##############################################################################
void dxl_set_txpacket_instruction(uint8_t instruction)
{
    gbInstructionPacket[DXL_PKT_INST] = instruction;
}

//##############################################################################
void dxl_set_txpacket_parameter(uint8_t index, uint8_t value )
{
    gbInstructionPacket[DXL_PKT_PARA+index] = value;
}

//##############################################################################
void dxl_set_txpacket_length(uint8_t length)
{
    gbInstructionPacket[DXL_PKT_LEN] = length;
}

//##############################################################################
uint8_t dxl_get_rxpacket_error(uint8_t errbit)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0x80;

    if (gbStatusPacket[DXL_PKT_ERR]&errbit)
        return 1;

    return 0;
}

//##############################################################################
uint8_t dxl_get_rxpacket_length(void)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_LEN];
}

//##############################################################################
uint8_t dxl_get_rxpacket_parameter(uint8_t index)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_PARA+index];
}

//##############################################################################
uint16_t dxl_makeword(uint8_t lowbyte, uint8_t highbyte)
{
    uint16_t word;

    word = highbyte;
    word = word<<8;
    word = word+lowbyte;
    return word;
}

//##############################################################################
uint8_t dxl_get_lowbyte(uint16_t word)
{
    uint16_t temp = (word&0x00FF);
    return (uint8_t) temp;
}

//##############################################################################
uint8_t dxl_get_highbyte(uint16_t word)
{
    uint16_t temp = ((word&0xFF00)>>8);
    return (uint8_t) temp;
}

//##############################################################################
void dxl_ping(uint8_t id)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_PING;
    gbInstructionPacket[DXL_PKT_LEN] = 2;

    dxl_txrx_packet();
}

//##############################################################################
uint8_t dxl_read_byte(uint8_t id, uint8_t address)
{
    while(giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = 1;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();

    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_PARA];
}

//##############################################################################
void dxl_write_byte(uint8_t id, uint8_t address, uint8_t value)
{
    while(giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = value;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();
}

//##############################################################################
uint16_t dxl_read_word(uint8_t id, uint8_t address)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = 2;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();

    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return dxl_makeword(gbStatusPacket[DXL_PKT_PARA], gbStatusPacket[DXL_PKT_PARA+1]);
}

//##############################################################################
void dxl_write_word(uint8_t id, uint8_t address, uint16_t value)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = dxl_get_lowbyte(value);
    gbInstructionPacket[DXL_PKT_PARA+2] = dxl_get_highbyte(value);
    gbInstructionPacket[DXL_PKT_LEN] = 5;

    dxl_txrx_packet();
}

//##############################################################################
void dxl_clear_statpkt(void)
{
    uint8_t i, max=gbStatusPacket[DXL_PKT_LEN];
    if ( (max>0) && (max<DXL_MAXNUM_RXPARAM) )
    {
        for (i=0; i<(max+4); i++)
            gbStatusPacket[i]=0;
    }
    else
    {
        for (i=0; i<6; i++)
            gbStatusPacket[i]=0;
    }
}

//##############################################################################
void dxl_capture(uint8_t id)
{
//    while(giBusUsing);

//    gbInstructionPacket[DXL_PKT_ID] = id;
//    gbInstructionPacket[DXL_PKT_INST] = INST_CAP_REGION;
//    gbInstructionPacket[DXL_PKT_LEN] = 2;

//    dxl_txrx_packet();

    dxl_write_byte(id, 0, 0);
}

//##############################################################################
uint8_t dxl_recover(uint8_t id, HaViMo2_Region_Buffer_t* hvm2rb)
{
    if (hvm2rb==0)//NULL)
        return 0xFF;

    while (giBusUsing);

    uint8_t i;
    for (i=0; i<15; i++)
    {
        gbInstructionPacket[DXL_PKT_ID] = id;
        gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
        gbInstructionPacket[DXL_PKT_PARA] = ((i+1)*16);
        gbInstructionPacket[DXL_PKT_PARA+1] = 16;
        gbInstructionPacket[DXL_PKT_LEN] = 4;

        dxl_txrx_packet();

        if (gbStatusPacket[DXL_PKT_LEN]==(16+2))
        {
        	hvm2rb->valid++;
        }
        else
        {
//            PrintCommStatus(gbCommStatus);
//            PrintErrorCode();
            return hvm2rb->valid;
//            break;
        }

        hvm2rb->rb[i].Index=gbStatusPacket[DXL_PKT_PARA];
        hvm2rb->rb[i].Color=gbStatusPacket[DXL_PKT_PARA+1];
        hvm2rb->rb[i].NumPix=(
                (uint16_t)gbStatusPacket[DXL_PKT_PARA+2]+
                ((uint16_t)gbStatusPacket[DXL_PKT_PARA+3]<<8));
        hvm2rb->rb[i].SumX=
                (
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+4]+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+5]<<8)+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+6]<<16))
                );
        hvm2rb->rb[i].SumY=
                (
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+8]+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+9]<<8)+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+10]<<16))
                );
        hvm2rb->rb[i].MaxX=gbStatusPacket[DXL_PKT_PARA+12];
        hvm2rb->rb[i].MinX=gbStatusPacket[DXL_PKT_PARA+13];
        hvm2rb->rb[i].MaxY=gbStatusPacket[DXL_PKT_PARA+14];
        hvm2rb->rb[i].MinY=gbStatusPacket[DXL_PKT_PARA+15];
    }
    return hvm2rb->valid;
}

#endif



#ifdef USING_PC_UART
//##############################################################################
//##############################################################################
// Serial/PC_UART platform dependent source
//##############################################################################
//##############################################################################
#define PC_UART_BUFFER_LENGTH           128

static volatile uint16_t gbPcuWrite, gbPcuRead;
static volatile uint8_t gbpPcuBuffer[PC_UART_BUFFER_LENGTH]={0};
static volatile uint8_t ReBootToBootLoader;

uint8_t pcu_hal_open(uint32_t);
void pcu_hal_close(void);
void pcu_hal_set_timeout(uint8_t);
uint8_t pcu_hal_timeout(void);

void pcu_put_byte(uint8_t);
uint8_t pcu_get_queue(void);
uint8_t pcu_peek_queue(void);
//uint8_t pcu_get_qstate(void);
void pcu_clear_queue(void);
void pcu_put_queue(void);
void RxD_PCU_Interrupt(void);

//static FILE *PC_UART_Device;

//##############################################################################
uint8_t pcu_hal_open(uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_DeInit(USART3);
    mDelay(10);
    // Configure USART3 (PC_UART)
    USART_Init(USART3, &USART_InitStructure);

    // Enable USART3 (PC_UART) Receive interrupt
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    // Enable USART3 (PC_UART)
    USART_Cmd(USART3, ENABLE);

//    PC_UART_Device = fdevopen( std_putchar, std_getchar );

    return 1;
}

//##############################################################################
void pcu_hal_close(void)
{
    // Disable USART3 (PC UART)
    USART_Cmd(USART3, DISABLE);
}

//##############################################################################
void pcu_hal_set_timeout(uint8_t NumRcvByte)
{
    // 200us; ~180 us to transmit one byte at 57600 bps
    start_timeout_pcu(NumRcvByte*200);
}

//##############################################################################
uint8_t pcu_hal_timeout(void)
{
    if (glPcuTimeoutCounter==0)
        return 1;
    else
        return 0;
}

//##############################################################################
void pcu_put_byte(uint8_t bTxdData)
{
    SetLED(TXD, 1);

    USART_SendData(USART3,bTxdData);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);

    SetLED(TXD, 0);
}

//##############################################################################
uint8_t pcu_get_queue(void)
{
    if (gbPcuWrite==gbPcuRead)
        return 0xFF;

    uint8_t data = gbpPcuBuffer[gbPcuRead++];

    if (gbPcuRead>(PC_UART_BUFFER_LENGTH-1))
        gbPcuRead = 0;

    return data;
}

//##############################################################################
uint8_t pcu_peek_queue(void)
{
    if (gbPcuWrite==gbPcuRead)
        return 0xFF;

    uint8_t data = gbpPcuBuffer[gbPcuRead];

    return data;
}

//##############################################################################
void pcu_put_queue(void)
{
    uint8_t temp;
    if (USART_GetITStatus(USART3, USART_IT_RXNE)!=RESET)
    {
        temp = USART_ReceiveData(USART3);
        if (temp=='#')
            ReBootToBootLoader++;
        else
            ReBootToBootLoader=0;

        if (ReBootToBootLoader>15)
        {
#ifdef USING_BREAK_TO_BOOTLOADER
            //BreakToBootLoader();
#endif
        }
    }
    else
        return;

    SetLED(RXD, 1);

    if (gbPcuWrite<(PC_UART_BUFFER_LENGTH-1))
    {
        gbpPcuBuffer[gbPcuWrite++] = temp;
    }
    else
    {
        gbpPcuBuffer[gbPcuWrite] = temp;
        gbPcuWrite = 0;
    }

    if (gbPcuRead==gbPcuWrite)
        gbPcuRead++;
    if (gbPcuRead>(PC_UART_BUFFER_LENGTH-1))
        gbPcuRead=0;

    SetLED(RXD, 0);
}

//##############################################################################
void pcu_clear_queue(void)
{
    gbPcuWrite = 0;
    gbPcuRead = 0;
}

//##############################################################################
uint8_t pcu_get_qstate(void)
{
    if (gbPcuWrite==gbPcuRead)
    {
        pcu_clear_queue();
        return 0;
    }
    else if (gbPcuRead<gbPcuWrite)
        return (uint8_t) (gbPcuWrite-gbPcuRead);
    else
        return (uint8_t) (PC_UART_BUFFER_LENGTH-(gbPcuRead-gbPcuWrite));
}

//##############################################################################
void RxD_PCU_Interrupt(void)
{
    pcu_put_queue();
}



//##############################################################################
//##############################################################################
// PC UART platform independent source
//##############################################################################
//##############################################################################

//##############################################################################
uint8_t pcu_initialize(uint32_t baudrate)
{
    if (pcu_hal_open(baudrate)==0)
        return 0;

    return 1;
}

//##############################################################################
void pcu_terminate(void)
{
    pcu_hal_close();
}

//##############################################################################
int std_putchar(char c)
{
    if (c=='\n')
    {
        pcu_put_byte((uint8_t) '\r'); //0x0D
        pcu_put_byte((uint8_t) '\n'); //0x0A
    }
    else
    {
        pcu_put_byte((uint8_t) c);
    }

    return c;
}

//##############################################################################
int std_puts(const char *str)
{
    int n=0;
    while (str[n])
        std_putchar(str[n++]);

    return n;
}

//##############################################################################
int std_getchar(void)
{
    char c;

    pcu_hal_set_timeout(10);
    while ( (pcu_hal_timeout()==0) && (pcu_get_qstate()==0) );
    if (pcu_get_qstate()==0)
        return 0xFF;

    c = pcu_get_queue();

    if (c=='\r')
        c = '\n';

    return c;
}

//##############################################################################
char* std_gets(char *str)
{
    uint8_t c, len=0;

    while (len<128)
    {
        pcu_hal_set_timeout(10);
        while ( (pcu_hal_timeout()==0) && (pcu_get_qstate()==0) );
        if (pcu_get_qstate()==0)
        {
            if (len==0)
            {
                return 0;//NULL;
            }
            else
            {
                str[len] = '\0';
                return str;
            }
        }

        c = pcu_get_queue();
        if ( (c=='\n') || (c=='\0') )
        {
            if (len==0)
            {
                return 0;//NULL;
            }
            else
            {
                str[len] = '\0';
                return str;
            }
        }
        else
            str[len++] = (int8_t) c;
    }

    return str;
}


//##############################################################################
void PrintCommStatus(uint16_t Status)
{
    if (Status&DXL_TXFAIL)
        std_puts("\nDXL_TXFAIL: Failed transmit instruction packet!\n");

    if (Status&DXL_RXFAIL)
        std_puts("\nDXL_RXFAIL: Failed get status packet from device!\n");

    if (Status&DXL_TXERROR)
        std_puts("\nDXL_TXERROR: Incorrect instruction packet!\n");

    if (Status&DXL_BAD_INST)
        std_puts("\nDXL_BAD_INST: Invalid Instruction byte\n");

    if (Status&DXL_BAD_ID)
        std_puts("\nDXL_BAD_ID: ID's not same for instruction and status packets\n");

    if (Status&DXL_RXWAITING)
        std_puts("\nDXL_RXWAITING: Now receiving status packet!\n");

    if (Status&DXL_RXTIMEOUT)
        std_puts("\nDXL_RXTIMEOUT: There is no status packet!\n");

    if (Status&DXL_RXCHECKSUM)
        std_puts("\nDXL_RXCHECKSUM: Incorrect status packet checksum!\n");

//    else
//        std_puts("\nThis is unknown error code!\n");
}

//##############################################################################
void PrintErrorCode(void)
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        std_puts("\nInput voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        std_puts("\nAngle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        std_puts("\nOverheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        std_puts("\nOut of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        std_puts("\nChecksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        std_puts("\nOverload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        std_puts("\nInstruction code error!\n");
}

//##############################################################################
int PrintChar(char c){return std_putchar(c);}

//##############################################################################
int PrintString(const char* s){return std_puts(s);}

//##############################################################################
int GetChar(void){return std_getchar();}

//##############################################################################
char* GetString(char* s){return std_gets(s);}

//##############################################################################
void Printu32d(uint32_t lNum)
{
    uint32_t temp, div=1000000000;
    char out[11];
    uint8_t i, j;

    for (i=0; i<10; i++)
    {
        temp = (char) (lNum/div);
        lNum = (lNum%div);
//        lNum -= (uint32_t) (temp*div);
//        out[i] = (char) (temp&0x0000000F)+0x30;
        out[i] = (char) ((temp&0x0F)+0x30);
        div /= 10;
    }
    out[i] = '\0';

    for (i=0; i<10; i++)
    {
        if (out[0]=='0')
        {
            for (j=0; j<10; j++)
            {
                out[j] = out[j+1];
                if (out[j]=='\0')
                    break;
            }
        }
    }

    std_puts(out);
    return;
}

//##############################################################################
void Prints32d(int32_t lNumS)
{
    uint32_t temp, lNum, div=1000000000;
    char out[12];
    uint8_t i, j;

    if (lNum<0)
    {
        out[0] = '-';
        lNum = (uint32_t) ((~lNumS)+1);
    }
    else
    {
        out[0] = '+';
        lNum = (uint32_t) (lNumS);
    }

    for (i=1; i<11; i++)
    {
        temp = (lNum/div);
        lNum = (lNum%div);
//        lNum -= (uint32_t) (temp*div);
//        out[i] = (char) (temp&0x0000000F)+0x30;
        out[i] = (char) ((temp&0x0F)+0x30);
        div /= 10;
    }
    out[i] = '\0';

    for (i=0; i<11; i++)
    {
        if (out[0]=='0')
        {
            for (j=0; j<11; j++)
            {
                out[j] = out[j+1];
                if (out[j]=='\0')
                    break;
            }
        }
    }

    std_puts(out);
    return;
}

//##############################################################################
void Printu16h(uint16_t wNum)
{
    char out[7];
    out[0] = '0';
    out[1] = 'x';
    out[6] = '\0';

    out[2] = (char) ((wNum>>12)&0x0F)+0x30;
    if (out[2] > '9')
        out[2] += 7;

    out[3] = (char) ((wNum>>8)&0x0F)+0x30;
    if (out[3] > '9')
        out[3] += 7;

    out[4] = (char) ((wNum>>4)&0x0F)+0x30;
    if (out[4] > '9')
        out[4] += 7;

    out[5] = (char) (wNum&0x0F)+0x30;
    if (out[5] > '9')
        out[5] += 7;

    std_puts(out);
    return;
}

//##############################################################################
void Printu8h(uint8_t bNum)
{
    char out[5];
    out[0] = '0';
    out[1] = 'x';
    out[4] = '\0';

    out[2] = (char) ((bNum>>4)&0x0F)+0x30;
    if (out[2] > '9')
        out[2] += 7;

    out[3] = (char) (bNum&0x0F)+0x30;
    if (out[3] > '9')
        out[3] += 7;

    std_puts(out);
    return;
}

#endif





#ifdef USING_ZIGBEE
//##############################################################################
//##############################################################################
// Zigbee SDK platform dependent source
//##############################################################################
//##############################################################################
#define ZIGBEE_BUFFER_LENGTH            64

static volatile uint8_t gbZigWrite=0, gbZigRead=0;
static volatile uint8_t gbpZigBuffer[ZIGBEE_BUFFER_LENGTH];

uint8_t zgb_hal_open(uint32_t);
void zgb_hal_close(void);
uint8_t zgb_hal_tx(uint8_t*, uint8_t);
uint8_t zgb_hal_rx(uint8_t*, uint8_t);
void RxD_ZIG_Interrupt(void);

//##############################################################################
uint8_t zgb_hal_open(uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_DeInit(UART5);
    mDelay(10);
    // Configure UART5 (ZigBee)
    USART_Init(UART5, &USART_InitStructure);

    // Enable UART5 (ZigBee) Receive interrupt
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    // Enable UART5 (ZigBee)
    USART_Cmd(UART5, ENABLE);

    // De-activate Reset mode of ZIG-110 module
    GPIO_SetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);

    return 1;
}

//##############################################################################
void zgb_hal_close(void)
{
    // Disable UART5 (ZigBee)
    USART_Cmd(UART5, DISABLE);
    // Activate Reset mode of ZIG-110 module
//    GPIO_SetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);    // original
    GPIO_ResetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);    // correct?
}

//##############################################################################
uint8_t zgb_hal_tx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        SetLED(TXD, 1);

        USART_SendData(UART5,pPacket[i]);
        while (USART_GetFlagStatus(UART5, USART_FLAG_TC)==RESET);

        SetLED(TXD, 0);
    }

    return numPacket;
}

//##############################################################################
uint8_t zgb_hal_rx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        if (gbZigRead!=gbZigWrite)
        {
            pPacket[i] = gbpZigBuffer[gbZigRead++];
            if (gbZigRead>(ZIGBEE_BUFFER_LENGTH-1))
                gbZigRead = 0;
        }
        else
            return i;
    }

    return numPacket;
}

//##############################################################################
void RxD_ZIG_Interrupt(void)
{
    uint8_t temp;
    if (USART_GetITStatus(UART5, USART_IT_RXNE)!=RESET)
    {
        temp = USART_ReceiveData(UART5);
        if (temp=='#')
            ReBootToBootLoader++;
        else
            ReBootToBootLoader=0;

        if (ReBootToBootLoader>15)
        {
#ifdef USING_BREAK_TO_BOOTLOADER
            BreakToBootLoader();
#endif
        }
    }
    else
        return;

    SetLED(RXD, 1);

    if (gbZigWrite<(ZIGBEE_BUFFER_LENGTH-1))
    {
        gbpZigBuffer[gbZigWrite++] = temp;
    }
    else
    {
        gbpZigBuffer[gbZigWrite] = temp;
        gbZigWrite = 0;
    }

    if (gbZigRead==gbZigWrite)
        gbZigRead++;
    if (gbZigRead>(ZIGBEE_BUFFER_LENGTH-1))
        gbZigRead=0;

    SetLED(RXD, 0);
}



//##############################################################################
//##############################################################################
// Zigbee SDK platform independent source
//##############################################################################
//##############################################################################
#define PACKET_LENGTH                   6

static uint8_t gbRcvPacket[PACKET_LENGTH];
static uint8_t gbRcvPacketNum;
static uint16_t gwRcvData;
static volatile uint8_t gbRcvFlag;

//##############################################################################
uint8_t zgb_initialize(uint32_t baudrate)
{
    if (zgb_hal_open(baudrate)==0)
        return 0;

    gbRcvFlag = 0;
    gwRcvData = 0;
    gbRcvPacketNum = 0;
    return 1;
}

//##############################################################################
void zgb_terminate(void)
{
    zgb_hal_close();
}

//##############################################################################
uint8_t zgb_tx_data(uint16_t word)
{
    uint8_t SndPacket[6];
    uint8_t lowbyte = (uint8_t) (word&0xFF);
    uint8_t highbyte = (uint8_t) ((word>>8)&0xFF);

    SndPacket[0] = 0xFF;
    SndPacket[1] = 0x55;
    SndPacket[2] = lowbyte;
    SndPacket[3] = ~lowbyte;
    SndPacket[4] = highbyte;
    SndPacket[5] = ~highbyte;

    if (zgb_hal_tx(SndPacket, 6)!=6)
        return 0;

    return 1;
}

//##############################################################################
uint8_t zgb_rx_check(void)
{
    uint8_t RcvNum;
    uint8_t checksum;
    uint8_t i, j;

    if (gbRcvFlag==1)
        return 1;

    // Fill packet buffer
    if (gbRcvPacketNum<6)
    {
        RcvNum = zgb_hal_rx((uint8_t*)&gbRcvPacket[gbRcvPacketNum], (6-gbRcvPacketNum));
        if (RcvNum!=-1)
            gbRcvPacketNum += RcvNum;
    }

    // Find header
    if (gbRcvPacketNum>=2)
    {
        for (i=0; i<gbRcvPacketNum; i++)
        {
            if (gbRcvPacket[i]==0xFF)
            {
                if (i<=(gbRcvPacketNum-2))
                {
                    if (gbRcvPacket[i+1]==0x55)
                        break;
                }
            }
        }

        if (i>0)
        {
            if (i==gbRcvPacketNum)
            {
                // Cannot find header
                if (gbRcvPacket[i-1]==0xFF)
                    i--;
            }

            // Remove data before header
            for (j=i; j<gbRcvPacketNum; j++)
            {
                gbRcvPacket[j-i] = gbRcvPacket[j];
            }
            gbRcvPacketNum -= i;
        }
    }

    // Verify packet
    if (gbRcvPacketNum==6)
    {
        if ( (gbRcvPacket[0]==0xFF) && (gbRcvPacket[1]==0x55) )
        {
            checksum = ~gbRcvPacket[3];
            if (gbRcvPacket[2]==checksum)
            {
                checksum = ~gbRcvPacket[5];
                if (gbRcvPacket[4]==checksum)
                {
                    gwRcvData = (uint16_t) ((gbRcvPacket[4]<<8)&0xFF00);
                    gwRcvData += gbRcvPacket[2];
                    gbRcvFlag = 1;
                }
            }
        }
        gbRcvPacket[0] = 0x00;
        gbRcvPacketNum = 0;
    }

    return gbRcvFlag;
}

//##############################################################################
uint16_t zgb_rx_data(void)
{
    gbRcvFlag = 0;
    return gwRcvData;
}

#endif



//##############################################################################
//##############################################################################
// CM-530 Configuration functions
//##############################################################################
//##############################################################################
void ISR_Delay_Base(void);

void SysTick_Configuration(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);

//##############################################################################
void SysInit(void)
{
    // Clear the WatchDog Early Wakeup interrupt flag
//    WWDG_ClearFlag();
    ReBootToBootLoader = 0;

    // System Clocks Configuration
    RCC_Configuration();

    // NVIC configuration
    NVIC_Configuration();

    // GPIO configuration
    GPIO_Configuration();

    // System clock count configuration
    SysTick_Configuration();

    // Analog to Digital Converter Configuration
    ADC_Configuration();

//##############################################################################
    uint16_t error=0, tog=0;

#ifdef USING_PC_UART
    mDelay(100);
    if (!pcu_initialize(Baudrate_PCU))
        error|=(1<<0);
#endif
#ifdef USING_ZIGBEE
    mDelay(100);
    if (!zgb_initialize(Baudrate_ZIG))
        error|=(1<<1);
#endif
#ifdef USING_DYNAMIXEL
    mDelay(100);
    if (!dxl_initialize(Baudrate_DXL))
        error|=(1<<2);
#endif

    SetLED(PLAY, (error&(1<<0)));
    SetLED(PROGRAM, (error&(1<<1)));
    SetLED(MANAGE, (error&(1<<2)));

    while(error)
    {
        SetLED(POWER, tog);
        tog ^= 1;

        mDelay(500);
    }

    SetLED(POWER, 1);

    PrintString("\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    PrintString("CM-530 Experimental Example         ");
    PrintString(CM530_FIRMWARE_VERSION);

    PrintString("Battery Voltage: ");
    Printu32d((uint32_t)ReadAnalog(VBUS)>>4);
    PrintString("e-1 [Volts]\n");

    PrintString("PCU:");
#ifdef USING_PC_UART
    Printu32d(Baudrate_PCU);
    PrintString("(bps)\n");
#else
    PrintString("Not in use\n");
#endif

    PrintString("ZIG:");
#ifdef USING_ZIGBEE
    Printu32d(Baudrate_ZIG);
    PrintString("(bps)\n");
#else
    PrintString("Not in use\n");
#endif

    PrintString("DXL:");
#ifdef USING_DYNAMIXEL
    Printu32d(Baudrate_DXL);
    PrintString("(bps)\n");
#else
    PrintString("Not in use\n");
#endif

    Buzzed(150, 200);    // 2500 Hz ~ Ds_7/Eb_7

    uint8_t id, num=0;
    uint16_t wdata;
    for (id=1; id<(250); id++)
    {
        wdata = (dxl_read_byte(id, P_ID)&0x00FF);
        if (wdata==id)
        {
            wdata=0;
            num++;
            PrintString("{");
            Printu32d(id);
            PrintString(", ");

            wdata = dxl_read_word(id, P_MODEL_NUMBER_L);
            error = dxl_get_result();
            if (!(error&DXL_RXSUCCESS))
                PrintCommStatus(error);
            Printu32d(wdata);
            if (wdata==MODEL_AX12)
            {
                PrintString(" (AX-12)");
            }
            else if (wdata==MODEL_AX18)
            {
                PrintString(" (AX-18)");
            }
            else if (wdata==MODEL_AXS1)
            {
                PrintString(" (AX-S1)");
            }
            else if (wdata==MODEL_AXS20)
            {
                PrintString(" (AX-S20)");
            }
            else if (wdata==MODEL_JHFPS)
            {
                PrintString(" (JH-FPS)");
            }
            else if (wdata==MODEL_MX28)
            {
                PrintString(" (MX-28)");
            }
            else if (wdata==MODEL_HaViMo2)
            {
                PrintString(" (HaViMo2)");
            }

            PrintString(", ");
            Printu32d(dxl_read_byte(id, P_FIRMWARE_VERSION));
            PrintString("} \n");
        }
    }
    PrintString("\nDXL DEVICES:");
    Printu32d(num);
    PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    Buzzed(150, 2300);    // 217 Hz ~ A_4

    PrintString("Press START to begin User Program...\n\n");
    
// Wait for START button to be pressed (and toggle MANAGE LED)
    while (!ReadButton(START))
    {
        if (glCountdownCounter==0)
        {
            SetLED(MANAGE, tog);
            tog ^= 1;
            StartCountdown(500);
        }
    }
}

//##############################################################################
void ISR_Delay_Base(void)
{
    // User accessible delay counter
    if (glDelayCounter>1)
    {
        glDelayCounter--;
    }
    else if (glDelayCounter>0)
    {
        glDelayCounter--;
        gbCounterCount--;
    }

    // User accessible timeout/countdown counter
	if (glCountdownCounter>1)
    {
        glCountdownCounter--;
    }
	else if (glCountdownCounter>0)
    {
        glCountdownCounter--;
        gbCounterCount--;
    }

    // Buzzer countdown counter
    if (glBuzzerCounter>1)
        glBuzzerCounter--;
    else if (glBuzzerCounter>0)
    {
        glBuzzerCounter--;
        gbCounterCount--;
    }

    // Dynamixel timeout counter
    if (glDxlTimeoutCounter>1)
        glDxlTimeoutCounter--;
    else if (glDxlTimeoutCounter>0)
    {
        glDxlTimeoutCounter--;
        gbCounterCount--;
    }

    // PC UART timeout counter
    if (glPcuTimeoutCounter>1)
        glPcuTimeoutCounter--;
    else if (glPcuTimeoutCounter>0)
    {
        glPcuTimeoutCounter--;
        gbCounterCount--;
    }


    // Battery Monitor timeout counter
//    if (glBatTimeoutCounter>1)
//        glBatTimeoutCounter--;
//    else
//    {
//        Battery_Monitor_Alarm();
//        if (glBatTimeoutSet>100000)
//            glBatTimeoutCounter = glBatTimeoutSet;
//        else
//            glBatTimeoutCounter = 100000;
//    }

    // If no active counters, disable interrupt
    if (gbCounterCount==0)
    {
        // Disable SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Disable);
        // Clear SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Clear);
    }
}

//##############################################################################
void SysTick_Configuration(void)
{
#ifdef USING_SYSTICK_100US
    // SysTick end of count event each 100us with input clock equal to 9MHz (HCLK/8, default)
    SysTick_SetReload(900);
#elif defined USING_SYSTICK_10US
    // SysTick end of count event each 10us with input clock equal to 9MHz (HCLK/8, default)
    SysTick_SetReload(90);
#elif defined USING_SYSTICK_1US
    // SysTick end of count event each 1us with input clock equal to 9MHz (HCLK/8, default)
    SysTick_SetReload(9);
#endif
    // Enable SysTick interrupt
    SysTick_ITConfig(ENABLE);

    // Reset Active Counter count
    gbCounterCount = 0;
}

//##############################################################################
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    // RCC system reset(for debug purpose)
    RCC_DeInit();

    // Enable HSE
    RCC_HSEConfig(RCC_HSE_ON);

    // Wait till HSE is ready
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus==SUCCESS)
    {
        // Enable Prefetch Buffer
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        // Flash 2 wait state
        FLASH_SetLatency(FLASH_Latency_2);

        // HCLK = SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);

        // PCLK1 = HCLK/2
        RCC_PCLK1Config(RCC_HCLK_Div2);

        // PLLCLK = 8MHz * 9 = 72 MHz
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

        // Enable PLL
        RCC_PLLCmd(ENABLE);

        // Wait till PLL is ready
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET)
        {
        }

        // Select PLL as system clock source
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // Wait till PLL is used as system clock source
        while (RCC_GetSYSCLKSource()!=0x08)
        {
        }
    }

    // Enable peripheral clocks

    // Enable GPIOB and GPIOC clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);

    // Enable USART1 Clock (Dynamixel)
#ifdef USING_DYNAMIXEL
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
#ifdef USING_PC_UART
    // Enable USART3 Clock (PC_UART)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
#ifdef USING_ZIGBEE
    // Enable UART5 Clock (Zigbee)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
#endif

    PWR_BackupAccessCmd(ENABLE);
}

//##############################################################################
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    #ifdef  VECT_TAB_RAM
        // Set the Vector Table base location at 0x20000000
        NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
    #else  // VECT_TAB_FLASH
        // Set the Vector Table base location at 0x08003000
        NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
    #endif

    // Configure the NVIC Preemption Priority Bits
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // Enable the USART1 Interrupt (Dynamixel)
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the USART3 Interrupt (Serial/PC_UART)
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the UART5 Interrupt (Zigbee)
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//##############################################################################
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    // PORTA CONFIG
    GPIO_InitStructure.GPIO_Pin = PIN_SIG_MOT1P | PIN_SIG_MOT1M | PIN_SIG_MOT2P | PIN_SIG_MOT2M  | PIN_SIG_MOT5P | PIN_SIG_MOT5M;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_BUZZER | PIN_ZIGBEE_RESET;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_ADC1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_SW_RIGHT | PIN_SW_LEFT;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    // PORTB CONFIG
    GPIO_InitStructure.GPIO_Pin = PIN_LED_AUX | PIN_LED_MANAGE | PIN_LED_PROGRAM | PIN_LED_PLAY;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_ENABLE_TXD | PIN_ENABLE_RXD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_SW_START;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    // PORTC CONFIG
    GPIO_InitStructure.GPIO_Pin = PIN_SIG_MOT3P | PIN_SIG_MOT3M | PIN_SIG_MOT4P | PIN_SIG_MOT4M;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_ADC_SELECT0 | PIN_ADC_SELECT1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_LED_POWER | PIN_LED_TXD | PIN_LED_RXD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_TXD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_ADC0 | PIN_VDD_VOLT;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_MIC;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_SW_UP | PIN_SW_DOWN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    // PORTD CONFIG
    GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_RXD;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
    GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);
}


void ADC_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_StructInit(&ADC_InitStructure);

    // ADC1 configuration
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC2, &ADC_InitStructure);

    // ADC1 regular channels configuration
        // Set ADC1 to read SIG_ADC0 (ADC1 multiplexer output) on Channel 10
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1 , ADC_SampleTime_239Cycles5);
        // Set ADC1 to read VBUS on Channel 13
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1 , ADC_SampleTime_239Cycles5);    // SIG_VDD/VBUS
    //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    // ADC2 regular channels configuration
        // Set ADC2 to read SIG_ADC1 (ADC2 multiplexer output) on Channel 5
    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);    // SIG_ADC1
    //ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

    // Enable ADC1 DMA
    //ADC_DMACmd(ADC1, ENABLE);

    // Enable ADC1,2
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    // Enable ADC1,2 reset calibration register
    // Check the end of ADC1,2 reset calibration register
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));

    // Start ADC1,2 calibration
    // Check the end of ADC1,2 calibration
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));


    // Start ADC2 Software Conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}
