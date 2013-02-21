// STM32F103RE running at 3.3V 72MHz

// ADC function runs at ~4us per sample

// TSL1402R clock at 1/8e-6s = 125kHz
// TSL1402R has ~16mm wide active sensing area
 
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    |--         s         --||            /
    |            ________________________/______
                /\                      /    |
               /||\                    /     |
              / || \                  /
             /  ||  \                /
            /   ||   \              /
           /    ||    \            /         q
         d/     ||     \          /
         /      ||      \        /
        /       ||       \      /
       /        ||        \    /
    | /         ||         \  /              |
    |/__      __||__    ____\/_______________|__
   ///                      /\      |
  ///\                     /  \     
 /// Beta                 /    \    f
///    \                 /\Beta \   |
                     ---|========|-----
                        |-  x   -|
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ///                      ||
 Laser                     Lens
Laser                     Sensor
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
(f/x) = (q/s)
q=(f*s)/x
sin(Beta) = q/d
Beta = arctan(f/((n_p/2)*w_p))

x = 256 * 63.5e-6 [m] ~ 16e-3 [m]
s = 50e-3 [m]

f = 20e-3 [m] => q_min = 61.516mm
f = 16e-3 [m] => q_min = 49.213mm

Beta = arctan(f/((num_pixel/2)*width_pixel))

//      = arctan(16000um/((256/2)*63.5um))
//      = 63.069 degrees
//  need laser ~27 degrees from perpendicular to sensor
//    at 50mm to side of center and 16mm forward

// Revo: s=50mm, f=16mm, x=752*6e-3mm, dx=.6e-3mm (1/10 pixel), q=6000mm
// plot blue = q_min [cm], plot red = dq [mm] at 6m
fs=[200:10:2000]; q_min=(fs./(752*6e-3)); dq=(6000*6000).*(.6e-3./fs); hold off; plot(fs,q_min/10,'b'); hold on; plot(fs,dq,'r');hold off; grid on;

// Home: s=200mm, f=30mm, x=256*63.5e-3mm, dx=6.35e-3mm (1/10 pixel), q=6000mm
// plot blue = q_min [cm], plot red = dq [mm] at 6m
fs=[200:10:20000]; q_min=(fs./(256*63.5e-3)); dq=(6000*6000).*(6.35e-3./fs); hold off; plot(fs,q_min/10,'b'); hold on; plot(fs,dq,'r');hold off; grid on;




// plot blue = q_min [cm], plot red = dq [mm] at 6m (10x interpolation)
// plot black = q_min2 [cm], plot green = dq2 [mm] at 6m (100x interpolation)
I1=10;I2=100;q=6000;fs=[200:10:20000]; q_min=(fs./(752*6e-3)); dq=(q^2).*((6e-3/I1)./fs); q_min2=(fs./(256*63.5e-3)); dq2=(q^2).*((63.5e-3/I2)./fs); hold off; plot(fs,q_min./10,'b'); hold on; grid on; plot(fs,q_min2./10,'k'); plot(fs,dq,'r'); plot(fs,dq2,'g');

// plot blue = q_min [cm], plot red = dq [mm] at 6m (10x interpolation)
// plot black = q_min2 [cm], plot green = dq2 [mm] at 6m (100x interpolation)
I1=10;I2=100;q=3000;fs=[200:10:20000]; q_min=(fs./(752*6e-3)); dq=(q^2).*((6e-3/I1)./fs); q_min2=(fs./(256*63.5e-3)); dq2=(q^2).*((63.5e-3/I2)./fs); hold off; plot(fs,q_min./10,'b'); hold on; grid on; plot(fs,q_min2./10,'k'); plot(fs,dq,'r'); plot(fs,dq2,'g');


s=50mm, f=16mm, FOV=2*atan((256*63.5e-6)/(2*16e-3))=~53, beta=~63


*/

// TSL14xx
//   T_int(min) = (256-18)*(1/clk)+20e-6 [s]
//     62.5kHz  = (256-18)*(1/62500)+20e-6 = 3.8280e-3 [s]
//   1000.0kHz  = (256-18)*(1/1e6)+20e-6 = 258e-6 [s]
//   8000.0kHz  = (256-18)*(1/8e6)+20e-6 = 49.75e-6 [s]

// EM-3242 => PC0 => ADC0
// TSL-A12 => PC1 => ADC1
// TSL-CLK => PD5 => OC0B
// TSL-SI1 => PD7

//	Binary output:
//		MAGIC/SYNC:		42
//		ANGLE_MSB:		((angle_>>8)&0xFF)
//		ANGLE_LSB:		(angle_&0xFF)
//		VELOCITY_MSB:	((velocity_>>8)&0xFF)
//		VELOCITY_LSB:	(velocity_&0xFF)
//		DISTANCE_MSB:	((distance_>>8)&0xFF)
//		DISTANCE_LSB:	(distance_&0xFF)
//   	QUALITY_MSB:	((quality_>>8)&0xFF)
//		QUALITY_LSB:	(quality_&0xFF)
//		ERROR:			error_
//		CHECKSUM:		checksum_
//			checksum = ~(

//	ASCII output:
//		ANGLE,DISTANCE,QUALITY,ERROR\r\n


// New plan
// Buffered output variables
volatile uint16_t angle_    = 0;
volatile uint32_t distance_ = 0;
volatile uint32_t quality_  = 0;
volatile uint8_t  error_    = 0;
volatile int16_t  velocity_ = 0;
volatile uint8_t  checksum_ = 0;

// Callback variables
volatile uint16_t angle     = 0;
volatile uint32_t distance  = 0;
volatile uint32_t quality   = 0;
volatile uint8_t  error     = 0;
volatile uint32_t divisor   = 0;
volatile uint16_t counter   = 256;

// Others
volatile uint8_t  dark_threshold = 0;
volatile uint16_t milliseconds_ = 0;

#define OUTPUT_MODE_TEXT			(1<<7)
#define OUTPUT_MODE_ANGL			(1<<6)
#define OUTPUT_MODE_DIST			(1<<5)
#define OUTPUT_MODE_VELO			(1<<4)
#define OUTPUT_MODE_QUAL			(1<<3)
//#define OUTPUT_MODE_TEXT			(1<<2)
//#define OUTPUT_MODE_TEXT			(1<<1)
//#define OUTPUT_MODE_TEXT			(1<<0)
volatile uint8_t output_mode_ = OUTPUT_MODE_ANGL | OUTPUT_MODE_DIST | OUTPUT_MODE_QUAL;

#define ENCODER_HIGH				200
#define ENCODER_LOW					50
#define ENCODER_DIVISOR				(ENCODER_HIGH-ENCODER_LOW)
#define TSL1402_MULTIPLIER			(1)


#define OUTPUT_DIV_HIGH				100000
#define OUTPUT_DIV_LOW				100
#define OUTPUT_DIST_FAR				10000
#define OUTPUT_DIST_NEAR			50



// TSL1402 TIMINGS

// Interrupt
//     B    A    B    O    B    A    B    O    B    A    B    O    B    A    B
// Counter (ADC Sample retrieved)
//         255                 256              pos1/257              pos2/0
// ADC Sample started
//         256              pos1/257              pos2/0                1
// TCNT0
//|____|----|----|____|____|----|----|____|____|----|----|____|____|----|----|__
// CLK
//    256                 257                 258                  1
//|____|----|----|____|____|----|----|____|____|----|----|____|____|----|----|__
// SI1
//|____|____|____|____|____|____|____|____|____|____|____|____|----|----|____|__
// AO1
//|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|~~~~|~~~~|~~
// AO2
//|~~~~|~~~~|~~~~|~~~~|~~~~|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZ
// 255} {    pixel 256    } { rising edge to end + 20us Qtransfer } {    pixel 1


// Interrupt
//  O    B    A    B    O    B    A    B    O    B    A    B    O    B    A    B
// Counter (ADC Sample retrieved)
//            1                   2                   3                   4
// ADC Sample started
//            2                   3                   4                   5
// TCNT0
//__|____|----|----|____|____|----|----|____|____|----|----|____|____|----|----|
// CLK
//       2                   3                   4                   5
//__|____|----|----|____|____|----|----|____|____|----|----|____|____|----|----|
// SI1
//__|____|____|____|____|____|____|____|____|____|____|____|____|____|____|____|
// AO1
//~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|~~~~|
// AO2
//ZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|ZZZZ|
//      } {     pixel 2     } {     pixel 3     } {     pixel 4     } {  pixel 5










#include "cm530.h"

//##############################################################################
//##############################################################################
// Main function of User Program
//##############################################################################
//##############################################################################
int main(void)
{
// Must include this line in all programs using "cm530.h/.c"
	SysInit();

// Reset the LEDs
	SetLED(TXD, 0);
	SetLED(RXD, 0);
	SetLED(AUX, 0);

	SetLED(MANAGE, 0);
	SetLED(PROGRAM, 0);
	SetLED(PLAY, 0);


	// Select EPORT1A and EPORT4A via multiplexer
	GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
	GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);



}

uint32_t sample_pixel(void)
{
	// Ollo Port 1; Pin 5; TSL1402R CLK pin
	SetEPort(EPORT15, 1);

	// Wait until middle of high clock pulse
	uDelay(2);

	// Start ADC1 Software Conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	// Wait for end of high clock pulse
	uDelay(2);

	// Ollo Port 1; Pin 5; TSL1402R CLK pin
	SetEPort(EPORT15, 0);

	// Wait for ADC to finish sample
	uDelay(2);

	uint32_t temp = (uint32_t)(ADC_GetConversionValue(ADC1));

	// Wait for end of low clock pulse
	StartMicroCountdown(2);

	return temp;
}

uint32_t sample_encoder(void)
{
	// Ollo Port 1; Pin 5; TSL1402R CLK pin
	SetEPort(EPORT15, 1);

	// Wait until middle of high clock pulse
	uDelay(2);

	// Start ADC2 Software Conversion
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);

	// Wait for end of high clock pulse
	uDelay(2);

	// Ollo Port 1; Pin 5; TSL1402R CLK pin
	SetEPort(EPORT15, 0);

	// Wait for ADC to finish sample
	uDelay(2);

	uint32_t temp = (uint32_t)(ADC_GetConversionValue(ADC2));

	// Wait for end of low clock pulse
	StartMicroCountdown(2);

	return temp;
}

// Called at TOP=OCR0A; middle of high pulse of TSL1402 clock
void grabber(void)
{
	uint32_t value = 0;

	uint32_t clock_counter = 1;



// Begin capture of first pixel

	// Ollo Port 1; Pin 1; TSL1402R SI pin High for 2us
	SetEPort(EPORT11, 1);

	uDelay(2);

	// Ollo Port 1; Pin 5; TSL1402R CLK pin High for 2us
	SetEPort(EPORT15, 1);

	// Wait until middle of high clock pulse
	uDelay(2);

	// End high pulse to shift register input
	SetEPort(EPORT11, 0);

	// Start ADC1 Software Conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	// Wait for end of high clock pulse
	uDelay(2);

	// Ollo Port 1; Pin 5; TSL1402R CLK pin
	SetEPort(EPORT15, 0);

	// Wait for ADC to finish sample
	uDelay(2);

	value = (uint32_t)(ADC_GetConversionValue(ADC1));

	// Wait for end of low clock pulse
	StartMicroCountdown(2);

	quality = value;
	distance = (value*clock_counter);



// Begin capture of all remaining pixels
	while (clock_counter<256)
	{
		// Wait for last low clock pulse to end
		while (glCountdownCounter>0);

		clock_counter++;

		value = sample_pixel();

		// Some distance calculations
		quality += value;
		distance += (value*clock_counter);
	}
// Done grabbing pixel intensities
// Must wait for falling edge of 257th clock cycle + 20us for charge transfer

// Start capture from encoder
	// EM-3242 encoder on ADC2

	// Wait for last low clock pulse (256) to end
	while (glCountdownCounter>0);

	clock_counter++;

	value = sample_encoder();

	// Wait for last low clock pulse (257) to end
	while (glCountdownCounter>0);
// End of 257th falling edge
// Wait 20us for charge transfer
	StartMicroCountdown(20);





	// Find angle
	angle = angle*180;
	angle_ = (angle)/(ENCODER_DIVISOR);

	if (divisor > OUTPUT_DIV_HIGH)
	{
		distance_ = 0;
		quality_ = 0;
		error_ = 1;
		print();
	}
	else if (divisor < OUTPUT_DIV_LOW)
	{
		distance_ = 0;
		quality_ = 0;
		error_ = 2;
		print();
	}

	clock_counter++;
	}
	else if (clock_counter==257)
	{

		angle = (uint32_t) value;

		uint16_t temp = (uint16_t) distance/divisor;
		if (temp > OUTPUT_DIST_FAR)
		{
			distance_ = 0;
			quality_ = 0;
			error_ = 3;
		}
		else if (temp < OUTPUT_DIST_NEAR)
		{
			distance_ = 0;
			quality_ = 0;
			error_ = 4;
		}
		else
		{
			distance_ = temp;
			quality_ = divisor;
			error_ = 0;
		}
		print();
		clock_counter++;
	}

	// Wait for charge transfer to finish before starting next capture
	while (glCountdownCounter>0);
}

