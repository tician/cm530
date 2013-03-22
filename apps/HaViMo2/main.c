/*
 *******************************************************************************
 *  main.c - HaViMo2 Pan-Tilt Tracking Example (CM-530 version)
 *******************************************************************************
 *  -A very basic program to track an object using the HaViMo2 vision module
 *    mounted on a pan-tilt dynamixel turret and all controlled by a (beta?)
 *    CM-530 from Robotis, Inc. <http://www.robotis.com>.
 *******************************************************************************
 *  -A far more in-depth Embedded-C based introduction to using the HaViMo2
 *    from its creator is at:
 *      <http://robosavvy.com/site/Builders/hamid_m/example.c>
 *    That CM-5 example source code was posted on the RoboSavvy forums in the
 *    "Bioloid Vision Module" thread (page 4, post 9).
 *      <http://robosavvy.com/forum/viewtopic.php?t=1341&postdays=0&postorder=asc&start=45>
 *  
 *******************************************************************************
 *  LEGAL STUFF
 *******************************************************************************
 *  "THE RAMEN-WARE LICENSE":
 *  <ticiane1@uga.edu> wrote this file in all its glorious failure.
 *    You can do absolutely whatever you want with this collection of bits.
 *    It is free software, and comes without any warranty whatsoever. Usage is
 *    entirely at your own risk. If you happen to find it useful, then why not
 *    spend some of the limited time/energy/money you save to have a good meal
 *    with family, friends, and even perfect strangers.
 *      --Matthew Paulishen. 2011, 2012.
 *******************************************************************************
 */

#include "cm530.h"

//#define VERBOSE_IMAGE_PROCESSING
void ProcessImage(uint8_t TrackingColor);
HaViMo2_Region_Buffer_t h2rb;

volatile uint8_t Targetx, Targety;
volatile uint8_t TrackFound;

#define DXL_PAN_SERVO                   19
#define DXL_TILT_SERVO                  20

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


// Main Code for Testing HaViMo2

	volatile uint16_t HeadVPos, HeadHPos;
	char input=0;

	SetLED(MANAGE, 1);
	SetLED(PROGRAM, 0);
	SetLED(PLAY, 0);

	dxl_write_byte(DXL_PAN_SERVO, AXM_TORQUE_ENABLE, 1);
	dxl_write_byte(DXL_TILT_SERVO, AXM_TORQUE_ENABLE, 1);
	dxl_write_word(DXL_PAN_SERVO, AXM_GOAL_POSITION_L, 512);
	dxl_write_word(DXL_TILT_SERVO, AXM_GOAL_POSITION_L, 512);

	SetLED(MANAGE, 0);
	SetLED(PROGRAM, 0);
	SetLED(PLAY, 1);
	mDelay(2000);


	while(1) 
	{
		// For debug/testing without a serial connection and keyboard
		mDelay(100);
		while (!ReadButton(UP) && !ReadButton(LEFT) && !ReadButton(RIGHT));

		if (ReadButton(UP))
			input='T';
		else if (ReadButton(LEFT))
			input='L';
		else if (ReadButton(RIGHT))
			input='R';
		else
			input='0';

		PrintString("\nInput=");
		PrintChar(input);
		mDelay(2000);

	//Track Target
		if (input=='T')
		{
			HeadHPos=512;
			HeadVPos=512;
			PrintString("\nStart tracking\n");

		// Track for 20 seconds
//			StartCountdown(20000);
//			while (glCountdownCounter>0)
		// Track Indefinitely
			while (!ReadButton(DOWN))
			{
#ifdef VERBOSE_IMAGE_PROCESSING
				PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
				PrintString("\nBegin Processing: ");
				Printu32d(glCountdownCounter);
#endif
				ProcessImage(White);
#ifdef VERBOSE_IMAGE_PROCESSING
				PrintString("\nDone Processing:  ");
				Printu32d(glCountdownCounter);
#endif
				if (TrackFound)
				{
					HeadHPos-=((int)Targetx-80)/2;
					HeadVPos-=((int)Targety-60)/2;
					dxl_write_word(DXL_PAN_SERVO, AXM_GOAL_POSITION_L, HeadHPos);
					dxl_write_word(DXL_TILT_SERVO, AXM_GOAL_POSITION_L, HeadVPos);
					PrintString("\n Target X,Y: ");
					Printu32d(Targetx);
					PrintString(",");
					Printu32d(Targety);
				}
#ifdef VERBOSE_IMAGE_PROCESSING
				PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
#endif
			}
			PrintString("\nDone tracking\n");
		}
	}
	return 0;
}



//##############################################################################
//##############################################################################
// Function to retrieve (x,y) coordinates of tracked color
//##############################################################################
//##############################################################################
void ProcessImage(uint8_t TrackingColor)
{
	TrackFound=0;

	// Ping HaViMo2
		// If responds -> done processing last image, get results
		// Else -> still processing, wait/try again later
	dxl_ping( HaViMo2_ID );
#ifdef VERBOSE_IMAGE_PROCESSING
	PrintCommStatus(dxl_get_result());
#endif
	if (!(dxl_get_result()&DXL_RXSUCCESS))
	{
#ifdef VERBOSE_IMAGE_PROCESSING
		PrintString("\nNo response...");
#endif
		return;
	}
#ifdef VERBOSE_IMAGE_PROCESSING
	PrintString("\nImage ready...");
#endif

// Recover current Region Buffer
	dxl_recover( HaViMo2_ID, &h2rb );
// Start capture of next image
	dxl_capture( HaViMo2_ID );

#ifdef VERBOSE_IMAGE_PROCESSING
	PrintString("\nNumber of Regions found: ");
	Printu32d(h2rb.valid);
	PrintString("\n");
#endif

	uint8_t i;

	// Examine the Region Buffer
	for (i=0; i<15; i++)
	{
		// is the region is valid?
		if (h2rb.rb[i].Index!=0)
		{
			// Colors are:
			//   0/Unknown/Black, 1/Ball/Teal, 2/Field/Red, 3/MyGoal/Green,
			//   4/OppGoal/Purple, 5/Robot/White, 6/Cyan, 7/Magenta

			if (h2rb.rb[i].Color==TrackingColor)
			{
				// bigger than the last region found
				if(h2rb.rb[i].NumPix>TrackFound)
				{
					Targetx=h2rb.rb[i].SumX/h2rb.rb[i].NumPix;
					Targety=h2rb.rb[i].SumY/h2rb.rb[i].NumPix;
					TrackFound=h2rb.rb[i].NumPix;
				}
			}
		}
	}
	return;
}
