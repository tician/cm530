#include "cm530.h"

int main(void)
{
// Must include this line in all programs using "cm530.h/.c"
	SysInit();

	while(1)
	{
		// Reset the LEDs
		SetLED(TXD, 0);
		SetLED(RXD, 0);
		SetLED(AUX, 0);

		SetLED(MANAGE, 0);
		SetLED(PROGRAM, 0);
		SetLED(PLAY, 0);

		mDelay(1000);

		SetLED(TXD, 1);
		SetLED(RXD, 1);
		SetLED(AUX, 1);

		SetLED(MANAGE, 1);
		SetLED(PROGRAM, 1);
		SetLED(PLAY, 1);

		mDelay(1000);
	}

	return 0;
}
