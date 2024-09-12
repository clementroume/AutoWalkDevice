#include "RTClib.h"
#include <Arduino.h>
#include <SPI.h>

RTC_PCF8523 rtc;

void setup()
{
	Serial.begin(57600);

	if (!rtc.begin())
	{
		Serial.println("Couldn't find RTC");
		Serial.flush();
		while (1)
			delay(10);
	}
	rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	rtc.start();
}

void loop()
{
	DateTime now = rtc.now();

	Serial.print(now.hour(), DEC);
	Serial.print(':');
	Serial.print(now.minute(), DEC);
	Serial.print(':');
	Serial.print(now.second(), DEC);
	Serial.println();

	delay(1000);
}