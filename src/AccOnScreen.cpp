#include <Adafruit_ST7789.h>
#include <Arduino.h>
#include <Fonts/FreeSans9pt7b.h>
#include "dso32.h"
#include "RTClib.h"

DSO32 dso32 = DSO32(&Wire, LSM6DSO32_I2C_ADD_L);
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);
RTC_PCF8523 rtc;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  dso32.begin();

  display.init(135, 240);
  display.setRotation(3);
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(ST77XX_WHITE);

  dso32.Set_X(LSM6DSO32_XL_ODR_104Hz_NORMAL_MD);
  dso32.Set_X_FS(LSM6DSO32_4g);

  dso32.Set_G(LSM6DSO32_GY_ODR_104Hz_NORMAL_MD);
  dso32.Set_G_FS(LSM6DSO32_250dps);

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop()
{
  canvas.fillScreen(ST77XX_BLACK);
  canvas.setCursor(0, 17);
  // Read temperature
  canvas.setTextColor(ST77XX_RED);
  canvas.print("Temperature :");
  canvas.setTextColor(ST77XX_WHITE);
  uint8_t tempStatus;
  dso32.Get_T_DRDY_Status(&tempStatus);
  if (tempStatus == 1)
  {
    float_t temp;
    dso32.Get_Temp(&temp);
    canvas.print(temp);
  }
  canvas.println("C");
  // Read accelerometer
  canvas.setTextColor(ST77XX_YELLOW);
  canvas.println("Acceleration :");
  canvas.setTextColor(ST77XX_WHITE);
  canvas.print("X: ");
  uint8_t acceleroStatus;
  dso32.Get_X_DRDY_Status(&acceleroStatus);
  if (acceleroStatus == 1)
  { // Status == 1 means a new data is available
    float_t acceleration[3];
    dso32.Get_X_Axes(acceleration);
    // Plot data for each axis in mg
    canvas.print(acceleration[0] / 100, 2);
    canvas.print(" Y: ");
    canvas.print(acceleration[1] / 100), 2;
    canvas.print(" Z: ");
    canvas.println(acceleration[2] / 100), 2;
  }

  // Read gyroscope
  canvas.setTextColor(ST77XX_GREEN);
  canvas.println("Rotation :");
  canvas.setTextColor(ST77XX_WHITE);
  canvas.print("X: ");
  uint8_t gyroStatus;
  dso32.Get_G_DRDY_Status(&gyroStatus);
  if (gyroStatus == 1)
  { // Status == 1 means a new data is available
    float_t rotation[3];
    dso32.Get_G_Axes(rotation);
    // Plot data for each axis in milli degrees per second
    canvas.print(rotation[0] / 1000, 2);
    canvas.print(" Y: ");
    canvas.print(rotation[1] / 1000, 2);
    canvas.print(" Z: ");
    canvas.println(rotation[2] / 1000, 2);
  }

  // Read Time
  DateTime now = rtc.now();
  canvas.setTextColor(ST77XX_BLUE);
  canvas.print("Time : ");
  canvas.setTextColor(ST77XX_WHITE);
  canvas.print(now.hour());
  canvas.print(":");
  canvas.print(now.minute());
  canvas.print(":");
  canvas.println(now.second());

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  delay(1000);
  return;
}
