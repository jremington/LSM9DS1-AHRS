/*****************************************************************
  Modified from LSM9DS1_Basic_I2C.ino
  SFE_LSM9DS1 Library Simple Example Code - I2C Interface
  Jim Lindblom @ SparkFun Electronics
  Original Creation Date: April 30, 2015
  https://github.com/sparkfun/LSM9DS1_Breakout

  SFE example Modified to print the raw sensor data
  using default settings. SJR 2/2021

  Adafruit 3V or 5V board
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
	LSM9DS1 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VIN ------------- 5V
	 GND ------------- GND

   CSG, CSXM, SDOG, and SDOXM should all be pulled high.
   pullups on the ADAFRUIT breakout board do this.

  This code is beerware. If you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!

  Distributed as-is; no warranty is given.
*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// default settings gyro  245 d/s, accel = 2g, mag = 4G
LSM9DS1 imu;.

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
// #define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
// #define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////

#define PRINT_SPEED 300 // 300 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

void setup()
{
  Serial.begin(9600);
  while (!Serial); //wait for connection

  Wire.begin();

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }
  Serial.println("Collecting gyro data, hold still");
  delay(3000);
  // get gyro offset
  long gxa = 0, gya = 0, gza = 0;
  for (int i = 0; i < 300; i++) {
    if ( imu.gyroAvailable() )
    {
      imu.readGyro();
      gxa += imu.gx;
      gya += imu.gy;
      gza += imu.gz;
    }
  }
  Serial.println(F("gyro offsets"));
  Serial.print(gxa / 300);
  Serial.print(", ");
  Serial.print(gya / 300);
  Serial.print(", ");
  Serial.println(gza / 300);
  Serial.println();
  
  Serial.println(F("rotate slowly and carefully in 3D"));
  delay(3000);
  Serial.println("starting");
}

void loop()
{
  static int n = 0; //count values transmitted
  // Update the sensor values whenever new data is available

  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    imu.readMag();
  }

  if (millis() - lastPrint > PRINT_SPEED)
  {
    Serial.print(imu.ax);
    Serial.print(", ");
    Serial.print(imu.ay);
    Serial.print(", ");
    Serial.print(imu.az);
    Serial.print(", ");
    Serial.print(imu.mx);
    Serial.print(", ");
    Serial.print(imu.my);
    Serial.print(", ");
    Serial.println(imu.mz);
    if (n++ > 299) {
      Serial.println("Done.");
      while (1); //hang here
    }
    lastPrint = millis(); // Update lastPrint time
  }
}
