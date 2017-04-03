/*  *****************************************
    ADXL345_Calibration
    ADXL345 Hook Up Guide Calibration Example

    Utilizing Sparkfun's ADXL345 Library
    Bildr ADXL345 source file modified to support
    both I2C and SPI Communication

    E.Robert @ SparkFun Electronics
    Created: Jul 13, 2016
    Updated: Sep 13, 2016

    Development Environment Specifics:
    Arduino 1.6.11

    Hardware Specifications:
    SparkFun ADXL345
    Arduino Uno
 *  *****************************************/

#include <SparkFun_ADXL345.h>
#include <Servo.h>

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
//ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** VARIABLES ******************/
/*                                             */
double AccelMinX = 100;
double AccelMaxX = 0;
double AccelMinY = 100;
double AccelMaxY = 0;
double AccelMinZ = 100;
double AccelMaxZ = 0;

int accX = 0;
int accY = 0;
int accZ = 0;

Servo myservo;

/************** DEFINED VARIABLES **************/
/*                                             */
#define offsetX   -123       // OFFSET values
#define offsetY   -16
#define offsetZ   -10

#define gainX     133        // GAIN factors
#define gainY     261
#define gainZ     248

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup()
{
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);
  myservo.writeMicroseconds(1500);   //1000 & 2000 seems to be 45 degrees from centre, tilt of 500
  Serial.begin(115200);                 // Start the serial terminal
  Serial.println("SparkFun ADXL345 Accelerometer Breakout Calibration");
  Serial.println();

  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(2);           // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                // Configure the device: 4 wire SPI mode = '0' or 3 wire SPI mode = 1
  // Default: Set to 1
  // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library
}

/****************** MAIN CODE ******************/
/*  Accelerometer Readings and Min/Max Values  */
void loop()
{
  Serial.println("Send any character to display values.");
  while (!Serial.available()) {}      // Waiting for character to be sent to Serial
  Serial.println();

  // Get the Accelerometer Readings
  int x, y, z;                        // init variables hold results
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store in variables x,y,z

  if (x < AccelMinX) AccelMinX = x;
  if (x > AccelMaxX) AccelMaxX = x;

  if (y < AccelMinY) AccelMinY = y;
  if (y > AccelMaxY) AccelMaxY = y;

  if (z < AccelMinZ) AccelMinZ = z;
  if (z > AccelMaxZ) AccelMaxZ = z;

  Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  "); Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
  Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  "); Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
  Serial.println();


  /* Note: Must perform offset and gain calculations prior to seeing updated results
    /  Refer to SparkFun ADXL345 Hook Up Guide: https://learn.sparkfun.com/tutorials/adxl345-hookup-guide
    /  offsetAxis = 0.5 * (Acel+1g + Accel-1g)
    /  gainAxis = 0.5 * ((Acel+1g - Accel-1g)/1g) */

  // UNCOMMENT SECTION TO VIEW NEW VALUES
  accX = (x - offsetX) / gainX;       // Calculating New Values for X, Y and Z
  accY = (y - offsetY) / gainY;
  accZ = (z - offsetZ) / gainZ;

  Serial.print("New Calibrated Values: "); Serial.print(accX); Serial.print("  "); Serial.print(accY); Serial.print("  "); Serial.print(accZ);
  Serial.println();

  while (Serial.available())
  {
    Serial.read();                    // Clear buffer
  }
}
