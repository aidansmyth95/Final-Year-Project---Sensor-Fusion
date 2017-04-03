/*  *********************************************
    SparkFun_ADXL345_Example
    Triple Axis Accelerometer Breakout - ADXL345
    Hook Up Guide Example

    Utilizing Sparkfun's ADXL345 Library
    Bildr ADXL345 source file modified to support
    both I2C and SPI Communication

    E.Robert @ SparkFun Electronics
    Created: Jul 13, 2016
    Updated: Sep 06, 2016

    Development Environment Specifics:
    Arduino 1.6.11

    Hardware Specifications:
    SparkFun ADXL345
    Arduino Uno
 *  *********************************************/

#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library
#include <Servo.h>
/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
//ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION
Servo myservo;
int pos = 1500;    // variable to store the servo position
int servo_tilt = 250;
long interval = 5;  //in milliseconds
long previousMillis;
boolean up = true;
/****************** INTERRUPT ******************/
/*      Uncomment If Attaching Interrupt       */
//int interruptPin = 2;                 // Setup pin 2 to be the interrupt pin (for most Arduino Boards)

double accX = 0.0;
double accY = 0.0;
double accZ = 0.0;

double timeStamp;
uint32_t timer;

#define RADIANS_TO_DEGREES 57.2958 // 180/3.14159

#define minx 23.00
#define maxx 28.00
#define miny 7.00
#define maxy 8.00
#define minz 0.00
#define maxz 247.00


  /* Note: Must perform offset and gain calculations prior to seeing updated results
    /  Refer to SparkFun ADXL345 Hook Up Guide: https://learn.sparkfun.com/tutorials/adxl345-hookup-guide
    /  offsetAxis = 0.5 * (Acel+1g + Accel-1g)
    /  gainAxis = 0.5 * ((Acel+1g - Accel-1g)/1g) */

/************** DEFINED VARIABLES **************/
/*                                             */
double offsetX = (minx+maxx)/2;       // OFFSET values
double offsetY = (miny+maxy)/2 ;      // OFFSET values
double offsetZ = (minz+maxz)/2  ;     // OFFSET values

double gainX = (minx-maxx)/1     ;  // OFFSET values
double gainY = (miny-maxy)/1      ; // OFFSET values
double gainZ = (minz-maxz)/1       ;// OFFSET values

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);
  myservo.writeMicroseconds(1500);   //1000 & 2000 seems to be 45 degrees from centre, tilt of 500
  previousMillis = 0;        // will store last time LED was updated

  Serial.begin(115200);                 // Start the serial terminal

  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(2);           // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  // Default: Set to 1
  // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library

  adxl.setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)

  adxl.setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)

  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment

  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment

  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  // This library may have a problem using INT2 pin. Default to INT1 pin.

  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(1);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);

  //attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt
  //start a timer
  timer = micros();
  timeStamp = 0.00;

}

/****************** MAIN CODE ******************/
/*     Accelerometer Readings and Interrupt    */
void loop() {

  // Accelerometer Readings
  int x, y, z;
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

  double dt = (double)(micros() - timer) / 1000000; //stops the timer, convert to seconds from microseconds, and cast as a double
  timer = micros(); //start timer again for the next dt.
  timeStamp = timeStamp + dt;

  accX = (x - offsetX) / gainX;
  accY = (y - offsetY) / gainY;
  accZ = z;
  double roll =  atan2(y, z) * RADIANS_TO_DEGREES;
  double pitch = atan2(-x, z) * RADIANS_TO_DEGREES;
  // Output Results to Serial
  Serial.print(timeStamp); Serial.print("\t");
  Serial.print(roll); Serial.print("\t");
  Serial.println(pitch);

  //ADXL_ISR();
  // You may also choose to avoid using interrupts and simply run the functions within ADXL_ISR();
  //  and place it within the loop instead.
  // This may come in handy when it doesn't matter when the action occurs.

  // Servo control ///////////////////////////////////////////////////////
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval) {
    previousMillis = currentMillis;

    if ( pos >= 1500 + servo_tilt) {
      up = false;
    }
    if ( pos <= 1500 - servo_tilt) {
      up = true;
    }
    if (up == true) {
      pos = pos + 10;
    }
    else {
      pos = pos - 10;
    }
    if(pos >= 1500 - servo_tilt && pos <= 1500 + servo_tilt)  myservo.writeMicroseconds(pos);   //1000 & 2000 seems to be 45 degrees from centre, tilt of 500
  }
}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {

  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();

  // Free Fall Detection
  if (adxl.triggered(interrupts, ADXL345_FREE_FALL)) {
    Serial.println("*** FREE FALL ***");
    //add code here to do when free fall is sensed
  }

  // Inactivity
  if (adxl.triggered(interrupts, ADXL345_INACTIVITY)) {
    Serial.println("*** INACTIVITY ***");
    //add code here to do when inactivity is sensed
  }

  // Activity
  if (adxl.triggered(interrupts, ADXL345_ACTIVITY)) {
    Serial.println("*** ACTIVITY ***");
    //add code here to do when activity is sensed
  }

  // Double Tap Detection
  if (adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)) {
    Serial.println("*** DOUBLE TAP ***");
    //add code here to do when a 2X tap is sensed
  }

  // Tap Detection
  if (adxl.triggered(interrupts, ADXL345_SINGLE_TAP)) {
    Serial.println("*** TAP ***");
    //add code here to do when a tap is sensed
  }
}




