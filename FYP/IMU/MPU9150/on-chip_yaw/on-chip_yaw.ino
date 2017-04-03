
/*
   Sources included in code implementation:
     http://www.tkjelectronics.com
     http://www.pitt.edu/~mpd41/Angle.ino
     https://www.google.ie/url?sa=t&rct=j&q=&esrc=s&source=web&cd=3&cad=rja&uact=8&ved=0ahUKEwjKxLPDrIjSAhVBAsAKHZY8DxkQFggrMAI&url=http%3A%2F%2Fwww.geekmomprojects.com%2Fmpu-6050-redux-dmp-data-fusion-vs-complementary-filter%2F&usg=AFQjCNGZhpCB0ZnuQ1yK7MNKn72BWPFPuA&sig2=bfmnJZdS2C-KLjzIB7jm6Q&bvm=bv.146786187,d.ZGg

*/

#include <Servo.h>
#include "Kalman.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define RADIANS_TO_DEGREES 57.2958 // 180/3.14159

//  DMP Variables
MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

const int MPU_addr = 0x68;
const int FS_SEL = 0;         // Default sensitiviy full scale range
double timeStamp;
double DMP_roll, DMP_pitch, DMP_yaw;
uint32_t timer;

// calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;


// INTERRUPT DETECTION ROUTINE //////////////////////////
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Wire.begin();
  Serial.flush();
  Serial.end();
  Serial.begin(115200);
  mpu.initialize();
  delay(100);

  // DMP SETUP
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

  //start a timer
  timer = micros();
  timeStamp = 0.00;
}




void loop() {
  
  double dt = (double)(micros() - timer) / 1000000; //stops the timer, convert to seconds from microseconds, and cast as a double
  timer = micros(); //start timer again for the next dt.
  timeStamp = timeStamp + dt;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //DMP
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt
  }

  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  }

  DMP_roll = ypr[2] * RADIANS_TO_DEGREES;
  DMP_pitch = -ypr[1] * RADIANS_TO_DEGREES;
  DMP_yaw = ypr[0] * RADIANS_TO_DEGREES;

  Serial.print(timeStamp); Serial.print("\t");

  if (timeStamp > 40.0) { //give time to settle, especially DMP

    Serial.println(DMP_yaw);


  }


}


