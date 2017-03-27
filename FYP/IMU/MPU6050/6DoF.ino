
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

Servo myservo;  // create servo object to control a servo. 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
int pos = 1500;    // variable to store the servo position
int servo_tilt = 500;
long interval = 5;  //in milliseconds
long previousMillis;
boolean up = true;

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

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

const int MPU_addr = 0x68;
const int FS_SEL = 0;         // Default sensitiviy full scale range
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //These will be the raw data from the MPU6050.
double roll, pitch, gyroXrate, gyroYrate;
double alpha = 0.98;
double compAngleX, compAngleY; // Calculated angles in the complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double GYRO_FACTOR = 131.0 / (FS_SEL + 1);  // Default sensitiviy full scale range factor
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

void set_last_read_angle_data(float x, float y, float x_gyro, float y_gyro) {
  AcX = x;
  AcY = y;
  GyX = x_gyro;
  GyY = y_gyro;
}

// INTERRUPT DETECTION ROUTINE //////////////////////////
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// CALIBRATION_ROUTINE /////////////
void calibrate_sensors() {
  int num_readings = 1000;
  int16_t ax = AcX;
  int16_t ay = AcY;
  int16_t az = AcZ;
  int16_t gx = GyX;
  int16_t gy = GyY;
  int16_t gz = GyZ;

  // Discard the first reading
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;

  //set DMP offsets, see DMP [dmpbots] source
  mpu.setXGyroOffset(base_x_gyro / GYRO_FACTOR);
  mpu.setYGyroOffset(base_y_gyro / GYRO_FACTOR);
  mpu.setZGyroOffset(base_z_gyro / GYRO_FACTOR);
}

void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);
  previousMillis = 0;        // will store last time LED was updated

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

  // collect the data
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // get calibration values for sensors
  calibrate_sensors();
  set_last_read_angle_data(0, 0, 0, 0);

  // calculate pitch and roll
  roll = atan2(AcY - base_y_accel, AcZ - base_z_accel) * RADIANS_TO_DEGREES;
  pitch = atan2(-(AcX - base_x_accel), AcZ - base_x_accel) * RADIANS_TO_DEGREES;

  // set the starting angle to this pitch and roll
  gyroXrate = roll;
  gyroYrate = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  //start a timer
  timer = micros();
  timeStamp = 0.00;
}




void loop() {
  //Collect raw data
  mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  double dt = (double)(micros() - timer) / 1000000; //stops the timer, convert to seconds from microseconds, and cast as a double
  timer = micros(); //start timer again for the next dt.
  timeStamp = timeStamp + dt;

  //orientation of the accelerometer relative to the earth in degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  roll =  atan2(AcY, AcZ) * RADIANS_TO_DEGREES;
  pitch = atan2(-AcX, AcZ) * RADIANS_TO_DEGREES;

  //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by GYRO_FACTOR.
  gyroXrate = (GyX - base_x_gyro) / GYRO_FACTOR;
  gyroYrate = (GyY - base_y_gyro) / GYRO_FACTOR;


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //KALMNAN FILTER
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //COMPLEMENTARY FILTER
  compAngleX = alpha * (compAngleX + gyroXrate * dt) + (1 - alpha) * roll; // Calculate the angle using a Complimentary filter
  compAngleY = alpha * (compAngleY + gyroYrate * dt) + (1 - alpha) * pitch;

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
    Serial.print(compAngleX); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");
    Serial.print(DMP_roll);  Serial.print("\t");
    Serial.print(DMP_pitch); Serial.print("\t");
    // Serial.print(DMP_yaw);   Serial.print("\t");

    Serial.print(gyroXrate); Serial.print("\t");
    Serial.print(gyroYrate); Serial.print("\t");
    Serial.print(roll);      Serial.print("\t");
    Serial.println(pitch);

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
      myservo.writeMicroseconds(pos);   //1000 & 2000 seems to be 45 degrees from centre, tilt of 500
    }

  }
}




