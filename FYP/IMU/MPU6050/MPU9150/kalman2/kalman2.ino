
#include "Wire.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "MPU6050_9Axis_MotionApps41.h" 
#include "I2Cdev.h"


#define RADIANS_TO_DEGREES 57.2958 // 180/3.14159

// From magnetometer calibration script
#define minx -213.00 
#define maxx 197.00
#define miny -96.00 
#define maxy 446.00
#define minz -381.00 
#define maxz 181.00


#define declination_angle -3.35 //finglas & DCU -3.13, Rathkenny is -3.35


long previousMillis;


MPU6050 mpu;    //for DMP
// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float ypr[3];
//  DMP Variables
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
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, Mx, My, Mz, magbias_x, magbias_y, magbias_z, XH, YH; //These will be the raw data from the MPU6050.
double alpha = 0.98;
double compAngleX, compAngleY; // Calculated angles in the complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double roll,pitch,yaw;
double gyroXrate,gyroYrate;
double GYRO_FACTOR = 32767/250;  // Default sensitiviy full scale range factor FS0, 131
double MAG_FACTOR = 1229.0/4095.0;  //  Default sensitiviy full scale range factor magnetometer
double timeStamp;
uint32_t timer;

  int16_t ax = AcX;
  int16_t ay = AcY;
  int16_t az = AcZ;
  int16_t gx = GyX;
  int16_t gy = GyY;
  int16_t gz = GyZ;
  int16_t mx = Mx;
  int16_t my = My;
  int16_t mz = Mz;

// calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

void set_last_read_angle_data(float x, float y, float x_gyro, float y_gyro) {         //only these should be zero at start
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


// CALIBRATION_ROUTINE for Accelerometer & Gyroscope/////////////
void calibrate_sensors() {
  int num_readings = 100;

  // Discard the first reading
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
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

//////////////////////////////////////////////////////////////////////////////////////////


void setup() {
/*  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);
  previousMillis = 0;        // will store last time LED was updated */

  Wire.begin();
  Serial.end();
  Serial.begin(115200);
  mpu.initialize();
  delay(100);


  // DMP SETUP /////////////////////////////////
  

  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  
  // get calibration values for sensors
  calibrate_sensors();
  set_last_read_angle_data(0, 0, 0, 0);
  ///////////////////////////////////////////////

  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);    //get raw values

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
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz); //get raw values

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

  magbias_x = MAG_FACTOR * (maxx + minx) / 2;
  magbias_y = MAG_FACTOR * (maxy + miny) / 2;
  magbias_z = MAG_FACTOR * (maxz + minz) / 2;

  Mx = (MAG_FACTOR * mx) - magbias_x;
  My = (MAG_FACTOR * my) - magbias_y;
  Mz = (MAG_FACTOR * mz) - magbias_z;

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
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  9 DoF Tilt Compensation using Magnetometer  ////////////////////////////////////////////////////////////////////////////////////


     XH = Mx * cos(kalAngleY / RADIANS_TO_DEGREES) + My * sin(kalAngleX/ RADIANS_TO_DEGREES) + Mz * cos(kalAngleX / RADIANS_TO_DEGREES) * sin(kalAngleY / RADIANS_TO_DEGREES);
     YH = My * cos(kalAngleX/ RADIANS_TO_DEGREES) - Mz * sin(kalAngleX/ RADIANS_TO_DEGREES);
     yaw  = atan2(+YH , XH) * RADIANS_TO_DEGREES;                                                                           //convert back to degrees
     yaw += declination_angle;    // http://www.magnetic-declination.com/

     if (yaw < -180 ){
        yaw += 180;
     }
     if (yaw > 180){
        yaw -= 180;
     }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* DMP 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Obtain YPR angles from buffer
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  }
  double DMP_roll = ypr[2] * RADIANS_TO_DEGREES;
  double DMP_pitch = -ypr[1] * RADIANS_TO_DEGREES;
  double DMP_yaw = ypr[0] * RADIANS_TO_DEGREES;
  */
  Serial.print(timeStamp);  Serial.print("\t");
  if(timeStamp > 20.0){   //give time to settle

     Serial.println(yaw);
     // Serial.println(DMP_yaw); // Uncomment for DMP output
  }
  
}





