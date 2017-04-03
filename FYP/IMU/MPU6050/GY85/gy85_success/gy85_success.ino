#include <Servo.h>
#include "Wire.h"
#include "Kalman.h"
#include "GY_85.h"
#include "I2Cdev.h"

#define RADIANS_TO_DEGREES 57.2958 // 180/3.14159

Servo myservo;  // create servo object
int pos = 1500;    // variable to store the servo position
int servo_tilt = 500;
long interval = 5;  //in milliseconds
long previousMillis;
boolean up = true;

// From magnetometer calibration script
#define minx -759.00
#define maxx 784.00
#define miny -877.00
#define maxy 478.00
#define minz -781.00
#define maxz 670.00


#define declination_angle -3.35
//finglas & DCU -3.13, Rathkenny is -3.35


GY_85 GY85;    //for DMP

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;


double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, Mx, My, Mz, magbias_x, magbias_y, magbias_z, XH, YH; //These will be the raw data from the MPU6050.
double alpha = 0.95;
double compAngleX, compAngleY; // Calculated angles in the complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double roll, pitch, yaw;
double gyroXrate, gyroYrate;
double GYRO_FACTOR = 32767 / 250; // Default sensitiviy full scale range factor FS0, 131
double MAG_FACTOR = 1229.0 / 4095.0; //  Default sensitiviy full scale range factor magnetometer
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


// CALIBRATION_ROUTINE for Accelerometer & Gyroscope/////////////
void calibrate_sensors() {
  int num_readings = 100;

  // Discard the first reading
  ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  az = GY85.accelerometer_z( GY85.readFromAccelerometer() );
  gx = GY85.gyro_x( GY85.readGyro() );
  gy = GY85.gyro_y( GY85.readGyro() );
  gz = GY85.gyro_z( GY85.readGyro() );

  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
    ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
    az = GY85.accelerometer_z( GY85.readFromAccelerometer() );
    gx = GY85.gyro_x( GY85.readGyro() );
    gy = GY85.gyro_y( GY85.readGyro() );
    gz = GY85.gyro_z( GY85.readGyro() );
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

}

//////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(1500);
  previousMillis = 0;        // will store last time LED was updated */

  Wire.begin();
  Serial.end();
  Serial.begin(9600);
  GY85.init();
  delay(100);


  // DMP SETUP /////////////////////////////////

  // get calibration values for sensors
  calibrate_sensors();
  set_last_read_angle_data(0, 0, 0, 0);
  ///////////////////////////////////////////////

  ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  az = GY85.accelerometer_z( GY85.readFromAccelerometer() );

  mx = GY85.compass_x( GY85.readFromCompass() );
  my = GY85.compass_y( GY85.readFromCompass() );
  mz = GY85.compass_z( GY85.readFromCompass() );

  gx = GY85.gyro_x( GY85.readGyro() );
  gy = GY85.gyro_y( GY85.readGyro() );
  gz = GY85.gyro_z( GY85.readGyro() );

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
  ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  az = GY85.accelerometer_z( GY85.readFromAccelerometer() );

  mx = GY85.compass_x( GY85.readFromCompass() );
  my = GY85.compass_y( GY85.readFromCompass() );
  mz = GY85.compass_z( GY85.readFromCompass() );

  gx = GY85.gyro_x( GY85.readGyro() );
  gy = GY85.gyro_y( GY85.readGyro() );
  gz = GY85.gyro_z( GY85.readGyro() );

  AcX = ax; AcY = ay; AcZ = az;
  GyX = gx; GyY = gy; GyZ = gz;

  double dt = (double)(micros() - timer) / 1000000; //stops the timer, convert to seconds from microseconds, and cast as a double
  timer = micros(); //start timer again for the next dt.
  timeStamp = timeStamp + dt;

  //orientation of the accelerometer relative to the earth in degrees
  //corrects any cumulative errors in the orientation that the gyroscope develops.
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
  //COMPLEMENTARY FILTER
  compAngleX = alpha * (compAngleX + gyroXrate * dt) + (1 - alpha) * roll; // Calculate the angle using a Complimentary filter
  compAngleY = alpha * (compAngleY + gyroYrate * dt) + (1 - alpha) * pitch;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  9 DoF Tilt Compensation using Magnetometer  ////////////////////////////////////////////////////////////////////////////////////


  XH = Mx * cos(kalAngleY / RADIANS_TO_DEGREES) + My * sin(kalAngleX / RADIANS_TO_DEGREES) + Mz * cos(kalAngleX / RADIANS_TO_DEGREES) * sin(kalAngleY / RADIANS_TO_DEGREES);
  YH = My * cos(kalAngleX / RADIANS_TO_DEGREES) - Mz * sin(kalAngleX / RADIANS_TO_DEGREES);
  yaw  = atan2(+YH , XH) * RADIANS_TO_DEGREES;                                                                           //convert back to degrees
  yaw += declination_angle;    // http://www.magnetic-declination.com/

  if (yaw < -180 ) {
    yaw += 180;
  }
  if (yaw > 180) {
    yaw -= 180;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.print(timeStamp);  Serial.print("\t");
  if (timeStamp > 20.0) { //give time to settle

    // for pitch, roll
    /*
      Serial.print(compAngleX); Serial.print("\t");
      Serial.print(compAngleY); Serial.print("\t");
      Serial.print(kalAngleX); Serial.print("\t");
      Serial.print(kalAngleY); Serial.print("\t");
      Serial.print(gyroXrate); Serial.print("\t");
      Serial.print(gyroYrate); Serial.print("\t");
      Serial.print(roll);      Serial.print("\t");
      Serial.println(pitch);
    */

    //for yaw
    Serial.println(yaw);
  }
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

