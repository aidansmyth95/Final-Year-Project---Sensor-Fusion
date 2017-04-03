
#include "Wire.h"
#include "Kalman.h"
#include "GY_85.h"
#include "I2Cdev.h"

double magMin[3], magMax[3];
GY_85 GY85;
int16_t ax, ay, az, gx, gy, gz, m[3];

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  GY85.init();

  for (int i = 0; i < 3; i++) {
    magMin[i] = 10000000;
    magMax[i] = -10000000;
  }
  Serial.println("Magnetometer calibration starting...");
}

void loop()
{
  boolean changed = false;
    
    m[0] = GY85.compass_x( GY85.readFromCompass() );
    m[1] = GY85.compass_y( GY85.readFromCompass() );
    m[2] = GY85.compass_z( GY85.readFromCompass() );

  for (int i = 0; i < 3; i++) {
    if (m[i] < magMin[i]) {
      magMin[i] = m[i];
      changed = true;
    }
    if (m[i] > magMax[i]) {
      magMax[i] = m[i];
      changed = true;
    }
  }

  if (changed) {
    Serial.println("-------");
    Serial.print("minX: ");   Serial.print(magMin[0]);
    Serial.print(" maxX: ");  Serial.println(magMax[0]);
    Serial.print("minY: ");   Serial.print(magMin[1]);
    Serial.print(" maxY: ");  Serial.println(magMax[1]);
    Serial.print("minZ: ");   Serial.print(magMin[2]);
    Serial.print(" maxZ: ");  Serial.println(magMax[2]);
  }
}


