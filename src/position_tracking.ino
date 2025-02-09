
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Adafruit_AHRS.h>      // Sensor fusion

ICM_20948_I2C myIMU;
Adafruit_Madgwick filter;

float vx = 0, vy = 0;
float px = 0, py = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Wire.begin();
  if (myIMU.begin() != ICM_20948_Stat_Ok) {
    Serial.println("IMU not detected!");
    while (1);
  }

  filter.begin(100.0);  // 100Hz sample rate
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  myIMU.getAGMT();  // Read accel, gyro, mag data

  // Get raw sensor data
  float ax = myIMU.accX() / 1000 * 9.81 + 0.0927; //Adjusting for empirically derived offset value
  float ay = myIMU.accY() / 1000 * 9.81 - 0.2315;
  float az = myIMU.accZ() / 1000 * 9.81 - 0.3534;
  float gx = myIMU.gyrX() * 3.1415926 / 180 + .0055;
  float gy = myIMU.gyrY() * 3.1415926 / 180 - .00084;
  float gz = myIMU.gyrZ() * 3.1415926 / 180 - .0045;
  float mx = myIMU.magX();
  float my = myIMU.magY();
  float mz = myIMU.magZ();

  // Update sensor fusion
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();  // Rotation about Z-axis

  // Gravity correction
  /*
  float g_x = 9.81 * sin(pitch * DEG_TO_RAD);
  float g_y = -9.81 * sin(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
  float g_z = 9.81 * cos(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
  
  ax -= g_x;
  ay -= g_y;
  az -= g_z;
  */
  // Transform acceleration to global frame using yaw
  float a_X = ax * cos(yaw * DEG_TO_RAD) - ay * sin(yaw * DEG_TO_RAD);
  float a_Y = ax * sin(yaw * DEG_TO_RAD) + ay * cos(yaw * DEG_TO_RAD);

  // Integrate acceleration to velocity
  vx += a_X * dt;
  vy += a_Y * dt;

  // Integrate velocity to position
  px += vx * dt;
  py += vy * dt;

  // Print results
  
  Serial.print("X: "); Serial.print(px);
  Serial.print(" Y: "); Serial.print(py);
  Serial.print(" Yaw: "); Serial.print(yaw);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Accel X: "); Serial.print(ax);
  Serial.print(" Accel Y: "); Serial.print(ay);
  Serial.print(" Accel Z: "); Serial.print(az);
  Serial.print(" Gyro X: "); Serial.print(gx);
  Serial.print(" Gyro Y: "); Serial.print(gy);
  Serial.print(" Gyro Z: "); Serial.print(gz);
  Serial.print(" Mag X: "); Serial.print(mx);
  Serial.print(" Mag Y: "); Serial.print(my);
  Serial.print(" Mag Z: "); Serial.println(mz);
  
  delay(10);
}
