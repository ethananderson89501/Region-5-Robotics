
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Adafruit_AHRS.h>      // Sensor fusion

ICM_20948_I2C myIMU;
Adafruit_Madgwick filter;

float vx = 0, vy = 0;
float px = 0, py = 0;
unsigned long lastTime = 0;
float ax_adjust = 0;
float ay_adjust = 0;
float az_adjust = 0;
float gx_adjust = 0;
float gy_adjust = 0;
float gz_adjust = 0;

float ax_old = 0;
float ay_old = 0;
float az_old = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Wire.begin();
  if (myIMU.begin() != ICM_20948_Stat_Ok) {
    Serial.println("IMU not detected!");
    while (1);
  }

  //ilter.begin(100.0);  // 100Hz sample rate

  float ax_calib = 0;
  float ay_calib = 0;
  float az_calib = 0;
  float gx_calib = 0;
  float gy_calib = 0;
  float gz_calib = 0;

  Serial.println("Calibration Pending. Hold IMU still and level.");
  delay(2000);
  Serial.println("Beginning calibration.");
  for (int i = 0; i < 1000; i++){
    myIMU.getAGMT();
    ax_calib += myIMU.accX() / 1000 * 9.81;
    ay_calib += myIMU.accY() / 1000 * 9.81;
    az_calib += myIMU.accZ() / 1000 * 9.81;
    gx_calib += myIMU.gyrX();
    gy_calib += myIMU.gyrY();
    gz_calib += myIMU.gyrZ();
    delay(5);
  }
  
  ax_adjust = ax_calib / 1000.0;
  ay_adjust = ay_calib / 1000.0;
  az_adjust = az_calib / 1000.0 - 9.81;
  gx_adjust = gx_calib / 1000.0;
  gy_adjust = gy_calib / 1000.0;
  gz_adjust = gz_calib / 1000.0;

  Serial.print("Adjustments: ");
  Serial.print(ax_adjust); Serial.print(" ");
  Serial.print(ay_adjust); Serial.print(" ");
  Serial.print(az_adjust); Serial.print(" ");
  Serial.print(gx_adjust); Serial.print(" ");
  Serial.print(gy_adjust); Serial.print(" ");
  Serial.println(gz_adjust);

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  myIMU.getAGMT();  // Read accel, gyro, mag data

  // Get raw sensor data
  float ax = myIMU.accX() / 1000 * 9.81 - ax_adjust; //Adjusting for empirically derived offset value
  float ay = myIMU.accY() / 1000 * 9.81 - ay_adjust;
  float az = myIMU.accZ() / 1000 * 9.81 - az_adjust;
  float gx = (myIMU.gyrX()- gx_adjust) * 3.1415926 / 180;
  float gy = (myIMU.gyrY()- gy_adjust) * 3.1415926 / 180;
  float gz = (myIMU.gyrZ()- gz_adjust) * 3.1415926 / 180;
  float mx = myIMU.magX();
  float my = myIMU.magY();
  float mz = myIMU.magZ();

  Serial.print("Acceleration: "); Serial.print(ax); Serial.print(" "); Serial.print(ay); Serial.print(" "); Serial.print(az); Serial.print(" ");
  Serial.print("Gryo: "); Serial.print(gx); Serial.print(" "); Serial.print(gy); Serial.print(" "); Serial.print(gz); Serial.print(" ");
  Serial.print("Magnetometer: "); Serial.print(mx); Serial.print(" "); Serial.print(my); Serial.print(" "); Serial.println(mz);

  // Update sensor fusion
  //Lowpass filter
  ax = 0.9*ax_old + 0.1*ax;
  ay = 0.9*ay_old + 0.1*ay;
  az = 0.9*az_old + 0.1*az;
  ax_old = ax;
  ay_old = ay;
  az_old = az;
  Serial.print("FILTERED: ");
  Serial.print("Acceleration: "); Serial.print(ax); Serial.print(" "); Serial.print(ay); Serial.print(" "); Serial.print(az); Serial.print(" ");
  Serial.print("Gryo: "); Serial.print(gx); Serial.print(" "); Serial.print(gy); Serial.print(" "); Serial.print(gz); Serial.print(" ");
  Serial.print("Magnetometer: "); Serial.print(mx); Serial.print(" "); Serial.print(my); Serial.print(" "); Serial.println(mz);

  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();  // Rotation about Z-axis

  // Gravity correction
  
  float g_x = 9.81 * sin(pitch * DEG_TO_RAD);
  float g_y = -9.81 * sin(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
  float g_z = 9.81 * cos(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
  
  ax -= g_x;
  ay -= g_y;
  az -= g_z;

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
  
  delay(1);
}
