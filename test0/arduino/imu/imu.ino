#include "mpu9250.h"

/* Mpu9250 object */
bfs::Mpu9250 imu;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus, 0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while (1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while (1) {}
  }
  Serial.print("new_imu,new_mag,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,temp_c,heading");
}

void loop() {
  /* Check if data read */
  if (imu.Read()) {
    
    Serial.print(imu.new_imu_data());
    Serial.print(",");
    Serial.print(imu.new_mag_data());
    Serial.print(",");
    Serial.print(imu.accel_x_mps2());
    Serial.print(",");
    Serial.print(imu.accel_y_mps2());
    Serial.print(",");
    Serial.print(imu.accel_z_mps2());
    Serial.print(",");
    Serial.print(imu.gyro_x_radps());
    Serial.print(",");
    Serial.print(imu.gyro_y_radps());
    Serial.print(",");
    Serial.print(imu.gyro_z_radps());
    Serial.print(",");
    Serial.print(imu.mag_x_ut());
    Serial.print(",");
    Serial.print(imu.mag_y_ut());
    Serial.print(",");
    Serial.print(imu.mag_z_ut());
    Serial.print(",");
    Serial.print(imu.die_temp_c());
    Serial.print(",");

    // Calculate heading in degrees
    float heading = atan2(imu.mag_y_ut(), imu.mag_x_ut()) * 180.0 / PI;

    // Normalize the heading to 0-360 degrees
    if (heading < 0) {
      heading += 360.0;
    }
    Serial.print(heading);
    Serial.print("|\n");

  }


}
