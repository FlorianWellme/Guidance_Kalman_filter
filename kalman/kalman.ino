#include <Wire.h>
#include <TimerOne.h>
#include <math.h>

/* =========================================================
   MPU ADDRESS AND CONFIGURATION
   ========================================================= */

// I2C address of MPU6500 / MPU9250
#define MPU_ADDR 0x68

// Full-scale configuration
#define GYRO_CONFIG_1000DPS 0x10   // ±1000 degrees per second
#define ACC_CONFIG_4G       0x08   // ±4 g

// Gyroscope sensitivity from datasheet
#define GYRO_SENS_1000DPS 32.8f    // 32.8 LSB = 1 °/s

// Sampling period
const float dt = 0.01f;            // 10 ms → 100 Hz

/* =========================================================
   KALMAN FILTER PARAMETERS
   ========================================================= */

// Process noise (gyro trust)
const float Q_angle   = 0.001f;    // Model uncertainty
const float Q_bias    = 0.003f;    // Gyro bias drift

// Measurement noise (accelerometer)
const float R_measure = 0.03f;

/* =========================================================
   GLOBAL VARIABLES
   ========================================================= */

// Timer interrupt flag
volatile bool intFlag = false;

// Gyroscope bias (measured at rest)
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

// Yaw angle (gyro only, no absolute reference)
float yawAngle = 0.0f;

/* =========================================================
   KALMAN FILTER STRUCTURE (1D)
   ========================================================= */

typedef struct {
  float angle;        // Estimated angle
  float bias;         // Estimated gyro bias
  float P[2][2];      // Error covariance matrix
} Kalman_t;

// One Kalman filter for roll and one for pitch
Kalman_t kalmanRoll;
Kalman_t kalmanPitch;

/* =========================================================
   LOW-LEVEL I2C FUNCTIONS
   ========================================================= */

// Write a byte to an I2C register
void I2CwriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// Read N bytes from an I2C register
void I2Cread(uint8_t addr, uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, n);
  for (uint8_t i = 0; i < n; i++)
    buf[i] = Wire.read();
}

/* =========================================================
   TIMER INTERRUPT (100 Hz)
   ========================================================= */

// Called every 10 ms
void timerCallback() {
  intFlag = true;
}

/* =========================================================
   1D KALMAN FILTER UPDATE FUNCTION
   ========================================================= */

float kalmanUpdate(Kalman_t *k, float accAngle, float gyroRate, float dt) {

  /* ------------------ PREDICTION STEP ------------------
     Integrate gyro angular velocity
     while compensating estimated bias
  */
  k->angle += (gyroRate - k->bias) * dt;

  // Update covariance matrix
  k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + Q_angle);
  k->P[0][1] -= dt * k->P[1][1];
  k->P[1][0] -= dt * k->P[1][1];
  k->P[1][1] += Q_bias * dt;

  /* ------------------ CORRECTION STEP ------------------
     Correct prediction using accelerometer angle
  */

  // Innovation covariance
  float S = k->P[0][0] + R_measure;

  // Kalman gains
  float K0 = k->P[0][0] / S;
  float K1 = k->P[1][0] / S;

  // Measurement residual
  float y = accAngle - k->angle;

  // Update state estimate
  k->angle += K0 * y;
  k->bias  += K1 * y;

  // Update covariance matrix
  float P00 = k->P[0][0];
  float P01 = k->P[0][1];

  k->P[0][0] -= K0 * P00;
  k->P[0][1] -= K0 * P01;
  k->P[1][0] -= K1 * P00;
  k->P[1][1] -= K1 * P01;

  return k->angle;
}

/* =========================================================
   GYROSCOPE CALIBRATION
   ========================================================= */

void calibrateGyro() {
  const int samples = 500;
  long sx = 0, sy = 0, sz = 0;

  // Average gyro values while IMU is perfectly still
  for (int i = 0; i < samples; i++) {
    uint8_t buf[14];
    I2Cread(MPU_ADDR, 0x3B, 14, buf);

    sx += (buf[8]  << 8) | buf[9];
    sy += (buf[10] << 8) | buf[11];
    sz += (buf[12] << 8) | buf[13];

    delay(5);
  }

  gyroBiasX = sx / (float)samples;
  gyroBiasY = sy / (float)samples;
  gyroBiasZ = sz / (float)samples;
}

/* =========================================================
   SETUP
   ========================================================= */

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU
  I2CwriteByte(MPU_ADDR, 0x6B, 0x00);
  delay(100);

  // Low-pass filter configuration
  I2CwriteByte(MPU_ADDR, 0x1A, 0x06);

  // Sensor ranges
  I2CwriteByte(MPU_ADDR, 0x1B, GYRO_CONFIG_1000DPS);
  I2CwriteByte(MPU_ADDR, 0x1C, ACC_CONFIG_4G);

  // Timer at 100 Hz
  Timer1.initialize(10000);
  Timer1.attachInterrupt(timerCallback);

  Serial.println("Calibrating gyro...");
  calibrateGyro();
  Serial.println("Calibration done");

  Serial.println(
  "ax,ay,az,"
  "gx,gy,gz,"
  "rollAcc,pitchAcc,yawGyro,"
  "rollKalman,pitchKalman,yawKalman"
  ); 

}

/* =========================================================
   MAIN LOOP
   ========================================================= */

void loop() {
  // Wait for 100 Hz timer tick
  while (!intFlag);
  intFlag = false;

  uint8_t buf[14];
  I2Cread(MPU_ADDR, 0x3B, 14, buf);

  /* ---------- ACCELEROMETER ---------- */
  float ax = (int16_t)((buf[0] << 8) | buf[1]);
  float ay = (int16_t)((buf[2] << 8) | buf[3]);
  float az = (int16_t)((buf[4] << 8) | buf[5]);

  /* ---------- GYROSCOPE ---------- */
  float gx = ((int16_t)((buf[8]  << 8) | buf[9])  - gyroBiasX) / GYRO_SENS_1000DPS;
  float gy = ((int16_t)((buf[10] << 8) | buf[11]) - gyroBiasY) / GYRO_SENS_1000DPS;
  float gz = ((int16_t)((buf[12] << 8) | buf[13]) - gyroBiasZ) / GYRO_SENS_1000DPS;

  /* ---------- ACCELEROMETER ANGLES ---------- */
  float rollAcc  = atan2(ay, az) * 180.0 / PI;
  float pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

  /* ---------- KALMAN FUSION ---------- */
  float roll  = kalmanUpdate(&kalmanRoll,  rollAcc,  gx, dt);
  float pitch = kalmanUpdate(&kalmanPitch, pitchAcc, gy, dt);

  /* ---------- YAW (GYRO ONLY) ---------- */
  yawAngle += gz * dt;

  /* ---------- SERIAL OUTPUT ---------- */


  // Raw accelerometer
  Serial.print(ax, 6); Serial.print(",");
  Serial.print(ay, 6); Serial.print(",");
  Serial.print(az, 6); Serial.print(",");

  // Raw gyroscope (deg/s)
  Serial.print(gx, 6); Serial.print(",");
  Serial.print(gy, 6); Serial.print(",");
  Serial.print(gz, 6); Serial.print(",");

  // Angles BEFORE Kalman (accelerometer only)
  Serial.print(rollAcc, 6);  Serial.print(",");
  Serial.print(pitchAcc, 6); Serial.print(",");
  Serial.print(yawAngle, 6); Serial.print(",");

  // Angles AFTER Kalman
  Serial.print(roll, 6);  Serial.print(",");
  Serial.print(pitch, 6); Serial.print(",");
  Serial.println(yawAngle, 6); 



}
