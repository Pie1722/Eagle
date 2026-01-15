#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>

Servo FR, FL, BR, BL;
char input = 0;
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float angleX, angleY;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY;
float elapsedTime, currentTime, previousTime;
int c = 0;

// ===== Limits =====
#define MAX_ANGLE 30.0f   // degrees

// ===== PID gains (start small) =====
float Kp = 4.0;
float Ki = 0.03;
float Kd = 1.8;

// ===== Setpoints =====
float rollSetpoint  = 0;
float pitchSetpoint = 0;

// ===== PID memory =====
float rollError, rollPrevError;
float pitchError, pitchPrevError;

float rollIntegral = 0;
float pitchIntegral = 0;

// ===== PID outputs =====
float rollOutput = 0;
float pitchOutput = 0;

#define CE_PIN 9
#define CSN_PIN 10
#define en_esc A0
#define PLD_S sizeof(joy)

RF24 radio(CE_PIN, CSN_PIN);
byte rxAddr[5] = {0x45, 0x55, 0x67, 0x10, 0x21};

struct JoystickData {
  uint16_t code;
  int16_t xAxis;
  int16_t yAxis;
  int16_t rXaxis;
  int16_t rYaxis;
  uint16_t buttons;    // 16 bits, 16 buttons
};

JoystickData joy;

void setup() {

  Serial.begin(500000);

  pinMode(en_esc, OUTPUT);
  digitalWrite(en_esc, HIGH);

  FR.attach(2);
  FL.attach(3);
  BR.attach(4);
  BL.attach(5);
  
  // so that it wont write any random values
  FR.writeMicroseconds(1000);
  FL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);

  Serial.println("Calibration for ESC");
  Serial.println("Enter y to start calibration");
  Serial.println("Enter n to skip");
  
  unsigned long esc_cal = millis();
  while (millis() - esc_cal < 5000) {
    if (Serial.available()) {
      input = Serial.read();

      if (input == 'y') {
        Serial.println("Calibrating...");
        break;
      }

      if (input == 'n') {
        break;
      }
    }
  }

  if (input == 'y') {
    calibrate_esc();
  } else {
    Serial.print("ESC calibration skipped.");
  }
  
  if (!radio.begin()) {
    Serial.println("NRF24 init FAIL");
    while (1);
  }

  initialize_MPU6050();
  
  radio.setPALevel(RF24_PA_MAX);    // matches STM32 _0dbm power
  radio.setDataRate(RF24_2MBPS);    // match
  radio.setChannel(76);             // match
  radio.setPayloadSize(PLD_S);      // 32 bytes
  radio.setAddressWidth(5);
  radio.setAutoAck(true);          // match
  radio.disableAckPayload();
  radio.disableDynamicPayloads();   // match
  //radio.enableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);  // match
  //radio.disableCRC();
  radio.openReadingPipe(0, rxAddr);
  radio.startListening();

}

void calibrate_esc() {

  FR.writeMicroseconds(2000);   // full throttle
  FL.writeMicroseconds(2000);   
  BR.writeMicroseconds(2000);   
  BL.writeMicroseconds(2000); 
  digitalWrite(en_esc, LOW);  
  delay(300);
  FR.writeMicroseconds(1000);   // zero throttle
  FL.writeMicroseconds(1000);   
  BR.writeMicroseconds(1000);   
  BL.writeMicroseconds(1000);  
  Serial.println("Calibration of ESC done"); 
}

void getData() {

  Serial.print("Left Y axis: ");
  Serial.print(joy.yAxis);
  Serial.print("\t Left X axis: ");
  Serial.print(joy.xAxis);
  Serial.print("\t Right Y axis: ");
  Serial.print(joy.rXaxis);
  Serial.print("\t Right Y axis: ");
  Serial.println(joy.rYaxis);
}

void loop() {
  
  if (radio.available()) {
    radio.read(&joy, sizeof(joy));
    getData();
    
    read_IMU();
    
    rollSetpoint  = ((float)joy.xAxis / 32767.0f) * MAX_ANGLE;
    pitchSetpoint = ((float)joy.yAxis / 32767.0f) * MAX_ANGLE;

    runPID(elapsedTime); 

    int throttle = map(joy.rYaxis, -32768, 32767, 1000, 2000);

    if (throttle < 1050) {
      rollIntegral = 0;
      pitchIntegral = 0;
    }

    int FRspeed = throttle - rollOutput - pitchOutput;
    int FLspeed = throttle + rollOutput - pitchOutput;
    int BRspeed = throttle - rollOutput + pitchOutput;
    int BLspeed = throttle + rollOutput + pitchOutput;

    FRspeed = constrain(FRspeed, 1000, 2000);
    FLspeed = constrain(FLspeed, 1000, 2000);
    BRspeed = constrain(BRspeed, 1000, 2000);
    BLspeed = constrain(BLspeed, 1000, 2000);

    FR.writeMicroseconds(FRspeed);
    FL.writeMicroseconds(FLspeed);
    BR.writeMicroseconds(BRspeed);
    BL.writeMicroseconds(BLspeed);
  }

}

void runPID(float dt) {

  // ===== ROLL =====
  rollError = rollSetpoint - angleX;
  rollIntegral += rollError * dt;
  rollIntegral = constrain(rollIntegral, -50, 50);   // anti-windup
  float rollDerivative = (rollError - rollPrevError) / dt;

  rollOutput = Kp*rollError + Ki*rollIntegral + Kd*rollDerivative;
  rollPrevError = rollError;

  // ===== PITCH =====
  pitchError = pitchSetpoint - angleY;
  pitchIntegral += pitchError * dt;
  pitchIntegral = constrain(pitchIntegral, -50, 50);
  float pitchDerivative = (pitchError - pitchPrevError) / dt;

  pitchOutput = Kp*pitchError + Ki*pitchIntegral + Kd*pitchDerivative;
  pitchPrevError = pitchError;
}

void initialize_MPU6050() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Configure Accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 32.8);
    GyroErrorY = GyroErrorY + (GyroY / 32.8);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
}

void read_IMU() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value

  // Calculating angle values using
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.15; // AccErrorX ~(-1.15) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 0.52; // AccErrorX ~(0.5)

  // === Read gyro data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;   // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX + 1.85; //// GyroErrorX ~(-1.85)
  GyroY = GyroY - 0.15; // GyroErrorY ~(0.15)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;

  // Read Temperature
  Wire.beginTransmission(MPU);
  Wire.write(0x41);          // TEMP_OUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);

  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  float temperature = (rawTemp / 340.0) + 36.53;

  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print(" C");

  // Complementary filter - combine acceleromter and gyro angle values
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;
  // Map the angle values from -90deg to +90 deg into values from 0 to 255, like the values we are getting from the Joystick
//  data.j1PotX = map(angleX, -90, +90, 255, 0);
//  data.j1PotY = map(angleY, -90, +90, 0, 255);
  Serial.print("\t Y axis: ");
  Serial.print(angleY);
  Serial.print("\t X axis: ");
  Serial.println(angleX);
}
