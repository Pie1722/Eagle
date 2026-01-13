#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>

Servo FR, FL, BR, BL;
char c = 0;

#define CE_PIN 9
#define CSN_PIN 10
#define en_esc A0

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

#define PLD_S sizeof(joy)

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
      c = Serial.read();

      if (c == 'y') {
        Serial.println("Calibrating...");
        break;
      }

      if (c == 'n') {
        break;
      }
    }
  }

  if (c == 'y') {
    calibrate_esc();
  } else {
    Serial.print("ESC calibration skipped.");
  }
  
  if (!radio.begin()) {
    Serial.println("NRF24 init FAIL");
    while (1);
  }
  
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
  }
}
