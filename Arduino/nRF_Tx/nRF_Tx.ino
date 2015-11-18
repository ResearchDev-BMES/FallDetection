#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

RF24 radio(7,8);
unsigned long timeoutPeriod = 1000;
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

union Payload
{
  struct{
    unsigned long Time;
    float Acc_X;
    float Acc_Y;
    float Acc_Z;
    //float Mag_X;
    //float Mag_Y;
    //float Mag_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    char ID;
    char pad;
  } Sensor;
  byte Modulator[sizeof(Sensor)];
};

union Payload BlackBox;

unsigned long Timer;

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };

void setup() {

  // Sensor Initialization
  Serial.begin(57600);
  accel.begin();
  gyro.begin();
  printf_begin();
  
  // Change ID from L to R in different sensor
  BlackBox.Sensor.ID = 'L';
  
  // Radio Setup
  radio.begin();
  radio.setChannel(1);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(1);
  radio.setRetries(2, 15);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
  radio.setPayloadSize(32);
  radio.stopListening();
  radio.powerUp();
}

void loop() {
  // Sampling at 100 Hz
  if (millis() - Timer > 9UL) {
    Timer = millis();
    sensors_event_t event;

    // Timer, used if Debugging
    BlackBox.Sensor.Time = Timer;

    // Absolute Acceleration. Used if axial information is non-relevant.
    //BlackBox.Sensor.Accelerometer = sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.y,2)+pow(event.acceleration.z,2));
    
    // Get Accelerometer Data
    accel.getEvent(&event);
    BlackBox.Sensor.Acc_X = event.acceleration.x;
    BlackBox.Sensor.Acc_Y = event.acceleration.y;
    BlackBox.Sensor.Acc_Z = event.acceleration.z;

    // Get Gyroscope Data. Convert to Degree per Second
    gyro.getEvent(&event);
    BlackBox.Sensor.Gyro_X = event.gyro.x*57.2958;
    BlackBox.Sensor.Gyro_Y = event.gyro.y*57.2958;
    BlackBox.Sensor.Gyro_Z = event.gyro.z*57.2958;

  /* Use only if you want to display the data for Debug
    
    Serial.print(BlackBox.Sensor.Gyro_X);
    Serial.print(",");
    Serial.print(BlackBox.Sensor.Gyro_Y);
    Serial.print(",");
    Serial.print(BlackBox.Sensor.Gyro_Z);
    Serial.println(",");

    Serial.print(event.gyro.x);
    Serial.print(",");
    Serial.print(event.gyro.y);
    Serial.print(",");
    Serial.print(event.gyro.z);
    Serial.println(",");
    
    Serial.print(event.gyro.x);
    Serial.print(",");
    Serial.print(event.gyro.y);
    Serial.print(",");
    Serial.print(event.gyro.z);
    Serial.println(",");
    
  */
    radio.writeBlocking(&BlackBox.Modulator,sizeof(BlackBox.Modulator),timeoutPeriod);
    radio.txStandBy(timeoutPeriod);
  }
}
