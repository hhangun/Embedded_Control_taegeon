/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lim Soon Ho, Han Taegeon
Modified         : 2023-12-15
Language/ver     : C++ in Arduino
Description      : Make automatic entrance system
/----------------------------------------------------------------*/

  #include <SoftwareSerial.h>
  #include "Wire.h"
  #include <MPU6050_light.h>

  MPU6050 mpu(Wire);
  unsigned long timer = 0;

  SoftwareSerial HC06(8,9);

  void setup() {
    Serial.begin(9600);
    Wire.begin();

    pinMode(2, INPUT);          // MPU6050 센서 INT핀모드 설정
    HC06.begin(9600);
    
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(500);
     mpu.calcOffsets(); // gyro and accelero
  }

  char val_X = 'S';
  char val_Z = 's';

  void loop() {
    mpu.update();
    
    HC06.write(val_X);  // send Alphabet of X angle to MCU
    HC06.write(val_Z);  // send Alphabet of Z angle to MCU
    
    if((millis()-timer)>10){ // print data every 100ms
    Serial.print("X : ");
    Serial.print(mpu.getAngleX());  
    //Serial.print(val_X);
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
    //Serial.println(val_Z);
    timer = millis();  
    }

    // DC Motor
    if(mpu.getAngleX()>=10 && mpu.getAngleX()<=30)  val_X = 'A';    // Go Back
    else if(mpu.getAngleX() > 30)  val_X = 'B';      // Speed up
    else if(mpu.getAngleX()<10 && mpu.getAngleX()>-10) val_X = 'S'; // Stop
    else if(mpu.getAngleX()<=-10 && mpu.getAngleX() >= -30) val_X = 'C';   // Go straight
    else if(mpu.getAngleX()<-30) val_X = 'D';   // Speed up

    // USE Z
     if(mpu.getAngleZ()<=20 && mpu.getAngleZ()>=-20)  val_Z = 's';  // Stop
    else if(mpu.getAngleZ()>20)  val_Z = 'a'; // Left
    else if(mpu.getAngleZ()<-20)   val_Z = 'b';  // Right
  
  }
