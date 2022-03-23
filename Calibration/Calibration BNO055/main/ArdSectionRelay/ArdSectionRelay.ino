
#include "BNO055_AOG.h"
#include <Wire.h>

#define A 0X28  //I2C address selection pin LOW
#define B 0x29  //                          HIGH

BNO055 IMU(A);

int inByte = 0; // incoming serial byte
/*
const unsigned int calibrationDat[11] = 
{ 65511,65425,65535,        //accel
  65534,65535,1,            //gyro
  65300,15,55,              //mag
  100,980};                 //acc radius, mag radius
  */

  void setup() {
    Wire.begin();
    Serial.begin(19200);
    IMU.init();

    // Restore calibration data from zeroing
    bno055_offsets_t calibrationData;
    
    calibrationData.accel_offset_x = 65511;
    calibrationData.accel_offset_y = 65425;
    calibrationData.accel_offset_z = 65535;
    
    calibrationData.gyro_offset_x = 65534;
    calibrationData.gyro_offset_y = 65533;
    calibrationData.gyro_offset_z = 1;
    
    calibrationData.mag_offset_x = 65300;
    calibrationData.mag_offset_y = 15;
    calibrationData.mag_offset_z = 55;
    
    calibrationData.accel_radius = 1000;
    calibrationData.mag_radius = 980;
    
    IMU.setSensorOffsets(calibrationData);

    //use external 32K crystal
    IMU.setExtCrystalUse(true);    
    
    //Pins 8 thru 12 as output, 8 is LSB
    DDRB = B00011111;
  
    //PORTB turn off 8 thru 12
    PORTB = B00000000;
  
    //Pin D2 as input, pulled up.
    pinMode(2, INPUT_PULLUP);   

  establishContact(); // send a byte to establish contact until receiver responds
}

void loop() 
{
  if (Serial.available() > 0)
  {
    inByte = Serial.read();   // read relay control from AgOpenGPS
    PORTB = inByte;           // write out to relays
    inByte = digitalRead(2);  // read work switch
    Serial.print(inByte);  // write byte as ascii 1 or 0 to AgOpenGPS

    IMU.readIMU();
    Serial.print(",");    
    Serial.print(IMU.euler.roll);
    Serial.print(","); 
    Serial.print(IMU.euler.pitch); 
    Serial.print(","); 
    Serial.println(IMU.euler.angVel);  
  
    Serial.flush();           // flush out buffer
  }
  delay(50);   
}

void establishContact() 
{
  while (Serial.available() <= 0) 
  {
    Serial.print("X"); // send an initial string
    delay(500);
  }
}
