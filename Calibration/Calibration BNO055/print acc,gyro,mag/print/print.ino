#include <Wire.h>
#include <Adafruit Sensor.h>
#include <Adafruit BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
    Serial.begin(115200);
    myIMU.begin();
    delay(1000);
    int8_t temp=myIMU.getTemp();
    Serial.printIn(temp);
    muIMU.setExtCrystalUse(true);
}

void loop(){
    uint8_t system, gyro, accel , mg =0;
    myIMU.getCalibration(&system, &gyro, &accel, &mg);
    imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    Serial.print(acc.x());
    Serial.print(",");
    Serial.print(acc.y());
    Serial.print(",");
    Serial.print(acc.z());
    Serial.print(",");
    
    Serial.print(accel);
    Serial.print(",");
    Serial.print(gyro);
    Serial.print(",");
    Serial.print(mg);
    Serial.print(",");
    Serial.printIn(system);

    delay(BNO055_SAMPLERATE_DELAY_MS);
}
