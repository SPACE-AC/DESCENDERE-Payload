#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
// #include <Comp6DOF_n0m1.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <Wire.h>
#include <utility/imumaths.h>

// Define connection pins

// CONFIGURATION - START

#define TEAM_ID 1022

#define BUZZER_PIN 2
#define LED_PIN 3

#define BME_ADDR 0x76
#define BNO_PCB_ADDR 0x29
#define BNO_GIMBAL_ADDR 0x28

#define XBEE_RX_PIN 7
#define XBEE_TX_PIN 8

#define SD_CS_PIN 10

#define VOLTAGE_PIN 20
#define R1_OHM 3000.0F
#define R2_OHM 1740.0F

#define SERVO_HEADING_PIN 5
#define SERVO_PITCH_PIN 6
#define CAMERA_PIN 0

// CONFIGURATION - END

#define operationEEAddr 0
#define packetCountEEAddr 1
#define groundEEAddr 19

#define SEALEVELPRESSURE_HPA 1013.25

Adafruit_BNO055 bnoPcb = Adafruit_BNO055(55, BNO_PCB_ADDR);
Adafruit_BNO055 bnoGimbal = Adafruit_BNO055(55, BNO_GIMBAL_ADDR);
Adafruit_BME280 bme;

Servo headingServo;
Servo pitchServo;

time_t RTCTime;

SoftwareSerial xbee(XBEE_RX_PIN, XBEE_TX_PIN);

bool shouldOperate;
char fileName[100];
float groundAlt;

struct Packet {
    unsigned long packetCount;  // tried to make this a static variable, but it didn't work
    char time[32] = "xx:xx:xx";
    float altitude;
    float temp;
    float voltage;
    float gyro_r;
    float gyro_p;
    float gyro_y;
    float accel_r;
    float accel_p;
    float accel_y;
    float mag_r;
    float mag_p;
    float mag_y;
    float pointingError;
    String state;

    String combine() {
        return String(TEAM_ID) + ',' + String(time) + ',' + packetCount + ",T," + String(altitude) + ',' +
               String(temp) + ',' + String(voltage) + ',' + gyro_r + ',' + gyro_p + ',' + gyro_y +
               ',' + accel_r + ',' + accel_p + ',' + accel_y + ',' + mag_r + ',' +
               mag_p + ',' + mag_y + ',' + pointingError + ',' + state + "$$$";
    }
};
Packet packet;

void blink(const unsigned int&, const unsigned int& delay_ms = 100);
void beep(const unsigned int&, const unsigned int& delay_ms = 100);
void blinkbeep(const unsigned int&, const unsigned int& delay_ms = 100);
void recovery();
void getBmeData();
void getBnoData();
void getVoltage();
void gimbalCalibration();
time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void setup() {
    // Components setup
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(CAMERA_PIN, OUTPUT);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    digitalWrite(CAMERA_PIN, HIGH);
    headingServo.attach(SERVO_HEADING_PIN);
    pitchServo.attach(SERVO_PITCH_PIN);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);

    Wire.setTimeout(3000000);

    Serial.begin(115200);
    xbee.begin(115200);
    if (bme.begin(BME_ADDR, &Wire))
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        blinkbeep(1);
        delay(500);
    }
    if (bnoPcb.begin())
        Serial.println("✔ SUCCEED: PCB BNO055");
    else {
        Serial.println("[FAILED] Unable to set up PCB BNO055!");
        blinkbeep(2);
        delay(500);
    }
    if (bnoGimbal.begin())
        Serial.println("✔ SUCCEED: Gimbal BNO055");
    else {
        Serial.println("[FAILED] Unable to set up Gimbal BNO055!");
        blinkbeep(4);
        delay(500);
    }
    bnoPcb.setExtCrystalUse(true);
    bnoGimbal.setExtCrystalUse(true);

    // // get bno055 offset
    // bnoPcb.setMode(OPERATION_MODE_CONFIG);
    // bnoGimbal.setMode(OPERATION_MODE_CONFIG);
    // delay(1000);
    // bnoPcb.setMode(OPERATION_MODE_NDOF);
    // bnoGimbal.setMode(OPERATION_MODE_NDOF);
    // delay(1000);
    // bnoPcb.getSensorOffsets(bnoPcb.accelOffset, bnoPcb.gyroOffset, bnoPcb.magOffset);
    // bnoGimbal.getSensorOffsets(bnoGimbal.accelOffset, bnoGimbal.gyroOffset, bnoGimbal.magOffset);
    // bnoPcb.setMode(OPERATION_MODE_CONFIG);
    // bnoGimbal.setMode(OPERATION_MODE_CONFIG);
    // delay(1000);
    // bnoPcb.setMode(OPERATION_MODE_NDOF);
    // bnoGimbal.setMode(OPERATION_MODE_NDOF);
    // delay(1000);

    if (SD.begin(SD_CS_PIN))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        blinkbeep(5);
    }

    delay(1000);

    setSyncProvider(getTeensy3Time);

    recovery();  // Recovery from EEPROM in case of power failure
    Serial.println("READY NOW");
    delay(3000);
}

uint32_t lastOn = 0, lastOff = 0;
void doCommand(String telem) {
    telem.trim();
    Serial.println("IN: " + telem);
    if (telem == "ON") {
        if (millis() - lastOn < 1000) return;
        shouldOperate = true;
        Serial.println("OPERATION ON");
        packet.packetCount = 0;
        EEPROM.update(packetCountEEAddr, packet.packetCount);
        groundAlt = round(bme.readAltitude(SEALEVELPRESSURE_HPA));
        EEPROM.update(groundEEAddr, groundAlt);
        EEPROM.update(operationEEAddr, true);
        digitalWrite(CAMERA_PIN, LOW);
        delay(550);
        digitalWrite(CAMERA_PIN, HIGH);
        lastOn = millis();
    } else if (telem == "OFF") {
        if (millis() - lastOff < 1000) return;
        shouldOperate = false;
        Serial.println("OPERATION OFF");
        EEPROM.update(operationEEAddr, 0);
        headingServo.write(90);
        pitchServo.write(90);
        digitalWrite(CAMERA_PIN, LOW);
        delay(550);
        digitalWrite(CAMERA_PIN, HIGH);
        lastOff = millis();
        for (int i = 0; i < 100; i++) {  // Prepare EEPROM for next session
            EEPROM.update(i, 0);
        }
    } else if (telem == "POLL") {
        // beep(1);

        packet.packetCount += 1;
        sprintf(packet.time, "%02d:%02d:%02d", hour(), minute(), second());
        String outTelemetry = packet.combine();
        // String outTelemetry = packet.combine() + "\r\r\r";

        digitalWrite(LED_PIN, HIGH);
        String outTelemetry1 = outTelemetry.substring(0, outTelemetry.length() / 2);
        String outTelemetry2 = outTelemetry.substring(outTelemetry.length() / 2, outTelemetry.length());
        // xbee.print(outTelemetry1);
        // Serial.print(outTelemetry1);
        // // delay(50);
        // xbee.print(outTelemetry2);
        // Serial.print(outTelemetry2);
        xbee.print(outTelemetry);
        Serial.print(outTelemetry);

        digitalWrite(LED_PIN, LOW);

        EEPROM.update(packetCountEEAddr, packet.packetCount);
        File file = SD.open(fileName, FILE_WRITE);
        if (file) {
            file.println(outTelemetry);
            file.close();
        }
    }
}

unsigned long time_lastrun = 0;
unsigned long timeLastGimbal = 0;
void loop() {
    if (millis() - time_lastrun > 200) {
        time_lastrun = millis();
        getVoltage();
        getBnoData();
        getBmeData();
    }
    if (Serial.available()) {
        String inTelemetry = Serial.readStringUntil('\n');
        doCommand(inTelemetry);
    }
    if (xbee.available()) {
        String inTelemetry = xbee.readStringUntil('\r');
        doCommand(inTelemetry);
    }
    if (shouldOperate && millis() - timeLastGimbal > 10) {
        timeLastGimbal = millis();
        gimbalCalibration();
    } else {
        // digitalWrite(BUZZER_PIN, LOW);
    }
}

void recovery() {
    shouldOperate = EEPROM.read(operationEEAddr);
    if (shouldOperate) {
        Serial.println("Recovery status: Operating, recovering...");
        EEPROM.get(packetCountEEAddr, packet.packetCount);
        EEPROM.get(groundEEAddr, groundAlt);
        digitalWrite(CAMERA_PIN, LOW);
        delay(550);
        digitalWrite(CAMERA_PIN, HIGH);
    } else {
        Serial.println("Recovery status: Not Operating");
    }

    // Determine mission file name
    int fileIndex = 0;
    do {
        fileIndex++;
        String("TP_" + String(fileIndex) + ".txt").toCharArray(fileName, 100);
    } while (SD.exists(fileName));
    Serial.print("Selected file name: ");
    Serial.println(fileName);
    blinkbeep(3);
}

void getBmeData() {
    packet.temp = bme.readTemperature();
    packet.altitude = (bme.readAltitude(SEALEVELPRESSURE_HPA)) - groundAlt;
    // packet.altitude = (bme.readAltitude(SEALEVELPRESSURE_HPA)) - 0;
}
void getBnoData() {
    imu::Vector<3> gyro = bnoPcb.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    packet.gyro_r = gyro.x();
    packet.gyro_p = gyro.y();
    packet.gyro_y = gyro.z();
    imu::Vector<3> accel = bnoPcb.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    packet.accel_r = accel.x();
    packet.accel_p = accel.y();
    packet.accel_y = accel.z();
    imu::Vector<3> mag = bnoPcb.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    packet.mag_r = mag.x();
    packet.mag_p = mag.y();
    packet.mag_y = mag.z();
}

void getVoltage() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    packet.voltage = apparentVoltage * (R1_OHM + R2_OHM) / R2_OHM;
    if (packet.voltage < 5.3) {
        blink(10, 25);
    }
}

void blink(const unsigned int& count, const unsigned int& delay_ms) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(LED_PIN, LOW);
        delay(delay_ms);
    }
}

void beep(const unsigned int& count, const unsigned int& delay_ms) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delay_ms);
    }
}

void blinkbeep(const unsigned int& count, const unsigned int& delay_ms) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delay_ms);
    }
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double qw, qx, qy, qz;
double roll, pitch, heading;
double pcb_roll, pcb_pitch, pcb_heading;
double lengthRotated = 0;
void adjustPitch() {
    // roll is, in fact, pitch in real world (due to sensor placement)
    if (roll >= 130 && roll <= 140) {
        pitchServo.write(90);
        return;
    }
    if (roll > -45 && roll < 135) {
        pitchServo.write(mapf(roll, 135, -45, 85, 50));  // 85
    } else {
        pitchServo.write(98);  // 98
    }
}
void adjustHeading() {
    // magnetic north (0) is south pole
    // if (heading >= 175 && heading <= 180 || heading >= -180 && heading <= -175) {

    // if (heading >= -10 && heading <= 10) {
    //     headingServo.write(90);
    //     return;
    // }

    // prevent wire twisting
    // if (heading >= 175 && heading <= 180) {
    //     headingServo.write(180);
    //     delay(500);
    //     headingServo.write(90);
    //     return;
    // } else if (heading >= -180 && heading <= -175) {
    //     headingServo.write(0);
    //     delay(500);
    //     headingServo.write(90);
    //     return;
    // }

    if (heading > -180 && heading < 0) {
        headingServo.write(mapf(heading, 0, -180, 90, -20));  // 85 CW 86 88
        lengthRotated -= 1;
    } else {
        headingServo.write(mapf(heading, 0, 180, 90, 200));  // 98 CCW 95 93
        lengthRotated += 1;
    }
}

void (*resetFunc)(void) = 0;

bool initialRound = true;
// float initialDiff = 0;
double previousDegree;
double degreesRotated = 0;
void getRotatedDegrees() {
    // straighten degrees
    // if (heading < 0) heading = mapf(heading, -180, 0, 180, 360);
    // if (pcb_heading < 0) pcb_heading = mapf(pcb_heading, -180, 0, 180, 360);
    // const float currentDiff = heading - pcb_heading;
    if (!initialRound) degreesRotated += heading - previousDegree;
    previousDegree = pcb_heading;
    initialRound = false;

    Serial.print(heading);
    Serial.print(" ");
    Serial.println(degreesRotated);
    if (degreesRotated > 270) {
        headingServo.write(70);
    } else if (degreesRotated < -270) {
        headingServo.write(110);
    }
    // if (initialRound) {
    //     return;
    //     // initialDiff = currentDiff;
    // }else{
    //     // if (abs(currentDiff - initialDiff) > 360) {
    //     //     lengthRotated = 0;
    //     //     initialDiff = currentDiff;
    //     // }
    // }

    // if(heading - previousDegree>0){

    // }else{
    // degreesRotated+=heading - previousDegree;
    // }

    qw = bnoPcb.getQuat().w();
    qx = bnoPcb.getQuat().x();
    qy = bnoPcb.getQuat().y();
    qz = bnoPcb.getQuat().z();
    double qnorm = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);  // normalize the quaternion
    qw *= qnorm;
    qx *= qnorm;
    qy *= qnorm;
    qz *= qnorm;
    pcb_roll = 180 / M_PI * atan2(qw * qx + qy * qz, 0.5 - qx * qx - qy * qy);
    pcb_pitch = 180 / M_PI * asin(2 * (qw * qy - qx * qz));
    pcb_heading = 180 / M_PI * atan2(qw * qz + qx * qy, 0.5 - qy * qy - qz * qz);
}

void gimbalCalibration() {
    // Do gimbal calibration logicWaiting for logic after testing
    // Don't forgot to set 'pointing error' and 'state'
    // if (!bnoGimbal.begin()) resetFunc();
    qw = bnoGimbal.getQuat().w();
    qx = bnoGimbal.getQuat().x();
    qy = bnoGimbal.getQuat().y();
    qz = bnoGimbal.getQuat().z();
    // Serial.println("Running gimbal calibration...");
    double qnorm = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);  // normalize the quaternion
    qw *= qnorm;
    qx *= qnorm;
    qy *= qnorm;
    qz *= qnorm;
    roll = 180 / M_PI * atan2(qw * qx + qy * qz, 0.5 - qx * qx - qy * qy);
    pitch = 180 / M_PI * asin(2 * (qw * qy - qx * qz));
    heading = 180 / M_PI * atan2(qw * qz + qx * qy, 0.5 - qy * qy - qz * qz);

    const imu::Vector<3> mag = bnoGimbal.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    double heading2 = atan2(mag.y(), mag.x()) * 180 / M_PI;
    // double heading3 =

    if ((isnan(roll) || isnan(pitch) || isnan(heading)) && millis() > 3000) {
        headingServo.write(90);
        pitchServo.write(90);
        // Wire.endTransmission();
        // bno.begin();
        delay(100);
        // resetFunc();
        return;
    }
    // Serial.print(roll);
    // Serial.print("\t");
    // Serial.print(pitch);
    // Serial.print("\t");
    // Serial.print(heading);
    // Serial.print("\t");
    // Serial.println(lengthRotated);
    if (millis() > 3000) {
        adjustPitch();
        adjustHeading();
        getRotatedDegrees();
        // if (heading > 0) {
        //     packet.pointingError = 180 - heading;
        // } else if (heading < 0) {
        //     packet.pointingError = -180 - heading;
        // }
        packet.pointingError = heading;
    }
}