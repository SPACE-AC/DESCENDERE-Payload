#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <InternalTemperature.h>
#include <PID_v1.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include <queue>

// *** CONFIGURATION - START ***

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
#define R1_OHM 2000.0F
#define R2_OHM 1250.0F

#define SERVO_HEADING_PIN 5
#define SERVO_PITCH_PIN 6
#define CAMERA_PIN 9

#define GIMBAL_CHECK_RATE 20

// *** CONFIGURATION - END ***

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
bool isBnoGimbalOk = true;
float ptrErr = 0;
uint32_t fullRangeTime;
float initialDifference;
double pitch, heading;
double pcb_roll, pcb_pitch, pcb_heading;
double angleRotated = 0, gimbalAngleRotated = 0;
bool shouldComputePid = true;

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
    double pointingError = 0;
    String state;

    String combine() {
        return String(TEAM_ID) + "," + String(time) + "," + packetCount + ",T," + String(altitude) + "," +
               String(temp) + "," + String(voltage) + "," + gyro_r + "," + gyro_p + "," + gyro_y +
               "," + accel_r + "," + accel_p + "," + accel_y + "," + mag_r + "," +
               mag_p + "," + mag_y + "," + pointingError + "," + state + "$";
    }
};
Packet packet;

double Setpoint = 0, pidOutput;
double tError;
PID pid(&tError, &pidOutput, &Setpoint, 0.8, 0.000001, 0.02, REVERSE);
// PID pid(&packet.pointingError, &servoSpeed, &Setpoint, 0, 100, 3100, DIRECT);
// AutoPID pid(&packet.pointingError, &Setpoint, &servoSpeed, 0, 180, 5, 5, 0.1);

class SimpleMovingAverage {
    int size;
    int count;
    std::queue<float> buffer;
    float sum;

   public:
    SimpleMovingAverage(int size) {
        this->size = size;
        count = 0;
        sum = 0;
    }

    void add(float value) {
        if (count < size) {
            buffer.push(value);
            sum += value;
            count++;
        } else {
            sum -= buffer.front();
            buffer.pop();
            buffer.push(value);
            sum += value;
        }
    }

    float getAverage() {
        return sum / count;
    }
}(sma)(100);

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

    // headingServo.write(180);
    // delay(430);
    // headingServo.write(90);
    // while (true)
    //     ;

    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
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
        isBnoGimbalOk = false;
        blinkbeep(4);
        delay(500);
    }
    bnoPcb.setExtCrystalUse(true);
    bnoGimbal.setExtCrystalUse(true);

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(GIMBAL_CHECK_RATE);

    if (SD.begin(SD_CS_PIN))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        blinkbeep(5);
    }

    delay(1000);

    setSyncProvider(getTeensy3Time);

    // headingServo.write(180);
    // delay(545);  // 430
    // headingServo.write(90);
    // while (true)
    //     ;

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
        beep(3);
        packet.state = "TARGET_POINTING";
        shouldOperate = true;
        Serial.println("OPERATION ON");
        packet.packetCount = 0;
        EEPROM.update(packetCountEEAddr, packet.packetCount);
        EEPROM.update(groundEEAddr, groundAlt);
        EEPROM.update(operationEEAddr, true);
        angleRotated = 0;
        digitalWrite(CAMERA_PIN, LOW);
        delay(550);
        digitalWrite(CAMERA_PIN, HIGH);
        lastOn = millis();
    } else if (telem == "OFF") {
        if (millis() - lastOff < 1000) return;
        beep(2);
        packet.state = "IDLE";
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

        digitalWrite(LED_PIN, HIGH);
        xbee.print(outTelemetry);
        Serial.print(outTelemetry);

        digitalWrite(LED_PIN, LOW);

        EEPROM.update(packetCountEEAddr, packet.packetCount);
        File file = SD.open(fileName, FILE_WRITE);
        if (file) {
            file.println(outTelemetry);
            file.close();
        }
    } else if (telem.startsWith("ST,")) {
        beep(1);
        int hr = telem.substring(3, 5).toInt();
        int min = telem.substring(6, 8).toInt();
        int sec = telem.substring(9, 11).toInt();
        setTime(hr, min, sec, day(), month(), year());
    } else if (telem == "ALT") {
        beep(1);
        groundAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
        EEPROM.put(groundEEAddr, groundAlt);
    }
}

unsigned long time_lastrun = 0;
unsigned long timeLastGimbal = 0;
void loop() {
    if (millis() - time_lastrun > 20) {
        time_lastrun = millis();
        getVoltage();
        getBnoData();
        getBmeData();
        // doCommand("POLL");
    }
    if (Serial.available()) {
        String inTelemetry = Serial.readStringUntil('\n');
        doCommand(inTelemetry);
    }
    if (xbee.available()) {
        String inTelemetry = xbee.readStringUntil('\r');
        doCommand(inTelemetry);
    }
    if (shouldOperate && millis() - timeLastGimbal > GIMBAL_CHECK_RATE) {
        timeLastGimbal = millis();
        // Serial.print(pcb_heading);
        // Serial.print(",");
        // Serial.println(getRotatedDegrees());

        if (isBnoGimbalOk) gimbalCalibration();
    } else {
        // digitalWrite(BUZZER_PIN, LOW);
    }
    if (shouldComputePid) pid.Compute();
}

void recovery() {
    shouldOperate = EEPROM.read(operationEEAddr);
    packet.packetCount = EEPROM.read(packetCountEEAddr);
    EEPROM.get(groundEEAddr, groundAlt);
    if (shouldOperate) {
        Serial.println("Recovery status: Operating, recovering...");
        digitalWrite(CAMERA_PIN, LOW);
        delay(550);
        digitalWrite(CAMERA_PIN, HIGH);
        packet.state = "ACQUIRING_TARGET";
    } else {
        Serial.println("Recovery status: Not Operating");
        packet.state = "IDLE";
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

    // checkSwingBack();
    // Serial.println("Swing test complete");
    // Serial.print(fullRangeTime);
    // Serial.print(" ms,");
    // Serial.print(" Angle: ");
    // Serial.println(gimbalAngleRotated);

    // while (true)
    //     ;

    // initialDifference = getRotatedDegrees();
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

    imu::Quaternion quat = bnoPcb.getQuat();
    double qw = quat.w();
    double qx = quat.x();
    double qy = quat.y();
    double qz = quat.z();
    double qnorm = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);  // normalize the quaternion
    qw *= qnorm;
    qx *= qnorm;
    qy *= qnorm;
    qz *= qnorm;
    double roll = 180 / M_PI * atan2(qw * qx + qy * qz, 0.5 - qx * qx - qy * qy);
    double pitch = 180 / M_PI * asin(2 * (qw * qy - qx * qz));
    pcb_heading = 180 / M_PI * atan2(qw * qz + qx * qy, 0.5 - qy * qy - qz * qz);
    // Serial.print(roll);
    // Serial.print(",");
    // Serial.print(pitch);
    // Serial.print(",");
    // Serial.println(yaw);
}

void getVoltage() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    packet.voltage = apparentVoltage * (R1_OHM + R2_OHM) / R2_OHM;
    // if (packet.voltage < 5.3) {
    //     blink(10, 25);
    // }
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
void getGimbalIMU() {
    // Do gimbal calibration logicWaiting for logic after testing
    // Don't forgot to set 'pointing error' and 'state'
    // if (!bnoGimbal.begin()) resetFunc();
    imu::Quaternion quat = bnoGimbal.getQuat();
    qw = quat.w();
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();
    // Serial.println("Running gimbal calibration...");
    double qnorm = 1 / sqrt(qw * qw + qx * qx + qy * qy + qz * qz);  // normalize the quaternion
    qw *= qnorm;
    qx *= qnorm;
    qy *= qnorm;
    qz *= qnorm;
    pitch = 180 / M_PI * atan2(qw * qx + qy * qz, 0.5 - qx * qx - qy * qy);
    // pitch = 180 / M_PI * asin(2 * (qw * qy - qx * qz));
    heading = 180 / M_PI * atan2(qw * qz + qx * qy, 0.5 - qy * qy - qz * qz);
    // Serial.println(bnoGimbal.getVector(Adafruit_BNO055::VECTOR_EULER).z());
}

float getRotatedDegrees() {
    static double lastAngle = pcb_heading;
    angleRotated += circleDifference(pcb_heading, lastAngle);
    lastAngle = pcb_heading;
    return angleRotated;
}

float getGimbalRotatedDegrees() {
    static double lastGimbalAngle = heading;
    gimbalAngleRotated += circleDifference(heading, lastGimbalAngle);
    lastGimbalAngle = heading;
    return gimbalAngleRotated;
}

void adjustPitch() {
    // pitch = pitch - 135;
    // if (pitch < -180) pitch += 360;
    if (pitch >= 130 && pitch <= 140) {
        pitchServo.write(90);
        return;
    }
    if (pitch > -45 && pitch < 135) {
        pitchServo.write(mapf(pitch, 135, 125, 90, 85));  // 85 50
    } else {
        // pitchServo.write(mapf(pitch, 135, 180, 98, 100));  // 98
        pitchServo.write(98);  // 98
    }
}
void adjustHeading() {
    // ptrErr = bnoGimbal.getVector(Adafruit_BNO055::VECTOR_EULER).z() - 180;
    tError = ptrErr < 0 ? -ptrErr : ptrErr;

    // PID ver. starts here
    double outcome = ptrErr < 0 ? -pidOutput + 88 : pidOutput + 95;
    if (tError > 5) {
        shouldComputePid = true;
        headingServo.write(outcome);
    } else {
        shouldComputePid = false;
        headingServo.write(90);
    }
    getGimbalIMU();

    packet.pointingError = heading;
    // Serial.print(ptrErr);
    // Serial.print(" ");
    // Serial.println(outcome);
    float rotated = getRotatedDegrees();
    Serial.print(pcb_heading);
    Serial.print(",");
    Serial.print(rotated);
    Serial.print(",");
    Serial.print(outcome);
    Serial.print(",");
    Serial.println(packet.pointingError);
    // if (rotated > 540) {
    //     headingServo.write(0);
    //     delay(500);  // 430 545
    //     headingServo.write(90);
    //     angleRotated -= 360;
    // }
    // if (rotated < 0) {
    //     headingServo.write(180);
    //     delay(500);
    //     headingServo.write(90);
    //     angleRotated += 360;
    // }
}

void (*resetFunc)(void) = 0;

bool initialRound = true;
// float initialDiff = 0;
double previousDegree;

double circleDifference(double left, double right) {
    if (left > 0 && right < 0 && abs(left - right) > 180) {
        right += 360;
        return right - left;
    }
    if (left < 0 && right > 0 && abs(left - right) > 180) {
        left += 360;
        return left - right;
    }
    return left - right;
}

bool isHeadingGood(double heading) {
    return heading >= -20 && heading <= 20;
}

void checkSwingBack() {
    // Move to one end
    headingServo.write(45);
    delay(3000);
    getGimbalRotatedDegrees();
    headingServo.write(135);

    uint32_t startTime = millis();
    gimbalAngleRotated = 0;
    double lastAngle = 0;
    do {
        lastAngle = gimbalAngleRotated;
        getGimbalIMU();
        getGimbalRotatedDegrees();

        // if () {
        fullRangeTime = millis() - startTime;
        //     return;
        // }
        delay(50);
        Serial.print(gimbalAngleRotated);
        Serial.print(",");
        Serial.println(lastAngle);
    } while (abs(lastAngle - gimbalAngleRotated) > 0);
    headingServo.write(90);
}

short driftness;

void gimbalCalibration() {
    getGimbalIMU();
    getBnoData();
    const imu::Vector<3> mag = bnoGimbal.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    if ((isnan(pitch) || isnan(heading)) && millis() > 3000) {
        headingServo.write(90);
        pitchServo.write(90);
        // Wire.endTransmission();
        // bno.begin();
        delay(100);
        // resetFunc();
        return;
    }

    // If error stays, bounce back
    //
    // if (!isHeadingGood(heading)) {
    //     if (heading > 0)
    //         driftness += 1;
    //     else if (heading < 0)
    //         driftness -= 1;
    // } else {
    //     driftness > 0 ? driftness-- : driftness++;
    // }
    // if (driftness > 400) {
    //     driftness = 0;

    //     headingServo.write(180);
    //     delay(200);
    //     headingServo.write(90);
    //     return;
    // }
    // if (driftness < -400) {
    //     driftness = 0;

    //     headingServo.write(0);
    //     delay(200);
    //     headingServo.write(90);
    //     return;
    // }

    if (millis() > 3000) {
        adjustPitch();
        adjustHeading();
        ptrErr = heading;
        // sma.add(packet.pointingError);
    }
}