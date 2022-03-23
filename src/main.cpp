#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
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
#define BNO_PCB_ADDR 0x28
#define BNO_GIMBAL_ADDR 0x29

#define XBEE_RX_PIN 7
#define XBEE_TX_PIN 8

#define SD_CS_PIN 10

#define VOLTAGE_PIN 20
#define R1_OHM 3000.0F
#define R2_OHM 1740.0F

// CONFIGURATION - END

#define operationEEAddr 0
#define packetCountEEAddr 1
#define groundEEAddr 19

#define SEALEVELPRESSURE_HPA 1013.25

Adafruit_BNO055 bnoPcb = Adafruit_BNO055(55, BNO_PCB_ADDR);
Adafruit_BNO055 bnoGimbal = Adafruit_BNO055(55, BNO_GIMBAL_ADDR);
Adafruit_BME280 bme;

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
        return String(TEAM_ID) + ',' + String(time) + ',' + packetCount + ",T," + altitude + ',' +
               temp + ',' + String(voltage) + ',' + gyro_r + ',' + gyro_p + ',' + gyro_y +
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
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);

    Serial.begin(115200);
    xbee.begin(115200);
    if (bme.begin(BME_ADDR, &Wire))
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        beep(1);
        delay(500);
    }
    if (bnoPcb.begin())
        Serial.println("✔ SUCCEED: PCB BNO055");
    else {
        Serial.println("[FAILED] Unable to set up PCB BNO055!");
        beep(2);
        delay(500);
    }
    if (bnoGimbal.begin())
        Serial.println("✔ SUCCEED: Gimbal BNO055");
    else {
        Serial.println("[FAILED] Unable to set up Gimbal BNO055!");
        beep(4);
        delay(500);
    }
    bnoPcb.setExtCrystalUse(true);
    bnoGimbal.setExtCrystalUse(true);

    if (SD.begin(SD_CS_PIN))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        beep(5);
    }

    delay(1000);

    setSyncProvider(getTeensy3Time);

    recovery();  // Recovery from EEPROM in case of power failure
    Serial.println("READY NOW");
    delay(3000);
}

void doCommand(String telem) {
    telem.trim();
    Serial.println("IN: " + telem);
    if (telem == "ON") {
        packet.packetCount = 0;
        EEPROM.update(packetCountEEAddr, packet.packetCount);
        groundAlt = round(bme.readAltitude(SEALEVELPRESSURE_HPA));
        EEPROM.update(groundEEAddr, groundAlt);
        EEPROM.update(operationEEAddr, true);
        shouldOperate = true;
    } else if (telem == "OFF") {
        EEPROM.update(operationEEAddr, 0);
        shouldOperate = false;
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
    if (shouldOperate) {
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
        blinkbeep(3);

    } else {
        Serial.println("Recovery status: Not Operating");
        blinkbeep(4);
    }

    // Determine mission file name
    int fileIndex = 0;
    do {
        fileIndex++;
        String("TP_" + String(fileIndex) + ".txt").toCharArray(fileName, 100);
    } while (SD.exists(fileName));
    Serial.print("Selected file name: ");
    Serial.println(fileName);
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
        blinkbeep(10, 25);
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

void gimbalCalibration() {
    // Do gimbal calibration logicWaiting for logic after testing
    // Don't forgot to set 'pointing error' and 'state'
}