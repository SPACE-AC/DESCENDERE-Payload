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

#define LED_PIN 3

#define BME_ADDR 0x76
#define BNO_PCB_ADDR 0x28
#define BNO_GIMBAL_ADDR 0x29

#define XBEE_RX_PIN 22
#define XBEE_TX_PIN 21

#define SD_CS_PIN 10

#define VOLTAGE_PIN 15
#define R1_OHM 1000
#define R2_OHM 1250

// CONFIGURATION - END

#define operationEEAddr 0
#define packetCountEEAddr 1
#define groundEEAddr 19

#define SEALEVELPRESSURE_HPA 1013.25

Adafruit_BNO055 bnoPcb = Adafruit_BNO055(55, BNO_PCB_ADDR);
Adafruit_BNO055 bnoGimbal = Adafruit_BNO055(55, BNO_GIMBAL_ADDR);
Adafruit_BME280 bme;

// SoftwareSerial xbee(XBEE_RX_PIN, XBEE_TX_PIN, true);
// auto Serial5 = Serial5;

unsigned long time_lastrun = 0;
unsigned long time_current = 0;

bool shouldOperate;
char fileName[100];
float groundAlt;

struct Packet {
    unsigned int packetCount;  // tried to make this a static variable, but it didn't work
    char* time;
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
        return TEAM_ID + ',' + String(time) + ',' + packetCount + ",P," + altitude + ',' +
               temp + ',' + voltage + ',' + gyro_r + ',' + gyro_p + ',' + gyro_y +
               ',' + accel_r + ',' + accel_p + ',' + accel_y + ',' + mag_r + ',' +
               mag_p + ',' + mag_y + ',' + pointingError + ',' + state;
    }
};
Packet packet;

void blink(uint8_t, const unsigned int&, const unsigned int& delay_ms = 100);
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
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(9600);
    Serial5.begin(9600);
    if (bme.begin(BME_ADDR, &Wire))
        Serial.println("✔ SUCCEED: BME280");
    else
        Serial.println("[FAILED] Unable to set up BME280!");
    if (bnoPcb.begin())
        Serial.println("✔ SUCCEED: PCB BNO055");
    else
        Serial.println("[FAILED] Unable to set up PCB BNO055!");
    if (bnoGimbal.begin())
        Serial.println("✔ SUCCEED: Gimbal BNO055");
    else
        Serial.println("[FAILED] Unable to set up Gimbal BNO055!");
    bnoPcb.setExtCrystalUse(true);
    bnoGimbal.setExtCrystalUse(true);

    if (SD.begin(SD_CS_PIN))
        Serial.println("✔ SUCCEED: SD card");
    else
        Serial.println("[FAILED] SD card initialization failed!");

    setSyncProvider(getTeensy3Time);

    recovery();  // Recovery from EEPROM in case of power failure
    Serial5.print("READY NOW");
    delay(3000);
}

void loop() {
    // Serial.println(Serial5.available());
    // if (Serial5.available()) {
    //     Serial.println("XBEE Data Available");
    //     String inTelemetry = Serial5.readStringUntil('$');
    //     inTelemetry.trim();
    //     Serial.println("Incoming Telem: " + inTelemetry);
    //     if (inTelemetry == "CMD,1022,TP,ON") {
    //         packet.packetCount = 0;
    //         groundAlt = round(bme.readAltitude(SEALEVELPRESSURE_HPA));
    //         EEPROM.update(groundEEAddr, groundAlt);
    //         EEPROM.update(operationEEAddr, true);
    //         shouldOperate = true;
    //     } else if (inTelemetry == "CMD,1022,TP,OFF") {
    //         EEPROM.update(operationEEAddr, 0);
    //         shouldOperate = false;
    //         for (int i = 0; i < 100; i++) {  // Prepare EEPROM for next session
    //             EEPROM.update(i, 0);
    //         }
    //     } else if (inTelemetry == "CMD,1022,TP,POLL") {
    digitalWrite(LED_PIN, HIGH);

    getBnoData();
    getBmeData();
    packet.packetCount += 1;
    sprintf(packet.time, "%02d:%02d:%02d", hour(), minute(), second());
    String outTelemetry = packet.combine();
    // Serial5.print(outTelemetry);
    Serial.println("outputing");
    Serial.print(outTelemetry);
    Serial.println("Out: " + outTelemetry);
    EEPROM.put(packetCountEEAddr, packet.packetCount);
    File file = SD.open(fileName, FILE_WRITE);
    if (file) {
        file.println(outTelemetry);
        file.close();
    }

    //         digitalWrite(LED_PIN, LOW);
    //     }
    // }
    if (shouldOperate) {
        gimbalCalibration();
    } else {
        digitalWrite(LED_PIN, LOW);
    }
    delay(1000);
}

void recovery() {
    shouldOperate = EEPROM.read(operationEEAddr);
    if (shouldOperate) {
        Serial.println("Recovery status: Operating, recovering...");
        EEPROM.get(packetCountEEAddr, packet.packetCount);
        EEPROM.get(groundEEAddr, groundAlt);
        blink(LED_PIN, 3);

    } else {
        Serial.println("Recovery status: Not Operating");
        blink(LED_PIN, 4);
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
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1024.0;
    packet.voltage = apparentVoltage * (R1_OHM + R2_OHM) / R2_OHM;
}

void blink(uint8_t pin, const unsigned int& count, const unsigned int& delay_ms) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(pin, HIGH);
        delay(delay_ms);
        digitalWrite(pin, LOW);
        delay(delay_ms);
    }
}

void gimbalCalibration() {
    // Do gimbal calibration logicWaiting for logic after testing
    // Don't forgot to set 'pointing error' and 'state'
}