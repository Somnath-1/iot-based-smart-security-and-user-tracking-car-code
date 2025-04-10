#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_VL53L0X.h>

// Bluetooth & GPS Pins
#define HC05_TX 2
#define HC05_RX 3
#define GPS_TX 4
#define GPS_RX 13

// Motor Driver Pins
#define ENA 6
#define ENB 10
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 12
#define ENA2 5
#define ENB2 11
#define IN5 A0
#define IN6 A1
#define IN7 A2
#define IN8 A3

#define SLAVE_ADDRESS 8

SoftwareSerial btSerial(HC05_TX, HC05_RX);
SoftwareSerial gpsSerial(GPS_TX, GPS_RX);
TinyGPSPlus gps;
Adafruit_VL53L0X lox;

// Status Variables
bool objectDetected = false, alertDetected = false, userTracking = false;
bool faceRecognition = false, liveStream = false, captureImage = false;
float userLat, userLon;
String wifiSSID = "", wifiPASS = "";
void sendToSecondaryArduino(String data);

void setup() {
    Wire.begin();
    Serial.begin(115200);
    btSerial.begin(9600);
    gpsSerial.begin(9600);
    Serial.println("Initializing LiDAR...");
    if (!lox.begin()) {
        Serial.println("Failed to initialize LiDAR!");
        while (1);   
    }
    Serial.println("LiDAR Initialized Successfully.");


    // Motor Pins
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA2, OUTPUT); pinMode(ENB2, OUTPUT);
    pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
    pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);

    Serial.println("Primary Arduino Ready.");
}

void loop() {

    receiveSensorData();
    handleBluetoothCommands();

    static unsigned long lastGPSUpdate = 0;
    if (millis() - lastGPSUpdate > 5000) {
        trackUserGPS();
        lastGPSUpdate = millis();
    }

    if (!alertDetected) {
        navigate();
    } else {
        stopCar();
        waitForUserResponse();
    }
}

void receiveSensorData() {
    Wire.requestFrom(SLAVE_ADDRESS, 10); // Increased to match the new secondary Arduino data size
    if (Wire.available() == 10) {
        int frontDist = (Wire.read() << 8) | Wire.read();
        int leftDist = (Wire.read() << 8) | Wire.read();
        int rightDist = (Wire.read() << 8) | Wire.read();
        int rearDist = (Wire.read() << 8) | Wire.read();
        bool smokeDetected = Wire.read();
        bool fireDetected = Wire.read();
        bool weaponDetected = Wire.read();
        bool knifeDetected = Wire.read();

        objectDetected = (frontDist < 30 || leftDist < 30 || rightDist < 30 || rearDist < 30);

        if (smokeDetected) sendBluetoothAlert("SMOKE DETECTED");
        if (fireDetected) sendBluetoothAlert("FIRE DETECTED");
        if (weaponDetected) sendBluetoothAlert("WEAPON DETECTED");
        if (knifeDetected) sendBluetoothAlert("KNIFE DETECTED");

        alertDetected = smokeDetected || fireDetected || weaponDetected || knifeDetected;
    }
}


void trackUserGPS() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
        if (gps.location.isValid() && gps.hdop.value() < 200) {
            userLat = gps.location.lat();
            userLon = gps.location.lng();
            btSerial.print("CAR_GPS:");
            btSerial.print(userLat, 6);
            btSerial.print(",");
            btSerial.println(userLon, 6);
        }
    }
}

void navigate() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    int lidarDist = measure.RangeMilliMeter / 10;

    if (userTracking) {
        moveForwardSmoothly();
        if (objectDetected || lidarDist < 30) {
            stopCar();
            avoidObstacle();
        }
    } else {
        stopCar();
    }
}

void moveForwardSmoothly() {
    analogWrite(ENA, 100);
    analogWrite(ENB, 100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
    digitalWrite(IN7, HIGH);
    digitalWrite(IN8, LOW);
}

void stopCar() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
}

void avoidObstacle() {
    stopCar();
    delay(300);
    analogWrite(ENA, 80);
    analogWrite(ENB, 80);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(500);
    stopCar();
    delay(300);
}

void sendBluetoothAlert(String threat) {
    btSerial.print("ALERT: ");
    btSerial.print(threat);
    btSerial.println("! Type 'RESUME' to continue.");
}

void waitForUserResponse() {
    while (alertDetected) {
        if (btSerial.available()) {
            String command = btSerial.readString();
            if (command.indexOf("RESUME") >= 0) {
                alertDetected = false;
                break;
            }
        }
        delay(500);
    }
}
void handleBluetoothCommands() {
    if (btSerial.available()) {
        String input = btSerial.readStringUntil('\n');
        input.trim();

        Serial.print("BT Cmd: "); Serial.println(input); // Keep short
        if (input == "T") {
            userTracking = !userTracking;
            btSerial.println(userTracking ? "T_ON" : "T_OFF");  // Shorter Response
        } else if (input == "S") {
            stopCar();
            btSerial.println("STOP");  
        } else if (input == "R") {
            alertDetected = false;
            userTracking = true;
            btSerial.println("RESUME");
        } else if (input == "FR_ON") {
            faceRecognition = true;
            sendToSecondaryArduino("FACE_ON");
            btSerial.println("FR_ON");
        } else {
            btSerial.println("ERR");
        }
         if (wifiSSID.length() > 0 && wifiPASS.length() > 0) {
            sendToSecondaryArduino("SSID:" + wifiSSID);
            sendToSecondaryArduino("PASS:" + wifiPASS);
            wifiSSID = "";
            wifiPASS = "";
        }
    }
}
void sendToSecondaryArduino(String data) {
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(data.c_str());
    Wire.endTransmission();
    delay(100);
}
