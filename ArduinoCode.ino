#include <PS2X_lib.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Define pins
#define RE 10
#define DE 11
#define stepPin 7   // Pulse pin for TB6600 Stepper Motor Driver
#define dirPin 8    // Direction pin for TB6600 Stepper Motor Driver
#define enPin 9     // Enable pin for TB6600 Stepper Motor Driver

// BTS7960 Motor Driver Pins
int R_EN = 2;    // Enable/disable right motor
int L_EN = 3;    // Enable/disable left motor
int RPWM = 5;   // PWM input for right motor (EN)
int LPWM = 6;   // PWM input for left motor (EN)

// Define constants
const byte nitro[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};
byte values[11];

// Software Serial for modbus communication
SoftwareSerial mod(12, 13);

PS2X ps2x;    // Create PS2 Controller Class
TinyGPS gps;  // Create GPS object

// Variables
int startTime = 0;
int error = 0;
byte type = 0;
byte vibrate = 0;
float lat, lon;
bool controllerEnable = true; // Variable to control if controller input should be processed

// Function prototypes
void readGPS();
byte nitrogen();
byte phosphorous();
byte potassium();
void stepMotor(int steps);

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);

    // Initialize PS2 Controller
    error = ps2x.config_gamepad(53, 51, 50, 52, true, true);
    if (error == 0) {
        Serial.println("Found Controller, configured successfully");
    } else if (error == 1) {
        Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    } else if (error == 2) {
        Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
    } else if (error == 3) {
        Serial.println("Controller refusing to enter Pressures mode, may not support it.");
    }

    type = ps2x.readType();
    switch (type) {
        case 0:
            Serial.println("Unknown Controller type");
            break;
        case 1:
            Serial.println("DualShock Controller Found");
            break;
        case 2:
            Serial.println("GuitarHero Controller Found");
            break;
    }

    // Initialize GPS Serial
    mod.begin(9600);

    // Initialize motor pins as outputs
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);

    // Enable the stepper driver
    digitalWrite(enPin, LOW);
}

void loop() {
    if (error == 1)
        return;

    if (type == 2)
        return;

    // Read PS2 controller input
    ps2x.read_gamepad(false, vibrate);

    // Process controller input for movement
    if (controllerEnable) {
        int ry = ps2x.Analog(PSS_RY);
        if (ry < 128) {
            digitalWrite(dirPin, HIGH); // Move up
            stepMotor(500);
        } else if (ry > 128) {
            digitalWrite(dirPin, LOW); // Move down
            stepMotor(500);
        }
    }

    // Example controller input handling for GPS and soil data
    if (ps2x.ButtonPressed(PSB_BLUE)) {
        startTime = millis() / 1000;
        while (true) {
            readGPS();
            if (lon != 0 && lat != 0) {
                byte val1, val2, val3;
                val1 = nitrogen();
                delay(250);
                val2 = phosphorous();
                delay(250);
                val3 = potassium();
                delay(250);

                String dataString = "data,latitude:" + String(lat, 6) + ",longitude:" + String(lon, 6) + ",nitrogen:" + String(val1) + ",phosphorous:" + String(val2) + ",potassium:" + String(val3);
                Serial.println(dataString);

                // Send data to Raspberry Pi via Serial
                Serial1.println(dataString);
                break;
            }
            if (millis() / 1000 - startTime >= 5) {
                Serial.println("Aborting action, no GPS signal.");
                break;
            }
            delay(500);
            Serial.println("Waiting for GPS signal...");
        }
    }

    // Read PS2 controller input for motor control
    ps2x.read_gamepad(false, 0);
    int leftX = ps2x.Analog(PSS_LX);
    int leftY = ps2x.Analog(PSS_LY);

    int baseSpeed = map(abs(leftY), 0, 255, 0, 255);
    int turnAmount = map(leftX, 0, 255, -127, 127);

    int leftMotorSpeed = baseSpeed - turnAmount;
    int rightMotorSpeed = baseSpeed + turnAmount;

    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    if (leftMotorSpeed > 0) {
        analogWrite(LPWM, leftMotorSpeed);
        digitalWrite(L_EN, HIGH);
    } else {
        analogWrite(LPWM, -leftMotorSpeed);
        digitalWrite(L_EN, LOW);
    }

    if (rightMotorSpeed > 0) {
        analogWrite(RPWM, rightMotorSpeed);
        digitalWrite(R_EN, HIGH);
    } else {
        analogWrite(RPWM, -rightMotorSpeed);
        digitalWrite(R_EN, LOW);
    }

    if (ps2x.ButtonPressed(PSB_GREEN)) {
        Serial.print("LeftX: ");
        Serial.print(leftX);
        Serial.print("\tLeftY: ");
        Serial.print(leftY);
        Serial.print("\tLeft Motor: ");
        Serial.print(leftMotorSpeed);
        Serial.print("\tRight Motor: ");
        Serial.println(rightMotorSpeed);
    }

    delay(50);
}

void readGPS() {
    while (Serial1.available()) {
        if (gps.encode(Serial1.read())) {
            gps.f_get_position(&lat, &lon);
        }
    }
}

byte nitrogen() {
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    if (mod.write(nitro, sizeof(nitro)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for (byte i = 0; i < 7; i++) {
            values[i] = mod.read();
        }
    }
    return values[4];
}

byte phosphorous() {
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    if (mod.write(phos, sizeof(phos)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for (byte i = 0; i < 7; i++) {
            values[i] = mod.read();
        }
    }
    return values[4];
}

byte potassium() {
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    if (mod.write(pota, sizeof(pota)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for (byte i = 0; i < 7; i++) {
            values[i] = mod.read();
        }
    }
    return values[4];
}

void stepMotor(int steps) {
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
    }
}
