// Necessary libraries
#include <SoftwareSerial.h>
#include <NewPing.h>

// Define software serial pins for GSM module
SoftwareSerial gsmSerial(7, 8); // RX, TX pins for GSM

// Define L298N motor driver pins for motor 1
const int enA = 2;
const int in1 = 3;
const int in2 = 4;

// Define L298N motor driver pins for motor 2
const int enB = 10;
const int in3 = 11;
const int in4 = 12;

// Define ultrasonic sensor pins
const int trigPin = 5;
const int echoPin = 6;

// Define buzzer pin
const int buzzerPin = 9;

// Maximum distance for the ultrasonic sensor (in cm)
const int maxDistance = 4;

// Create a NewPing object
NewPing sonar(trigPin, echoPin, maxDistance);

void setup() {
    Serial.begin(9600); // Initialize serial communication
    gsmSerial.begin(9600); // Initialize software serial for GSM

    // Set L298N motor driver pins as outputs for motor 1
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // Set L298N motor driver pins as outputs for motor 2
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output
}

void loop() {
    if (true) {
        // Check if received
        if (true) {
            // Close the barricade
            controlMotor(true, false);

            // Check for objects near the barricade
            unsigned int distance = sonar.ping_cm();
            if (distance != 0 && distance < maxDistance) {
                // Object detected, activate buzzer
                digitalWrite(buzzerPin, HIGH);
                delay(5000); // Wait for 5 seconds
                digitalWrite(buzzerPin, LOW);
            } else {
                // No object detected, open the barricade after a delay
                delay(3000); // Wait for 3 seconds
                controlMotor(false, true);
                delay(5000); // Wait for the barricade to open
            }

            // Stop the motors
            controlMotor(false, false);
        }
    }
}

// Function to control the L298N motor drivers
void controlMotor(bool closeBarricade, bool openBarricade) {
    if (closeBarricade) {
        // Close the barricade
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(enA, 255); // Maximum speed

        // For motor 2
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enB, 255); // Maximum speed
    } else if (openBarricade) {
        // Open the barricade
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(enA, 255); // Maximum speed

        // For motor 2
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enB, 255); // Maximum speed
    } else {
        // Stop the motors
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(enA, 0); // Stop motor 1

        // For motor 2
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(enB, 0); // Stop motor 2
    }
}
