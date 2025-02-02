#include <Servo.h>

Servo myServo;

const int servoPin = 18;   // Change this to your servo's actual pin
const int OPEN_POS = 0;    // Adjust if needed
const int CLOSE_POS = 180; // Adjust if needed
const unsigned long INTERVAL = 1000;  // 1 second delay between movements

bool isOpen = false;  // Track servo state
unsigned long lastToggleTime = 0;  // Store last toggle time

void setup() {
    Serial.begin(9600);
    myServo.attach(servoPin);

    Serial.println("🔒 Closing Claw at Startup...");
    myServo.write(CLOSE_POS);  // Start by closing the claw
}

void loop() {
    // Check if it's time to toggle the servo
    if (millis() - lastToggleTime >= INTERVAL) {
        lastToggleTime = millis();  // Reset the timer

        if (isOpen) {
            Serial.println("🔒 Closing Claw...");
            myServo.writeMicroseconds(2000);
        } else {
            Serial.println("🔓 Opening Claw...");
            myServo.writeMicroseconds(1000);
        }

        isOpen = !isOpen;  // Toggle the state
    }
}
