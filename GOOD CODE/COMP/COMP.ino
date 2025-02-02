#include <Servo.h>
#include <vector>

Servo myServo;

/* 

DRIVE CONSTANTS 

*/
const int EN_A = 11; // Enable pin for first motor
const int IN1 = 9;   // Control pin for first motor
const int IN2 = 8;   // Control pin for first motor
const int IN3 = 7;   // Control pin for second motor
const int IN4 = 6;   // Control pin for second motor
const int EN_B = 10; // Enable pin for second motor
const float MOTOR_POWER = 0.30;
const int ninety_time = 1600;
const float adj_const = 1;
const float speed_per_second = 40.72;
const int turn_direction = "left";

/* 

ULTRASONIC CONSTANTS 

*/
const int trigPin = 13;
const int echoPin = 12;
const float soundSpeed = 0.034;  // Speed of sound in cm/Âµs

/* 

COLOR CONSTANTS 

*/
const int s0 = 1; 
const int s1 = 2; 
const int s2 = 3; 
const int s3 = 4; 
const int outPin = 5; 
const int oe = 0;

// Calibration Values
const int r_black = 0;
const int g_black = 0;
const int b_black = 0;
const int r_red = 0;
const int g_green = 0;
const int b_blue = 0;

const int ref_black[] = {0, 10, 0};
const int ref_blue[] = {145, 255, 246};
const int ref_green[] = {150, 255, 165};
const int ref_red[] = {250, 150, 130};

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;

float avgRed = 0, avgGreen = 0, avgBlue = 0;
const float alpha = 0.33;  // Smoothing factor (adjustable for responsiveness)

/* 

CLAW CONSTANTS 

*/
const int servoPin = 18;
const int OPEN_POS = 180;
const int CLOSE_POS = 0;
const int STEP_DELAY = 10;
const int STEP_SIZE = 10; 

void setup() {
  Serial.begin(9600);
    
  /* 

  MOTOR INIT 

  */
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN_B, OUTPUT);

  /* 

  ULTRASONIC INIT 

  */
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

	/* 

  COLOR INIT 

  */
	pinMode(s0, OUTPUT);
	pinMode(s1, OUTPUT);
	pinMode(s2, OUTPUT);
	pinMode(s3, OUTPUT);

	// Set Pulse Width scaling to 20%
	digitalWrite(s0,HIGH);
	digitalWrite(s1,LOW);

	// Set Sensor output as input
	pinMode(outPin, INPUT);
  pinMode(oe, OUTPUT);
  digitalWrite(oe, LOW);

  /* 

  SERVO INIT 

  */
  myServo.attach(servoPin);
}
// helper jits

/* 

ULTRASONIC HELPER 

*/
float getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000);  // Timeout to prevent lock-up
    return (duration == 0) ? -1 : (duration * soundSpeed) / 2;
}

/* 

COLOR HELPER 

*/

float calculateConfidence(int r, int g, int b, const int refRGB[], int tolerance = 50) {
    float score = 0;
    score += max(0, (1 - abs(r - refRGB[0]) / (float)tolerance));
    score += max(0, (1 - abs(g - refRGB[1]) / (float)tolerance));
    score += max(0, (1 - abs(b - refRGB[2]) / (float)tolerance));
    return (score / 3) * 100;  // Convert to percentage confidence
}

// Function to map pulse width to 0-255 range
int mapColor(int pw, int minPW, int maxPW) {
    return constrain(map(pw, minPW, maxPW, 255, 0), 0, 255);
}

// Function to read Red Pulse Widths
int getRedPW() {
	// Set sensor to read Red only
	digitalWrite(s2,LOW);
	digitalWrite(s3,LOW);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(outPin, LOW);
	// Return the value
	return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
	// Set sensor to read Green only
	digitalWrite(s2,HIGH);
	digitalWrite(s3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(outPin, LOW);
	// Return the value
	return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {
	// Set sensor to read Blue only
	digitalWrite(s2,LOW);
	digitalWrite(s3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(outPin, LOW);
	// Return the value
	return PW;
}

/* 

CLAW HELPER 

*/
void openServo() {
    Serial.println("Opening Claw...");
    for (int pos = CLOSE_POS; pos <= OPEN_POS; pos += STEP_SIZE) {
        myServo.write(pos);
        delay(STEP_DELAY);
    }
}

// Function to Close Servo
void closeServo() {
    Serial.println("Closing Claw...");
    for (int pos = OPEN_POS; pos >= CLOSE_POS; pos -= STEP_SIZE) {
        myServo.write(pos);
        delay(STEP_DELAY);
    }
}

/* 

GENERAL HELPER 

*/

float calculateEMA(float newSample, float prevEMA, float alpha) {
    return alpha * newSample + (1 - alpha) * prevEMA;
}

// Timing variables
unsigned long lastColorUpdate = 0;
const unsigned long COLOR_UPDATE_INTERVAL = 100;  // Update interval (ms)

// Stable color tracking variables
String stableColor = "Unknown";   // Last confirmed stable color
String potentialColor = "Unknown"; // Color being evaluated
int stabilityCounter = 0;         // Number of consistent detections required
const int STABILITY_THRESHOLD = 5; // Number of times a color must be detected consecutively

std::vector<String> colorsSeen; // Vector to store detected stable colors
std::vector<int> times;
bool isDriving = true;
int timer = 0;

// Reverse & Turn Timing Variables
unsigned long reverseStartTime = 0;
unsigned long turnStartTime = 0;
unsigned long finalReverseStartTime = 0;

bool isDrivingInitially = true;
bool isReversing = false;
bool hasReversed = false;
bool isTurningLeft = false;
bool hasTurnedLeft = false;
bool isDrivingToRings = false;
bool hasCrossedRings = false;
bool isFinalReversing = false;
bool hasFinalReversed = false;
bool blackRingDetected = false;  // Track if a black ring has been detected
bool blackRingDetectionEnabled = false;  // Only enable black ring detection after turning left

int rings_to_cross = 0;
int stableColorsSinceTurn = 0;  // Track stable colors seen after turn


int offset = 1300;  // Set the final reverse time in milliseconds


void updateColorNonBlocking() {
    if (millis() - lastColorUpdate >= COLOR_UPDATE_INTERVAL) {
        lastColorUpdate = millis();  // Reset timer

        int redPW = getRedPW();
        int greenPW = getGreenPW();
        int bluePW = getBluePW();

        // Map pulse width to 0-255 range
        int redValue = mapColor(redPW, r_red, r_black);
        int greenValue = mapColor(greenPW, g_green, g_black);
        int blueValue = mapColor(bluePW, b_blue, b_black);

        // Apply EMA filtering
        avgRed = calculateEMA(redValue, avgRed, alpha);
        avgGreen = calculateEMA(greenValue, avgGreen, alpha);
        avgBlue = calculateEMA(blueValue, avgBlue, alpha);

        // Compute confidence scores
        float confBlack = calculateConfidence(avgRed, avgGreen, avgBlue, ref_black);
        float confBlue = calculateConfidence(avgRed, avgGreen, avgBlue, ref_blue);
        float confGreen = calculateConfidence(avgRed, avgGreen, avgBlue, ref_green);
        float confRed = calculateConfidence(avgRed, avgGreen, avgBlue, ref_red);

        // Determine the most confident color
        String detectedColor;
        float maxConfidence = max(confBlack, max(confBlue, max(confGreen, confRed)));

        if (maxConfidence == confBlack) detectedColor = "Black";
        else if (maxConfidence == confBlue) detectedColor = "Blue";
        else if (maxConfidence == confGreen) detectedColor = "Green";
        else detectedColor = "Red";

        // 🔹 Stabilization Logic 🔹
        if (detectedColor == potentialColor) {
            stabilityCounter++; // Increase stability counter if the color remains the same
        } else {
            potentialColor = detectedColor; // Reset potential color
            stabilityCounter = 1; // Reset stability count
        }

        // If the same color has been detected for STABILITY_THRESHOLD readings, confirm it
        if (stabilityCounter >= STABILITY_THRESHOLD && stableColor != potentialColor) {
            stableColor = potentialColor;  // Confirm new stable color

            colorsSeen.push_back(stableColor);
            times.push_back(millis());

            // **Check for black ring detection only AFTER turning left**
            if (blackRingDetectionEnabled && stableColor == "Black" && !blackRingDetected) {
                Serial.println("⚠️ Black Ring Detected AFTER Turn! Reversing Movement...");
                blackRingDetected = true;
                rings_to_cross = 5;  // Reset rings to cross to 5
                stableColorsSinceTurn = 0;  // Reset count
                drive("stop", 0);
                isDrivingToRings = false;
                driveUntilRingsCrossed();
            }

            // Track colors after turning to count rings crossed
            if (hasTurnedLeft && !hasCrossedRings) {
                stableColorsSinceTurn++;
                Serial.print("🎯 Rings Crossed: ");
                Serial.println(stableColorsSinceTurn);
            }

            Serial.print("🔹 New Stable Color Detected: ");
            Serial.println(stableColor);
        }
        // Serial.print("Detected Color: ");
        // Serial.print(detectedColor);
        // Serial.print(" | Confidence: ");
        // Serial.print(maxConfidence);
        // Serial.print("% | EMA RGB: R=");
        // Serial.print(avgRed);
        // Serial.print(", G=");
        // Serial.print(avgGreen);
        // Serial.print(", B=");
        // Serial.print(avgBlue);
        // Serial.print(" | Stability Count: ");
        // Serial.print(stabilityCounter);
        // Serial.print(" | Stable Color: ");
        // Serial.println(stableColor);
    }
}
// Function to Drive the Robot Forward
void driveStraight() {
    if (isDriving) {
        // Serial.println("🚗 Driving Forward...");
        drive("forward", MOTOR_POWER);  // Move at 50% speed
    } else {
        // Serial.println("🛑 Stopping!");
        drive("stop", 0);  // Stop the robot
    }
}



// Function to Check Stop Condition Based on Seen Colors
void checkStopCondition() {
    if (colorsSeen.size() >= 3) {
        if (colorsSeen.back() == colorsSeen[colorsSeen.size() - 3]) {
            Serial.println("🛑 Pattern Detected! Stopping Robot...");
            isDrivingInitially = false;
            timer = times[colorsSeen.size() - 1] - times[colorsSeen.size() - 3];
            Serial.print(timer);
            rings_to_cross = 7 - colorsSeen.size();
            isDriving = false;  // Stop moving
        }
    }
}

void printStoredColors() {
    Serial.print("Colors Seen: ");
    for (size_t i = 0; i < colorsSeen.size(); i++) {
        Serial.print(colorsSeen[i]);
        if (i < colorsSeen.size() - 1) Serial.print(", ");
    }
    Serial.println();
}


// **Modify drive logic to reverse movement when a black ring is detected AFTER TURN**
void drive(String direction, float speedFraction) {
    speedFraction = constrain(speedFraction, 0, 1);
    int speed = speedFraction * 255;

    // **Reverse all movements if black ring was detected after the turn**
    if (blackRingDetected) {
        if (direction == "forward") direction = "backward";
        else if (direction == "backward") direction = "forward";
        else if (direction == "left") direction = "right";
        else if (direction == "right") direction = "left";
    }

    if (direction == "forward") {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else if (direction == "backward") {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else if (direction == "left") {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else if (direction == "right") {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
        speed = 0;
    }

    int l_speed = speed * adj_const;
    int r_speed = speed;

    analogWrite(EN_A, l_speed);
    analogWrite(EN_B, r_speed);
}


void reverseForTime(unsigned long duration) {
    if (!isReversing) {
        Serial.println("🔄 Reversing...");
        drive("backward", MOTOR_POWER);  // Start reversing
        reverseStartTime = millis();  // Record reverse start time
        isReversing = true;
        //isDrivingInitially = false;
    }
    
    // Check if reverse duration has passed
    if (millis() - reverseStartTime >= duration) {
        Serial.println("🛑 Stopping Reverse...");
        drive("stop", 0);  // Stop after reverse duration
        isReversing = false;
        hasReversed = true;  // Ensure it only reverses once
    }
}

void turnLeftForTime(unsigned long duration) {
    if (!isTurningLeft) {
        Serial.println("↩️ Turning Left...");
        drive(turn_direction, MOTOR_POWER);  // Start turning left
        turnStartTime = millis();  // Record turn start time
        isTurningLeft = true;
    }

    // Check if turn duration has passed
    if (millis() - turnStartTime >= duration) {
        Serial.println("🛑 Stopping Turn...");
        drive("stop", 0);  // Stop turning
        isTurningLeft = false;
        hasTurnedLeft = true;  // Ensure it only turns once
    }
}

void driveUntilRingsCrossed() {
    if (!isDrivingToRings) {
        Serial.println(blackRingDetected ? "🚗 Driving in Reverse to Cross Rings..." : "🚗 Driving Forward to Cross Rings...");
        drive("forward", MOTOR_POWER);
        isDrivingToRings = true;
    }
    
    if (stableColorsSinceTurn >= rings_to_cross) {
        Serial.println("🏁 Crossed All Rings! Stopping...");
        drive("stop", 0);
        hasCrossedRings = true;
        isDrivingToRings = false;
    }
}


void finalReverse(unsigned long duration) {
    if (!isFinalReversing) {
        Serial.println("🔄 Final Reverse...");
        drive("backward", MOTOR_POWER);  // Start final reversing
        finalReverseStartTime = millis();  // Record start time
        isFinalReversing = true;
    }

    // Check if reverse duration has passed
    if (millis() - finalReverseStartTime >= duration) {
        Serial.println("🛑 Stopping Final Reverse...");
        drive("stop", 0);  // Stop after final reverse duration
        isFinalReversing = false;
        hasFinalReversed = true;  // Ensure it only reverses once
    }
}

unsigned long servoOpenStartTime = 0;  // Store the time when opening starts
bool isServoOpening = false;  // Track if the servo is in the process of opening

void loop() {
    updateColorNonBlocking();  // Update colors independently
    if (timer == 0) checkStopCondition();  // Check if we need to stop

    if (isDrivingInitially) {
        driveStraight();  // Drive forward at the start
    } else if (!hasReversed && timer > 0) {
        reverseForTime(timer / 1.5 + 100);  // Reverse for half the detected time
    } else if (hasReversed && !hasTurnedLeft) {
        turnLeftForTime(ninety_time);  // Turn left for 90-degree turn
    } else if (hasTurnedLeft && !blackRingDetectionEnabled) {
        blackRingDetectionEnabled = true;  // Enable black ring detection AFTER turn
    } else if (hasTurnedLeft && !hasCrossedRings) {
        driveUntilRingsCrossed();  // Drive forward until rings are crossed
    } else if (hasCrossedRings && !hasFinalReversed) {
        finalReverse(offset);  // Drive backward for offset time
    } 

    // **Non-blocking servo open logic**
    if (hasFinalReversed && !isServoOpening) {
        Serial.println("🔓 Opening Claw in 0.5 second...");
        servoOpenStartTime = millis();  // Store the start time
        isServoOpening = true;
    }

    // Wait 1 second before opening the servo
    if (isServoOpening && millis() - servoOpenStartTime >= 500) {
        Serial.println("🔓 Opening Claw Now!");
        myServo.writeMicroseconds(1000);
        isServoOpening = false;  // Ensure this only happens once
    }

    // Print colors every 5 seconds
    if (millis() % 5000 < 1) {
        printStoredColors();
    }
}