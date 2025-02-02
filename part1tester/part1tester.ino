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

const float motor_power = 0.5;
const float adj_const = 0.92;
const float speed_per_second = 40.72;

const float pre_calculated_distance = 72.5; // mm
float actual_distance = 0;

int processTracker = 1;

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
const int ref_black[] = {0, 0, 30};
const int ref_blue[] = {0, 150, 230};
const int ref_green[] = {0, 200, 120};
const int ref_red[] = {140, 130, 160};

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;

float avgRed = 0, avgGreen = 0, avgBlue = 0;
const float alpha = 0.33;  // Smoothing factor (adjustable for responsiveness)

String cur_color;
std::vector<String> colors;
bool movingForward = true, spinning = false;
unsigned long spinStartTime = 0;
const int SPIN_DURATION = 1500;

int time = 0, step = 1, colors_crossed = 0, start = 0, stop = 0;
bool reverse = false;
/* 

CLAW CONSTANTS 

*/
const int servoPin = 18;
const int OPEN_POS = 60;
const int CLOSE_POS = 0;
const int STEP_DELAY = 10;
const int STEP_SIZE = 5; 

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
  //colors.push_back("Black");
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

float calculateSMA(float newSample, float prevSMA) {
    static float buffer[5] = {0};  // Store last 5 samples
    static int index = 0;
    static bool filled = false;  // Tracks if buffer has enough samples

    buffer[index] = newSample;  // Store new sample
    index = (index + 1) % 5;    // Increment index, loop back after 5 samples

    // Compute the average only when buffer is filled
    float sum = 0;
    int count = filled ? 5 : index;
    for (int i = 0; i < count; i++) {
        sum += buffer[i];
    }

    if (index == 0) filled = true;  // Mark buffer as filled after 5 samples
    return sum / count;  // Return average
}

String getColor() {
    int redPW = getRedPW();
    int greenPW = getGreenPW();
    int bluePW = getBluePW();

    // Map pulse width to 0-255 range
    int redValue = mapColor(redPW, 250, 600);
    int greenValue = mapColor(greenPW, 550, 1000);
    int blueValue = mapColor(bluePW, 430, 980);

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

    // Print results
    // Serial.print("Detected Color: ");
    // Serial.print(detectedColor);
    // Serial.print(" | Confidence: ");
    // Serial.print(maxConfidence);
    // Serial.print("% | EMA RGB: R=");
    // Serial.print(avgRed);
    // Serial.print(", G=");
    // Serial.print(avgGreen);
    // Serial.print(", B=");
    // Serial.println(avgBlue);
    return detectedColor;
    // delay(200);
}

void loop() {
  Serial.print("Hello World");
}