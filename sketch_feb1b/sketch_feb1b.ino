#include <Wire.h>
#include <Servo.h>
Servo myServo;
// #include <everything_code.ino>

// Motor & Servo pins
#define MOTOR_LEFT 5
#define MOTOR_RIGHT 6
#define SERVO_PIN 9

// TCS230 Color Sensor pins
#define S2 8
#define S3 7
#define OUT 4

// Ultrasonic Sensor pins (for minor adjustments)
#define TRIG 10
#define ECHO 11

Servo flagServo;
const float pre_calculated_distance = 72.5; // mm
const float velocity = 100.0; // mm/s (Example value, should be calibrated)
int detected_colors = 0;
bool center_reached = false;
static bool color_detected = false;
static unsigned long start_time = 0;
unsigned long end_time = 0;
String prev_color = "";
String cur_color = "";
float actual_distance = 0;
int count = 0;

// Function to read RGB values from TCS230
String readColor() {
  return getColor();
}

// Function to calculate theta
float calculateTheta(float actual_distance) {
    return acos(pre_calculated_distance / actual_distance) * 180.0 / PI;
}

// // Function to detect first color
// void moveStart() {
// // drive
// if
//   while (readColor() == "Black"){
//     drive_dist("forward", 0.5);
    
//   }
//   prev_color = "Black";
//   cur_color = readColor();
// }

// Function to move straight
void moveDetect_Distance() {
  start_time = millis();
  while (readColor() == cur_color){
    analogWrite(MOTOR_LEFT, 150);
    analogWrite(MOTOR_RIGHT, 150);
  }
  end_time = millis();
  prev_color = cur_color;
  cur_color = readColor();
  
  float travel_time = (end_time - start_time) / 1000.0; // Convert ms to seconds
  actual_distance = velocity * travel_time;
}

// Function to adjust the angle of the jit
float angle_adjustment(float theta) {
  // FUNCTION TO TURN ROBOT THETA DEGREES RIGHT
    analogWrite(MOTOR_LEFT, 50);
    analogWrite(MOTOR_RIGHT, 50);

    if (readColor() == prev_color){
      analogWrite(MOTOR_LEFT, -50);   // move in reverse edit if needed
      analogWrite(MOTOR_RIGHT, -50);  // move in reverse edit if needed

      // FUNCTION TO TURN ROBOT 2 * THETA DEGREEES LEFT
    } else {
      analogWrite(MOTOR_LEFT, -50);   // move in reverse edit if needed
      analogWrite(MOTOR_RIGHT, -50);  // move in reverse edit if needed
    }
}

// Function to stop motors
void stopMotors() {
  analogWrite(MOTOR_LEFT, 0);
  analogWrite(MOTOR_RIGHT, 0);
}

void setup() {
  Serial.begin(9600);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  flagServo.attach(SERVO_PIN);
  flagServo.write(90); // Initial position

  // moveStart();

  cur_color = readColor();
}

void loop() {
  moveDetect_Distance();
  float theta = calculateTheta(actual_distance);
  angle_adjustment(theta);  // robot pointing in the right(ish) direction now
  count++;
  if (count == 4){
    // ADJUST ROBOT OFFSET
    openServo();
    stopMotors();
    while (1);
  }

  prev_color = cur_color;
  cur_color = readColor();

  drive("forward", 0.5);
  if (cur_color != prev_color){
    drive("stop", 0);
    while (1);
  }
}



//yiyi
// DRIVE CONSTANTS 

const int EN_A = 11; // Enable pin for first motor
const int IN1 = 9;   // Control pin for first motor
const int IN2 = 8;   // Control pin for first motor
const int IN3 = 7;   // Control pin for second motor
const int IN4 = 6;   // Control pin for second motor
const int EN_B = 10; // Enable pin for second motor

const float motor_power = 0.5;
const float adj_const = 0.92;
const float speed_per_second = 40.72;

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
const int ref_black[] = {5, 2, 1};
const int ref_blue[] = {30, 200, 250};
const int ref_green[] = {100, 250, 170};
const int ref_red[] = {250, 220, 205};

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
const int OPEN_POS = 60;
const int CLOSE_POS = 0;
const int STEP_DELAY = 10;
const int STEP_SIZE = 5; 

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
    Serial.print("Detected Color: ");
    Serial.println(detectedColor);
    // Serial.print(" | Confidence: ");
    // Serial.print(maxConfidence);
    // Serial.print("% | EMA RGB: R=");
    // Serial.print(avgRed);
    // Serial.print(", G=");
    // Serial.print(avgGreen);
    // Serial.print(", B=");
    // Serial.println(avgBlue);
    return detectedColor;

    delay(200);
}
void drive(String direction, float speedFraction) {
    speedFraction = constrain(speedFraction, 0, 1);
    int speed = speedFraction * 255;

    if (direction == "forward") {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else if (direction == "backward") {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else if (direction == "right") {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else if (direction == "left") {
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

void drive_dist(String direction, float distance) {
  // v = d/t
  float time = distance / speed_per_second * 1000; // time in ms
  drive(direction, motor_power);
  delay(time);
  drive(direction, 0);
}

void check_wall(float distance, String color) {
    if (distance <= 15) {
        if (color == "red") {
            u_turn();
        } else if (color == "blue") {
            turn_left();
        } else if (color == "green") {
            turn_right();
        }
    }
}
void turn_left() {
    // right motor forward
    // left motor back
    // until 90 deg
}
void turn_right() {
    // right motor back
    // left motor forward
    // until 90 deg
}
void u_turn() {
    // right motor back
    // left motor forward
    // until 180 deg
}