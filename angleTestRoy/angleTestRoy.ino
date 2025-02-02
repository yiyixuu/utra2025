#include <Wire.h>
#include <Servo.h>
Servo myServo;

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

void setup() {
  Serial.begin(9600);
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN_B, OUTPUT);
}
const int COLOR_UPDATE_INTERVAL = 100;
unsigned long last_color_update = 0;

void updateColorBackground() {
  if (millis() >= last_color_update + COLOR_UPDATE_INTERVAL) {
    last_color_update = millis();
    cur_color = getColor(); 
  }
}
void loop() {
  if ()
  while(1);
}