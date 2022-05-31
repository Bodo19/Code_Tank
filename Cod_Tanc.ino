#define LED_PIN 13
#define BUTTON_PIN 12
#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7
#include <Servo.h> // include servo library

Servo servo1; // define servos
Servo servo2;

const int swPin = 12;
int switchState = 1;

int joyX = 0; // give variable to joystick readings
int joyY = 1;

int joyValX;
int joyValY;
int pos_Y;

int difference_X = 0;
int difference_Y = 0;
byte lastButtonState = HIGH;
byte ledState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

int motorSpeedA = 0;
int motorSpeedB = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(swPin, INPUT);
  digitalWrite(swPin, HIGH);
  
  servo1.attach(2); // start servos
  servo2.attach(3);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  turela();
  
  if (millis() - lastTimeButtonStateChanged > debounceDuration) {
    byte buttonState = digitalRead(swPin);
    if (buttonState != lastButtonState) {
      lastTimeButtonStateChanged = millis();
      lastButtonState = buttonState;
      if (buttonState == LOW) {
      // turn LED on:
        digitalWrite(LED_PIN, HIGH);
      } else {
      // turn LED off:
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
  motoare();
}

void motoare() {
  int xAxis = analogRead(A5); // Read Joysticks X-axis
  int yAxis = analogRead(A4); // Read Joysticks Y-axis

  // Y-axis used for forward and backward control
  if (xAxis < 470) {
    // Set Motor A backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the declining X-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(xAxis, 470, 0, 0, 255);
    motorSpeedB = map(xAxis, 470, 0, 0, 255);
  }
  else if (xAxis > 550) {
    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the increasing X-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(xAxis, 550, 1023, 0, 255);
    motorSpeedB = map(xAxis, 550, 1023, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }

  // X-axis used for left and right control
  if (yAxis < 470) {
    // Convert the declining Y-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(yAxis, 470, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
  }
  if (yAxis > 550) {
    // Convert the increasing Y-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(yAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}


void turela(){
  joyValY = analogRead(A1); // read value from joystick
  joyValY = map(joyValY, 0, 1023, 0, 180); // change value range to 0-180
  move_motor_Y();
  
  joyValX = analogRead(A0); // repeat same for x axis
  joyValX = map(joyValX, 0, 1023, 0, 180);
  move_motor_X();  
}
void move_motor_X(){
  difference_X = (joyValX - 90) / 4 + 90;
  servo1.write(difference_X);
  delay(15);
  servo1.write(90);
}
void move_motor_Y(){
  difference_Y = (joyValY - 90) / 30;
  pos_Y = pos_Y + difference_Y;
  servo2.write(pos_Y);
}
