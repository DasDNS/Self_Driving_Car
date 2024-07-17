#include <AFMotor.h>
#include <Servo.h>

//Pin definitions
#define ultrasonicTrigPin 20
#define ultrasonicEchoPin 21

int leftIRPin = 24;
int rightIRPin = 25;

AF_DCMotor leftMotor (1); //channel 1
AF_DCMotor rightMotor(2); //channel 2

int encoderPin = 7;
int encoderCount = 0; //counter for encoder pulses
float wheelCircumference = 20.0; //in centimeters
float speedConstant = 1000.0 / wheelCircumference; //Conversion factor (ms to seconds)

Servo Steeringservo;

void setup() {
  
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  Serial.begin(9600);
  Steeringservo.attach(28); //servo is attched to pin 28
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), updateEncoder, RISING);
  
}

void loop() {

  //Ultrasonic sensor - obstacle detection
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);

  long duration = pulseIn(ultrasonicEchoPin, HIGH);
  int distance = duration / 29.1; //calculate distance in centimeters

  if (distance < 4) { //stop if obstacle is too close
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    delay(100);
  } else {
    //Move forward
    goForward();
  }

  //IR sensor - lane detection
  int leftIRValue = digitalRead(leftIRPin);
  int rightIRValue = digitalRead(rightIRPin);

  //Adjusting motor control based on IR sensor values (lane detection)
  int threshold = 500;

  if (leftIRValue > threshold && rightIRValue > threshold) {
    //Both sensors detect white line
    goBackward;
    turnRight;
  } else if (leftIRValue > threshold) {
    //left sensor detects white line (turn right)
    turnRight();
  } else if (rightIRvalue > threshold) {
    //right sensor detects white line (turn left)
    turnLeft();
  } else {
    //no white line detected (go forward)
    goForward();
  }

  //Read encoder pulses
  //Calculate speed (in cm/s)
  float speed = encoderCount * speedConstant;
  Serial.print(" Speed: ");
  Serial.print(speed);
  Serial.println(" cm/s ");
  delay(1000);

}

void updateEncoder () {
  if (digitalRead(encoderPin) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void goForward () {
  leftMotor.setSpeed(150);
  rightMotor.setSpeed(150);
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}

void stop() {
  leftMotor.setSpeed(150);
  rightMotor.setSpeed(150);
  delay(100);
}

void turnRight() {
  //increase left motor speed
  leftMotor(200);
  rightMotor(150);
}

void turnLeft() {
  
}
