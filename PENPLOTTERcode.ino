#include <Wire.h>
#include <BluetoothSerial.h>
#include <SCMD.h>
#include "SCMD_config.h"
#include <ESP32Servo.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_i2cexp.h>

#define LEFT_MOTOR 1
#define RIGHT_MOTOR 0

#define ENCODER_LEFT_A 12
#define ENCODER_LEFT_B 13
#define ENCODER_RIGHT_A 17
#define ENCODER_RIGHT_B 16
#define HALL_EFFECT_SENSOR 19

#define LIMIT_SWITCH_LEFT 15
#define LIMIT_SWITCH_RIGHT 15
#define LIMIT_SWITCH_TOP 32
#define LIMIT_SWITCH_BOTTOM 32
#define EMERGENCY_STOP 14

#define SERVO_PIN 21
#define PEN_UP_ANGLE 120
#define PEN_DOWN_ANGLE 0

const int LED_RED = A0;
const int LED_BLUE = A1;
//#define LED_GREEN 33
SCMD qwiicMotorDriver;
BluetoothSerial SerialBT;
Servo penServo;

hd44780_I2Cexp lcd;

volatile long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;
volatile bool emergencyStopTriggered = false;
volatile long leftEncoderLastChangeTime = 0;
volatile long rightEncoderLastChangeTime = 0;
const long debounceDelay = 5; // 10 ms debounce delay

void IRAM_ATTR leftEncoderISR() {
  long currentTime = millis();
  if (currentTime - leftEncoderLastChangeTime > debounceDelay) {
    if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) {
      leftEncoderValue++;
    } else {
      leftEncoderValue--;
    }
    leftEncoderLastChangeTime = currentTime;
  }
}

void IRAM_ATTR rightEncoderISR() {
  long currentTime = millis();
  if (currentTime - rightEncoderLastChangeTime > debounceDelay) {
    if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B)) {
      rightEncoderValue--;
    } else {
      rightEncoderValue++;
    }
    rightEncoderLastChangeTime = currentTime;
  }
}

void IRAM_ATTR emergencyStopISR() {
  emergencyStopTriggered = true;
  Serial.println("Emergency stop triggered");
}

void configureMotorDriver() {
  qwiicMotorDriver.settings.commInterface = I2C_MODE;
  qwiicMotorDriver.settings.I2CAddress = 0x5D;
  qwiicMotorDriver.settings.chipSelectPin = 10;

  while (qwiicMotorDriver.begin() != 0xA9) {
    Serial.println("ID mismatch, trying again");
    delay(500);
  }
  Serial.println("ID matches 0xA9");

  Serial.print("Waiting for enumeration...");
  while (!qwiicMotorDriver.ready());
  Serial.println("Done.");

  configureMotors();
  qwiicMotorDriver.enable();
  Serial.println("Motor driver configured and enabled");
}

void configureMotors() {
  while (qwiicMotorDriver.busy());
  qwiicMotorDriver.inversionMode(1, 1); // Invert motor 1
  while (qwiicMotorDriver.busy());
  qwiicMotorDriver.enable(); // Enables the output driver hardware
  Serial.println("Motors configured");
}

void moveToHomePosition() {
  analogWrite(LED_RED, 0);
  Serial.println("Moving to home position");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moving to Home");
  bool leftAtHome = false;
  bool rightAtHome = false;
  penServo.write(PEN_UP_ANGLE);
  while (!leftAtHome || !rightAtHome) {
    if (emergencyStopTriggered) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
      return;
    }

    int leftLimitSwitchState = digitalRead(LIMIT_SWITCH_LEFT);
    int bottomLimitSwitchState = digitalRead(LIMIT_SWITCH_BOTTOM);
    
    Serial.print("Left Limit Switch: ");
    Serial.println(leftLimitSwitchState);
    Serial.print("Bottom Limit Switch: ");
    Serial.println(bottomLimitSwitchState);
    
    if (leftLimitSwitchState == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 100); // Move left motor backward
      Serial.println("Moving left motor backward to home position");
    } else {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 1, 0);
      leftAtHome = true;
      Serial.println("Left motor at home position");
    }
    
    if (bottomLimitSwitchState == HIGH) {
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 1, 100); // Move right motor downward
      Serial.println("Moving right motor downward to home position");
    } else {
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
      rightAtHome = true;
      Serial.println("Right motor at home position");
    }
    
    Serial.print("Left Encoder Value: ");
    Serial.println(leftEncoderValue);
    Serial.print("Right Encoder Value: ");
    Serial.println(rightEncoderValue);
    
    delay(100); // Add delay to reduce the frequency of serial output
  }
  
  qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
  qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);

  // Check Hall Effect sensor state
  if (digitalRead(HALL_EFFECT_SENSOR) == LOW) { // Assuming LOW indicates active
    analogWrite(LED_BLUE, 200);
    Serial.println("Hall Effect sensor active, LED turned on");
  } else {
    analogWrite(LED_BLUE, 0);
    Serial.println("Hall Effect sensor not active, LED turned off");
  }

  // Reset encoder values to zero after reaching home position
  leftEncoderValue = 0;
  rightEncoderValue = 0;
  Serial.println("Encoders reset to zero");
  
  Serial.println("Reached home position");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("At Home Position");
}

void drawHouse() {
  
  Serial.println("Drawing Nicholas' House");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Drawing House");
  // Reset encoder values to zero after reaching home position
  leftEncoderValue = 0;
  rightEncoderValue = 0;
  penServo.write(PEN_UP_ANGLE);
  long countsPerCm = 6; // Assuming 1cm corresponds to 40 encoder counts

  //Check for emergency stop in each move
  analogWrite(LED_BLUE, 0);
  analogWrite(LED_RED, 0);
  moveRelative(-countsPerCm * 70, countsPerCm * 50);
  delay(50);
  Serial.println("Move to initial position");

  penServo.write(PEN_DOWN_ANGLE);

  moveRelative(0, countsPerCm * 52); 
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
      
    }
  delay(50);
  Serial.println("Moved up 6cm");

  moveRelative(-countsPerCm * 60, -countsPerCm * 60);
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
      
    } 
  delay(50);
  Serial.println("Moved diagonally down");

  moveRelative(0, countsPerCm * 55);
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
      
    }
  delay(50); 
  Serial.println("Moved up");

  moveRelative(countsPerCm * 57, 0);
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
      
    }
  delay(50); 
  Serial.println("Moved left");
  moveRelative(-countsPerCm * 32, countsPerCm * 32);
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
      
    }
  delay(50);
  Serial.println("roof");
  moveRelative(-countsPerCm * 32, -countsPerCm * 32);
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
      
    }
  delay(50);
  Serial.println("roof");
  moveRelative(countsPerCm * 60, -countsPerCm * 60);
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
      
    }
  delay(50);
  Serial.println("Moved diagonally down");

  moveRelative(-countsPerCm * 62, 0);
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH || digitalRead(LIMIT_SWITCH_RIGHT) == HIGH || digitalRead(LIMIT_SWITCH_TOP) == HIGH || digitalRead(LIMIT_SWITCH_BOTTOM) == HIGH) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Limit switch pressed, stopping motors");
    
    }
  penServo.write(PEN_UP_ANGLE);
  
  Serial.println("FINISH DRAWING HOUSE");
}

void moveRelative(long xCounts, long yCounts) {
  long targetLeftEncoder = leftEncoderValue + xCounts;
  long targetRightEncoder = rightEncoderValue + yCounts;
  Serial.print("Moving relative to target - Left: ");
  Serial.print(targetLeftEncoder);
  Serial.print(", Right: ");
  Serial.println(targetRightEncoder);

  int tolerance = 3; // Allowable position error

  while (abs(leftEncoderValue - targetLeftEncoder) > tolerance || abs(rightEncoderValue - targetRightEncoder) > tolerance) {
    // Debug current encoder values
    Serial.print("Current Left Encoder Value: ");
    Serial.println(leftEncoderValue);
    Serial.print("Current Right Encoder Value: ");
    Serial.println(rightEncoderValue);
 
    if (emergencyStopTriggered) {
      analogWrite(LED_RED, 200);
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Emergency stop triggered, stopping motors");
      return; // Exit the function if emergency stop is triggered
    }

    if (leftEncoderValue < targetLeftEncoder - tolerance) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 75); // Move left motor forward
      Serial.println("Moving left motor forward");
    } else if (leftEncoderValue > targetLeftEncoder + tolerance) {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 1, 75); // Move left motor backward
      Serial.println("Moving left motor backward");
    } else {
      qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      Serial.println("Stopping left motor");
    }

    if (rightEncoderValue < targetRightEncoder - tolerance) {
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 122); // Move right motor upward
      Serial.println("Moving right motor upward");
    } else if (rightEncoderValue > targetRightEncoder + tolerance) {
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 1, 122); // Move right motor downward
      Serial.println("Moving right motor downward");
    } else {
      qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      Serial.println("Stopping right motor");
    }
    
    delay(10); // Small delay to stabilize control loop
  }

  // Ensure motors are stopped after reaching target
  qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
  qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
  Serial.println("Reached target position");
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Plotter");
  analogWrite(LED_RED, 0);
  analogWrite(LED_BLUE, 0);
  //digitalWrite(LED_GREEN, HIGH);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  //lcd.print("Initializing...");

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_TOP, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_BOTTOM, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP), emergencyStopISR, FALLING);

  Serial.println("Configuring motor driver...");
  configureMotorDriver();
  penServo.attach(SERVO_PIN);
  penServo.write(PEN_UP_ANGLE); // Initialize pen in the up position
  lcd.setCursor(0, 1);
  lcd.print("Ready");
}

void loop() {
  if (emergencyStopTriggered) {
    analogWrite(LED_RED, 200);
    qwiicMotorDriver.disable(); // Disable motors
    Serial.println("Motors disabled due to emergency stop");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EMERGENCY STOP");
    delay(100); 
    emergencyStopTriggered = false;
    qwiicMotorDriver.enable(); // Re-enable motors
    Serial.println("Motors re-enabled after emergency stop");
  }

  if (SerialBT.available()) {
    int command = SerialBT.read();
    switch (command) {
      case '2':
        Serial.println("Command received: Move to home position");
        moveToHomePosition();
        break;
      case '1':
        Serial.println("Command received: Draw house");
        drawHouse();
        break;
      case '0':
        ESP.restart();
        Serial.println("Command received: Emergency stop");
        break;
      case '3':
        penServo.write(PEN_UP_ANGLE);
        //statusMessage = "Pen Up";
        Serial.println("Pen Up");
        break;
      case '4':
        penServo.write(PEN_DOWN_ANGLE);
        //statusMessage = "Pen Down";
        Serial.println("Pen Down");
        break;
      default:
        Serial.print("Unknown command received: ");
        Serial.println(command);
        break;
    }
  }
  Serial.print("Left Encoder Value: ");
  Serial.println(leftEncoderValue);
  Serial.print("Right Encoder Value: ");
  Serial.println(rightEncoderValue);
  delay(500);
}
