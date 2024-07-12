  #include <Wire.h>
  #include <BluetoothSerial.h>
  #include <SCMD.h>
  #include "SCMD_config.h"
  #include <LiquidCrystal_I2C.h>
  #include <ESP32Servo.h>

  // Motor and Encoder Definitions
  #define LEFT_MOTOR 1
  #define RIGHT_MOTOR 0
  #define ENCODER_LEFT_A 12
  #define ENCODER_LEFT_B 13
  #define ENCODER_RIGHT_A 33
  #define ENCODER_RIGHT_B 27

  // Limit Switch and Emergency Stop Definitions
  #define LIMIT_SWITCH_LEFT 15
  #define LIMIT_SWITCH_RIGHT 15
  #define LIMIT_SWITCH_TOP 32
  #define LIMIT_SWITCH_BOTTOM 32
  #define EMERGENCY_STOP 14

  // Hall Effect Sensor and LED Definitions
  #define HALL_EFFECT_SENSOR 16
  #define LED_PIN 17

  // Servo Motor Definition
  #define SERVO_PIN 18
  #define PEN_UP_ANGLE 90
  #define PEN_DOWN_ANGLE 0

  SCMD qwiicMotorDriver;
  BluetoothSerial SerialBT;
  LiquidCrystal_I2C lcd(0x27, 16, 2);
  Servo penServo;

  volatile long leftEncoderValue = 0;
  volatile long rightEncoderValue = 0;
  volatile bool emergencyStopTriggered = false;
  volatile long leftEncoderLastChangeTime = 0;
  volatile long rightEncoderLastChangeTime = 0;
  const long debounceDelay = 10; // 10 ms debounce delay

  // Status message
  String statusMessage = "Idle";

  // ISR for Encoders
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
        rightEncoderValue++;
      } else {
        rightEncoderValue--;
      }
      rightEncoderLastChangeTime = currentTime;
    }
  }

  // Task for LCD display
  void lcdTask(void *pvParameters) {
    lcd.init();
    lcd.backlight();
    while (true) {
      lcd.clear();
      lcd.setCursor(0, 0); // Set cursor to the first line
      lcd.print(statusMessage.substring(0, 16)); // Display first 16 characters

      if (statusMessage.length() > 16) {
        lcd.setCursor(0, 1); // Set cursor to the second line
        lcd.print(statusMessage.substring(16)); // Display remaining characters
      }

      delay(1000); // Update every second
    }
  }

  // Task for Emergency Stop
  void emergencyStopTask(void *pvParameters) {
    pinMode(EMERGENCY_STOP, INPUT_PULLUP);
    while (true) {
      if (digitalRead(EMERGENCY_STOP) == LOW) { // Assuming LOW indicates the stop button is pressed
        emergencyStopTriggered = true;
        statusMessage = "Emergency Stop Triggered";
        qwiicMotorDriver.disable(); // Disable motors
        Serial.println("Motors disabled due to emergency stop");
        while (digitalRead(EMERGENCY_STOP) == LOW) { // Wait until the button is released
          delay(100);
        }
        emergencyStopTriggered = false;
        qwiicMotorDriver.enable(); // Re-enable motors
        Serial.println("Motors re-enabled after emergency stop");
      }
      delay(100);
    }
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
    statusMessage = "Moving to Home Position";
    Serial.println(statusMessage);
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
        qwiicMotorDriver.setDrive(LEFT_MOTOR, 1, 100); // Move left motor backward
        Serial.println("Moving left motor backward to home position");
      } else {
        qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
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
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Hall Effect sensor active, LED turned on");
    } else {
      digitalWrite(LED_PIN, LOW);
      Serial.println("Hall Effect sensor not active, LED turned off");
    }

    // Reset encoder values to zero after reaching home position
    leftEncoderValue = 0;
    rightEncoderValue = 0;
    statusMessage = "Pen at Home Position";
    Serial.println(statusMessage);
  }

  void drawHouse() {
    statusMessage = "Drawing Nicholas' House";
    Serial.println(statusMessage);
    penServo.write(PEN_UP_ANGLE);
    // Reset encoder values to zero after reaching home position
    leftEncoderValue = 0;
    rightEncoderValue = 0;
    long countsPerCm = 40; // Assuming 1cm corresponds to 40 encoder counts

    // Check for emergency stop in each move
    if (!moveRelative(countsPerCm * 2, countsPerCm * 2)) return; // Move right 2cm
    Serial.println("Moved right 2cm");
    penServo.write(PEN_DOWN_ANGLE);

    if (!moveRelative(countsPerCm * 5, 0)) return; // Move right 5cm
    Serial.println("Moved right 5cm");

    if (!moveRelative(0, countsPerCm * 5)) return; // Move up 5cm
    Serial.println("Moved up 5cm");

    if (!moveRelative(-countsPerCm * 5, 0)) return; // Move left 5cm
    Serial.println("Moved left 5cm");

    if (!moveRelative(countsPerCm * 5, -countsPerCm * 5)) return; // Move right and down 5cm
    Serial.println("Moved right and down 5cm");

    if (!moveRelative(0, -countsPerCm * 5)) return; // Move down 5cm
    Serial.println("Moved down 5cm");

    if (!moveRelative(-countsPerCm * 2, 0)) return; // Move left 2cm
    Serial.println("Moved left 2cm");

    penServo.write(PEN_UP_ANGLE);

    statusMessage = "Drawing Complete";
    Serial.println(statusMessage);
  }

  bool moveRelative(long targetLeftEncoder, long targetRightEncoder) {
    while (leftEncoderValue != targetLeftEncoder || rightEncoderValue != targetRightEncoder) {
      if (emergencyStopTriggered) {
        qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
        qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
        return false;
      }

      Serial.print("Current Left Encoder Value: ");
      Serial.print(leftEncoderValue);
      Serial.print(", Target Left Encoder Value: ");
      Serial.println(targetLeftEncoder);
      Serial.print("Current Right Encoder Value: ");
      Serial.print(rightEncoderValue);
      Serial.print(", Target Right Encoder Value: ");
      Serial.println(targetRightEncoder);

      if (leftEncoderValue < targetLeftEncoder) {
        qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 50); // Move left motor forward
      } else if (leftEncoderValue > targetLeftEncoder) {
        qwiicMotorDriver.setDrive(LEFT_MOTOR, 1, 50); // Move left motor backward
      } else {
        qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0); // Stop left motor
      }

      if (rightEncoderValue < targetRightEncoder) {
        qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 50); // Move right motor forward
      } else if (rightEncoderValue > targetRightEncoder) {
        qwiicMotorDriver.setDrive(RIGHT_MOTOR, 1, 50); // Move right motor backward
      } else {
        qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0); // Stop right motor
      }

      delay(100); // Add delay to reduce the frequency of serial output
    }

    qwiicMotorDriver.setDrive(LEFT_MOTOR, 0, 0);
    qwiicMotorDriver.setDrive(RIGHT_MOTOR, 0, 0);
    return true;
  }

  void setup() {
    Serial.begin(115200);
    SerialBT.begin("Nicholas_House_Drawing");

    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_TOP, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_BOTTOM, INPUT_PULLUP);
    pinMode(EMERGENCY_STOP, INPUT_PULLUP);
    pinMode(HALL_EFFECT_SENSOR, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);

    configureMotorDriver();

    penServo.attach(SERVO_PIN);
    penServo.write(PEN_UP_ANGLE); // Initialize pen in the up position

    // Create tasks for LCD and Emergency Stop
    xTaskCreatePinnedToCore(lcdTask, "LCD Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(emergencyStopTask, "Emergency Stop Task", 4096, NULL, 1, NULL, 0);

    Serial.println("Setup complete");
  }

  void loop() {
    if (SerialBT.available()) {
      char command = SerialBT.read();
      Serial.print("Received command: ");
      Serial.println(command);

      switch (command) {
        case '0':
          emergencyStopTriggered = true;
          Serial.println("Command received: Emergency stop");
          break;
        case '1':
          moveToHomePosition();
          break;
        case '2':
          drawHouse();
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
          Serial.println("Unknown command");
          break;
      }
    }

    delay(100);
  }
