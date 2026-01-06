//Here is the code for Smart Dustbin Robot using Bluetooth Control 

#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// Bluetooth setup
BluetoothSerial SerialBT;

// Motor driver pins
#define MOTOR_A_IN1 12
#define MOTOR_A_IN2 14
#define MOTOR_B_IN3 27
#define MOTOR_B_IN4 26
#define MOTOR_ENABLE_A 25
#define MOTOR_ENABLE_B 33
int motorspeed2=150;
int motorspeed=90;
unsigned long lastCommandTime = 0;   // track last Bluetooth command time
const unsigned long commandTimeout = 200; // 1 second timeout


// Ultrasonic sensor pins
#define TRIG_PIN 5
#define ECHO_PIN 18

// IR sensor pin
#define IR_SENSOR_PIN 13

// Servo motor pin
#define SERVO_PIN 19

// ISD1820 pins
#define ISD_REC_PIN 4   // Not used in playback mode
#define ISD_PLAY_PIN 2  // Trigger playback
#define ISD_VCC_PIN 15  // Control power to save battery

// Variables
Servo myServo;
long duration;
int distance;
bool obstacleDetected = false;
bool objectDetected = false;
unsigned long lastAudioPlay = 0;
unsigned long servoOpenTime = 0;
bool servoActive = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize Bluetooth
  SerialBT.begin("ESP32_Robot"); // Bluetooth device name
  
  // Motor pins setup
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  
  // Ultrasonic sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // IR sensor setup
  pinMode(IR_SENSOR_PIN, INPUT);
  
  // Servo setup
  myServo.attach(SERVO_PIN);
  myServo.write(90); // Start with servo closed
  
  // ISD1820 setup
  pinMode(ISD_PLAY_PIN, OUTPUT);
  pinMode(ISD_VCC_PIN, OUTPUT);
  digitalWrite(ISD_PLAY_PIN, LOW);
  digitalWrite(ISD_VCC_PIN, HIGH); // Power on the module
  digitalWrite(ISD_PLAY_PIN, HIGH);
  delay(4000);
  digitalWrite(ISD_PLAY_PIN, LOW);
  
  Serial.println("ESP32 Robot Controller Ready!");
}

void loop() {
  // Sensors & actions
  checkObstacle();
  readIRSensor();
  handleServo();
   
  if (millis() - lastAudioPlay >= 20000) {
  playAudioPeriodically();
  lastAudioPlay = millis();
  }
  
  // Process Bluetooth commands
  if (!obstacleDetected) {
    processBluetoothCommands();
  } else {
    stopMotors();
  }

  // If no command for > 1s, stop motors
  if (millis() - lastCommandTime > commandTimeout) {
    stopMotors();
  }

  delay(50);
}


void checkObstacle() {
  // Clear the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  distance = duration * 0.034 / 2;
  
  // Check if obstacle is closer than 5cm
  if (distance < 5) {
    obstacleDetected = true;
    Serial.println("Obstacle detected! Stopping motors.");
  } else {
    obstacleDetected = false;
  }
}

void readIRSensor() {
  int irValue = digitalRead(IR_SENSOR_PIN);
  
  if (irValue == LOW) { // Assuming LOW means object detected
    if (!objectDetected) {
      objectDetected = true;
      servoOpenTime = millis();
      Serial.println("Object detected by IR sensor");
    }
  } else {
    objectDetected = false;
  }
}

void handleServo() {
  if (objectDetected && !servoActive) {
    // Open servo
    myServo.write(20);
    servoActive = true;
    Serial.println("Servo opened");
  }
  
  if (servoActive && (millis() - servoOpenTime >= 5000)) {
    // Close servo after 5 seconds
    myServo.write(90);
    servoActive = false;
    Serial.println("Servo closed");
  }
}

void playAudioPeriodically() {
 
    digitalWrite(ISD_PLAY_PIN, HIGH);
    delay(4000); // Short pulse to trigger playback
    digitalWrite(ISD_PLAY_PIN, LOW);
    Serial.println("Audio played");
 
 }

void processBluetoothCommands() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("Received command: ");
    Serial.println(command);

    lastCommandTime = millis(); // update on any command

    switch(command) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopMotors(); break;
      default: stopMotors(); break;
    }
  }
}


void moveForward() {
  analogWrite(MOTOR_ENABLE_A,motorspeed);
  analogWrite(MOTOR_ENABLE_B,motorspeed);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, HIGH);
  digitalWrite(MOTOR_B_IN4, LOW);
  Serial.println("Moving forward");
}

void moveBackward() {
  analogWrite(MOTOR_ENABLE_A,motorspeed);
  analogWrite(MOTOR_ENABLE_B,motorspeed);
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2,LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
  Serial.println("Moving backward");
}

void turnLeft() {
  analogWrite(MOTOR_ENABLE_A,motorspeed2);
  analogWrite(MOTOR_ENABLE_B,motorspeed2);
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH);
  digitalWrite(MOTOR_B_IN4, LOW);
  Serial.println("Turning left");
}

void turnRight() {
  analogWrite(MOTOR_ENABLE_A,motorspeed2);
  analogWrite(MOTOR_ENABLE_B,motorspeed2);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
  Serial.println("Turning right");
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
  Serial.println("Motors stopped");
}
