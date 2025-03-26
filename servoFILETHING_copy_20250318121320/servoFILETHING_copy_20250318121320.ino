#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Using the default address 0x40 for the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo constants
#define SERVO_FREQ 50    // Analog servos run at ~50 Hz updates
#define SG90MIN 150      // Minimum pulse length for SG90 servos
#define SG90MAX 500      // Maximum pulse length for SG90 servos

// Define servo pins
#define SERVO_X 0        // X-axis servo channel
#define SERVO_Y 1        // Y-axis servo channel
#define SERVO_Z 2        // Z-axis servo channel

// Variables for face coordinates
int faceX = 0;
int faceY = 0;
int faceZ = 0;
bool faceDetected = false;

// Buffer for incoming data
const int BUFFER_SIZE = 16;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Debug buffer
char debugBuf[64];

// LED pins for debugging
const int ledPin = 13;
const int ledX = 5;  // LED for X coordinate
const int ledY = 6;  // LED for Y coordinate
const int ledZ = 7;  // LED for Z coordinate

void setup() {
  // Initialize main serial communication at 9600 baud
  Serial.begin(9600);
 
  // Set up the LED pins
  pinMode(ledPin, OUTPUT);
  pinMode(ledX, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(ledZ, OUTPUT);
 
  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
 
  // Center the servos initially
  moveServo(SERVO_X, 325); // Center X
  moveServo(SERVO_Y, 325); // Center Y
  moveServo(SERVO_Z, 325); // Center Z
 
  // Visual indication that system is ready
  for(int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
 
  delay(1000);
}

void loop() {
  // Check if data is available from serial
  while (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();
   
    // Blink LED to indicate data received
    digitalWrite(ledPin, HIGH);
    delay(10);
    digitalWrite(ledPin, LOW);
   
    // Process the received data
    processIncomingByte(incomingByte);
  }
 
  // If face coordinates are received, move servos accordingly
  if (faceDetected) {
    // Visual indication of face coordinates using LEDs
    // The brightness of each LED corresponds to the coordinate value
    analogWrite(ledX, map(faceX, 0, 999, 0, 255));
    analogWrite(ledY, map(faceY, 0, 999, 0, 255));
    analogWrite(ledZ, map(faceZ, 0, 999, 0, 255));
   
    // Map face coordinates to servo positions
    int servoX = mapCoordToServo(faceX);
    int servoY = mapCoordToServo(faceY);
    int servoZ = mapCoordToServo(faceZ);
   
    // Move servos to track the face
    moveServo(SERVO_X, servoX);
    moveServo(SERVO_Y, servoY);
    moveServo(SERVO_Z, servoZ);
   
    // Reset the face detected flag
    faceDetected = false;
  }
 
  delay(10); // Small delay to prevent overwhelming the serial buffer
}

// Process each incoming byte
void processIncomingByte(char inByte) {
  // Check if this is the start of coordinates
  if (inByte == '(') {
    bufferIndex = 0;
    buffer[bufferIndex++] = inByte;
  }
  // Add the incoming byte to the buffer
  else if (bufferIndex < BUFFER_SIZE - 1 && bufferIndex > 0) {
    buffer[bufferIndex++] = inByte;
   
    // Check if we have a complete message (ending with ')')
    if (inByte == ')') {
      // Null-terminate the string
      buffer[bufferIndex] = '\0';
     
      // Parse the coordinates
      parseCoordinates(buffer);
     
      // Reset the buffer index for the next message
      bufferIndex = 0;
    }
  }
}

// Parse the received coordinates in format "(x,y,z)"
void parseCoordinates(char* data) {
  // Check if the data starts with '(' and ends with ')'
  if (data[0] == '(' && data[bufferIndex-1] == ')') {
    // Format should be "(xxx,yyy,zzz)"
    char xStr[4] = {data[1], data[2], data[3], '\0'};
    char yStr[4] = {data[5], data[6], data[7], '\0'};
    char zStr[4] = {data[9], data[10], data[11], '\0'};
   
    // Convert strings to integers
    faceX = atoi(xStr);
    faceY = atoi(yStr);
    faceZ = atoi(zStr);
    faceDetected = true;
  }
}

// Map face coordinates to servo positions
int mapCoordToServo(int position) {
  // Map the position from 0-999 to servo range
  return map(position, 0, 999, SG90MIN, SG90MAX);
}

// Move a servo to a specific position
void moveServo(uint8_t servoNum, uint16_t position) {
  // Ensure position is within bounds
  position = constrain(position, SG90MIN, SG90MAX);
 
  // Set the servo position
  pwm.setPWM(servoNum, 0, position);
}

// If you want to add printf-like debugging later,
// you can connect a second Arduino and use this function:
/*
void debugPrint(const char* format, ...) {
  // This function is commented out but can be implemented
  // if you add a second Arduino for debugging
}
*/