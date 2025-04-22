#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Constants
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SG90MIN 0
#define SG90MAX 500

// Constants for the arm dimensions in cm
#define BICEP_LENGTH 15.0     // Length of the bicep segment (15 cm)
#define FOREARM_LENGTH 18.0   // Length of the forearm segment (18 cm)

// Position of the arm base (capstan drive) relative to the camera's coordinate system
#define ARM_BASE_X 0.0  // X coordinate of arm base in camera space (adjust as needed)
#define ARM_BASE_Y 0.0  // Y coordinate of arm base in camera space (adjust as needed)
#define ARM_BASE_Z 0.0  // Z coordinate of arm base in camera space (adjust as needed)

#define Wrist 0
#define Elbow 1
#define BicepSwivel 2
#define CapstanShoulder 3

// External variables declarations
extern Adafruit_PWMServoDriver pwm;
extern uint8_t servonum;
extern int rev;
extern int x, y, z;
extern uint8_t xarray[], yarray[], zarray[];
extern int FACE_DETECTION_FLAG;
extern int xidx, yidx, zidx;
extern char xval, yval, zval;
extern int cartesiancount;
extern float gear1, gear2, gear3, gear4;
extern int testflag;
extern char buffer[];
extern int bufferIndex;
extern bool newDataAvailable;

// Function prototypes
void DriveEachServoOneAtATime();
void SetMinPositions();
void ONEWAYWristDegrees(int Degrees);
void WristDegrees(int Degrees);
void ONEWAYBicepSwivelDegrees(int Degrees);
void ONEWAYBicepSwivelDegreesBACK(int final, int initial);
void BicepSwivelDegrees(int Degrees);
void INITIALFINALBicepSwivelDegrees(int Degrees1, int Degrees2);
void ONEWAYElbowDegrees(int Degrees);
void ONEWAYElbowDegreesBACK(int final, int inital);
void ElbowDegrees(int Degrees);
void INITIALFINALElbowDegrees(int Degrees1, int Degrees2);
void CapstanShoulderDegrees(int Degrees);
void readFaceCoordinates();
void parseCoordinates();
void moveServosToFace();
void setServoPulse(uint8_t n, double pulse);
// Add these function prototypes
void ONEWAYWristDegreesBACK(int final, int initial);
void INITIALFINALWristDegrees(int Degrees1, int Degrees2);
void ONEWAYCapstanShoulderDegrees(int Degrees);
void ONEWAYCapstanShoulderDegreesBACK(int final, int initial);
void INITIALFINALCapstanShoulderDegrees(int Degrees1, int Degrees2);

// Implementation of the functions
inline void DriveEachServoOneAtATime()
{
  while(1)
  {
    // Drive each servo one at a time using setPWM()
    if(servonum==3)
    {
      continue;
    }
    
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) 
    {
      pwm.setPWM(servonum, 0, pulselen); // command that moves the servo
      delay(10);
    }

      delay(500);
      for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) 
      {
        pwm.setPWM(servonum, 0, pulselen);
        delay(10);
      }

      delay(500);

      servonum++;
      
      if (servonum > 7) 
      {
        servonum = 0; // Testing the first 8 servo channels
        break; //if you want it to exit the function instead of carrying on forever
      }
  }
}

inline void SetMinPositions()
{
  pwm.setPWM(0, 0, SG90MIN);
  int WristNewMin = map(90, 0, 200, SG90MIN, SG90MAX);
  pwm.setPWM(0, 0, WristNewMin);
  for (uint16_t pulselen = SG90MAX; pulselen > WristNewMin; pulselen--) 
  {
    pwm.setPWM(0, 0, pulselen);
    delay(20);
  }
  pwm.setPWM(0, 0, WristNewMin);

  pwm.setPWM(1, 0, SERVOMIN);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(25);
  }
  pwm.setPWM(1, 0, SERVOMIN);

  //This will work the other way around. "SERVOMAX" is closer to BicepNewMin, and SERVOMIN is analagous to the max of this servo.
  int BicepNewMin = map(90, 0, 200, SERVOMIN, SERVOMAX);
  pwm.setPWM(2, 0, BicepNewMin);
  for (uint16_t pulselen = SERVOMIN; pulselen < BicepNewMin; pulselen++) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(50);
  }
  pwm.setPWM(2, 0, BicepNewMin);
}

inline void Kill()
{
  delay(500);
  
  // BicepSwivel - Move from 90 degrees position to min position
  int BicepNewMin = map(90, 0, 200, SERVOMIN, SERVOMAX);
  pwm.setPWM(BicepSwivel, 0, BicepNewMin);
  for (uint16_t pulselen = BicepNewMin; pulselen > 190; pulselen--) 
  {
    pwm.setPWM(BicepSwivel, 0, pulselen);
    delay(10);
  }
  pwm.setPWM(BicepSwivel, 0, 190);
  
  delay(500);
  
  // Wrist - Move from 90 degrees position to max position
  int WristNewMin = map(90, 0, 200, SG90MIN, SG90MAX);
  pwm.setPWM(Wrist, 0, WristNewMin);
  for (uint16_t pulselen = WristNewMin; pulselen < SG90MAX; pulselen++) 
  {
    pwm.setPWM(Wrist, 0, pulselen);
    delay(10);
  }
  pwm.setPWM(Wrist, 0, SG90MAX);
  
  delay(500);
  
  // Elbow - Move from min position to max position
  pwm.setPWM(Elbow, 0, SERVOMIN);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) 
  {
    pwm.setPWM(Elbow, 0, pulselen);
    delay(10);
  }
  pwm.setPWM(Elbow, 0, SERVOMAX);
  
  delay(500);
}


inline void ONEWAYWristDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SG90MIN, SG90MAX);
  for (uint16_t pulselen = SG90MIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(0, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

inline void WristDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SG90MIN, SG90MAX);
  for (uint16_t pulselen = SG90MIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(0, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);

  for (uint16_t pulselen = Pos; pulselen > SG90MIN; pulselen--) 
  {
    pwm.setPWM(0, 0, pulselen);
    delay(10);
  }
}

inline void ONEWAYBicepSwivelDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

inline void ONEWAYBicepSwivelDegreesBACK(int final, int initial)
{
  int Pos = map(final, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = initial; pulselen > Pos; pulselen--) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

inline void BicepSwivelDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);

  for (uint16_t pulselen = Pos; pulselen > SERVOMIN; pulselen--) 
  {
    pwm.setPWM(2, 0, pulselen);
    delay(10);
  }
}

inline void INITIALFINALBicepSwivelDegrees(int Degrees1, int Degrees2)
{
  int Pos1 = map(Degrees1, 0, 200, SERVOMIN, SERVOMAX);
  int Pos2 = map(Degrees2, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = Pos1; pulselen < Pos2; pulselen++) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);

  for (uint16_t pulselen = Pos2; pulselen > Pos1; pulselen--) 
  {
    pwm.setPWM(2, 0, pulselen);
    delay(10);
  }
}

inline void ONEWAYElbowDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

inline void ONEWAYElbowDegreesBACK(int final, int inital)
{
  int Pos = map(final, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = inital; pulselen > Pos; pulselen--) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

inline void ElbowDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);

  for (uint16_t pulselen = Pos; pulselen > SERVOMIN; pulselen--) 
  {
    pwm.setPWM(1, 0, pulselen);
    delay(10);
  }
}

inline void INITIALFINALElbowDegrees(int Degrees1, int Degrees2)
{
  int Pos1 = map(Degrees1, 0, 200, SERVOMIN, SERVOMAX);
  int Pos2 = map(Degrees2, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = Pos1; pulselen < Pos2; pulselen++) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);

  for (uint16_t pulselen = Pos2; pulselen > Pos1; pulselen--) 
  {
    pwm.setPWM(1, 0, pulselen);
    delay(10);
  }
}

inline void readFaceCoordinates() 
{
  while (Serial.available() > 0) 
  {
    char inChar = Serial.read();
   
    // Check if this is the start of a coordinate packet
    if (inChar == '(') 
    {
      bufferIndex = 0;
      buffer[bufferIndex++] = inChar;
    }
    // Add character to buffer
    else if (bufferIndex > 0 && bufferIndex < 15) 
    {
      buffer[bufferIndex++] = inChar;
     
      // Check if we have complete coordinates
      if (inChar == ')') 
      {
        buffer[bufferIndex] = '\0'; // Null terminate
        parseCoordinates();
        newDataAvailable = true;
        
        // Blink LED to show data received
        digitalWrite(2, HIGH);
        delay(10);
        digitalWrite(2, LOW);
        
        break;
      }
    }
  }
}

inline void parseCoordinates() 
{
  // Check if the data format is correct
  if (buffer[0] == '(' && buffer[bufferIndex-1] == ')') 
  {
    // Reset indices
    xidx = yidx = zidx = 0;
    
    // Initialize parsing state (0=parsing x, 1=parsing y, 2=parsing z)
    int parseState = 0;
    bool isNegative[3] = {false, false, false}; // Track negative sign for each coordinate
    
    // Go through the buffer character by character
    for (int i = 1; i < bufferIndex-1; i++) 
    {
      // If we find a comma, move to the next coordinate
      if (buffer[i] == ',') 
      {
        parseState++;
        continue;
      }
      
      // Check for negative sign
      if (buffer[i] == '-' && ((parseState == 0 && xidx == 0) || 
                               (parseState == 1 && yidx == 0) || 
                               (parseState == 2 && zidx == 0))) 
      {
        isNegative[parseState] = true;
        continue;
      }
      
      // If it's a digit, add it to the appropriate array
      if (buffer[i] >= '0' && buffer[i] <= '9') 
      {
        switch (parseState) 
        {
          case 0: // X coordinate
            if (xidx < 2) // Prevent buffer overflow (max index is 2 for a 3-element array)
              xarray[xidx++] = buffer[i];
            break;
          case 1: // Y coordinate
            if (yidx < 2) // Prevent buffer overflow
              yarray[yidx++] = buffer[i];
            break;
          case 2: // Z coordinate
            if (zidx < 2) // Prevent buffer overflow
              zarray[zidx++] = buffer[i];
            break;
        }
      }
    }
    
    // Null-terminate the coordinate strings
    xarray[xidx] = '\0';
    yarray[yidx] = '\0';
    zarray[zidx] = '\0';
    
    // Only convert to integers if we have valid data in each coordinate
    if (xidx > 0 && yidx > 0 && zidx > 0) 
    {
      // Convert string arrays to integers
      x = atoi((char*)xarray);
      y = atoi((char*)yarray);
      z = atoi((char*)zarray);
      
      // Apply negative sign if needed
      if (isNegative[0]) x = -x;
      if (isNegative[1]) y = -y;
      if (isNegative[2]) z = -z;
      
      // Set flag to indicate valid face detection data
      FACE_DETECTION_FLAG = 1;
      
      // Debug output
      Serial.print("Parsed coordinates: x=");
      Serial.print(x);
      Serial.print(", y=");
      Serial.print(y);
      Serial.print(", z=");
      Serial.println(z);
    } 
    else 
    {
      Serial.println("Invalid coordinate format");
    }
  }
  else 
  {
    Serial.println("Invalid packet format");
  }
}

inline void moveServosToFace() 
{
  // Convert the input coordinates to real-world coordinates in cm
  // Handling the -1000 to +1000 range
  float worldX = map(x, -1000, 1000, -1000, 1000);  // Map x to a range of -1000 to +1000 cm
  float worldY = map(y, -1000, 1000, -1000, 1000);  // Map y to a range of -1000 to +1000 cm
  float worldZ = map(z, -1000, 1000, -1000, 1000);  // Map z to a range of -1000 to +1000 cm
  
  // Debug output of raw coordinates
  Serial.print("Target coordinates: (");
  Serial.print(worldX);
  Serial.print(", ");
  Serial.print(worldY);
  Serial.print(", ");
  Serial.print(worldZ);
  Serial.println(") cm");
  
  // Adjust coordinates relative to arm base (capstan drive position)
  float targetX = worldX - ARM_BASE_X;
  float targetY = worldY - ARM_BASE_Y;
  float targetZ = worldZ - ARM_BASE_Z;
  
  // Calculate bicep swivel angle (horizontal rotation, like azimuth)
  // This is the angle in the XZ plane from the positive X axis
  float bicepSwivelAngle = atan2(targetZ, targetX) * 180.0 / PI;
  
  // Adjust to match your servo's orientation - you may need to modify this offset
  bicepSwivelAngle = bicepSwivelAngle + 90; // Add 90 degrees so 0 is forward
  
  // Ensure angle is in 0-180 range for servo
  while (bicepSwivelAngle < 0) bicepSwivelAngle += 360;
  while (bicepSwivelAngle > 360) bicepSwivelAngle -= 360;
  
  // Clamp to servo range
  if (bicepSwivelAngle > 180) bicepSwivelAngle = 180;
  if (bicepSwivelAngle < 0) bicepSwivelAngle = 0;
  
  // Calculate the horizontal distance to the target
  float horizontalDistance = sqrt(targetX*targetX + targetZ*targetZ);
  
  // Calculate the elevation angle (vertical angle)
  // This is the angle in the vertical plane from the horizontal
  float elevationAngle = atan2(targetY, horizontalDistance) * 180.0 / PI;
  
  // Map elevation angle to elbow angle based on arm geometry
  // Assuming 90° elbow angle gives horizontal aim
  float elbowAngle = 90 - elevationAngle;
  
  // Adjust elbow angle into valid servo range (0-180)
  if (elbowAngle < 0) elbowAngle = 0;
  if (elbowAngle > 180) elbowAngle = 180;
  
  // For the wrist, use a fixed angle or adjust based on target distance if needed
  float wristAngle = 90; // Adjust based on your needs
  
  // Debug output of calculated angles
  Serial.print("Bicep Swivel Angle: ");
  Serial.print(bicepSwivelAngle);
  Serial.println(" degrees");
  
  Serial.print("Elevation Angle: ");
  Serial.print(elevationAngle);
  Serial.println(" degrees");
  
  Serial.print("Elbow Angle: ");
  Serial.print(elbowAngle);
  Serial.println(" degrees");
  
  // Call the servo movement functions to position the arm
  ONEWAYBicepSwivelDegrees(bicepSwivelAngle);
  ONEWAYElbowDegrees(elbowAngle);
  ONEWAYWristDegrees(wristAngle);
  
  // Reset the flags
  FACE_DETECTION_FLAG = 0;
  newDataAvailable = false;
}

inline void setServoPulse(uint8_t n, double pulse) 
{
  double pulselength;
 
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

// Additional wrist functions
inline void ONEWAYWristDegreesBACK(int final, int initial)
{
  int Pos = map(final, 0, 200, SG90MIN, SG90MAX);
  for (uint16_t pulselen = initial; pulselen > Pos; pulselen--) 
  {
    pwm.setPWM(0, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

inline void INITIALFINALWristDegrees(int Degrees1, int Degrees2)
{
  int Pos1 = map(Degrees1, 0, 200, SG90MIN, SG90MAX);
  int Pos2 = map(Degrees2, 0, 200, SG90MIN, SG90MAX);
  for (uint16_t pulselen = Pos1; pulselen < Pos2; pulselen++) 
  {
    pwm.setPWM(0, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);

  for (uint16_t pulselen = Pos2; pulselen > Pos1; pulselen--) 
  {
    pwm.setPWM(0, 0, pulselen);
    delay(10);
  }
}

// Additional capstan functions
inline void ONEWAYCapstanShoulderDegrees(int Degrees)
{
  // Calculate how many full rotations we need
  int fullRotations = Degrees / 360;
  int remainingDegrees = Degrees % 360;
  
  // Map the remaining degrees to servo pulse values
  int finalPos = map(remainingDegrees, 0, 180, SERVOMIN, SERVOMAX);
  
  // Perform full 360-degree rotations first
  for (int rotation = 0; rotation < fullRotations; rotation++) {
    // One full rotation: 0 -> 180 -> 0 -> 180 -> 0
    // First half rotation (0 to 180)
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
    
    // Second half rotation (180 to 360, represented as 180 back to 0)
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  
  // Handle any remaining degrees (less than 360)
  if (remainingDegrees > 0) {
    for (uint16_t pulselen = SERVOMIN; pulselen < finalPos; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
}

inline void ONEWAYCapstanShoulderDegreesBACK(int final, int initial)
{
  // For the backward rotation, we need to compute positions differently
  int startPos = map(initial % 180, 0, 180, SERVOMIN, SERVOMAX);
  int endPos = map(final % 180, 0, 180, SERVOMIN, SERVOMAX);
  
  // Calculate full rotations needed
  int degreesDiff = initial - final;
  int fullRotations = degreesDiff / 360;
  int remainingDegrees = degreesDiff % 360;
  
  // First handle remaining degrees if starting position is higher than ending
  if (startPos > endPos) {
    for (uint16_t pulselen = startPos; pulselen > endPos; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  // If starting position is lower, we need to go back to SERVOMIN and then from SERVOMAX down
  else if (startPos < endPos) {
    // Go from start to SERVOMIN
    for (uint16_t pulselen = startPos; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
    // Go from SERVOMAX to end
    for (uint16_t pulselen = SERVOMAX; pulselen > endPos; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  
  // Perform full 360-degree rotations
  for (int rotation = 0; rotation < fullRotations; rotation++) {
    // One full rotation backward: 180 -> 0 -> 180 -> 0
    // First half rotation (180 to 0)
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
    
    // Second half rotation (0 to 180)
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  delay(500);
}

inline void INITIALFINALCapstanShoulderDegrees(int Degrees1, int Degrees2)
{
  // Map initial and final positions to servo values
  int Pos1 = map(Degrees1 % 180, 0, 180, SERVOMIN, SERVOMAX);
  int Pos2 = map(Degrees2 % 180, 0, 180, SERVOMIN, SERVOMAX);
  
  // Calculate number of full rotations
  int degreesDiff = abs(Degrees2 - Degrees1);
  int fullRotations = degreesDiff / 360;
  
  // Move from initial to final position
  if (Pos1 < Pos2) {
    for (uint16_t pulselen = Pos1; pulselen < Pos2; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  } else if (Pos1 > Pos2) {
    // Need to complete a full rotation
    // First go to max
    for (uint16_t pulselen = Pos1; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
    // Then from min to final position
    for (uint16_t pulselen = SERVOMIN; pulselen < Pos2; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  delay(500);
  
  // Perform additional rotations if needed
  for (int i = 0; i < fullRotations; i++) {
    // Complete a full 360 degree rotation
    for (uint16_t pulselen = Pos2; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
    for (uint16_t pulselen = SERVOMIN; pulselen < Pos2; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  
  // Return to the initial position
  if (Pos2 > Pos1) {
    for (uint16_t pulselen = Pos2; pulselen > Pos1; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  } else if (Pos2 < Pos1) {
    // Need to complete a full rotation backward
    // First go to min
    for (uint16_t pulselen = Pos2; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
    // Then from max to initial position
    for (uint16_t pulselen = SERVOMAX; pulselen > Pos1; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
}

// Updated CapstanShoulderDegrees to handle unlimited rotation
inline void CapstanShoulderDegrees(int Degrees)
{
  // Calculate how many full rotations we need
  int fullRotations = Degrees / 360;
  int remainingDegrees = Degrees % 360;
  
  // Map the remaining degrees to servo pulse values
  int finalPos = map(remainingDegrees, 0, 180, SERVOMIN, SERVOMAX);
  
  // Perform full 360-degree rotations first
  for (int rotation = 0; rotation < fullRotations; rotation++) {
    // First half rotation (0 to 180)
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
    
    // Second half rotation (180 to 360, represented as 180 back to 0)
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  
  // Handle any remaining degrees (less than 360)
  if (remainingDegrees > 0) {
    for (uint16_t pulselen = SERVOMIN; pulselen < finalPos; pulselen++) {
      pwm.setPWM(3, 0, pulselen);
      delay(1);
    }
  }
  delay(500);

  // Return to the starting position
  for (uint16_t pulselen = finalPos; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(3, 0, pulselen);
    delay(1);
  }
}

#endif // FUNCTIONS_