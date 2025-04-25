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
#define FIXED_BICEP_ANGLE 45

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
void ONEWAYWristDegreesBACK(int final, int initial);
void INITIALFINALWristDegrees(int Degrees1, int Degrees2);
void ONEWAYCapstanShoulderDegrees(int Degrees);
void ONEWAYCapstanShoulderDegreesBACK(int final, int initial);
void INITIALFINALCapstanShoulderDegrees(int Degrees1, int Degrees2);
void Kill();
void PointToCoordinates(float targetX, float targetY, float targetZ);
void SummonPotOfGreed();
void BeatAlbinsAss();
void resetBadmintonPosition();
void I_SUMMON_POT_OF_GREED_TO_DRAW_3_ADDITIONAL_CARDS_FROM_MY_DECK();


inline void PointToCoordinates(float targetX, float targetY, float targetZ) {
  Serial.println("Pointing to coordinates:");
  Serial.print("X: "); Serial.print(targetX);
  Serial.print(" Y: "); Serial.print(targetY);
  Serial.print(" Z: "); Serial.println(targetZ);
  
  // STEP 1: Calculate bicep swivel angle using arctan(Z/X)
  // This rotates the arm in the horizontal plane
  float bicepSwivelAngle;
  
  // Handle special case to prevent division by zero
  if (targetX == 0) {
    bicepSwivelAngle = (targetZ >= 0) ? 0 : 180; // 0° for +Z, 180° for -Z
  } else {
    // Calculate angle in the XZ plane
    bicepSwivelAngle = atan2(targetZ, targetX) * 180.0 / PI;
    
    // Convert from -180°/180° to 0°-180° for servo control
    bicepSwivelAngle = 90 - bicepSwivelAngle; // Adjust based on your servo orientation
    
    // Ensure the angle is within the servo's range
    if (bicepSwivelAngle < 0) bicepSwivelAngle = 0;
    if (bicepSwivelAngle > 180) bicepSwivelAngle = 180;
  }
  
  // STEP 2: Calculate elbow angle using arctan(Y/horizontal_distance)
  // Horizontal distance is the straight-line distance in the XZ plane
  float horizontalDistance = sqrt(targetX*targetX + targetZ*targetZ);
  float elbowAngle;
  
  // Handle special case to prevent division by zero
  if (horizontalDistance == 0) {
    elbowAngle = (targetY >= 0) ? 0 : 180; // 0° for up, 180° for down
  } else {
    elbowAngle = atan2(targetY, horizontalDistance) * 180.0 / PI;
    
    // Convert to servo angle - assuming 90° is horizontal
    elbowAngle = 90 - elbowAngle;
    
    // Account for the 1.6:1 gear ratio mentioned in the code
    elbowAngle = elbowAngle * 1.6;
    
    // Ensure angle is within servo's range
    if (elbowAngle < 0) elbowAngle = 0;
    if (elbowAngle > 180) elbowAngle = 180;
  }
  
  // STEP 3: Calculate wrist angle to maintain level pointing
  // In simple terms, the wrist needs to compensate for the elbow angle
  float wristAngle = 90 + (elbowAngle / 1.6 - 90); // Undo gear ratio for calculation
  
  // Ensure angle is within servo's range
  if (wristAngle < 0) wristAngle = 0;
  if (wristAngle > 180) wristAngle = 180;
  
  // Debug output
  Serial.print("Bicep swivel angle: "); Serial.println(bicepSwivelAngle);
  Serial.print("Elbow angle: "); Serial.println(elbowAngle);
  Serial.print("Wrist angle: "); Serial.println(wristAngle);
  
  // STEP 4: Move servos to calculated positions
  // For bicep swivel (servo 2) - assuming it's inverted
  int bicepPos = map(bicepSwivelAngle, 0, 180, SERVOMAX, SERVOMIN); // Inverted mapping
  pwm.setPWM(BicepSwivel, 0, bicepPos);
  
  // For elbow (servo 1)
  int elbowPos = map(elbowAngle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(Elbow, 0, elbowPos);
  
  // For wrist (servo 0)
  int wristPos = map(wristAngle, 0, 180, SG90MIN, SG90MAX);
  pwm.setPWM(Wrist, 0, wristPos);
  
  Serial.println("Movement complete");
}


inline void SummonPotOfGreed() {
  // Make sure we have valid coordinates
  if (x == 0 && y == 0 && z == 0) {
    Serial.println("Cannot calculate angles, coordinates are at origin");
    return;
  }
  
  Serial.println("I SUMMON POT OF GREED");
  
  // Scale factor correction - If input is being scaled down by 10, we'll compensate
  float scaleFactor = 10.0;
  
  // Convert the input coordinates to real-world coordinates in cm
  float worldX = x * scaleFactor;  
  float worldY = y * scaleFactor;
  float worldZ = z * scaleFactor;
  
  // Debug output of scaled coordinates
  Serial.print("Target coordinates: (");
  Serial.print(worldX);
  Serial.print(", ");
  Serial.print(worldY);
  Serial.print(", ");
  Serial.print(worldZ);
  Serial.println(") cm");
  
  // Calculate the horizontal distance to the target
  float horizontalDistance = sqrt(worldX*worldX + worldZ*worldZ);
  
  // Calculate bicep swivel angle (horizontal rotation)
  // For bicep swivel that moves from 12 o'clock to 3 o'clock
  // 12 o'clock would be 0 degrees, 3 o'clock would be 90 degrees
  
  // atan2 returns angle counterclockwise from positive X axis
  float rawSwivelAngle = atan2(worldZ, worldX) * 180.0 / PI;
  
  // Convert from atan2 convention to our servo convention
  // In atan2, positive Z is forward (12 o'clock), positive X is right (3 o'clock)
  float bicepSwivelAngle;
  
  // If in first quadrant (positive X, positive Z) -> 0 to 90 degrees
  if (worldX >= 0 && worldZ >= 0) {
    bicepSwivelAngle = 90 - rawSwivelAngle;  // Convert to our convention
  }
  // If in second quadrant (negative X, positive Z) -> not reachable, set to 0 (12 o'clock)
  else if (worldX < 0 && worldZ >= 0) {
    bicepSwivelAngle = 0;
  }
  // If in third or fourth quadrant (negative Z) -> not reachable with 12-3 o'clock range
  else {
    bicepSwivelAngle = 90;  // 3 o'clock position
  }
  
  // Ensure swivel angle is within the 0-90 degree range (12 o'clock to 3 o'clock)
  if (bicepSwivelAngle < 0) bicepSwivelAngle = 0;
  if (bicepSwivelAngle > 90) bicepSwivelAngle = 90;
  
  // Account for the fixed 45-degree downward tilt of the bicep
  // First, calculate the effective position of the elbow joint
  float BICEP_ANGLE_RAD = 45.0 * PI / 180.0; // 45 degrees in radians
  
  // Depending on the bicep swivel angle, the elbow position will vary
  float swivelRad = bicepSwivelAngle * PI / 180.0;
  
  // Calculate elbow position components due to the fixed 45-degree tilt
  // When bicep swivel is at 0 (12 o'clock), the tilt affects Z and Y
  // When bicep swivel is at 90 (3 o'clock), the tilt affects X and Y
  float elbowOffsetX = BICEP_LENGTH * sin(BICEP_ANGLE_RAD) * sin(swivelRad);
  float elbowOffsetY = BICEP_LENGTH * sin(BICEP_ANGLE_RAD) * cos(swivelRad);
  float elbowOffsetZ = BICEP_LENGTH * cos(BICEP_ANGLE_RAD);
  
  // Calculate the vector from elbow to target
  float targetVectorX = worldX - elbowOffsetX;
  float targetVectorY = worldY - elbowOffsetY;
  float targetVectorZ = worldZ - elbowOffsetZ;
  
  // Calculate the direct distance from elbow to target
  float targetDistance = sqrt(targetVectorX*targetVectorX + targetVectorY*targetVectorY + targetVectorZ*targetVectorZ);
  
  // Check if the point is reachable with the forearm
  if (targetDistance > FOREARM_LENGTH) {
    // Target is too far - scale down to maximum reach
    float scalingFactor = FOREARM_LENGTH / targetDistance;
    targetVectorX *= scalingFactor;
    targetVectorY *= scalingFactor;
    targetVectorZ *= scalingFactor;
    targetDistance = FOREARM_LENGTH;
    
    Serial.println("Target out of reach for forearm, scaling to max reach");
  }
  
  // Calculate the elevation angle for the elbow joint
  // Project the target vector onto planes determined by the bicep swivel angle
  float projectedDistInSwivelPlane;
  
  // At 0 degrees swivel (12 o'clock), we project onto Y-Z plane
  // At 90 degrees swivel (3 o'clock), we project onto X-Y plane
  if (bicepSwivelAngle <= 45) {
    // Closer to 12 o'clock, primarily use Y-Z plane
    projectedDistInSwivelPlane = sqrt(targetVectorY*targetVectorY + targetVectorZ*targetVectorZ);
  } else {
    // Closer to 3 o'clock, primarily use X-Y plane
    projectedDistInSwivelPlane = sqrt(targetVectorX*targetVectorX + targetVectorY*targetVectorY);
  }
  
  // Calculate elevation angle relative to the horizontal plane
  float elevationAngle = atan2(targetVectorY, projectedDistInSwivelPlane) * 180.0 / PI;
  
  // For elbow: starting position is perpendicular to bicep (90 degrees)
  // Since bicep is tilted 45 degrees down, the elbow at 90 degrees points 45 degrees up
  // Calculate the raw elbow angle needed
  float rawElbowAngle = 90 - elevationAngle;
  
  // Account for the 1.6:1 gear ratio for the elbow servo
  float elbowAngle = rawElbowAngle * 1.6;
  
  // Ensure elbow angle is within physical limits
  if (elbowAngle < 0) elbowAngle = 0;
  if (elbowAngle > 180) elbowAngle = 180;
  
  // For the wrist, we need to maintain a consistent aim at the target
  // Calculate the wrist angle to compensate for both bicep and elbow positions
  // Start with a neutral position of 90 degrees (perpendicular to forearm)
  float wristAngle = 90;
  
  // When elbow is at 90 degrees (perpendicular to bicep), the laser should be level
  // if the bicep is pointed down 45 degrees and elbow is at 90 degrees from the bicep,
  // then the forearm is pointing 45 degrees up from horizontal.
  // So to make the laser level, we need to adjust wrist down by 45 degrees
  float wristCompensation = 45;
  
  // Further adjust wrist based on changes to elbow from its neutral position
  wristAngle = 90 - wristCompensation + (rawElbowAngle - 90);
  
  // Ensure wrist angle is within physical limits
  if (wristAngle < 0) wristAngle = 0;
  if (wristAngle > 180) wristAngle = 180;
  
  // Debug output of calculated angles
  Serial.print("Bicep Swivel Angle (0=12 o'clock, 90=3 o'clock): ");
  Serial.print(bicepSwivelAngle);
  Serial.println(" degrees");
  
  Serial.print("Raw Elbow Angle (before gear ratio): ");
  Serial.print(rawElbowAngle);
  Serial.println(" degrees");
  
  Serial.print("Commanded Elbow Angle (after 1.6:1 gear ratio): ");
  Serial.print(elbowAngle);
  Serial.println(" degrees");
  
  Serial.print("Wrist Angle: ");
  Serial.print(wristAngle);
  Serial.println(" degrees");
  
  // Gradually move servos to their targeted positions for dramatic effect
  
  // Move bicep swivel - USING BARE FOR LOOP
  // Move bicep swivel - USING BARE FOR LOOP
  Serial.println("ACTIVATING BICEP SWIVEL...");
  int bicepPos = map(bicepSwivelAngle, 0, 200, SERVOMAX, SERVOMIN); // Inverted mapping
  for (uint16_t pulselen = SERVOMAX; pulselen > bicepPos; pulselen--) { // Inverted direction
    pwm.setPWM(BicepSwivel, 0, pulselen);
    delay(5); // Using 5ms delay as requested
  }

  
    // Move elbow - USING BARE FOR LOOP
  Serial.println("EXTENDING ELBOW SERVO...");
  int elbowPos = map(elbowAngle, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < elbowPos; pulselen++) {
    pwm.setPWM(Elbow, 0, pulselen);
    delay(5); // Using 5ms delay as requested
  }
  delay(200);
  
  // Move wrist - USING BARE FOR LOOP
  Serial.println("ADJUSTING WRIST SERVO...");
  int wristPos = map(wristAngle, 0, 200, SG90MIN, SG90MAX);
  for (uint16_t pulselen = SG90MIN; pulselen < wristPos; pulselen++) {
    pwm.setPWM(Wrist, 0, pulselen);
    delay(5); // Using 5ms delay as requested
  }
  delay(200);
  
  // POT OF GREED ACTIVATION
  Serial.println("TO DRAW 3 ADDITIONAL CARDS FROM MY DECK!");
  
  // Reset the FACE_DETECTION_FLAG
  FACE_DETECTION_FLAG = 0;
  while(1); // Lock in position
}




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
    delay(10);
  }
  pwm.setPWM(0, 0, WristNewMin);

  pwm.setPWM(1, 0, SERVOMIN);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(15);
  }
  pwm.setPWM(1, 0, SERVOMIN);

  //This will work the other way around. "SERVOMAX" is closer to BicepNewMin, and SERVOMIN is analagous to the max of this servo.
  int BicepNewMin = map(90, 0, 200, SERVOMIN, SERVOMAX);
  pwm.setPWM(2, 0, BicepNewMin);
  for (uint16_t pulselen = SERVOMIN; pulselen < BicepNewMin; pulselen++) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(15);
  }
  pwm.setPWM(2, 0, BicepNewMin);
}

inline void Kill()
{
  delay(50);
  
  // BicepSwivel - Move from 90 degrees position to min position
  int BicepNewMin = map(90, 0, 200, SERVOMIN, SERVOMAX);
  pwm.setPWM(BicepSwivel, 0, BicepNewMin);
  for (uint16_t pulselen = BicepNewMin; pulselen > 190; pulselen--) 
  {
    pwm.setPWM(BicepSwivel, 0, pulselen);
    delay(10);
  }
  pwm.setPWM(BicepSwivel, 0, 190);
  
  delay(50);
  
  // Wrist - Move from 90 degrees position to max position
  int WristNewMin = map(90, 0, 200, SG90MIN, SG90MAX);
  pwm.setPWM(Wrist, 0, WristNewMin);
  for (uint16_t pulselen = WristNewMin; pulselen < SG90MAX; pulselen++) 
  {
    pwm.setPWM(Wrist, 0, pulselen);
    delay(10);
  }
  pwm.setPWM(Wrist, 0, SG90MAX);
  
  delay(50);
  
  // Elbow - Move from min position to max position
  pwm.setPWM(Elbow, 0, SERVOMIN);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) 
  {
    pwm.setPWM(Elbow, 0, pulselen);
    delay(10);
  }
  pwm.setPWM(Elbow, 0, SERVOMAX);
  
  delay(50);
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
  // Scale factor correction - If input is being scaled down by 10, we'll compensate
  float scaleFactor = 10.0;
  
  // Convert the input coordinates to real-world coordinates in cm
  // Adjust scaling to compensate for the factor of 10 reduction
  float worldX = x * scaleFactor;  
  float worldY = y * scaleFactor;
  float worldZ = z * scaleFactor;
  
  // Debug output of scaled coordinates
  Serial.print("Scaled target coordinates: (");
  Serial.print(worldX);
  Serial.print(", ");
  Serial.print(worldY);
  Serial.print(", ");
  Serial.print(worldZ);
  Serial.println(") cm");
  
  // The reference point (0,0,0) is at the junction of bicep and shoulder
  // So we don't need to adjust coordinates relative to ARM_BASE
  float targetX = worldX;
  float targetY = worldY;
  float targetZ = worldZ;
  
  // Calculate the horizontal distance to the target
  float horizontalDistance = sqrt(targetX*targetX + targetZ*targetZ);
  
  // Calculate bicep swivel angle (horizontal rotation)
  // For bicep swivel that moves from 12 o'clock to 3 o'clock
  // 12 o'clock would be 0 degrees, 3 o'clock would be 90 degrees
  // We need to adjust atan2 calculation accordingly
  
  // atan2 returns angle counterclockwise from positive X axis
  float rawSwivelAngle = atan2(targetZ, targetX) * 180.0 / PI;
  
  // Convert from atan2 convention to our servo convention
  // In atan2, positive Z is forward (12 o'clock), positive X is right (3 o'clock)
  // For our servo, 0 is 12 o'clock, 90 is 3 o'clock
  float bicepSwivelAngle;
  
  // If in first quadrant (positive X, positive Z) -> 0 to 90 degrees
  if (targetX >= 0 && targetZ >= 0) {
    bicepSwivelAngle = 90 - rawSwivelAngle;  // Convert to our convention
  }
  // If in second quadrant (negative X, positive Z) -> not reachable
  // Set to closest reachable angle (0 degrees = 12 o'clock)
  else if (targetX < 0 && targetZ >= 0) {
    bicepSwivelAngle = 0;
  }
  // If in third or fourth quadrant (negative or positive X, negative Z)
  // These are also not reachable with 12 o'clock to 3 o'clock range
  // Set to closest reachable angle
  else {
    bicepSwivelAngle = 90;  // 3 o'clock position
  }
  
  // Ensure swivel angle is within the 0-90 degree range (12 o'clock to 3 o'clock)
  if (bicepSwivelAngle < 0) bicepSwivelAngle = 0;
  if (bicepSwivelAngle > 90) bicepSwivelAngle = 90;
  
  // Calculate the elevation angle (vertical angle from horizontal)
  float elevationAngle = atan2(targetY, horizontalDistance) * 180.0 / PI;
  
  // Calculate the direct distance to the target point
  float targetDistance = sqrt(targetX*targetX + targetY*targetY + targetZ*targetZ);
  
  // Check if the point is reachable
  float maxReach = BICEP_LENGTH + FOREARM_LENGTH;
  if (targetDistance > maxReach) {
    // Target is too far - scale down to maximum reach
    float scalingFactor = maxReach / targetDistance;
    targetX *= scalingFactor;
    targetY *= scalingFactor;
    targetZ *= scalingFactor;
    targetDistance = maxReach;
    
    // Recalculate horizontal distance
    horizontalDistance = sqrt(targetX*targetX + targetZ*targetZ);
    
    // Update elevation angle
    elevationAngle = atan2(targetY, horizontalDistance) * 180.0 / PI;
    
    Serial.println("Target out of reach, scaling to max reach");
  }
  
  // For elbow: starting position is perpendicular to bicep (90 degrees)
  // Full extension is collinear with bicep (0 degrees)
  // Need to account for 1.6:1 gear ratio
  
  // Calculate actual mechanical angle needed for elbow
  float rawElbowAngle;
  
  // Account for elevation to point at target
  // When elevation is positive, we need to increase elbow angle from perpendicular
  // When elevation is negative, we need to decrease elbow angle from perpendicular
  rawElbowAngle = 90 - elevationAngle;
  
  // Compensate for the 1.6:1 gear ratio
  // If we need the arm to move X degrees, we need to command 1.6*X to the servo
  float elbowAngle = rawElbowAngle * 1.6;
  
  // Ensure elbow angle is within servo limits after gear ratio adjustment
  if (elbowAngle < 0) elbowAngle = 0;
  if (elbowAngle > 180) elbowAngle = 180;
  
  // For the wrist, we need to maintain a consistent aim at the target
  // Assuming wrist neutral position has the laser perpendicular to forearm
  // We need to adjust the wrist to compensate for bicep and elbow positions
  float wristAngle = 90;  // Start at neutral position
  
  // Adjust wrist to keep laser pointing at target
  // If elbow is at 90 degrees (perpendicular), wrist should be at 90 degrees (neutral)
  // As elbow changes from 90, wrist should compensate
  wristAngle = 90 + (rawElbowAngle - 90);
  
  // Ensure wrist angle is within servo limits
  if (wristAngle < 0) wristAngle = 0;
  if (wristAngle > 180) wristAngle = 180;
  
  // Debug output of calculated angles
  Serial.print("Bicep Swivel Angle (0=12 o'clock, 90=3 o'clock): ");
  Serial.print(bicepSwivelAngle);
  Serial.println(" degrees");
  
  Serial.print("Raw Elbow Angle (before gear ratio): ");
  Serial.print(rawElbowAngle);
  Serial.println(" degrees");
  
  Serial.print("Commanded Elbow Angle (after 1.6:1 gear ratio): ");
  Serial.print(elbowAngle);
  Serial.println(" degrees");
  
  Serial.print("Wrist Angle: ");
  Serial.print(wristAngle);
  Serial.println(" degrees");
  
  // Call the servo movement functions to position the arm
  // Use ONEWAYBicepSwivelDegrees for the bicep swivel
  ONEWAYBicepSwivelDegrees(bicepSwivelAngle);
  
  // Use ONEWAYElbowDegrees for the elbow
  ONEWAYElbowDegrees(elbowAngle);
  
  // Use ONEWAYWristDegrees for the wrist
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



// Function to play badminton stroke by simultaneously moving wrist and bicep swivel
// from minimum to maximum positions with a 5ms delay

void BeatAlbinsAss() {

  pwm.setPWM(Elbow , 0 , SERVOMAX); 
  // Define position variables for wrist and bicep swivel
  int wristStartPos = SG90MIN;
  int wristEndPos = map(200, 0, 200, SG90MIN, SG90MAX);  // Map to maximum wrist position
  
  int bicepStartPos = SERVOMIN;
  int bicepEndPos = map(200, 0, 200, SERVOMIN, SERVOMAX);  // Map to maximum bicep position
  
  // Get the range of movement for scaling
  int wristRange = wristEndPos - wristStartPos;
  int bicepRange = bicepEndPos - bicepStartPos;
  
  // Find the maximum range to use as our loop counter
  int maxRange = max(wristRange, bicepRange);
  
  Serial.println("Starting badminton stroke movement...");
  
  // Move both servos simultaneously using a single for loop with percentage-based positioning
  for (int i = 0; i <= 100; i++) {  // Use percentage (0-100) for precision
    // Calculate positions based on percentage
    int wristPos = wristStartPos + (wristRange * i / 100);
    int bicepPos = bicepStartPos + (bicepRange * i / 100);
    
    // Set servo positions
    pwm.setPWM(Wrist, 0, wristPos);
    pwm.setPWM(BicepSwivel, 0, bicepPos);
    
    // Use the specified 5ms delay
    delay(5);
  }
  
  Serial.println("Badminton stroke completed!");
}

// Function to reset servos to starting positions
void resetBadmintonPosition() {
  Serial.println("Resetting to starting positions...");
  
  // Get current positions (assuming we're at max)
  int currentWristPos = map(200, 0, 200, SG90MIN, SG90MAX);
  int currentBicepPos = map(200, 0, 200, SERVOMIN, SERVOMAX);
  
  // Return to min positions
  for (int i = 100; i >= 0; i--) {
    // Calculate positions
    int wristPos = SG90MIN + ((currentWristPos - SG90MIN) * i / 100);
    int bicepPos = SERVOMIN + ((currentBicepPos - SERVOMIN) * i / 100);
    
    // Set servo positions
    pwm.setPWM(Wrist, 0, wristPos);
    pwm.setPWM(BicepSwivel, 0, bicepPos);
    
    delay(5);
  }
  
  Serial.println("Reset complete");
}







// SummonPotOfGreed function - Moves bicep swivel and elbow based on coordinate calculations
// Bicep swivel moves based on arctan(y/x) measured from x-axis (3 o'clock position)
// Elbow moves based on arctan((y-(BICEP_LENGTH*sin(FIXED_BICEP_ANGLE)))/(z-(BICEP_LENGTH*cos(FIXED_BICEP_ANGLE))))
inline void I_SUMMON_POT_OF_GREED_TO_DRAW_3_ADDITIONAL_CARDS_FROM_MY_DECK() {
  // Make sure we have valid coordinates
  if (x == 0 && y == 0) {
    Serial.println("Cannot calculate bicep angle, both x and y are zero");
    return;
  }
  
  //====== BICEP SWIVEL CALCULATION ======
  // Calculate the angle in radians from positive x-axis (3 o'clock)
  float angle_rad_bicep = atan2(y, x);
  
  // Convert angle to degrees
  float angle_deg_bicep = angle_rad_bicep * 180.0 / PI;
  
  // Adjust angle to be between 0 and 90 degrees (our bicep servo range)
  // In atan2, 0 degrees is at 3 o'clock, 90 degrees is at 12 o'clock
  // We need to map this to our servo where 0 degrees is 12 o'clock, 90 degrees is 3 o'clock
  float bicepSwivelAngle;
  
  // Convert from atan2 convention to our servo convention
  if (angle_deg_bicep >= 0 && angle_deg_bicep <= 90) {
    // First quadrant: convert directly
    bicepSwivelAngle = 90 - angle_deg_bicep;
  } 
  else if (angle_deg_bicep > 90 && angle_deg_bicep <= 180) {
    // Second quadrant: limit to 0 degrees (12 o'clock)
    bicepSwivelAngle = 0;
  }
  else if (angle_deg_bicep < 0 && angle_deg_bicep >= -90) {
    // Fourth quadrant: limit to 90 degrees (3 o'clock)
    bicepSwivelAngle = 90;
  }
  else {
    // Third quadrant: limit to closest reachable angle
    bicepSwivelAngle = 0;
  }
  
  // Ensure the bicep angle is within physical limits
  if (bicepSwivelAngle < 0) bicepSwivelAngle = 0;  // 12 o'clock position
  if (bicepSwivelAngle > 90) bicepSwivelAngle = 90; // 3 o'clock position

  //====== ELBOW CALCULATION ======
  // Calculate the fixed bicep angle effects (convert degrees to radians for sin/cos)
  float bicep_angle_rad = FIXED_BICEP_ANGLE * PI / 180.0;
  float bicep_sin_component = BICEP_LENGTH * sin(bicep_angle_rad);
  float bicep_cos_component = BICEP_LENGTH * cos(bicep_angle_rad);
  
  // Adjust coordinates based on the fixed 45-degree bicep angle
  float adjusted_y = y - bicep_sin_component;
  float adjusted_z = z - bicep_cos_component;
  
  // Check if we can calculate a meaningful angle
  if (adjusted_z == 0) {
    Serial.println("Cannot calculate elbow angle, adjusted z is zero");
    return;
  }
  
  // Calculate the elevation angle using the corrected formula:
  // arctan((y-(BICEP_LENGTH*sin(FIXED_BICEP_ANGLE)))/(z-(BICEP_LENGTH*cos(FIXED_BICEP_ANGLE))))
  float angle_rad_elbow = atan2(adjusted_y, adjusted_z);
  
  // Convert angle to degrees
  float angle_deg_elbow = angle_rad_elbow * 180.0 / PI;
  
  // Calculate displacement from perpendicular (90 degrees is perpendicular to bicep)
  float rawElbowAngle = 90 - angle_deg_elbow;
  
  // Account for 1.6:1 gear ratio as in moveServosToFace() function
  float elbowAngle = rawElbowAngle * 1.6;
  
  // Ensure the elbow angle is within servo limits after gear ratio adjustment
  if (elbowAngle < 0) elbowAngle = 0;
  if (elbowAngle > 180) elbowAngle = 180;
  
  // Debug output
  Serial.print("Input coordinates: x=");
  Serial.print(x);
  Serial.print(", y=");
  Serial.print(y);
  Serial.print(", z=");
  Serial.println(z);
  
  Serial.print("Bicep raw angle from atan2(y,x): ");
  Serial.print(angle_deg_bicep);
  Serial.println(" degrees");
  
  Serial.print("Bicep Swivel Angle (0=12 o'clock, 90=3 o'clock): ");
  Serial.print(bicepSwivelAngle);
  Serial.println(" degrees");
  
  Serial.print("Bicep sin component: ");
  Serial.print(bicep_sin_component);
  Serial.println(" cm");
  
  Serial.print("Bicep cos component: ");
  Serial.print(bicep_cos_component);
  Serial.println(" cm");
  
  Serial.print("Adjusted y: ");
  Serial.print(adjusted_y);
  Serial.println(" cm");
  
  Serial.print("Adjusted z: ");
  Serial.print(adjusted_z);
  Serial.println(" cm");
  
  Serial.print("Elbow raw angle from atan2(adjusted_y, adjusted_z): ");
  Serial.print(angle_deg_elbow);
  Serial.println(" degrees");
  
  Serial.print("Raw Elbow displacement (90 - elbow angle): ");
  Serial.print(rawElbowAngle);
  Serial.println(" degrees");
  
  Serial.print("Commanded Elbow Angle (after 1.6:1 gear ratio): ");
  Serial.print(elbowAngle);
  Serial.println(" degrees");
  
  // Move the bicep swivel from min position to calculated angle
  ONEWAYBicepSwivelDegrees(bicepSwivelAngle);
  
  // Move the elbow from min position to calculated angle
  ONEWAYElbowDegrees(elbowAngle);
  
  // Added animation - summoning effect
  // Pot of Greed allows you to draw two new cards!
  Serial.println("SUMMONING POT OF GREED!");
  Serial.println("IT ALLOWS ME TO DRAW TWO NEW CARDS!");
  
  // Reset the FACE_DETECTION_FLAG
  FACE_DETECTION_FLAG = 0;
  while(1);
}



#endif // FUNCTIONS_