#include "RoboticArm.h"

// Initialize PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Global variables for coordinates
int x = 0;
int y = 0;
int z = 0;
uint8_t xarray[] = {0, 0, 0};
uint8_t yarray[] = {0, 0, 0};
uint8_t zarray[] = {0, 0, 0};
int xidx = 0;
int yidx = 0;
int zidx = 0;
int FACE_DETECTION_FLAG = 0;

// Buffer for serial data
char buffer[16];
int bufferIndex = 0;
bool newDataAvailable = false;

// Trigger servo functions
void ONEWAYTriggerDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SG90MIN, SG90MAX);
  for (uint16_t pulselen = SG90MIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(0, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

void TriggerDegrees(int Degrees)
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

// Bicep swivel servo functions
void ONEWAYBicepSwivelDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

void ONEWAYBicepSwivelDegreesBACK(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMAX; pulselen > Pos; pulselen--) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

void BicepSwivelDegrees(int Degrees)
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

void INITIALFINALBicepSwivelDegrees(int Degrees1, int Degrees2)
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

// Elbow servo functions
void ONEWAYElbowDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

void ONEWAYElbowDegreesBACK(int Degrees)
{
  int Pos = map(Degrees, 0, 200, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMAX; pulselen > Pos; pulselen--) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(10);
  }
  delay(500);
}

void ElbowDegrees(int Degrees)
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

void INITIALFINALElbowDegrees(int Degrees1, int Degrees2)
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

// Capstan shoulder servo function
void CapstanShoulderDegrees(int Degrees)
{
  int Pos = map(Degrees, 0, 180, SERVOMIN, SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(3, 0, pulselen); // command that moves the servo
    delay(1);
  }
  delay(500);

  for (uint16_t pulselen = Pos; pulselen > SERVOMIN; pulselen--) 
  {
    pwm.setPWM(3, 0, pulselen);
    delay(1);
  }
}

// Face detection and targeting functions
void readFaceCoordinates() 
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

void parseCoordinates() 
{
  // Check if the data format is correct
  if (buffer[0] == '(' && buffer[bufferIndex-1] == ')') 
  {
    // Reset indices
    xidx = yidx = zidx = 0;
    
    // Initialize parsing state (0=parsing x, 1=parsing y, 2=parsing z)
    int parseState = 0;
    
    // Go through the buffer character by character
    for (int i = 1; i < bufferIndex-1; i++) 
    {
      // If we find a comma, move to the next coordinate
      if (buffer[i] == ',') 
      {
        parseState++;
        continue;
      }
      
      // If it's a digit, add it to the appropriate array
      if (buffer[i] >= '0' && buffer[i] <= '9') 
      {
        switch (parseState) 
        {
          case 0: // X coordinate
            if (xidx < sizeof(xarray)-1) // Prevent buffer overflow
              xarray[xidx++] = buffer[i];
            break;
          case 1: // Y coordinate
            if (yidx < sizeof(yarray)-1) // Prevent buffer overflow
              yarray[yidx++] = buffer[i];
            break;
          case 2: // Z coordinate
            if (zidx < sizeof(zarray)-1) // Prevent buffer overflow
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

void moveServosToFace() 
{
  // Convert the input coordinates (0-999) to real-world coordinates in cm
  float worldX = map(x, 0, 999, 0, 1000);  // Map x to a range of 0-1000 cm
  float worldY = map(y, 0, 999, 0, 1000);  // Map y to a range of 0-1000 cm
  float worldZ = map(z, 0, 999, 0, 1000);  // Map z to a range of 0-1000 cm
  
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
  float bicepSwivelAngle = atan2(targetZ, targetX) * 180.0 / PI;
  
  // Adjust to match servo orientation
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
  float elevationAngle = atan2(targetY, horizontalDistance) * 180.0 / PI;
  
  // Map elevation angle to elbow angle assuming 90° elbow angle gives horizontal aim
  float elbowAngle = 90 - elevationAngle;
  
  // Adjust elbow angle into valid servo range (0-180)
  if (elbowAngle < 0) elbowAngle = 0;
  if (elbowAngle > 180) elbowAngle = 180;
  
  // For the trigger, use a fixed angle
  float triggerAngle = 90; // Adjust based on your needs
  
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
  
  // Call servo movement functions to position the arm
  ONEWAYBicepSwivelDegrees(bicepSwivelAngle);
  ONEWAYElbowDegrees(elbowAngle);
  ONEWAYTriggerDegrees(triggerAngle);
  
  // Reset the flags
  FACE_DETECTION_FLAG = 0;
  newDataAvailable = false;
}

void setServoPulse(uint8_t n, double pulse) 
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