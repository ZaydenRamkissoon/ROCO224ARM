#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo pulse length definitions
#define SERVOMIN  150  // Minimum pulse length count (out of 4096)
#define SERVOMAX  600  // Maximum pulse length count (out of 4096)
#define USMIN     600  // Minimum microsecond length
#define USMAX     2400 // Maximum microsecond length
#define SERVO_FREQ 50  // Servo update frequency in Hz

// SG90 servo definitions
#define SG90MIN 0
#define SG90MAX 500

// Arm dimensions in cm
#define BICEP_LENGTH 15.0     // Length of bicep segment
#define FOREARM_LENGTH 18.0   // Length of forearm segment

// Camera-to-arm base offset
#define ARM_BASE_X 0.0  // X coordinate offset
#define ARM_BASE_Y 0.0  // Y coordinate offset
#define ARM_BASE_Z 0.0  // Z coordinate offset

// Function prototypes for servo movement
void ONEWAYTriggerDegrees(int Degrees);
void TriggerDegrees(int Degrees);
void ONEWAYBicepSwivelDegrees(int Degrees);
void ONEWAYBicepSwivelDegreesBACK(int Degrees);
void BicepSwivelDegrees(int Degrees);
void INITIALFINALBicepSwivelDegrees(int Degrees1, int Degrees2);
void ONEWAYElbowDegrees(int Degrees);
void ONEWAYElbowDegreesBACK(int Degrees);
void ElbowDegrees(int Degrees);
void INITIALFINALElbowDegrees(int Degrees1, int Degrees2);
void CapstanShoulderDegrees(int Degrees);
void setServoPulse(uint8_t n, double pulse);

// Function prototypes for face detection and targeting
void readFaceCoordinates();
void parseCoordinates();
void moveServosToFace();

// External variable declarations
extern Adafruit_PWMServoDriver pwm;
extern int x, y, z;
extern uint8_t xarray[], yarray[], zarray[];
extern int xidx, yidx, zidx;
extern int FACE_DETECTION_FLAG;
extern char buffer[];
extern int bufferIndex;
extern bool newDataAvailable;

#endif // ROBOTIC_ARM_H