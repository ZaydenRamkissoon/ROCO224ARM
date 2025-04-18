#include "functions.h"

// Global variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
uint8_t servonum = 0;
int rev = 0;
int x = 0;
int y = 0;
int z = 0;
uint8_t xarray[] = {0, 0, 0};
uint8_t yarray[] = {0, 0, 0};
uint8_t zarray[] = {0, 0, 0};
int FACE_DETECTION_FLAG = 0;
int xidx = 0;
int yidx = 0;
int zidx = 0;
char xval = 0;
char yval = 0;
char zval = 0;
int cartesiancount = 0; //0 corresponds to x, 1 corresponds to y, 2 corresponds to z.
float gear1 = 1;
float gear2 = 1;
float gear3 = 1;
float gear4 = 1;
int testflag = 0;
char buffer[16];
int bufferIndex = 0;
bool newDataAvailable = false;

void setup() 
{
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
}

void loop() 
{
  pwm.setPWM(3, 0, 210);
      
  Serial.println("Ready for coordinate input in format (xxx,yyy,zzz)");
  
  // Check for incoming data from serial port
  readFaceCoordinates();
  
  // If coordinates received, move servos accordingly
  if (newDataAvailable) 
  {
    moveServosToFace();
    SetMinPositions(); // Reset to minimum positions after moving
    newDataAvailable = false;
  }
 
  // Original servo demo code (will run when no serial data is available)
  if (!Serial.available()) 
  {
    
    //Wrist = 0
    //Elbow = 1
    //Bicep Swivel = 2
    //Capstan Shoulder = 3
    //for the capstan, ++ makes the shaft rotate anti-clockwise (meaning the shoulder goes up/clockwise)
    //therefore, -- makes the shaft rotate clockwise (meaning the shoulder goes down/anti-clockwise)
    //capstan works on timers. capstandegrees(1) followed by time = how much it will rotate anglewise.

    delay(500); // Reduced delay for better responsiveness to serial input
    
    // Initialize the arm to a known position
    SetMinPositions();
    pwm.setPWM(3, 0, 210);
    delay(50000);
    
    
    
    delay(1000); // Wait a second before checking again
  }  
}