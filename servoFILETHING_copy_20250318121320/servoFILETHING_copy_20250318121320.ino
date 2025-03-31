#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SG90MIN 0
#define SG90MAX 500

// our servo # counter
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

// New variables for improved face detection
char buffer[16];
int bufferIndex = 0;
bool newDataAvailable = false;


void setup() 
{
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency.
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() 
{
  Serial.write("hi");
  // Check for incoming data from facial recognition system
  readFaceCoordinates();
  Serial.write("hi");
  // If face coordinates received, move servos accordingly
  if (newDataAvailable) 
  {
    moveServosToFace();
    newDataAvailable = false;
  }
 
  // Original servo demo code (will run when no face data is available)
  if (!Serial.available()) 
  {
    
    
    if (testflag == 1)
    {
      while(1)
      {
        if(Serial.read() == 's')
        {
          testflag = 0;
          break;
        }
      }
    }

    //Trigger Finger = 0
    //Elbow = 1
    //Bicep Swivel = 2
    //Capstan Shoulder = 3
    while(1) //being used for testing
    {

      if(Serial.read() == 't')
      {
        testflag == 1;
        break;
      }

      delay(10);
      CapstanShoulderDegrees(360); //this function works up to 180 degrees. the smaller the angle, the less accurate.
      delay(10);

      /*
      for (uint16_t pulselen = SG90MIN; pulselen < SG90MAX; pulselen++) 
      {
      pwm.setPWM(0, 0, pulselen); // command that moves the servo
      delay(1);
      }

      delay(500);

      for (uint16_t pulselen = SG90MAX; pulselen > SG90MIN; pulselen--) 
      {
        pwm.setPWM(0, 0, pulselen);
        delay(1);
      }*/







      /*
      Serial.write("hiiiiii");
      for (uint16_t pulselen = SG90MIN; pulselen < SG90MAX; pulselen++) 
      {
        pwm.setPWM(3, 0, pulselen); // command that moves the servo
        delay(5);
      }
      // pwm.setPWM(3, 0, 210); // command that stops the 360 motor
           //delay(100);

      for (uint16_t pulselen = SG90MIN; pulselen < SG90MAX; pulselen--) 
      {
        pwm.setPWM(3, 0, pulselen); // command that moves the servo
        delay(5);
      }*/

      //  pwm.setPWM(3, 0, 210); // command that stops the 360 motor
      //delay(10);
      //pwm.setPWM(0, 0, 80); // command that moves the servo
      delay(50);
      /*for (uint16_t pulselen = 500; pulselen > 0; pulselen--) {
            pwm.setPWM(0, 0, pulselen); // command that moves the servo
          }*/

    }
      


    // Drive each servo one at a time using setPWM()
    /*
    for (uint16_t pulselen = SG90MIN; pulselen < SG90MAX; pulselen++) {
      pwm.setPWM(servonum, 0, pulselen); // command that moves the servo
    }

    delay(500);
    for (uint16_t pulselen = SG90MAX; pulselen > SG90MIN; pulselen--) {
      pwm.setPWM(servonum, 0, pulselen);
    }

    delay(500);

    servonum++;
    if (servonum > 7) servonum = 0; // Testing the first 8 servo channels
    */
  }
  if (Serial.read() == 's'){(loop);}
}


void TriggerDegrees(int Degrees)
{
  int Pos = map (Degrees , 0 , 200 , SG90MIN , SG90MAX);
  for (uint16_t pulselen = SG90MIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(0, 0, pulselen); // command that moves the servo
    delay(1);
  }

   delay(500);

  for (uint16_t pulselen = Pos; pulselen > SG90MIN; pulselen--) 
  {
    pwm.setPWM(0, 0, pulselen);
    delay(1);
  }
}  


void BicepSwivelDegrees(int Degrees)
{
  int Pos = map (Degrees , 0 , 200 , SERVOMIN , SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(2, 0, pulselen); // command that moves the servo
    delay(1);
  }

   delay(500);

  for (uint16_t pulselen = Pos; pulselen > SERVOMIN; pulselen--) 
  {
    pwm.setPWM(2, 0, pulselen);
    delay(1);
  }
}  



void ElbowDegrees(int Degrees)
{
  int Pos = map (Degrees , 0 , 200 , SERVOMIN , SERVOMAX);
  for (uint16_t pulselen = SERVOMIN; pulselen < Pos; pulselen++) 
  {
    pwm.setPWM(1, 0, pulselen); // command that moves the servo
    delay(1);
  }

   delay(500);

  for (uint16_t pulselen = Pos; pulselen > SERVOMIN; pulselen--) 
  {
    pwm.setPWM(1, 0, pulselen);
    delay(1);
  }
}  


void CapstanShoulderDegrees(int Degrees)
{
  int Pos = map (Degrees , 0 , 200 , SERVOMIN , SERVOMAX);
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



// Read face coordinates from serial port
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

// Parse coordinates from buffer in format "(xxx,yyy,zzz)"
void parseCoordinates() 
{
  // Check if the data format is correct
  if (buffer[0] == '(' && buffer[bufferIndex-1] == ')') 
  {
    // Extract X value (chars 1,2,3)
    xidx = 0;
    for (int i = 1; i <= 3; i++) 
    {
      if (buffer[i] >= '0' && buffer[i] <= '9') 
      {
        xarray[xidx++] = buffer[i];
      }
    }
    xarray[xidx] = '\0';
   
    // Extract Y value (chars 5,6,7)
    yidx = 0;
    for (int i = 5; i <= 7; i++) 
    {
      if (buffer[i] >= '0' && buffer[i] <= '9') 
      {
        yarray[yidx++] = buffer[i];
      }
    }
    yarray[yidx] = '\0';
   
    // Extract Z value (chars 9,10,11)
    zidx = 0;
    for (int i = 9; i <= 11; i++) 
    {
      if (buffer[i] >= '0' && buffer[i] <= '9') 
      {
        zarray[zidx++] = buffer[i];
      }
    }
    zarray[zidx] = '\0';
   
    // Convert string arrays to integers
    x = atoi((char*)xarray);
    y = atoi((char*)yarray);
    z = atoi((char*)zarray);
   
    FACE_DETECTION_FLAG = 1;
  }
}

// Move servos based on face coordinates
void moveServosToFace() 
{
  // Map face coordinates to servo positions
  int xPos = map(x, 0, 999, SG90MIN, SG90MAX);
  int yPos = map(y, 0, 999, SG90MIN, SG90MAX);
  int zPos = map(z, 0, 999, SG90MIN, SG90MAX);
 
  // Move servos to follow face
  pwm.setPWM(0, 0, xPos); // X-axis servo
  pwm.setPWM(1, 0, yPos); // Y-axis servo
  pwm.setPWM(2, 0, zPos); // Z-axis servo
}

void setServoPulse(uint8_t n, double pulse) {
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