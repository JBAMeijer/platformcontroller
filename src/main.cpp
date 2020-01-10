// BallanceBot Project - Embedded Vision Design Minor 2019-2020
// By: Tren Baltussen, Joey Meijer, Kevin Veld
// Version: 1.0 - 07/01/2020


#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <ServoAnglesLUT.h>

enum class FrameState {
  STARTBYTE,
  LENGTHBYTE,
  DATAARRAY,
  CHECKSUMBYTE,
  STOPBYTE
};


void processReceivedBytes();
void processBuffer(byte);
byte unStuff(byte);
byte calculateChecksum();

const int bufferSize = 255;
int dataCounter = 0;
int dataLength = 0;
bool unStuffIncomingByte = false;

bool previousDataProcessed = true;

byte incomingByte = 0; // for incoming serial data

byte buffer[bufferSize];
FrameState expectedPartInFrame = FrameState::STARTBYTE;

#define MAX 2380
#define MIN 570
#define INV1 1
#define INV2 3
#define INV3 5
#define pi  3.14159

// PID Control definitions
#define PIN_INPUT_X   0       // X Coordinate of current ball location input from vision application (Raspberry PI)
#define PIN_INPUT_Y   1       // Y Coordinate of current ball location input from vision application (Raspberry PI)
// #define PIN_INPUT_Xr  2       // X Coordinate input of target point input from vision application (Raspberry PI)
// #define PIN_INPUT_Yr  3       // Y Coordinate input of target point input from vision application (Raspberry PI) 

//Define Variables we'll be connecting to
double Input_X, Input_Y;
double Setpoint_X, Output_X, Setpoint, Output_Y;

//Specify the links and initial tuning parameters
double Kp = 0.18, Ki = 0.1, Kd = 0;        //Kp = -8.96, Ki = -0.96, Kd = -20.51;
//double Kpy = 0.2, Kiy = 0.1, Kdy = 0;     

PID PID_X(&Input_X, &Output_X, &Setpoint, Kp, Ki, Kd, DIRECT);
PID PID_Y(&Input_Y, &Output_Y, &Setpoint, Kp, Ki, Kd, DIRECT);


enum class countdirection
{
   COUNT_UP,
   COUNT_DOWN
};

const float servo_mult=400/(pi/4);

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;

const int potpin = 0;
int val;

int x=25;
int y=25;

countdirection xdir = countdirection::COUNT_UP;
countdirection ydir = countdirection::COUNT_DOWN;

uint8_t MODE = 1; // 0 = Manual control; 1 = PID; 2 = Turn Circles;

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200); 
  
  Serial.println("Board started!");
  delay(15);

  Serial.println("Setting up Servo's!");
  delay(15);

  myservo1.attach(9);
  myservo2.attach(8);
  myservo3.attach(7);
  myservo4.attach(6);
  myservo5.attach(5);
  myservo6.attach(4);      

  // PID - Initialize the variables we're linked to
//   Input_X = analogRead(PIN_INPUT_X);                       // Replace with visual input x-coordinate


//   Input_Y = analogRead(PIN_INPUT_Y);                       // Replace with visual input y-coordinate

   Serial.println("Setting up PID controller");
   delay(15);
//   Setpoint_X = analogRead(PIN_INPUT_Xr);                 // X Coordinate input of target point input from vision application (Raspberry PI)
   Setpoint = 150;                                        // Manually set the X pixel coordinate of the center of the platform
                                                            // Setpoint is set to the center on startup, which is: 150

//   Setpoint_Y = analogRead(PIN_INPUT_Yr);                 // Y Coordinate input of target point input from vision application (Raspberry PI)
   //Setpoint_Y = 150;                                        // Manually set the Y pixel coordinate of the center of the platform
                                                            // Setpoint is set to the center on startup, which is: 150

  // Turn the PID on
  PID_X.SetMode(AUTOMATIC);
  PID_X.SetOutputLimits(-25.0, 25.0);
  
  delay(15);
  PID_Y.SetMode(AUTOMATIC);   
  PID_Y.SetOutputLimits(-25.0, 25.0);

  delay(15);

}

void loop() {  

   if(x > 50) x = 50;
   if(x < 0) x = 0;
   if(y > 50) y = 50;
   if(y < 0) y = 0;


   myservo1.writeMicroseconds(constrain(1500 + radiansPerServo[x][y][0] * servo_mult, MIN, MAX));
   myservo2.writeMicroseconds(constrain(1395 - radiansPerServo[x][y][1] * servo_mult, MIN, MAX));                  
   myservo3.writeMicroseconds(constrain(1630 + radiansPerServo[x][y][2] * servo_mult, MIN, MAX));                  
   myservo4.writeMicroseconds(constrain(1325 - radiansPerServo[x][y][3] * servo_mult, MIN, MAX));                  
   myservo5.writeMicroseconds(constrain(1620 + radiansPerServo[x][y][4] * servo_mult, MIN, MAX));                  
   myservo6.writeMicroseconds(constrain(1355 - radiansPerServo[x][y][5] * servo_mult, MIN, MAX));

   delay(5);
   
   

   if(MODE == 0)      // Manual control
   {
      if(Serial1.available() > 0)
      {
         if(previousDataProcessed)
            processBuffer(Serial1.read());
         
         else
         {
            
            if(buffer[0] == 'w' && !previousDataProcessed) 
            {
               if(x == 50) x = x;
               else x++;

               previousDataProcessed = true;
            }
            else if(buffer[0] == 'a' && !previousDataProcessed) 
            {
               if(y == 50) y = y;
               else y++;

               previousDataProcessed = true;
            }
            else if(buffer[0] == 's' && !previousDataProcessed) 
            {
               if(x == 0) x = x;
               else x--;

               previousDataProcessed = true;
            }
            else if(buffer[0] == 'd' && !previousDataProcessed) 
            {
               if(y == 0) y = y;
               else y--;

               previousDataProcessed = true;
            }
            Serial.println("X: " + String(x) + " Y: " + String(y));
         }
      }
      else if(Serial.available() > 0)
      {
         char incomingByte = Serial.read();
         Serial.println(incomingByte);
         if(incomingByte == 'w') 
         {
            if(x == 50) x = x;
            else x++;
         }
         else if(incomingByte == 'a') 
         {
            if(y == 50) y = y;
            else y++;
         }
         else if(incomingByte == 's') 
         {
            if(x == 0) x = x;
            else x--;
         }
         else if(incomingByte == 'd') 
         {
            if(y == 0) y = y;
            else y--;
         }
         Serial.println("X: " + String(x) + " Y: " + String(y));
      }
   }                   

   // PID Control
   
   else if(Serial1.available() > 0 && MODE == 1)   // Comment-in the Serial.available condition to ensure PID is performed only then.
   {
      if(previousDataProcessed)
      {
         processBuffer(Serial1.read());  // read the incoming byte
      }
      if(!previousDataProcessed)
      {
         // Input_X = analogRead(PIN_INPUT_X);
         // Input_Y = analogRead(PIN_INPUT_Y);

         uint16_t tempx = buffer[1] << 8 | buffer[0];
         uint16_t tempy = buffer[3] << 8 | buffer[2];  

         /*if(tempx > 150)
            PID_X.SetControllerDirection(REVERSE);
         else if(tempx <= 150)
            PID_X.SetControllerDirection(DIRECT);

         if(tempy > 150)
            PID_Y.SetControllerDirection(REVERSE);
         else if(tempy <= 150)
            PID_Y.SetControllerDirection(DIRECT);*/

         Input_X = tempx;                                            // Pre-process input
         Input_Y = tempy;                                            // Pre-process input

         Serial.println("Input_X: " + (String)Input_X);
         Serial.println("Input_Y: " + (String)Input_Y);


         //PID_X.Compute();
         if(!PID_X.Compute())
         {
            Serial.println("Error: PID_X failed!");
         }

         //PID_Y.Compute();
         if(!PID_Y.Compute())
         {
            Serial.println("Error: PID_Y failed!");
         }

         Serial.println("AngleOut_X: " + (String)Output_X);
         Serial.println("AngleOut_Y: " + (String)Output_Y);

         x = Output_X + 25;
         y = Output_Y + 25;


         Serial.println("AngleX_PID: " + (String)x);
         Serial.println("AngleY_PID: " + (String)y);

         previousDataProcessed = true;
      }
         
         
   }

   else if(Serial.available() > 0 && MODE == 2)      // Turn circles
   {
      if(x == 50) xdir = countdirection::COUNT_DOWN;
      else if(x == 0) xdir = countdirection::COUNT_UP;

      if(y == 50) ydir = countdirection::COUNT_DOWN;
      else if(y == 0) ydir = countdirection::COUNT_UP;

      if(xdir == countdirection::COUNT_UP) x++;
      else x--;

      if(ydir == countdirection::COUNT_UP) y++;
      else y--;
   }

}

void processBuffer(byte incomingByte)
{
   if(incomingByte == 0x13 && expectedPartInFrame == FrameState::STARTBYTE)
   {
      dataCounter = 0;
      dataLength = 0;
      Serial.println("New incoming frame!");
      Serial.print("StartByte found: ");
      Serial.println(incomingByte, HEX);
      expectedPartInFrame = FrameState::LENGTHBYTE;
      // Wait for endByte
   }
   else if(expectedPartInFrame == FrameState::LENGTHBYTE)
   {
      Serial.print("LengthByte found: ");
      Serial.println(incomingByte, HEX);
      dataLength = incomingByte;
      expectedPartInFrame = FrameState::DATAARRAY;
   }
   else if(expectedPartInFrame == FrameState::DATAARRAY)
   {
      if(unStuffIncomingByte) buffer[dataCounter++] = unStuff(incomingByte);
      else if(incomingByte == 0x11) unStuffIncomingByte = true;
      else buffer[dataCounter++] = incomingByte;

      Serial.print("DataByte ");
      Serial.print(dataCounter);
      Serial.print(" found: ");
      Serial.println(incomingByte, HEX);

      
      if(dataCounter == dataLength) expectedPartInFrame = FrameState::CHECKSUMBYTE;
   }
   else if(expectedPartInFrame == FrameState::CHECKSUMBYTE)
   {
      Serial.print("ChecksumByte found: ");
      Serial.println(incomingByte, HEX);

      if(calculateChecksum() == incomingByte)
         expectedPartInFrame = FrameState::STOPBYTE;
      else{
         Serial.println("Error: received checksum not the same as calculated checksum");
         while (Serial1.available() > 0)
         {
            Serial1.read();
         }
         expectedPartInFrame = FrameState::STARTBYTE;
      }      
   }
   else if(expectedPartInFrame == FrameState::STOPBYTE)
   {
      Serial.print("StopByte found: ");
      Serial.println(incomingByte, HEX);     

      expectedPartInFrame = FrameState::STARTBYTE;
      previousDataProcessed = false;
   }
}

byte unStuff(byte stuffByte)
{
  unStuffIncomingByte = false;
  if(stuffByte == 0x09)
    return 0x13;
  else if(stuffByte == 0x07)
    return 0x11;
}

byte calculateChecksum()
{
  uint32_t checksum = dataLength;
  for(int i = 0; i < dataLength; i++)
    checksum += buffer[i];

  return (byte)checksum;
}  