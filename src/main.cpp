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
//Define Variables we'll be connecting to
double Input_X, Input_Y;
double Setpoint, Output_X, Output_Y;


//Specify the links and initial tuning parameters
double Ktune = 0.5 /1023;                     // Pot-meter tuning gain.
double Kp = 0.015, Ki = 0.03, Kd = 0.01;       // Kp = 0.32, Ki = 0.40, Kd = 0.40;                //Kp = -8.96, Ki = -0.96, Kd = -20.51;

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


uint8_t MODE = 1; // SET STANDARD TO ZERO UNLESS DESIRED DIFFERENTLY; 0 = Manual control; 1 = PID; 2 = Demo;
uint8_t PrevMODE = 0;

long unsigned int DELAY_VALUE = 2000;     // Delay when ball is not found in ms.
long unsigned int prevTime = 0;           // Delay timer, previous time.

float voltage = 0.0;                      // voltage output of QR detector
long unsigned int delayQR = 1000;         // Delay QR
long unsigned int prevTimeQR = 0;         // Previous time QR

long unsigned int delayMODE2 = 50;       // Delay for MODE 2
long unsigned int prevTimeMODE2 = 0;         // Previous time delayMODE2


void setup() {
  Serial.begin(9600);
  Serial1.begin(115200); 
  
  Serial.println("Board started!");
  delay(15);

  Serial.println("Setting up Servo's!");
  delay(15);

  myservo1.attach(7);
  myservo2.attach(6);
  myservo3.attach(5);
  myservo4.attach(4);
  myservo5.attach(3);
  myservo6.attach(2);      

  // Digital pin inputs Mode Selector
  pinMode(27, INPUT);   // GREEN LED
  pinMode(29, INPUT);   // YELLOW LED
  pinMode(31, INPUT);   // RED LED   

  // PID - Initialize the variables we're linked to

   Serial.println("Setting up PID controller");
   delay(15);
   Setpoint = 150;                                        // Setpoint is set to the center on startup, which is: 150
                                                            

  // Turn the PID on
  PID_X.SetMode(AUTOMATIC);
  PID_X.SetOutputLimits(-25.0, 25.0);
  
  delay(15);
  PID_Y.SetMode(AUTOMATIC);   
  PID_Y.SetOutputLimits(-25.0, 25.0);

  delay(15);

  prevTime = millis();
  prevTimeQR = millis();
  prevTimeMODE2 = millis();

}

void loop() {  

   
   


   // PID Tuning - Un-comment to initiate PID tuning through analog read A5 to A7.    // prevTime = millis(); // micros() if you need accuracy
   // if (millis() - prevTime >= DELAY_VALUE)
   // {
      
   //    Kp = analogRead(A5) * Ktune;
   //    Ki = analogRead(A6) * Ktune;
   //    Kd = analogRead(A7) * Ktune;
   //    Serial.println("Kp: " + String(Kp) + " Ki: " + String(Ki) + " Kd: " + String(Kd));
   //    prevTime = millis();
   // }
   

   if (millis() - prevTimeQR >= delayQR)              // prevTimeQR = millis(); // micros() if you need accuracy
   {
         // Digital marker mode reader
         prevTimeQR = millis();
         // Serial.println("Millis! " + String(prevTimeQR));

         
         bool GREEN = digitalRead(27);
         bool YELLOW = digitalRead(29);
         bool RED = digitalRead(31);

         // Serial.println("GREEN: " + (String)GREEN);
         // Serial.println("YELLOW: " + (String)YELLOW);
         // Serial.println("RED: " + (String)RED);

   //    Digital marker mode reader
         // if(GREEN) MODE = 0;
         // else if (YELLOW) MODE = 1;
         // else if (RED) MODE = 2;
         // else MODE = 0;


   //    Analog marker mode reader
   //    voltage = analogRead(A0)*5.0f/1023.0f;
   
   //    if(0.8f <= voltage && voltage < 1.1f)        MODE = 0;
   //    else if(1.2f <= voltage && voltage < 1.6f)   MODE = 1;
   //    else if(2.2f <= voltage && voltage < 2.5f)   MODE = 2;
   //    else                                         MODE = 0;
   //    Serial.println("V: " + String(voltage) + " Mode: " + MODE);

   }

   if(x > 50) x = 50;
   if(x < 0) x = 0;
   if(y > 50) y = 50;
   if(y < 0) y = 0;

   // delay(5);
   
   if(MODE == 0)      // Manual control
   {
       if(Serial.available() > 0)
      {
         char incomingByte = Serial.read();
         Serial.println(incomingByte);
         if(incomingByte == 'w') 
         {
            if(x == 50) x = x;
            else x++;
            Serial.println("X: " + String(x) + " Y: " + String(y));
         }
         else if(incomingByte == 'a') 
         {
            if(y == 50) y = y;
            else y++;
            Serial.println("X: " + String(x) + " Y: " + String(y));
         }
         else if(incomingByte == 's') 
         {
            if(x == 0) x = x;
            else x--;
            Serial.println("X: " + String(x) + " Y: " + String(y));
         }
         else if(incomingByte == 'd') 
         {
            if(y == 0) y = y;
            else y--;
            Serial.println("X: " + String(x) + " Y: " + String(y));
         }
      }

      PrevMODE = 0;
      
   }                   

   // PID Control
   if(MODE == 1)
   {

      // PID Tuning through serial input

    if(Serial.available() > 0)
      {
         char incomingByte = Serial.read();
         Serial.println(incomingByte);
         if(incomingByte == 'u') 
         {
            Kp = Kp + 0.005;
         }
         else if(incomingByte == 'j') 
         {
            Kp = Kp - 0.005;
         }
         else if(incomingByte == 'i') 
         {
            Ki = Ki + 0.005;
         }
         else if(incomingByte == 'k') 
         {
            Ki = Ki - 0.005;
         }

         else if(incomingByte == 'o') 
         {
            Kd = Kd + 0.005;
         }
         else if(incomingByte == 'l') 
         {
            Kd = Kd - 0.005;
         }
         Serial.println("Kp: " + String(Kp) + " Ki: " + String(Ki) + " Kd: " + String(Kd));

         PID_X.SetTunings(Kp, Ki, Kd);
         PID_Y.SetTunings(Kp, Ki, Kd);

      }


      if(!previousDataProcessed)
      {
         prevTime = millis(); // micros() if you need accuracy

         uint16_t tempx = buffer[1] << 8 | buffer[0];
         uint16_t tempy = buffer[3] << 8 | buffer[2];  


         Input_X = tempx;                                            // Pre-process input
         Input_Y = tempy;                                            // Pre-process input

         Serial.println("Input_X: " + (String)Input_X);
         Serial.println("Input_Y: " + (String)Input_Y);


         
         if(!PID_X.Compute())
         {
            Serial.println("Error: PID_X failed!");
         }

         if(!PID_Y.Compute())
         {
            Serial.println("Error: PID_Y failed!");
         }

         Serial.println("AngleOut_X: " + (String)Output_X);
         Serial.println("AngleOut_Y: " + (String)Output_Y);

         x = Output_X + 25;
         y = Output_Y + 25;

         previousDataProcessed = true;
      }
      if(Serial1.available() > 0 && previousDataProcessed)   // Comment-in the Serial.available condition to ensure PID is performed only then.
      {
         processBuffer(Serial1.read());  // read the incoming byte
      }
      if (millis() - prevTime >= DELAY_VALUE)
      {
         x = 25;
         y = 25;
         Input_X = Setpoint;
         Input_Y = Setpoint;
         prevTime = millis();
      }

      PrevMODE = 1;

   }

   else if(MODE == 2)      // Turn circles
   {
      if(PrevMODE != 2)
      {
         x = 25;
         y = 25;
         Serial.println("X: " + String(x) + " Y: " + String(y));     
         delay(1000);
      }

      if (millis() - prevTimeMODE2 >= delayMODE2)
         {
            if(x == 50) xdir = countdirection::COUNT_DOWN;
            else if(x == 0) xdir = countdirection::COUNT_UP;

            // if(y == 50) ydir = countdirection::COUNT_DOWN;
            // else if(y == 0) ydir = countdirection::COUNT_UP;

            if(xdir == countdirection::COUNT_UP) x++;
            else x--;

            // if(ydir == countdirection::COUNT_UP) y++;
            // else y--;
            prevTimeMODE2 = millis();
         }
      PrevMODE = 2;

   }

   myservo1.writeMicroseconds(constrain(1500 + radiansPerServo[x][y][0] * servo_mult, MIN, MAX));
   myservo2.writeMicroseconds(constrain(1395 - radiansPerServo[x][y][1] * servo_mult, MIN, MAX));                  
   myservo3.writeMicroseconds(constrain(1630 + radiansPerServo[x][y][2] * servo_mult, MIN, MAX));                  
   myservo4.writeMicroseconds(constrain(1325 - radiansPerServo[x][y][3] * servo_mult, MIN, MAX));                  
   myservo5.writeMicroseconds(constrain(1620 + radiansPerServo[x][y][4] * servo_mult, MIN, MAX));                  
   myservo6.writeMicroseconds(constrain(1355 - radiansPerServo[x][y][5] * servo_mult, MIN, MAX));


}

void processBuffer(byte incomingByte)
{
   if(incomingByte == 0x13 && expectedPartInFrame == FrameState::STARTBYTE)
   {
      dataCounter = 0;
      dataLength = 0;
      //Serial.println("New incoming frame!");
      //Serial.print("StartByte found: ");
      //Serial.println(incomingByte, HEX);
      expectedPartInFrame = FrameState::LENGTHBYTE;
      // Wait for endByte
   }
   else if(expectedPartInFrame == FrameState::LENGTHBYTE)
   {
     // Serial.print("LengthByte found: ");
     // Serial.println(incomingByte, HEX);
      dataLength = incomingByte;
      expectedPartInFrame = FrameState::DATAARRAY;
   }
   else if(expectedPartInFrame == FrameState::DATAARRAY)
   {
      if(unStuffIncomingByte) buffer[dataCounter++] = unStuff(incomingByte);
      else if(incomingByte == 0x11) unStuffIncomingByte = true;
      else buffer[dataCounter++] = incomingByte;

     // Serial.print("DataByte ");
     // Serial.print(dataCounter);
     // Serial.print(" found: ");
     // Serial.println(incomingByte, HEX);

      
      if(dataCounter == dataLength) expectedPartInFrame = FrameState::CHECKSUMBYTE;
   }
   else if(expectedPartInFrame == FrameState::CHECKSUMBYTE)
   {
      //Serial.print("ChecksumByte found: ");
      //Serial.println(incomingByte, HEX);

      if(calculateChecksum() == incomingByte)
         expectedPartInFrame = FrameState::STOPBYTE;
      else{
         //Serial.println("Error: received checksum not the same as calculated checksum");
         while (Serial1.available() > 0)
         {
            Serial1.read();
         }
         expectedPartInFrame = FrameState::STARTBYTE;
      }      
   }
   else if(expectedPartInFrame == FrameState::STOPBYTE)
   {
      //Serial.print("StopByte found: ");
      //Serial.println(incomingByte, HEX);     

      expectedPartInFrame = FrameState::STARTBYTE;
      previousDataProcessed = false;
   }
}

byte unStuff(byte stuffByte)
{

   Serial.println("Stuffbyte" + stuffByte);

  unStuffIncomingByte = false;
  if(stuffByte == 0x09)
    return 0x13;
  else if(stuffByte == 0x07)
    return 0x11;
   else
      return 0x00;   // TRIAL: RAN INTO ISSUES WITH PID. TRIED TO GET RID OF WARNINGS.
}

byte calculateChecksum()
{
  uint32_t checksum = dataLength;
  for(int i = 0; i < dataLength; i++)
    checksum += buffer[i];

  return (byte)checksum;
}  