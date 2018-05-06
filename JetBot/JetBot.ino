#include <DualVNH5019MotorShield.h>

//Bluetooth, motor, and sonar

DualVNH5019MotorShield md;
 
// defines pins numbers
const int middleTrig = 47;
const int middleEcho = 49;

const int leftTrig = 51;
const int leftEcho = 53;

const int rightTrig = 43;
const int rightEcho = 45;

// defines variables
long duration;
int distance;
//sonar globals
long middleRange = 40, rightRange = 30, leftRange = 30,
leftDelay = 500, rightDelay = 500, backDelay = 300,
releaseDelay = 300, middleDistance, rightDistance, leftDistance;
//bluetooth globals
int RX = 0;
boolean remoteControlState = false;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial1.println("M1 fault");
    while(1);//do nothing
  }
  if (md.getM2Fault())
  {
    Serial1.println("M2 fault");
    while(1);
  }
}
 
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(middleTrig, OUTPUT); // Sets the trigPin as an Output
  pinMode(middleEcho, INPUT); // Sets the echoPin as an Input
  pinMode(leftTrig, OUTPUT); // Sets the trigPinLeft as an Output
  pinMode(leftEcho, INPUT); // Sets the echoPinLeft as an Input
  pinMode(rightTrig, OUTPUT); // Sets the trigPinRight as an Output
  pinMode(rightEcho, INPUT); // Sets the echoPinRight as an Input
  Serial1.println("Dual VNH5019 Motor Shield");
  md.init();
}

int limit=100; 
int check(int trig, int echo)
{
  // Clears the trigPin
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  return distance;
}
void set_Move(int lim1, int lim2, int del) {
  //object infront; go backwards
      md.setM1Speed(lim1);
      stopIfFault();
      md.setM2Speed(lim2);
      stopIfFault();
      delay(del);
}
void blocked() {
  int ran = rand() % 2; 
  set_Move(0, 0, releaseDelay);
      switch(ran) {
        case 0://TURN LEFT
          set_Move(limit, limit, leftDelay);
          break;
        case 1://TURN RIGHT
          set_Move(-limit, -limit, rightDelay);
          break;
      } 
}
boolean middleObjectAvoid() {
  // avoids object found by middle ultrasound sensor
      middleDistance = check(middleTrig,middleEcho);
      if(middleDistance < middleRange) {
        //RELEASE
        set_Move(0,0,releaseDelay);
        rightDistance = check(rightTrig,rightEcho);
        leftDistance = check(leftTrig,leftEcho);
        if(rightDistance < rightRange) {
          if(leftDistance < leftRange) {
            blocked();
          }
          else {
            //TURN LEFT
            set_Move(limit, limit, leftDelay);
          }
        }
        else {
          //TURN RIGHT
          set_Move(-limit,-limit, rightDelay);
        }
        //RELEASE
        set_Move(0,0, releaseDelay);
        return true;
      }
      else
        return false;
}
boolean rightObjectAvoid() {
  // avoids object found by right ultrasound sensor
      rightDistance = check(rightTrig,rightEcho);
      if(rightDistance < rightRange) {
        //RELEASE
        set_Move(0,0, releaseDelay);
        middleDistance = check(middleTrig,middleEcho);
        leftDistance = check(leftTrig,leftEcho);
        if(middleDistance < middleRange) {
          if(leftDistance < leftRange) {
            blocked();
          }
          else {
            //TURN LEFT
            set_Move(limit, limit, leftDelay);
          }
        }
        else {
          //TURN LEFT
          set_Move(limit, limit, leftDelay);
        }
        //RELEASE
        set_Move(0,0, releaseDelay);
        return true;
      }
      else
        return false;
}
boolean leftObjectAvoid() {
  // avoids object found by left ultrasound sensor
      leftDistance = check(leftTrig,leftEcho);
      if(leftDistance < leftRange) {
        //RELEASE
        set_Move(0,0, releaseDelay);
        middleDistance = check(middleTrig,middleEcho);
        rightDistance = check(rightTrig,rightEcho);
        if(middleDistance < middleRange) {
          if(rightDistance < rightRange) {
            blocked();
          }
          else {
            //TURN RIGHT
            set_Move(-limit, -limit, rightDelay);
          }
        }
        else {
          //TURN RIGHT
          set_Move(-limit, -limit, rightDelay);
        }
        //RELEASE
        set_Move(0,0, releaseDelay);
        return true;
      }
      else
        return false;
}
void scan() {
      middleObjectAvoid();
      leftObjectAvoid();
      rightObjectAvoid();
      if((leftObjectAvoid() && middleObjectAvoid()) || (middleObjectAvoid() && leftObjectAvoid())) {
        //MOVE BACKWARDS
        set_Move(limit, -limit, backDelay);
        //TURN AROUND
            //TURN RIGHT
            set_Move(-limit, -limit, 1000);
        //RELEASE
        set_Move(0,0, releaseDelay);
      }
      else if((rightObjectAvoid() && middleObjectAvoid()) || (middleObjectAvoid() && rightObjectAvoid())) {
        //MOVE BACKWARDS
        set_Move(limit, -limit, backDelay);
        //TURN AROUND
            //TURN LEFT
            set_Move(limit, limit, 1000);
        //RELEASE
        set_Move(0,0, releaseDelay);
      }
      else if(rightObjectAvoid() && leftObjectAvoid()) {
        //MOVE BACKWARDS
        set_Move(limit, -limit, backDelay);
        //TURN AROUND
            //TURN RIGHT
            set_Move(-limit, -limit, 1000);
        //RELEASE
        set_Move(0,0, releaseDelay);
      }
      else if(leftObjectAvoid() && rightObjectAvoid()) {
        //MOVE BACKWARDS
        set_Move(limit, -limit, backDelay);
        //TURN AROUND
            //TURN LEFT
            set_Move(limit, limit, 1000);
        //RELEASE
        set_Move(0,0, releaseDelay);
      }
}
void setPower(int RX) {
  while(RX == '1' || RX == -1) {
        scan();
        //FORWARD
        set_Move(-limit, limit, 0);
        RX = -1;
        
      if(Serial1.available() > 0)
        RX = Serial1.read();
        
        if(RX == '1') {
          set_Move(0,0, 2000);
          break;
        }
      }
}
//Bluetooth function
//recieve text from Android application
void setControl(int RX) {
      if(RX == 'D') {
        //FORWARD
        set_Move(-limit, limit, 0);
        RX = 0;
      }
      else if(RX == 'R') {
        //BACKWARD
        set_Move(limit, -limit, 0);
        RX = 0;
      }
      else if(RX == 'l') {
        //TURN LEFT
        set_Move(limit/2, limit, 0);
        RX = 0;
      }
      else if(RX == 'B' || RX == '<' || RX == '>') {
        //RELEASE
        set_Move(0,0, 0);
        RX = 0;
      }
      else if(RX == 'r') {
        //TURN RIGHT
        set_Move(-limit, -limit/2, 0);
        RX = 0;
      }
}
void loop()
{
  // put your main code here, to run repeatedly:
      if(Serial1.available() > 0)
        RX = Serial1.read();
        
        if(RX == 'E') {
          //EXIT REMOTE CONTROL
          set_Move(0,0, 0);
          RX = 0;
          remoteControlState = false;
        }
      if(remoteControlState || RX == '9') {
        //ENTER REMOTE CONTROL || BLUETOOTH CONTROL
        setControl(RX);
        RX = 0;
        remoteControlState = true;
      }
      setPower(RX);
      RX = 0;
    
}
