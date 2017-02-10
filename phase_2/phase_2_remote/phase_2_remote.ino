/*
 * CSC460
 * Project 1 - Phase 2
 * Remote station pan & tilt servos and laser respond
 * to base station commands sent via bluetooth
 * 
 */
#include <Roomba_Driver.h>
#include <Servo.h>

#include <scheduler.h>

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

Roomba roombie(2, 34);

const int laserPin = 4;             // Laser pin (pwm pin 4)
const int analogLInPin = A7;        // Analog pin for light sensor

int lightValue = 0;


int roomba_directionX = 122;
int roomba_directionY = 125;

int servoX = 122;                   // value read from the joystick X
int servoY = 125;                   // value read from the joystick Y

int servoX_steps = 0;
int servoY_steps = 0;

int buttonState = 1;                // button is not pressed

volatile int hw_addr =  0;               // variable to store which device to control
volatile int hw_instr = 0;              // variable to store an instruction to give the device





Servo myservoX;                     // create servo object to control a pan servo (X axis)
Servo myservoY;                     // servo object to control tilt servo ( Y axis) 
 
int lowpos = 1000;                  // lowest servo position
int highpos = 2000;                 // highest servo position
int currentX = 1500;                // current X servo position
int currentY = 1500;                // current Y servo position
int oldXPos = 1500;                 // old current X servo position
int oldYPos = 1500;                 // old current Y servo position

void task_read_bluetooth()
{
  digitalWrite(7,HIGH);

  hw_addr = Serial1.read();
  hw_instr = Serial1.read();
  
  switch(hw_addr)
  {
    // Servo pan (x axis)
    case 1:
      servoX = hw_instr;
      break;
    // Servo tilt (y axis)
    case 2:
      servoY = hw_instr;
      break;
    case 3:
      buttonState = hw_instr;     
      break;
    case 4:
      roomba_directionX = hw_instr;
      break;
    case 5:
      roomba_directionY = hw_instr;
      break;
    default:
      // If the hardware address is unknown, do nothing.
      break;
  }  
  digitalWrite(7,LOW);
}

void lightSensorTask()
{
 
   //read the analog in value for light sensor
  lightValue = analogRead(analogLInPin); 
  
}

void task_drive_roomba()
{
  digitalWrite(11, HIGH);  

  if(roomba_directionX == 122 && roomba_directionY == 125)
  {
    roombie.drive(0,0);
  }
  
  if(roomba_directionX < 100)
  {

    roombie.drive(50,-1);
  }
  else if(roomba_directionX > 150)
  {
    
    roombie.drive(50,1);
  }

  if(roomba_directionY < 100)
  {
  
    roombie.drive(-150,32768);
  }
  else if(roomba_directionY > 150)
  {

    roombie.drive(150,32768);
  }
   
  //reset direction variable to no movement
  roomba_directionX = 122;
  roomba_directionY = 125;

  digitalWrite(11, LOW);
}

void task_control_laser()
{
  digitalWrite(8,HIGH);

  // determine if laser should be on or off given the state of the button
  if (buttonState == 0)
  { 
   digitalWrite(laserPin, HIGH);                // Turn Laser On
  }
  else
  {
   digitalWrite (laserPin, LOW);               // Turn Laser off
  }
  digitalWrite(8,LOW);  
}

void task_control_servo()
{
  digitalWrite(9,HIGH);
  
//  Serial.print("servoX: ");
//  Serial.print(servoX);
//  Serial.println();
//  Serial.print("servoY: ");
//  Serial.print(servoY);
//  Serial.println();
  // determine which direction to move pan servo
  if( servoX > 150){
    servoX_steps = servoX - 150;
    // increment only 100 steps at one time     
    for(currentX = oldXPos; currentX >= (oldXPos-servoX_steps); currentX -= 1)     // moves servo X position to the left
    {                      
      myservoX.writeMicroseconds(currentX);                               // tell servo to go to position in variable 'currentX' 
      delayMicroseconds(2500);                                            // waits 2.5ms for the servo to reach the position 
      // limit the amount the servo can move right
      if(currentX < lowpos)
      {
        currentX = lowpos;
        break;
      }
    }
    //update last position       
    oldXPos=currentX;
  }
   else if( servoX < 100) 
  {
    servoX_steps = 100 - servoX;
    for(currentX = oldXPos; currentX <= (oldXPos+servoX_steps); currentX += 1)     // moves servo X position to the right
    {                              
      myservoX.writeMicroseconds(currentX);                               // tell servo to go to position in variable 'currentX' 
      delayMicroseconds(2500);                                            // waits 2.5ms for the servo to reach the position 
      if(currentX > highpos)
        {
          currentX = highpos;
          break;
        }
    }
    oldXPos=currentX;  
  }
  
  // determine which direction to move tilt servo
  if( servoY > 150)
  {
    servoY_steps = servoY - 150;
    for(currentY = oldYPos; currentY >= (oldYPos-servoY_steps); currentY -= 1)     // tilt servoY up
    {                              
      myservoY.writeMicroseconds(currentY);                               // tell servo to go to position in variable 'currentY' 
      delayMicroseconds(2500);                                            // waits 2.5ms for the servo to reach the position 
      if(currentY < lowpos)
      {
        currentY = lowpos;
        break;
      }      
    }
    oldYPos = currentY;  
  }
  else if( servoY < 100)
  {
    servoY_steps = 100 - servoY;
    for(currentY = oldYPos; currentY <= (oldYPos+servoY_steps); currentY += 1)     // tilt servoY down
    {                              
      myservoY.writeMicroseconds(currentY);                               // tell servo to go to position in variable 'currentY' 
      delayMicroseconds(2500);                                            // waits 2.5ms for the servo to reach the position 
      if(currentY > highpos)
        {
          currentY = highpos;
          break;
        }      
    }
    oldYPos=currentY;   
  }

  // reset servo values to non-movement values
  servoX = 122;
  servoY = 125;
  
  digitalWrite(9,LOW); 
}

// idle task
void task_idle(uint32_t idle_period)
{
  // this function can perform some low-priority task while the scheduler has nothing to run.
  // It should return before the idle period (measured in ms) has expired.  For example, it
  // could sleep or respond to I/O.
  digitalWrite(12,HIGH);
  delay(idle_period);
  digitalWrite(12,LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);                                     //used for testing values only
  Serial1.begin(9600);
  Serial2.begin(115200);

  roombie.init();
  delay(1000);
  
  myservoX.attach(2);                                     // attaches the servo on pin 9 to the servo object 
  myservoX.writeMicroseconds(1500);                       // set servo to mid-point
  myservoY.attach(3);                                     // attaches the servo on pin 9 to the servo object 
  myservoY.writeMicroseconds(1500);                       // set servo to mid-point
  
  pinMode (laserPin, OUTPUT);                             // Setting laser pin as output

  // Set pins for logic analyzer testing
  pinMode(7, OUTPUT);                                     // testing task_read_bluetooth
  pinMode(8,OUTPUT);                                      // testing task_control_servo
  pinMode(9,OUTPUT);                                     // testing task_control laser
  pinMode(11,OUTPUT);                                     // testing task_drive_roomba
  pinMode(12,OUTPUT);                                     // testing idle time
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12,LOW);

  Scheduler_Init();

  Scheduler_StartTask(30,25, task_read_bluetooth);
  Scheduler_StartTask(0,50, task_control_servo);
  Scheduler_StartTask(0,70, task_drive_roomba);
  Scheduler_StartTask(0,80, task_control_laser); 
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t idle_period = Scheduler_Dispatch();
  if (idle_period)
  {
    task_idle(idle_period);
  }
}
