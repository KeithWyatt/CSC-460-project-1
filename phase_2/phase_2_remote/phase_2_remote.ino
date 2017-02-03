/*
 * CSC460
 * Project 1 - Phase 2
 * Remote station pan & tilt servos and laser respond
 * to base station commands sent via bluetooth
 * 
 */

#include <Servo.h>

#include <scheduler.h>

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>



const int laserPin = 4;             // Laser pin (pwm pin 4)

int servoX = 122;                   // value read from the joystick X
int servoY = 125;                   // value read from the joystick Y

int buttonState = 1;                // button is not pressed

volatile int hw_addr;               // variable to store which device to control
volatile int hw_instr;              // variable to store an instruction to give the device





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
  digitalWrite(8,HIGH);

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
    default:
      // If the hardware address is unknown, do nothing.
      break;
  }
  
  digitalWrite(8,LOW);
}

void task_control_laser()
{
  digitalWrite(9,HIGH);

  // determine if laser should be on or off given the state of the button
  if (buttonState == 0)
  { 
   digitalWrite(laserPin, HIGH);                // Turn Laser On
  }
  else
  {
   digitalWrite (laserPin, LOW);               // Turn Laser off
  }
  
  digitalWrite(9,LOW);  
}

void task_control_servo()
{
  digitalWrite(10,HIGH);
  
  // determine which direction to move pan servo
  if( servoX > 150){
    
    // increment only 100 steps at one time     
    for(currentX = oldXPos; currentX >= (oldXPos-100); currentX -= 1)     // moves servo X position to the left
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
    for(currentX = oldXPos; currentX <= (oldXPos+100); currentX += 1)     // moves servo X position to the right
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
    for(currentY = oldYPos; currentY >= (oldYPos-100); currentY -= 1)     // tilt servoY up
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
    for(currentY = oldYPos; currentY <= (oldYPos+100); currentY += 1)     // tilt servoY down
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
  
  digitalWrite(10,LOW); 
}

void task_drive_roomba()
{
  //controls to drive roomba forward, back, left, and right
}

// idle task
void task_idle(uint32_t idle_period)
{
  // this function can perform some low-priority task while the scheduler has nothing to run.
  // It should return before the idle period (measured in ms) has expired.  For example, it
  // could sleep or respond to I/O.

  delay(idle_period);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);                                     //used for testing values only
  Serial1.begin(9600);

  myservoX.attach(2);                                     // attaches the servo on pin 9 to the servo object 
  myservoX.writeMicroseconds(1500);                       // set servo to mid-point
  myservoY.attach(3);                                     // attaches the servo on pin 9 to the servo object 
  myservoY.writeMicroseconds(1500);                       // set servo to mid-point
  
  pinMode (laserPin, OUTPUT);                             // Setting laser pin as output

  // Set pins for logic analyzer testing
  pinMode(8, OUTPUT);                                     // testing task_read_bluetooth
  pinMode(9,OUTPUT);                                      // testing task_control_servo
  pinMode(10,OUTPUT);                                     // testing task_control laser
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);

  Scheduler_Init();

  Scheduler_StartTask(0,100, task_read_bluetooth);
  //Scheduler_StartTask(0,100, task_drive_roomba);
  Scheduler_StartTask(30, 100, task_control_laser);
  Scheduler_StartTask(12, 100, task_control_servo);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t idle_period = Scheduler_Dispatch();
  if (idle_period)
  {
    task_idle(idle_period);
  }
}
