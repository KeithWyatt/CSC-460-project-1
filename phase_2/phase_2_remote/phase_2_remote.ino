#include <Servo.h>

//#include <arduino/Arduino.h>
#include <scheduler.h>

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>


//
const int laserPin = 4;            // Laser pin (pwm pin 10)
//
volatile uint8_t servoX = 122;               // value read from the joystick X
volatile uint8_t servoY = 125;               // value read from the joystick Y
//
//int oldXValue = 10;                 // can maybe change?
//int oldYValue = 10;                 // can maybe change?
//
//int buttonState = 0;
//int oldButtonState = 1;             // opposite of buttonState so the application can check button changes

volatile int hw_addr;
volatile int hw_instr;

uint8_t buttonState = 1;

//uint8_t instr_togg = 0;


Servo myservoX;                      // create servo object to control a pan servo (X axis)
Servo myservoY;                     // servo object to control tilt servo ( Y axis) 
 
int lowpos = 1000;                  // lowest servo position
int highpos = 2000;                 // highest servo position
int currentX = 1500;                // current X servo position
int currentY = 1500;                // current Y servo position
int oldXPos = 1500;          // old current X servo position
int oldYPos = 1500;          // old current Y servo position
uint8_t servo_value;

//TESTING 
//void task_send_bluetooth()
//{
//  Serial2.write(0x01);
//  Serial2.write(instr_togg);
// 
//  instr_togg ^= 1;
//  //Serial.print(instr_togg);
//}

void task_read_bluetooth()
{
  digitalWrite(8,HIGH);
//  if(Serial1.available())
//  {

    hw_addr = Serial1.read();
    hw_instr = Serial1.read();
    Serial.print(hw_addr);
    Serial.println();
    Serial.print(hw_instr);
    //delay(100);
    
    
    //Serial.print(int(hw_addr));
    switch(hw_addr)
    {
      // Servo pan (x axis)
      case 1:
        if(hw_instr != servoX)
        {
          servoX = hw_instr;
        }

        Serial.write(servoX);
        break;
      // Servo tilt (y axis)
      case 2:
        if(hw_instr != servoY)
        {
          servoY = hw_instr;
        }

        break;
      case 3:
        //Serial.print(buttonState);
//        if(hw_instr != buttonState)
//        {
          buttonState = hw_instr;
//        }      
        break;
      default:
        break;
    }
//  }
  digitalWrite(8,LOW);
}

void task_read_joystick_button()
{
  digitalWrite(9,HIGH);
  //Serial.print(buttonState);
  // determine if laser should be on or off given the state of the button
  if (buttonState == 0)
  {
    
   digitalWrite(laserPin, HIGH); // Turn Laser On
  }
  else
  {
   digitalWrite (laserPin, LOW); // Turn Laser off
  }
  digitalWrite(9,LOW);  
}

void task_control_servo()
{
    digitalWrite(10,HIGH);
    // create a deadzone around joystick center
    if( servoX > 150){
      // increment only 100 steps at one time     
      for(currentX = oldXPos; currentX >= (oldXPos-100); currentX -= 1)     // moves servo X position to the right
      {
                             
        myservoX.writeMicroseconds(currentX);              // tell servo to go to position in variable 'currentX' 
        delayMicroseconds(2500);                          // waits 2.5ms for the servo to reach the position 
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
      for(currentX = oldXPos; currentX <= (oldXPos+100); currentX += 1)     // moves servo X position to the left
      {                              
        myservoX.writeMicroseconds(currentX);              // tell servo to go to position in variable 'currentX' 
        delayMicroseconds(2500);                          // waits 2.5ms for the servo to reach the position 
        if(currentX > highpos)
          {
            currentX = highpos;
            break;
          }
      }
      oldXPos=currentX;  
    }
    
    if( servoY > 150)
    {
      for(currentY = oldYPos; currentY >= (oldYPos-100); currentY -= 1)     // moves servo X position to the left
      {                              
        myservoY.writeMicroseconds(currentY);             // tell servo to go to position in variable 'currentY' 
        delayMicroseconds(2500);                          // waits 2.5ms for the servo to reach the position 
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
      for(currentY = oldYPos; currentY <= (oldYPos+100); currentY += 1)     // moves servo X position to the left
      {                              
        myservoY.writeMicroseconds(currentY);              // tell servo to go to position in variable 'currentY' 
        delayMicroseconds(2500);                      // waits 2.5ms for the servo to reach the position 
        if(currentY > highpos)
          {
            currentY = highpos;
            break;
          }      
      }
      oldYPos=currentY;   
    }
  
//    oldXValue = servoX;
//    oldYValue = servoY;
    
//    oldXPos = currentX;
//    oldYPos = currentY; 
    digitalWrite(10,LOW); 
}

void task_drive_roomba()
{
  
}

// idle task
void task_idle(uint32_t idle_period)
{
  // this function can perform some low-priority task while the scheduler has nothing to run.
  // It should return before the idle period (measured in ms) has expired.  For example, it
  // could sleep or respond to I/O.
  if(Serial.available())
  {
//    Serial1.print((char)Serial.read());
    Serial.print("bye");
  }  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  //Serial2.begin(9600);
  myservoX.attach(2);                // attaches the servo on pin 9 to the servo object 
  myservoX.writeMicroseconds(1500);  // set servo to mid-point
  myservoY.attach(3);               // attaches the servo on pin 9 to the servo object 
  myservoY.writeMicroseconds(1500); // set servo to mid-point
  
  pinMode (laserPin, OUTPUT);       // Setting laser pin as output

  pinMode(8, OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);

  Scheduler_Init();

  Scheduler_StartTask(0,100, task_read_bluetooth);
  //Scheduler_StartTask(0,100, task_drive_roomba);
  Scheduler_StartTask(30, 100, task_read_joystick_button);
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

int main()
{
  init();
  setup();

  for (;;)
  {
    loop();
  }
  for (;;);
  return 0;
}

