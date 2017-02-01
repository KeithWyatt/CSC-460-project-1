/*

 Phase 2 of project 1 for CSC 460
 Controlling a roomba with pan and tilt kit
 Keith Wyatt & Becky Crouteau
 January 30th 2017

*/

#include <Wire.h>  // Comes with Arduino IDE

#include <LiquidCrystal_I2C.h>
//#include "arduino/Arduino.h"
#include "scheduler.h"


LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


const int servoXPin = A0;        // X axis pin for the joystick that controls the servo
const int servoYPin = A1;        // Y axis pin for the joystick that controls the servo


const int roombaXPin = A2;        // X axis pin for the joystick that controls the roomba
const int roombaYPin = A3;        // Y axis pin for the joystick that controls the roomba


const int analogLInPin = A7;        // Analog pin for light sensor
int lightValue = 0;


int servoXValue = 0;            // X axis value for the joystick that controls the servo
int servoYValue = 0;            // Y axis value for the joystick that controls the servo

int oldServoXValue = 0;        // old X axis value for the joystick that controls the servo
int oldServoYValue = 0;        // old Y axis value for the joystick that controls the servo


int roombaXValue = 0;            // X axis value for the joystick that controls the servo
int roombaYValue = 0;            // Y axis value for the joystick that controls the servo


int buttonPin = 15;                 // uses pin 15 for joystick button
int buttonState = 0;
int oldButtonState = 1;             // opposite of buttonState so the application can check button changes

 char buf[2] = {'a',1};
// buf = (char*) malloc(char*2);

void setup()
{
  
  Serial.begin(9600);
  Serial1.begin(9600);

  lcd.begin(16,2);                  // initialize the lcd for 16 chars 2 lines, turn on backlight

  pinMode(buttonPin, INPUT_PULLUP); //for joystick button

  Scheduler_Init();
 
  // Start task arguments are:
  // Start offset in ms, period in ms, function callback
 
  Scheduler_StartTask(0, 100, buttonTask);
  //Scheduler_StartTask(0, 300, pulse_pin2_task);
}

void buttonTask()
{
  
 buttonState = digitalRead(buttonPin); 
  
 if( Serial.available())
{

 Serial1.print(buf); 
}
//  NOT SURE WHY WE NEED THIS PART

if( Serial1.available())
 {
  Serial.print((char)Serial1.read()); 
 }  
  
}

void idle(uint32_t idle_period)
{
	// this function can perform some low-priority task while the scheduler has nothing to run.
	// It should return before the idle period (measured in ms) has expired.  For example, it
	// could sleep or respond to I/O.
 
	delay(idle_period);

}
  
void lightSensorTask()
{
 
   //read the analog in value for light sensor
  lightValue = analogRead(analogLInPin); 
  
}
  
  
void roombaTask()
{

   // reads the X value from the joystick for roomba
  roombaXValue = analogRead(roombaXPin);              
  // reads the Y value from the joystick for roomba
  roombaYValue = analogRead(roombaYPin);  
  
  
      // create a deadzone around joystick center, roomba going right
  if( roombaXValue < 400)
  {
  
    if( Serial.available())
    {

      Serial1.print("b");
      Serial1.print(roombaXValue); 
    }
/*  NOT SURE WHY WE NEED THIS PART

if( Serial1.available())
 {
  Serial.print(Serial1.read()); 
 }  
  */
    
  // roomba going right
  }else if( roombaXValue > 600) 
  {  
    if( Serial.available())
    {

      Serial1.print("c");
      Serial1.print(roombaXValue); 
    }
/*  NOT SURE WHY WE NEED THIS PART

if( Serial1.available())
 {
  Serial.print(Serial1.read()); 
 }  
  */
    
  }
    // roomba going backwards
    if( roombaYValue < 400)
    {
     if( Serial.available())
    {

      Serial1.print("d");
      Serial1.print(roombaYValue); 
    }
/*  NOT SURE WHY WE NEED THIS PART

if( Serial1.available())
 {
  Serial.print(Serial1.read()); 
 }  
  */
    }
    
    // roomba going forwards
    else if( roombaYValue > 600)
    {
        if( Serial.available())
    {

      Serial1.print("e");
      Serial1.print(roombaYValue); 
    }
/*  NOT SURE WHY WE NEED THIS PART

if( Serial1.available())
 {
  Serial.print(Serial1.read()); 
 }  
  */
    }
  
    oldServoXValue = servoXValue;
    oldServoYValue = servoYValue;
    oldButtonState = buttonState;

  }

void servoTask()
{
 
  // reads the X value from the joystick for servo
  servoXValue = analogRead(servoXPin);              
  // reads the Y value from the joystick for servo
  servoYValue = analogRead(servoYPin);  
  
}
  
void lcdTask()
{
  /*
  oldYValue < 30 || oldYValue > 990 || oldXValue <30 || oldXValue > 990)
  MIGHT NEED TO ADD BACK INTO THE IF STATEMENT
 
  // stops the LCD from updating joystick x and y axis info for small changes 
  if( (oldServoXValue > (servoXValue + 5)) || (oldServoXValue < (servoXValue - 5)) || 
  (oldServoYValue > (servoYValue + 5)) || (oldServoYValue < (servoYValue - 5)) ||   
  (oldButtonState != buttonState))
  {
  */  

  lcd.setCursor(0,0);
  lcd.print("X:");
  lcd.print(servoXValue);
  lcd.setCursor(5,0);
  lcd.print(" Y:");
  lcd.print(servoYValue);
  lcd.print(' ');
  lcd.setCursor(13,0);
  lcd.print("B:");
  lcd.print(buttonState);
 
 
  lcd.setCursor(0,1);
  lcd.print("L sensor: ");
  if(lightValue > 200)
  {
    lcd.print("ON ");
  }else
  {
    lcd.print("OFF");
  }

 
}

void loop()
{
   
  uint32_t idle_period = Scheduler_Dispatch();
  if (idle_period)
  {
    idle(idle_period);
  }  
  
}
