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

// uint8_t
int servoXValue = 0;            // X axis value for the joystick that controls the servo
int servoYValue = 0;            // Y axis value for the joystick that controls the servo

int oldServoXValue = 0;        // old X axis value for the joystick that controls the servo
int oldServoYValue = 0;        // old Y axis value for the joystick that controls the servo


int roombaXValue = 0;            // X axis value for the joystick that controls the servo
int roombaYValue = 0;            // Y axis value for the joystick that controls the servo


int buttonPin = 15;                 // uses pin 15 for joystick button
int buttonState = 1;
int oldButtonState = 1;             


void buttonTask()
{
  digitalWrite(37, HIGH);
  
  buttonState = digitalRead(buttonPin); 
  
  if( buttonState != oldButtonState)
  {
  
   Serial1.write(0x03);
   Serial1.write(buttonState); 
  
   oldButtonState = buttonState;
  
  }
  digitalWrite(37, LOW);
}

void idle(uint32_t idle_period)
{
	// this function can perform some low-priority task while the scheduler has nothing to run.
	// It should return before the idle period (measured in ms) has expired.  For example, it
	// could sleep or respond to I/O.
 digitalWrite(35, HIGH);
	delay(idle_period);
 digitalWrite(35, LOW);
}
  
void lightSensorTask()
{
 digitalWrite(39, HIGH);
   //read the analog in value for light sensor
  lightValue = analogRead(analogLInPin); 
 digitalWrite(39, LOW);
}
  
void roombaTask()
{

  digitalWrite(41, HIGH);
   // reads the X value from the joystick for roomba
  roombaXValue = analogRead(roombaXPin);              
  // reads the Y value from the joystick for roomba
  roombaYValue = analogRead(roombaYPin);  
  
  roombaYValue = map(roombaYValue,0,1023,0,255);
  
  roombaXValue = map(roombaXValue,0,1023,0,255);
  
      // create a deadzone around joystick center, roomba on X axis
  if( roombaXValue < 100 || roombaXValue > 150)
  {
  
      Serial1.write(0x04);
      Serial1.write(roombaXValue);
     
  }
  
    // roomba on Y axis
    if( roombaYValue < 100 || roombaYValue > 150)
    {

      Serial1.write(0x05);
      Serial1.write(roombaYValue); 
    
  }
  digitalWrite(41, LOW);
}

void servoTask()
{
 
  digitalWrite(43, HIGH);
  // reads the X value from the joystick for servo
  servoXValue = analogRead(servoXPin);  
  // reads the Y value from the joystick for servo
  servoYValue = analogRead(servoYPin); 
  
  servoYValue = map(servoYValue,0,1023,0,255);
  
  servoXValue = map(servoXValue,0,1023,0,255);

  if((servoXValue < 100) || (servoXValue > 150))
  {
    Serial1.write(0x01);
    Serial1.write(servoXValue); 
  }  
  if((servoYValue < 100) || (servoYValue > 150))
  {
    Serial1.write(0x02);
    Serial1.write(servoYValue); 
  }
  digitalWrite(43, LOW);
}
  
  
void lcdTask()
{
    
  digitalWrite(45, HIGH);

  lcd.setCursor(0,0);
  lcd.print("X:");
  lcd.print(servoXValue);
  lcd.print(' ');
  lcd.print("Y:");
  lcd.print(servoYValue);
  lcd.print(' ');
  lcd.print("B:");
  lcd.print(buttonState);
  if(servoYValue < 10)
  {
    lcd.print("     ");
  }
  else
  {
    lcd.print("  ");
  }
 
  lcd.setCursor(0,1);
  lcd.print("L sensor: ");
  if(lightValue > 200)
  {
    lcd.print("ON ");
  }else
  {
    lcd.print("OFF");
  }
 digitalWrite(45, LOW);
}
void setup()
{
  
  Serial.begin(9600);                // Don't need until remote station is sending status updates
  Serial1.begin(9600);            

  lcd.begin(16,2);                  // initialize the lcd for 16 chars 2 lines, turn on backlight

  pinMode(buttonPin, INPUT_PULLUP); //for joystick button

    pinMode(35, OUTPUT); 
    pinMode(37, OUTPUT);
    pinMode(39, OUTPUT); 
    pinMode(41, OUTPUT);
    pinMode(43, OUTPUT);
    pinMode(45, OUTPUT);

    digitalWrite(35, LOW);
    digitalWrite(37, LOW);
    digitalWrite(39, LOW);
    digitalWrite(41, LOW);
    digitalWrite(43, LOW);
    digitalWrite(45, LOW);
    
    
  Scheduler_Init();
 
  // Start task arguments are:
  // Start offset in ms, period in ms, function callback
 
  Scheduler_StartTask(0, 100, buttonTask);
  Scheduler_StartTask(35, 150, servoTask);
  Scheduler_StartTask(20, 150, roombaTask);
  Scheduler_StartTask(50, 200, lcdTask);
  Scheduler_StartTask(20, 100, lightSensorTask);
  
}
void loop()
{
   
  uint32_t idle_period = Scheduler_Dispatch();
  if (idle_period)
  {
    idle(idle_period);
  }  
  
}
