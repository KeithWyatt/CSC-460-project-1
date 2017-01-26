

/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE

#include <LiquidCrystal_I2C.h>
#include <Servo.h> 

/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x3F for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

/*-----( Declare Variables )-----*/

const int analogXInPin = A0;  // Analog X input pin that the potentiometer is attached to
const int analogXOutPin = 9; // Analog X output pin that the LED is attached to
const int analogYInPin = A1; // Analog Y input pin
const int analogYOutPin = 10;  // Analog Y output pin
const int analogLInPin = A3;  // Analog pin for light sensor


int sensorXValue = 0;        // value read from the pot
int outputXValue = 0;        // value output to the PWM (analog out)

int sensorYValue = 0;
int outputYValue = 0;

int oldXValue = 10;        // can maybe change?
int oldYValue = 10;        // can maybe change?

int buttonPin = 19;        // uses pin 19 for joystick button
int buttonState = 0;
int oldButtonState = 1;    // opposite of buttonState so the application can check button changes

int lightValue = 0;
int outputLightValue = 0;

Servo myservo;  // create servo object to control a servo 

//int defaultpos = 1500;    // variable to store the servo position 
int lowpos = 1000;        // lowest servo position
int highpos = 2000;      // highest servo position
int currentX = 0;          // current X servo position
int currentY = 0;          // current Y servo position
int oldCurrentXPos = 1500;    // old current X servo position
int oldCurrentYPos = 1500;    // old current Y servo position



void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);  // Used to type in characters

  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  
  pinMode(buttonPin, INPUT_PULLUP); //for joystick button
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.writeMicroseconds(1500);  // set servo to mid-point


}
void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  int oldButtonState = 1;

  //read the analog in value for light sensor
  lightValue = analogRead(analogLInPin);
  //outputLightValue = map(lightValue,0,1023,0,255);
  Serial.print(lightValue);
  Serial.println();
  lcd.setCursor(0,1);
  lcd.print("L sensor: ");
  if(lightValue > 0)
  {
    Serial.print("on ");
    lcd.print("ON ");
  }else
  {
    Serial.print("off ");
    lcd.print("OFF");
  }
  
  delay(100);

  // read the analog in value:
  sensorXValue = analogRead(analogXInPin);            
  // map it to the range of the analog out:
  outputXValue = map(sensorXValue, 0, 1023, 0, 255);  
  // change the analog out value:
  analogWrite(analogXOutPin, outputXValue);        

  // read the analog in value:
  sensorYValue = analogRead(analogYInPin);            
  // map it to the range of the analog out:
  outputYValue = map(sensorYValue, 0, 1023, 0, 255);  
  // change the analog out value:
  analogWrite(analogYOutPin, outputYValue);  
 

  buttonState = digitalRead(buttonPin);  
  
  if( (oldXValue > (sensorXValue + 25)) || (oldXValue < (sensorXValue - 25)) || 
  (oldYValue > (sensorYValue + 25)) || (oldYValue < (sensorYValue - 25)) || 
  (oldButtonState != buttonState || oldYValue < 30 || oldYValue > 990 || oldXValue <30 || oldXValue > 990)){
  

  // print the results to the serial monitor:
  Serial.print("X: " );                       
  Serial.print(sensorXValue);      
/*  Serial.print("\t X-output = ");      
  Serial.print(outputXValue);   
  */
    // print the results to the serial monitor:
  Serial.print(" | Y: " );                       
  Serial.print(sensorYValue);      
/*  Serial.print("\t Y-output = ");      
  Serial.println(outputYValue); 
*/
  Serial.print(" | B: ");
  Serial.println(buttonState);
  
  lcd.setCursor(0,0);
  lcd.print("X:");
  lcd.print(sensorXValue);
  lcd.setCursor(5,0);
  lcd.print(" Y:");
  lcd.print(sensorYValue);
  lcd.setCursor(12,0);
  lcd.print("B:");
  lcd.print(buttonState);

/*
  if( sensorXValue < 497){
    for(currentX = oldCurrentXPos; currentX >= (sensorXValue + 1000); currentX -= 1)     // moves servo X position to the left
  {                                
    myservo.writeMicroseconds(currentX);              // tell servo to go to position in variable 'currentX' 
    delayMicroseconds(2500);                       // waits 2.5ms for the servo to reach the position 
  }   
  }
 
  if( sensorXValue > 497){
    for(currentX = oldCurrentXPos; currentX <= (sensorXValue + 1000); currentX += 1)     // moves servo X position to the right
  {                                
    myservo.writeMicroseconds(currentX);              // tell servo to go to position in variable 'currentX' 
    delayMicroseconds(2500);                       // waits 2.5ms for the servo to reach the position 
  }   
  }
  
/*  if( sensorYValue < 508){
    for(currentY = oldCurrentYPos; currentY >= (sensorYValue + 1000); currentY -= 1)     // moves servo Y position to the down
  {                                
    myservo.writeMicroseconds(currentY);              // tell servo to go to position in variable 'currentY' 
    delayMicroseconds(2500);                       // waits 2.5ms for the servo to reach the position 
  }   
  }
  
  if( sensorYValue > 508){
    for(currentY = oldCurrentYPos; currentY <= (sensorYValue + 1000); currentY += 1)     // moves servo X position to the right
  {                                
    myservo.writeMicroseconds(currentY);              // tell servo to go to position in variable 'currentY' 
    delayMicroseconds(2500);                       // waits 2.5ms for the servo to reach the position 
  }   
  }
 */
  oldXValue = sensorXValue;
  oldYValue = sensorYValue;
  
  oldCurrentXPos = currentX;
  oldCurrentYPos = currentY;
  
  delay(2);
  }
}/* --(end main loop )-- */

//void updateLightSensor(lightValue)
//{
//  if(lightValue > 200)
//  {
//    Serial.print("on ");
//    lcd.print("ON ");
//  }else
//  {
//    Serial.print("off ");
//    lcd.print("OFF");
//  }  
//}
/* ( THE END ) */


