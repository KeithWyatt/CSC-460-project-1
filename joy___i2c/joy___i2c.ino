/* YourDuino.com Example Software Sketch
 16 character 2 line I2C Display
 Backpack Interface labelled "A0 A1 A2" at lower right.
 ..and
 Backpack Interface labelled "YwRobot Arduino LCM1602 IIC V1"
 MOST use address 0x27, a FEW use 0x3F
 terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include <LiquidCrystal_I2C.h>

/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

/*-----( Declare Variables )-----*/
//NONE

const int analogXInPin = A0;  // Analog X input pin that the potentiometer is attached to
const int analogXOutPin = 9; // Analog X output pin that the LED is attached to
const int analogYInPin = A1;
const int analogYOutPin = 10;
const int analogLInPin = A3;

int sensorXValue = 0;        // value read from the pot
int outputXValue = 0;        // value output to the PWM (analog out)
int sensorYValue = 0;
int outputYValue = 0;
int oldXValue = 10;
int oldYValue = 10;

int buttonPin = 19;
int buttonState = 0;
int oldButtonState = 1;

int lightValue = 0;
int outputLightValue = 0;


void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);  // Used to type in characters

  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  
  pinMode(buttonPin, INPUT_PULLUP); //for joystick button

}
void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  int oldButtonState = 1;


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

 //read the analog in value for light sensor
 lightValue = analogRead(analogLInPin);
 outputLightValue = map(lightValue,0,1023,0,255);

  buttonState = digitalRead(buttonPin);  
  
  if( (oldXValue > (sensorXValue + 25)) || (oldXValue < (sensorXValue - 25)) || 
  (oldYValue > (sensorYValue + 25)) || (oldYValue < (sensorYValue - 25)) || 
  (oldButtonState != buttonState)){
  
  oldXValue = sensorXValue;
  oldYValue = sensorYValue; 

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
  Serial.print(outputLightValue);                 

  lcd.setCursor(0,0);
  lcd.print("X:");
  lcd.print(sensorXValue);
  lcd.setCursor(5,0);
  lcd.print(" Y:");
  lcd.print(sensorYValue);
  lcd.setCursor(12,0);
  lcd.print("B:");
  lcd.print(buttonState);
  lcd.setCursor(0,1);
  lcd.print("L sensor: ");
  if(outputLightValue > 120)
  {
    Serial.print("on");
    lcd.print("ON");
  }else
  {
    Serial.print("off");
    lcd.print("OFF");
  }
  
  
  
  
  delay(2);
  }
}/* --(end main loop )-- */


/* ( THE END ) */


