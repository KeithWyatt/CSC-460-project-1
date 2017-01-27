

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

const int analogXInPin = A0;        // Analog X input pin that the joystick X axis is attached to

const int analogYInPin = A1;        // Analog Y input pin that the joystick Y is attached to

const int analogLInPin = A3;        // Analog pin for light sensor

const int laserPin = 10;            // Laser pin


int sensorXValue = 0;               // value read from the joystick X
int sensorYValue = 0;               // value read from the joystick Y

int oldXValue = 10;                 // can maybe change?
int oldYValue = 10;                 // can maybe change?

int buttonPin = 19;                 // uses pin 19 for joystick button
int buttonState = 0;
int oldButtonState = 1;             // opposite of buttonState so the application can check button changes

int lightValue = 0;

Servo myservo;                      // create servo object to control a pan servo (X axis)
Servo myservoY;                     // servo object to control tilt servo ( Y axis) 
 
int lowpos = 1000;                  // lowest servo position
int highpos = 2000;                 // highest servo position
int currentX = 1500;                // current X servo position
int currentY = 1500;                // current Y servo position
int oldCurrentXPos = 1500;          // old current X servo position
int oldCurrentYPos = 1500;          // old current Y servo position

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);               // Used to type in characters

  lcd.begin(16,2);                  // initialize the lcd for 16 chars 2 lines, turn on backlight
  
  pinMode(buttonPin, INPUT_PULLUP); //for joystick button
  
  myservo.attach(2);                // attaches the servo on pin 9 to the servo object 
  myservo.writeMicroseconds(1500);  // set servo to mid-point
  myservoY.attach(3);               // attaches the servo on pin 9 to the servo object 
  myservoY.writeMicroseconds(1500); // set servo to mid-point
  
  pinMode (laserPin, OUTPUT);       // Setting laser pin as output
}
void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  
  int oldButtonState = 1;
  
  //read the analog in value for light sensor
  lightValue = analogRead(analogLInPin);    

  // display light sensor info on LCD
  lcd.setCursor(0,1);
  lcd.print("L sensor: ");
  if(lightValue > 200)
  {
    lcd.print("ON ");
  }else
  {
    lcd.print("OFF");
  }

  // determine if laser should be on or off given the state of the button
  if (buttonState == 0)
  {
   digitalWrite (laserPin, HIGH); // Turn Laser On
  }
  else
  {
   digitalWrite (laserPin, LOW); // Turn Laser off
  }  
  delay(100);

  // read the analog in value for joystick x axis:
  sensorXValue = analogRead(analogXInPin);              

  // read the analog in value for joystick y axis:
  sensorYValue = analogRead(analogYInPin);                 

  // read the button state
  buttonState = digitalRead(buttonPin);  

  // stops the LCD from updating joystick x and y axis info for small changes
  if( (oldXValue > (sensorXValue + 35)) || (oldXValue < (sensorXValue - 35)) || 
  (oldYValue > (sensorYValue + 35)) || (oldYValue < (sensorYValue - 35)) || 
  (oldButtonState != buttonState || oldYValue < 30 || oldYValue > 990 || oldXValue <30 || oldXValue > 990))
  {
     
    lcd.setCursor(0,0);
    lcd.print("X:");
    lcd.print(sensorXValue);
    lcd.setCursor(5,0);
    lcd.print(" Y:");
    lcd.print(sensorYValue);
    lcd.setCursor(13,0);
    lcd.print("B:");
    lcd.print(buttonState);
  
    // create a deadzone around joystick center
    if( sensorXValue < 400){
      // increment only 100 steps at one time     
      for(currentX = oldCurrentXPos; currentX >= (oldCurrentXPos-100); currentX -= 1)     // moves servo X position to the right
      {
                             
        myservo.writeMicroseconds(currentX);              // tell servo to go to position in variable 'currentX' 
        delayMicroseconds(2500);                          // waits 2.5ms for the servo to reach the position 
        // limit the amount the servo can move right
        if(currentX < lowpos)
        {
          currentX = lowpos;
          break;
        }
      }
      //update last position       
      oldCurrentXPos=currentX;
    }
   
    else if( sensorXValue > 600) 
    {  
      for(currentX = oldCurrentXPos; currentX <= (oldCurrentXPos+100); currentX += 1)     // moves servo X position to the left
      {                              
        myservo.writeMicroseconds(currentX);              // tell servo to go to position in variable 'currentX' 
        delayMicroseconds(2500);                          // waits 2.5ms for the servo to reach the position 
        if(currentX > highpos)
          {
            currentX = highpos;
            break;
          }
      }
      oldCurrentXPos=currentX;  
    }
    
    if( sensorYValue < 400)
    {
      for(currentY = oldCurrentYPos; currentY >= (oldCurrentYPos-100); currentY -= 1)     // moves servo X position to the left
      {                              
        myservoY.writeMicroseconds(currentY);             // tell servo to go to position in variable 'currentY' 
        delayMicroseconds(2500);                          // waits 2.5ms for the servo to reach the position 
        if(currentY < lowpos)
        {
          currentY = lowpos;
          break;
        }      
      }   
    }
    
    else if( sensorYValue > 600)
    {
      for(currentY = oldCurrentYPos; currentY <= (oldCurrentYPos+100); currentY += 1)     // moves servo X position to the left
      {                              
        myservoY.writeMicroseconds(currentY);              // tell servo to go to position in variable 'currentY' 
        delayMicroseconds(2500);                      // waits 2.5ms for the servo to reach the position 
        if(currentY > highpos)
          {
            currentY = highpos;
            break;
          }      
      }
      oldCurrentYPos=currentY;   
    }
  
    oldXValue = sensorXValue;
    oldYValue = sensorYValue;
    
    oldCurrentXPos = currentX;
    oldCurrentYPos = currentY;

  }
}/* --(end main loop )-- */

/* ( THE END ) */


