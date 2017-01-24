/*
  Analog input, analog output, serial output
 
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.
 
 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground
 
 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
const int analogXInPin = A0;  // Analog X input pin that the potentiometer is attached to
const int analogXOutPin = 9; // Analog X output pin that the LED is attached to
const int analogYInPin = A1;
const int analogYOutPin = 10;

int sensorXValue = 0;        // value read from the pot
int outputXValue = 0;        // value output to the PWM (analog out)
int sensorYValue = 0;
int outputYValue = 0;

int buttonPin = 19;
int buttonState = 0;


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  pinMode(buttonPin, INPUT_PULLUP);
  
}

void loop() {
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
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);                     
}
