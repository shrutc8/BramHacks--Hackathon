// C++ code
//



#include <Wire.h>
#include "rgb_lcd.h"
#include <math.h>

const int buttonPin = 2;     // the number of the pushbutton pin


// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

rgb_lcd lcd;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

int counter = 0; 

const int B = 4275;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k
const int pinTempSensor = A0;     // Grove - Temperature Sensor connect to A0
const int buzzer = A1; 

// #if defined(ARDUINO_ARCH_AVR)
// #define debug  Serial
// #elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
// #define debug  SerialUSB
// #else
// #define debug  Serial
// #endif

void setup()
{
 //pinMode(2,INPUT);  
   pinMode(13,OUTPUT);
   pinMode(6, OUTPUT);


  Serial.begin(9600);
   //Serial.print("running");

  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(9, LOW);

    pinMode(buttonPin, INPUT);
}

void loop(){

	//digitalWrite(13, HIGH);

  //analogWrite(11, 255);
  //analogWrite(10, 255);
  //analogWrite(9, 255);

  int a = analogRead(pinTempSensor);


      
      digitalWrite(13, HIGH);

      if (counter < 3)
      {
        digitalWrite(6, HIGH);
        delay(600);
        digitalWrite(6, LOW);
        delay(600);
        counter++;
      }

      
    	
     
     // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    lcd.print(millis()/1000);

    buttonState = digitalRead(buttonPin);

    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    if (buttonState == HIGH) {

      digitalWrite(10, HIGH);

    }

    else {

      digitalWrite(10, LOW);
    }

   // int sensorState = digitalRead(A2);
    //Serial.println(sensorState);
    delay(100);
    

    delay(100);
}

   

  //digitalWrite(13, HIGH); //Turn the LED on
  //delay(1000); //wait one second
  //digitalWrite(13, LOW);
  //delay(1000);
