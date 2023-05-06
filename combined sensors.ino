#include <HCSR04.h>
#include <Servo.h>

UltraSonicDistanceSensor distanceSensor(6, 5);  // initializes sensor that uses digital pins 6(trig) and 5(echo)
Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position
int sensorPin = 11; //variable to store the IR sensor

void setup() {
  Serial.begin(9600);  // initializes serial connection so that we could print values from sensor
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(sensorPin,INPUT); // initializes the IR sensor as an input 
  pinMode(13,OUTPUT);
}

void loop() {
    if(distanceSensor.measureDistanceCm()<10.00){
       for (pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees
         // in steps of 1 degree
         myservo.write(pos);              // tell servo to go to position in variable 'pos'
         delay(15);                       // waits 15ms for the servo to reach the position
       }
       if(!digitalRead(sensorPin)){ 
         Serial.println("light up led");
         digitalWrite(13, HIGH);
         delay(1000);
       }
       for (pos = 90; pos <= 180; pos += 1) { // goes from 180 degrees to 0 degrees
         myservo.write(pos);              // tell servo to go to position in variable 'pos'
         delay(15);                       // waits 15ms for the servo to reach the position
       }
       for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
         myservo.write(pos);              // tell servo to go to position in variable 'pos'
         delay(15);                       // waits 15ms for the servo to reach the position
       }
       if(!digitalRead(sensorPin)){ 
         Serial.println("turn off led");
         digitalWrite(13, LOW);
         delay(1000);
       }
       for (pos = 90; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
         myservo.write(pos);              // tell servo to go to position in variable 'pos'
         delay(15);                       // waits 15ms for the servo to reach the position
       }
     }
 
}
