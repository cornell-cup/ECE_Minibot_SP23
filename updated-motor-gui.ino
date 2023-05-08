/* The script for Minibot motor control with the GUI.
   DC motors and Encoders 
        left: J6 port, (pwm1) digital pin 7, (pwm2) digital pin 3, (encoder) analog pin A1
        right: J5 port, (pwm1) digital pin 4, (pwm2) digital pin 6, (encoder) analog pin A0
          
   Functions: move forward/back/left/right
*/

#include <SPI.h>
#include <elapsedMillis.h>
#include <Wire.h>

elapsedMillis timeElapsed;

// Define constants for locomotion
/** Right motor drivers */
int motor0_pwmPinA = 7; // first pwm pin, controls voltage signal for speed
int motor0_pwmPinB = 3; // second pwm pin
int pwm0 = 80;      // initial pwm value (doesn't really matter)

/** Left motor drivers */
int motor1_pwmPinA = 4; // first pwm pin, controls voltage signal for speed
int motor1_pwmPinB = 6; // second pwm pin
int pwm1 = 80;      // initial pwm value

// Encoders regulate two motors to spin at same speed
// For more information, see PID algorithm on ECE documentation
int encoder0PinA = A1; // J6 motor on board
int encoder0Pos = 0; // Motor's angular position read by the encoder
int encoder0PinALast = LOW;

int encoder1PinA = A0; // J5 motor on board
int encoder1Pos = 0;
int encoder1PinALast = LOW;


int setpoint = 600; // turn rate for comparison (degrees/sec) Range: 0-800 (upper bound varies with timeSec value. Speed at pwm=255 is upper bound) 
double Integral0 = 0; // accumulated error with motors from desired number of turns
double Integral1 = 0; // accumulated error with motors from desired number of turns
int n = LOW;
int m = LOW;

int encoder0PrevCount = 0;
int lastSpeed0 = 0;
int encoder1PrevCount = 0;
int lastSpeed1 = 0;

double timeSec = 0.2; // update rate of the PID algorithm. Should match the timeElapsed < X in PID()

//PID constants
//P (proportional) is how much to adjust when turn rate is not equal to set rate. Matters most.
double kP = 0.3;
//I (integral) is how much to adjust based on accumulated error
double kI = 0;
//D (derivative) how quickly it deviates from set rate. Adjusts quicker for greater rates
double kD = 0;

// initialize the buffer
int bufSize = 4;
char buf[4];
volatile byte pos = 0;

int test;
//char buff [50]; Use multiple parameters
char updated;
volatile byte indx;
volatile boolean process;
int  interruptPin = 10;

int IRPin = 4; //S4 on J5
int in; 
int trigPin = 9; //J10 on board
int echoPin = A3; //this is the ADC pin
long duration,cm;



void setup() {  
  Serial.begin(115200);
  
// Locomotion
  pinMode (encoder0PinA, INPUT);
  pinMode (motor0_pwmPinA, OUTPUT);
  pinMode (motor0_pwmPinB, OUTPUT);

  pinMode (encoder1PinA, INPUT);
  pinMode (motor1_pwmPinA, OUTPUT);
  pinMode (motor1_pwmPinB, OUTPUT);

  pinMode(MISO,OUTPUT); //init spi
  pinMode(MOSI, INPUT);

  pinMode(20,OUTPUT);
  pinMode(22,OUTPUT);

// SPI
  SPCR |= bit (SPE); // slave control register
  indx = 0; //buffer empty
  process = false;

  pinMode(interruptPin,INPUT);
  int val = digitalRead(interruptPin);

  delay(1000);
  SPI.attachInterrupt();

  
  pinMode(IRPin, INPUT); 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  test = 0;
}

ISR (SPI_STC_vect) { //SPI Interrupt Service Routine
  Serial.println("entered ISR"); //debugging print line to indicate the beginning of the interrupt sequence
  byte c = SPDR; //read byte from SPI data register
  if (c != updated){ //if the new value does not equal the value already contained in SPDR
    Serial.println("Value has been changed"); //debugging print line to show value has changed successfully
    updated = c;// save data in the next index in the array buff
    Serial.print("ISR Value: ");
    Serial.println(updated); //debugging statement to check if value has been changed successfully
    process = true;
  }
}  


/** Adjust PWM for PID algorithm */
//add specification for PWM and pins
void adjustPWM() {
  Serial.println("in adjust PWM");
  int speedNow0 = calculateSpeed0(); // calculate the current speed for the right motor
  int error0 = setpoint - speedNow0; // calculate the error between the current speed and the set speed
  double dError0 = ((double)speedNow0 - (double)lastSpeed0) / timeSec;
  Integral0 += (double) error0; // update integral of the error

  int speedNow1 = calculateSpeed1(); // calculate the current speed for the left motor
  int error1 = setpoint - speedNow1;
  double dError1 = ((double)speedNow1 - (double)lastSpeed1) / timeSec;
  Integral1 += (double) error1;

  // cap the integral value within 0..255
  if (Integral0 > 255) Integral0 = 255;
  else if (Integral0 < 0) Integral0 = 0;

  if (Integral1 > 255) Integral1 = 255;
  else if (Integral1 < 0) Integral1 = 0;

  // calculate the value for speed adjustments
  int adjust0 = (kP * (double)error0) + kI * Integral0 + kD * dError0;
  int adjust1 = (kP * (double)error1) + kI * Integral1 + kD * dError1;

  // update pwm values according to the moving direction
  pwm0 += adjust0;
  pwm1 += adjust1;

  // cap the pwm values within 0..255
  if (pwm0 > 255) pwm0 = 255;
  else if (pwm0 < 0) pwm0 = 0;
  
  if (pwm1 > 255) pwm1 = 255;
  else if (pwm1 < 0) pwm1 = 0;

  // store the current speeds
  lastSpeed0 = speedNow0;
  lastSpeed1 = speedNow1;
  return;
}

/** Return the current rotational speed of right motor with encoder data. */
int calculateSpeed0() {
  int speedDetect = (encoder0Pos - encoder0PrevCount) / timeSec;
  encoder0PrevCount = encoder0Pos;
  return speedDetect;
}


/** Return the current rotational speed of left motor with encoder data. */
int calculateSpeed1() {
  int speedDetect = (encoder1Pos - encoder1PrevCount) / timeSec;
  encoder1PrevCount = encoder1Pos;
  return speedDetect;
}

/** Adjust the speed of motors with the PID algorithm. */
void PID(int fb) { //parameter allows to adjust pwm based off forward or backward, since there are two pwm values to account for
  
  Serial.println("in PID");
  // Adjust the rotational speeds by the calculated pwm values.
  if (fb){ //fb = 1, forward
    analogWrite(motor0_pwmPinA, pwm0);
    analogWrite(motor1_pwmPinB, pwm1);
  }
  else{
    analogWrite(motor0_pwmPinB, pwm0);
    analogWrite(motor1_pwmPinA, pwm1);
  }

   
  // Count the degrees of rotation in 0.2 seconds for each motor. 
  timeElapsed = 0;
  while ( timeElapsed < 200 ) {
    Serial.println("in while");
    n = digitalRead(encoder0PinA); // store the current digital signal of the encoder
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      // a switch from HIGH to LOW of the encoder signal marks rotation in 1 degree.
      encoder0Pos++;
    }
    encoder0PinALast = n; // update the last encoder signal for future comparison

    // same process for left encoder
    m = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (m == HIGH)) {
      encoder1Pos++;
    }
    encoder1PinALast = m;
  }
  adjustPWM();
  return;
}


void loop() {
    // clear the buffer when a command is executed
    Serial.println("process: " + String(process));
    if (process){
        process = false;
        switch(updated) { //function changes the letter value of updated to a command
          case 'F' : //fwd
            Serial.println("Forward");
            analogWrite(motor0_pwmPinB, 0); // pwma is high, so make pwmb to 0
            analogWrite(motor1_pwmPinA, 0); // pwmb 2 is high, so make pwma to 0
            PID(1);
            analogWrite(motor0_pwmPinA, pwm0); // set motor0 pwm1 to the calc value pwm0
            analogWrite(motor1_pwmPinB, pwm1); // set motor1 pwm 2 to calc value pwm1
            break; //breaks out of the switch loop and continues the original search
              
          case 'B' : //Backwards (back())
            Serial.println("back");
            analogWrite(motor0_pwmPinA, 0);
            analogWrite(motor1_pwmPinB, 0);
            PID(0);
            analogWrite(motor0_pwmPinB, pwm0);
            analogWrite(motor1_pwmPinA, pwm1);
            break; //breaks out of the switch loop and continues the original search
              
          case 'L' : //left
            Serial.println("Left");
            analogWrite(motor0_pwmPinA, 0);
            analogWrite(motor1_pwmPinA, 0);
            analogWrite(motor0_pwmPinB, 255);
            analogWrite(motor1_pwmPinB, 255);
            break; //breaks out of the switch loop and continues the original search
              
          case 'R' : //right
            Serial.println("RIGHT");
            analogWrite(motor0_pwmPinB, 0);
            analogWrite(motor1_pwmPinB, 0);
            analogWrite(motor0_pwmPinA, 255);
            analogWrite(motor1_pwmPinA, 255);
            break; //breaks out of the switch loop and continues the original search
              
          case 's' : //stop, makes all pins low
            Serial.println("Stop");
            analogWrite(motor0_pwmPinA, 0);
            analogWrite(motor1_pwmPinB, 0);
            analogWrite(motor0_pwmPinA, 0);
            analogWrite(motor1_pwmPinB, 0);
            break; //breaks out of the switch loop and continues the original search
              
         
          default: //code run when none of the cases are met
            break; //breaks out of the switch loop and continues the original search
        }
     }
}
