#include <Servo.h>



//  FL ----- FR 
//  |        |
//  |        |
//  |        |
//  BL ----- BR


#define ULTRASONIC_PIN 7

// Panning servo
#define PAN_SERVO_PIN 8

// Driving servos
#define FL_SERVO_PIN 9
#define FR_SERVO_PIN 10
#define BL_SERVO_PIN 11
#define BR_SERVO_PIN 12

// IR Sensors
#define FRONTIR_LEFT A3
#define FRONTIR_RIGHT A2
#define BOTIR_LEFT A1
#define BOTIR_RIGHT A0

// Thresholds
#define RAMP_THRES 4.5

// Servo Corrections
#define FL_CORR -2
#define FR_CORR 0
#define BL_CORR -2
#define BR_CORR 0

float ramp_R;
float ramp_L;
int distance;
int pwr = 15;

// Servo Bin
Servo Servo_pan;
Servo Servo_FL;
Servo Servo_FR;
Servo Servo_BL;
Servo Servo_BR;


void init_Servos()
{
  Servo_FL.attach(FL_SERVO_PIN);
  Servo_FR.attach(FR_SERVO_PIN);
  Servo_BL.attach(BL_SERVO_PIN);
  Servo_BR.attach(BR_SERVO_PIN);
  //Servo_pan.attach(PAN_SERVO_PIN);
}

// TODO: Calibrate servos for straight movement
void drive(int left_pow, int right_pow)
{
  //left side servo power
  Servo_FL.write(90 + left_pow + FL_CORR);
  Servo_BL.write(90 + left_pow + BL_CORR);
  Servo_FR.write(90 - right_pow + FR_CORR);
  Servo_BR.write(90 - right_pow + BR_CORR);
}

// Take multiple readings, and average them out to reduce false readings
int irRead(int pin) {
  int averaging = 0;             //  Holds value to average readings

  // Get a sampling of 5 readings from sensor
  for (int i=0; i<5; i++) {
    distance = analogRead(pin);
    averaging = averaging + distance;
    delay(55);      // Wait 55 ms between each read
    
                    // According to datasheet time between each read
                    //  is -38ms +/- 10ms. Waiting 55 ms assures each
                    //  read is from a different sample
  } 
  distance = averaging / 5;      // Average out readings 
  return (distance);
}

void setup() {
  Serial.begin(9600);
  init_Servos();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  ramp_L = 2797.1 * pow(irRead(BOTIR_RIGHT),-1.212);
  ramp_R = 7468.9 * pow(irRead(BOTIR_LEFT),-1.374);

  if (ramp_L > RAMP_THRES)
  {
     drive (pwr*3/4,pwr);
  }

  else if (ramp_R > RAMP_THRES)
  {
    drive (pwr,pwr*3/4);
  }

  else
  {
    drive(pwr,pwr);
  }
  delay(50);
  //drive(40,40);
  
  Serial.print(ramp_L);
  Serial.print(" ");
  Serial.print(ramp_R);
  Serial.println("");
    
}
