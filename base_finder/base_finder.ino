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
#define TARGET_THRES 50
#define END_THRES 10

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


bool target_locked = false;
bool target_found =  true;
int target_angle;

void init_Servos()
{
  Servo_FL.attach(FL_SERVO_PIN);
  Servo_FR.attach(FR_SERVO_PIN);
  Servo_BL.attach(BL_SERVO_PIN);
  Servo_BR.attach(BR_SERVO_PIN);
  Servo_pan.attach(PAN_SERVO_PIN);
}

void stop_Servos()
{
  Servo_FL.detach();
  Servo_FR.detach();
  Servo_BL.detach();
  Servo_BR.detach();
  Servo_pan.detach();
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

void turn_left()
{
  drive(-50,50);
  delay(850);
}

void L_find()
{
  long dist;
  Servo_pan.write(185);
  delay(100);
  drive(20,20);
  delay(1000);          // tuned
  stop_Servos();
  init_Servos();
  while (true)
  {
    drive(60,60);     
    dist = US_read();
    //Serial.println(dist);
    if (dist < TARGET_THRES)
    {
      drive(20,20);
      delay(1250);
      Servo_pan.write(90);
      turn_left();
      while(US_read() > END_THRES)
      {
        drive(20,20);
        delay(500);
      }
      stop_Servos();
      return;
    }
    delay(50); 
  }  

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

int US_read()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long us_distance, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_PIN, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ULTRASONIC_PIN, INPUT);
  return us_distance = pulseIn(ULTRASONIC_PIN, HIGH) / 58;
}


int US_sweepSearch()
{
  long dist;
  int ang; 
  for(ang = 0; ang <= 90; ang = ang + 10)
  {
    Servo_pan.write(90 + ang);
    dist = US_read();
    Serial.print("TRYING ANGLE :");
    Serial.println(90 + ang);
    if (dist < TARGET_THRES)
    {
     target_locked = true;
     Serial.println("TARGET LOCKED"); 
     return (90 + ang);
    }
    else
    {
      Serial.println("FINDING TARGET");
      target_locked = false;
    }
    delay(500 + 10 * ang);
    
    Servo_pan.write(90 - ang);
    dist = US_read();
    Serial.print("TRYING ANGLE :");
    Serial.println(90 - ang);
    if (dist < TARGET_THRES)
    {
     target_locked = true;
     Serial.println("TARGET LOCKED"); 
     return (90 - ang);
    }
    else
    {
      Serial.println("FINDING TARGET");
      target_locked = false;
    }
    delay(500 + 10 * ang);
  }
}

void setup() {
  Serial.begin(9600);
  init_Servos();
  
}
  
void loop() 
{
  /*
  long us_distance;
  
  us_distance = US_read();

  if (us_distance > TARGET_THRES)
  {
    target_locked == false;
    Serial.println("Initializing Search Ultrasonic...");
    target_angle = US_sweepSearch();
  }
  else
  {
    Serial.print("Target Angle: ");
    Serial.print(target_angle);
    Serial.print(" Target Distance: ");
    Serial.println(us_distance);
    Serial.println("IM IN THE ELSE");
  }  
  // put your main code here, to run repeatedly:
  ramp_L = 2797.1 * pow(irRead(BOTIR_RIGHT),-1.212);
  ramp_R = 7468.9 * pow(irRead(BOTIR_LEFT),-1.374);      
  */
  if (target_found == true)
  {
    turn_left();
    stop_Servos();
    init_Servos();
    L_find();
    target_found = false;
  }
  
}
