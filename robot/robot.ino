// PID
#include <PID_v1.h>

// IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Everest
#include <Accelerometer.h>
#include <Counter.h>
#include <Infrared.h>
#include <Pins.h>
#include <Ultra.h>
#include <Duration.h>
#include <Drive.h>
#include <Phase.h>

#include <Servo.h>

/***************************** defines ******************************/

// Set to 1 if the robot is in test mode, 0 if in competition mode.
#define TESTING 1
#define MOTORS_ON 1

#if TESTING
  #define PRINT(s) Serial.print(s);
  #define PRINTLN(s) Serial.println(s);
#else
  #define PRINT(s)
  #define PRINTLN(s)
#endif

// TODO get rid of this mess once we've done a little more testing with the servo correction factors.

// This makes us go pretty damn straight.
#define RAMP_UP_LEFT_SPEED 20
#define RAMP_UP_RIGHT_SPEED 20

#define RAMP_DOWN_LEFT_SPEED 10
#define RAMP_DOWN_RIGHT_SPEED 10

#define WALL_DISTANCE 20
#define WALL_THRESHOLD 1

<<<<<<< HEAD
#define TARGET_THRES 150  // at most
#define END_THRES_ULTRA 5
#define END_THRES_IR 7
#define END_THRES_FRONT_CASE 120 
=======
#define TARGET_THRES 150 // At most
#define END_THRES_ULTRA 8
#define END_THRES_IR 8
#define END_THRES_FRONT_CASE 120
>>>>>>> refs/remotes/adamheins/master

#define TURN_90_SPEED 45
#define TURN_90_TIME 750

/******************* Global Variables ******************************/

Phase phase = PhaseThree;
SubphaseOne subphaseOne = PHASE_ONE_TO_BACK_WALL;
SubphaseTwo subphaseTwo = PHASE_TWO_ASCEND_RAMP;

Accelerometer *accel;

// Ultrasonic sensor.
Ultra *ultra;
Servo pan_servo;

// Four-wheel drive system.
FWDrive *fwdrive;

// Infrared sensors.
Infrared *bottom_irs[2];
Infrared *front_irs[2];

Adafruit_BNO055 bno = Adafruit_BNO055(55);

bool run = false;

/****************************** Functions ****************************/

void waitForButton();

// Terminate the program by detaching motors and sitting in endless loop.
void stop() {
  fwdrive->stop();
  run = false;
  PRINTLN("Stopped. Press button to continue...");
  waitForButton();
}

void ir_ping_front() {
  front_irs[LEFT]->ping();
  front_irs[RIGHT]->ping();
}

int ir_avg_front_dist() {
  int ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
  int ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
  return (ir_left_dist + ir_right_dist) / 2;
}

void ir_front_flush() {
  front_irs[LEFT]->flush();
  front_irs[RIGHT]->flush();
}

<<<<<<< HEAD
void L_find()
{
  long dist;
  
  // Worst Case Forward Detection
   ultra->ping();
   ir_ping_front();
        
   ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
   ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
   dist = ultra->distance();
   Serial.println("farthest distance");
   Serial.print(ir_left_dist);
   Serial.print(" ");
   Serial.print(dist);
   Serial.print(" ");
   Serial.println(ir.ir_right_dist);
   stop();
   if (dist < END_THRES_FRONT_CASE)   
    {
      do
      {
        fwdrive->drive(60)
        ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
        ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
        dist = ultra->distance();
        Serial.print(ir_left_dist);
        Serial.print(" ");
        Serial.print(dist);
        Serial.print(" ");
        Serial.println(ir.ir_right_dist);
      }while(ir_left_dist > END_THRES_FRONT_CASE && ir_right_dist > END_THRES_FRONT_CASE && us_dist > END_THRES_FRONT_CASE);
    }

  pan_servo.write(ULTRA_LEFT);
  delay(100);
  fwdrive->drive(20);
  // Delay a bit so we get past the ramp.
  delay(2000);          // tuned

  PRINTLN("finding target");
  // --------------------[O-O]-D -------|| --------// 
  while (true) {
    fwdrive->drive(20);
    
    ultra->ping();
    dist = ultra->distance();
    
    Serial.println(dist);
    //PRINTLN(dist);

    if (dist < TARGET_THRES) {
      //PRINTLN("target found!");
      Serial.println("target found!");
      pan_servo.write(90);
      fwdrive->drive(20);
      
      // Continue moving forward a bit to align.
      delay(2000);      // XXX: TUNABLE

      turn90(LEFT);
      
      // Drive until we're close to the second base.
      int ir_left_dist, ir_right_dist, us_dist;
      do {
        fwdrive->drive(20);
        delay(50);
        
        // Sample all three front sensors to see if we're near base two.
        ultra->ping();
        ir_ping_front();
        
        ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
        ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
        us_dist = ultra->distance();
        Serial.print(ir_left_dist);
        Serial.print(" ");
        Serial.print(dist);
        Serial.print(" ");
        Serial.println(ir.ir_right_dist);
      } while(ir_left_dist > END_THRES_IR && ir_right_dist > END_THRES_ULTRA && us_dist > END_THRES_IR);
      
      return;
    }
    
    delay(50); 
  }  
}

=======
>>>>>>> refs/remotes/adamheins/master
double pid_out;
double pid_in;
double pid_ref = WALL_DISTANCE;

double corr = 0;

PID myPID(&pid_in, &pid_out, &pid_ref, 1, 0.1, 1, DIRECT);

double ramp_pid_in;
double ramp_pid_out;
double ramp_pid_ref = 0;

PID rampPID(&ramp_pid_in, &ramp_pid_out, &ramp_pid_ref, 3, 0.5, 0, DIRECT);

double turn_pid_in = 0;
double turn_pid_out = 0;
double turn_pid_ref = 0;

PID turnPID(&turn_pid_in, &turn_pid_out, &turn_pid_ref, 1, 0, 0.01, DIRECT);

double align_pid_in = 0;
double align_pid_out = 0;
double align_pid_ref = 0;

PID alignPID(&align_pid_in, &align_pid_out, &align_pid_ref, 1, 0, 0, DIRECT);

double drive_pid_in = 0;
double drive_pid_out = 0;
double drive_pid_ref = 0;

PID drivePID(&drive_pid_in, &drive_pid_out, &drive_pid_ref, 1, 0.1, 0, DIRECT);

// Turn 90 deg to either the left or right.
void turn90(Side side) {
  sensors_event_t event;
  bno.getEvent(&event);

  // Set our reference 90 deg away.
  if (side == LEFT) {
    turn_pid_ref = ((int)event.orientation.x - 90) % 360;
  } else {
    turn_pid_ref = ((int)event.orientation.x + 90) % 360;
  }

  int start = millis();

  while (true) {
    bno.getEvent(&event);

    turn_pid_in = event.orientation.x;

    // Correct the angle so it produces the correct error for the PID algorithm.
    if (turn_pid_in > turn_pid_ref + 180) {
      turn_pid_in -= 360;
    } else if (turn_pid_in < turn_pid_ref - 180) {
      turn_pid_in += 360;
    }
    float error = abs(turn_pid_in - turn_pid_ref);
    turnPID.Compute();

    PRINT("ref = ");
    PRINTLN(turn_pid_ref);

    PRINT("angle = ");
    PRINTLN(turn_pid_in);

    PRINT("speed = ");
    PRINTLN(turn_pid_out);

    PRINT("error = ");
    PRINTLN(error);

    fwdrive->pivot(turn_pid_out);

    if (abs(error) < 3) {
      return;
    }

    // Bail out after 1 sec - we fucked up.
    if (millis() - start > 5000) {
      return;
    }

    delay(20);
  }
}


bool drive_pid_flag = false;

void PIDDrive(int vel) {
  sensors_event_t event;
  bno.getEvent(&event);

  // Start the drive sequence.
  if (!drive_pid_flag) {
    drive_pid_flag = true;
    drive_pid_ref = event.orientation.x;
    return;
  }

  drive_pid_in = event.orientation.x;

  // Correct the angle so it produces the correct error for the PID algorithm.
  if (drive_pid_in > drive_pid_ref + 180) {
    drive_pid_in -= 360;
  } else if (drive_pid_in < drive_pid_ref - 180) {
    drive_pid_in += 360;
  }

  drivePID.Compute();

  PRINT("ref =");
  PRINTLN(drive_pid_ref);
  PRINT("angle =");
  PRINTLN(drive_pid_in);
  PRINT("control = ");
  PRINTLN(drive_pid_out);

  fwdrive->left(vel + drive_pid_out)->right(vel - drive_pid_out);
}

void PIDDriveDone() {
  drive_pid_flag = false;
}

void PIDDriveDuration(int vel, int period, int duration) {
  int num_periods = duration / period;
  for (int i = 0; i < num_periods; i++) {
    PIDDrive(vel);
    delay(period);
  }
  PIDDriveDone();
}

// Phase 3
void L_find() {

  // Worst Case Forward Detection
  ultra->ping();
  ir_ping_front();

  long ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
  long ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
  long us_dist = ultra->distance();

  Serial.println("farthest distance");
  Serial.print(ir_left_dist);
  Serial.print(" ");
  Serial.print(us_dist);
  Serial.print(" ");
  Serial.println(ir_right_dist);

  // The base may be in front of us.
  // If so, just drive forward until we hit it.
  if (ir_left_dist < END_THRES_FRONT_CASE && ir_right_dist < END_THRES_FRONT_CASE) {
    PRINTLN("base in front of us");
    do {
      PIDDrive(20);

      ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
      us_dist = ultra->distance();

      Serial.print(ir_left_dist);
      Serial.print(" ");
      Serial.print(us_dist);
      Serial.print(" ");
      Serial.println(ir_right_dist);

      delay(50);
    } while(ir_left_dist > END_THRES_FRONT_CASE
            && ir_right_dist > END_THRES_FRONT_CASE);
    // XXX watch out for the wall...
    PIDDriveDone();
    return;
  }

  pan_servo.write(ULTRA_LEFT);

  // Delay a bit so we get past the ramp.
  PIDDriveDuration(20, 50, 2000);

  PRINTLN("finding target");
  // --------------------[O-O]-D -------|| --------//
  while (true) {
    PIDDrive(20);

    ultra->ping();
    us_dist = ultra->distance();

    PRINTLN(us_dist);

    if (us_dist < TARGET_THRES) {
      PRINTLN("target found!");
      PRINTLN(us_dist);
      PRINTLN("------");

      pan_servo.write(ULTRA_CENTRE);

      // Continue moving forward a bit to align.
      PIDDriveDuration(20, 50, 2000);

      // Turn 90 deg toward the base.
      turn90(LEFT);

      // Drive until we're close to the second base.
      int ir_left_dist, ir_right_dist, us_dist;
      do {
        PIDDrive(20);
        delay(50);

        // Sample all three front sensors to see if we're near base two.
        ultra->ping();
        ir_ping_front();

        ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
        ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
        us_dist = ultra->distance();

        Serial.print(ir_left_dist);
        Serial.print(" ");
        Serial.print(us_dist);
        Serial.print(" ");
        Serial.println(ir_right_dist);

      } while(ir_left_dist > END_THRES_IR
              && ir_right_dist > END_THRES_ULTRA
              && us_dist > END_THRES_IR);
      PIDDriveDone();

      return;
    }

    delay(50);
  }
}



/******************* Setup ************************************/

void setup() {
#if TESTING
  Serial.begin(9600);
#endif

  // Push-button start
  pinMode(BUTTON_PUSH, INPUT_PULLUP);
  pinMode(DIGITAL_GROUND, OUTPUT);
  digitalWrite(DIGITAL_GROUND, LOW);

  // IMU.
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  if (!bno.begin()) {
    PRINTLN("IMU not detected gg...");
    stop();
  }
  bno.setExtCrystalUse(true);

  // Ultrasonic.
  ultra = new Ultra(ULTRASONIC_PIN);
  pan_servo.attach(PAN_SERVO_PIN);
  pan_servo.write(ULTRA_RIGHT);

  // Accelerometer.
  accel = new Accelerometer(ACCEL_X_PIN, ACCEL_Y_PIN, ACCEL_Z_PIN);

  // IRs.
  bottom_irs[LEFT] = new Infrared(BOTTOM_IR_LEFT_PIN);
  bottom_irs[RIGHT] = new Infrared(BOTTOM_IR_RIGHT_PIN);

  front_irs[LEFT] = new Infrared(FRONT_IR_LEFT_PIN);
  front_irs[RIGHT] = new Infrared(FRONT_IR_RIGHT_PIN);

  // Start the four-wheel drive system.
  fwdrive = new FWDrive();

  // PID systems.
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-10, 10);

  rampPID.SetMode(AUTOMATIC);
  rampPID.SetOutputLimits(-10, 10);

  turnPID.SetMode(AUTOMATIC);
  turnPID.SetOutputLimits(-20, 20);

  alignPID.SetMode(AUTOMATIC);
  alignPID.SetOutputLimits(-5, 5);

  drivePID.SetMode(AUTOMATIC);
  drivePID.SetOutputLimits(-20, 20);

  PRINTLN("Setup complete.");
  PRINTLN("Press button to continue...");
}


void waitForButton() {
  while (!run) {
     run = !digitalRead(BUTTON_PUSH);
     if (run) {
       fwdrive->start();
     }
     delay(20);
  }
}

/************************** Loop ****************************/

void loop() {

  waitForButton();

  if (phase == PhaseOne) {
   switch(subphaseOne) {
    case PHASE_ONE_TO_BACK_WALL: {
      PIDDrive(20);

      ir_ping_front();
      int front_dist = ir_avg_front_dist();

      PRINTLN(front_dist);

      // Next phase transition.
      if (front_dist < WALL_DISTANCE - 5) {
        ir_front_flush();

        pan_servo.write(ULTRA_RIGHT);
        turn90(LEFT);

        subphaseOne = PHASE_ONE_WALL_FOLLOWING;

        PRINTLN("Phase 1: Wall Following");
      } else {
        break;
      }
    }
    case PHASE_ONE_WALL_FOLLOWING: {
      // Follow the wall until the robot is some distance from the side wall.

      /*
      ultra->ping();
      int wall_dist = ultra->distance();

      // Negative error = we are too far from the wall - turn right
      // Positive error = we are too close to the wall - turn left
      pid_in = wall_dist;
      if (myPID.Compute()) {
        corr = pid_out;
      }
      PRINTLN(corr);

      fwdrive->left(20 - corr)->right(20 + corr);
      */
      PIDDrive(20);

      // Monitor front distance.
      ir_ping_front();
      int front_dist = ir_avg_front_dist();

      // Check if we proceed to next subphase.
      // NOTE that we need to compare against 0 in case the buffer is empty.
      if (front_dist <= 20 && front_dist > 0) {
        ir_front_flush();

        // Turn left and hope the ramp is there.
        turn90(LEFT);

        fwdrive->drive(20);

        subphaseOne = PHASE_ONE_MOUNTING;
        PRINTLN("Phase 1: Mounting");
      } else {
        break;
      }
    }
    case PHASE_ONE_MOUNTING: {
      ir_ping_front();
      int front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      int front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());

      while (abs(front_dist1 - front_dist2) > 2) {
        ir_ping_front();
        front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
        front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());

        PRINT(front_dist1);
        PRINT(" ");
        PRINTLN(front_dist2);

        /*
        bool turn_left = front_dist2 > front_dist1;
        bool turn_right = front_dist1 > front_dist2;

        align_pid_in = 0;
        if (turn_right) {
          align_pid_in = 1;
        } else if (turn_left) {
          align_pid_in = -1;
        }*/

        align_pid_ref = 4;
        align_pid_in = front_dist1 - front_dist2;

        alignPID.Compute();

        PRINT("control = ");
        PRINTLN(align_pid_out);

        fwdrive->left(20 - align_pid_out)->right(20 + align_pid_out);

        // Head toward ramp.
        /*
        if (front_dist1 > front_dist2 + 20) {
          PRINTLN("turn right");
          fwdrive->left(20)->right(10);
        } else if (front_dist2 > front_dist1) {
          PRINTLN("turn left");
          fwdrive->left(10)->right(20);
        } else {
          PRINTLN("straight");
          fwdrive->left(20)->right(20);
        }*/
        delay(20);
      }
      PRINTLN("driving");
      PIDDriveDuration(20, 50, 3000);
      stop();
      break;
    }
    default: ;
   }

   // If the robot is on the ramp, go to phase two.
   sensors_event_t event;
   bno.getEvent(&event);

   if (abs(event.orientation.z) >= 20.0) {
     phase = PhaseTwo;
     subphaseOne = PHASE_ONE_DONE;
     PRINTLN("PHASE 2");
   } else {
     delay(50);
   }

  } else if (phase == PhaseTwo) {
    int left_speed, right_speed;

    sensors_event_t event;
    bno.getEvent(&event);

    switch(subphaseTwo) {
      case PHASE_TWO_ASCEND_RAMP:
        if (accel->z() > 425) {
          subphaseTwo = PHASE_TWO_TOP_OF_RAMP;
          PRINTLN("top of ramp");
        } else {
          left_speed = RAMP_UP_LEFT_SPEED;
          right_speed = RAMP_UP_RIGHT_SPEED;
          break;
        }
      case PHASE_TWO_TOP_OF_RAMP:
        if (accel->onSlope()) {
          subphaseTwo = PHASE_TWO_DESCEND_RAMP;
          PRINTLN("descend ramp");
        } else {
          left_speed = RAMP_DOWN_LEFT_SPEED;
          right_speed = RAMP_DOWN_RIGHT_SPEED;
          break;
        }
      case PHASE_TWO_DESCEND_RAMP:
        if (accel->onFlat()) {
          subphaseTwo = PHASE_TWO_DONE;
          phase = PhaseThree;
          PRINTLN("phase 3 start");
          break;
        } else {
          left_speed = RAMP_DOWN_LEFT_SPEED;
          right_speed = RAMP_DOWN_RIGHT_SPEED;
          break;
        }
        break;
      default: ;
    }

    bottom_irs[LEFT]->ping();
    bottom_irs[RIGHT]->ping();

    bool left_edge = BOTTOM_IR_LEFT_SCALE(bottom_irs[LEFT]->distance()) > BOTTOM_IR_RAMP_THRESHOLD;
    bool right_edge = BOTTOM_IR_RIGHT_SCALE(bottom_irs[RIGHT]->distance()) > BOTTOM_IR_RAMP_THRESHOLD;

    //PRINT(left_edge);
    //PRINT(" ");
    //PRINTLN(right_edge);

    ramp_pid_in = 0;
    if (left_edge) {
      ramp_pid_in = 1;
    } else if (right_edge) {
      ramp_pid_in = -1;
    }

    if (rampPID.Compute()) {
      corr = ramp_pid_out;
    }
    PRINTLN(corr);

    fwdrive->left(left_speed - corr)->right(right_speed + corr);

    delay(PHASE_TWO_DELAY);

  } else {
<<<<<<< HEAD
  // *************************** PHASE THREE ***********************************// 
    // Turn left.
    turn90(LEFT);
    fwdrive->drive(20);
    delay(1000);
=======
    PRINTLN("turn left");

    turn90(LEFT);

    for (int i = 0; i < 20; ++i) {
      PIDDrive(20);
      delay(50);
    }
    PIDDriveDone();

>>>>>>> refs/remotes/adamheins/master
    // Find that damn base. Stranded mountaineers better be grateful.
    L_find();
    stop();
  }
}
