#include <Accelerometer.h>
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

#if TESTING
  #define PRINT(s) Serial.print(s);
  #define PRINTLN(s) Serial.println(s);
#else
  #define PRINT(s)
  #define PRINTLN(s)
#endif


#define LEFT_SERVO_CORR 0
#define RIGHT_SERVO_CORR 0
#define VELOCITY 15

/*
#define LEFT_SERVO_CORR -2
#define RIGHT_SERVO_CORR 8
#define VELOCITY 5
*/

#define WALL_DISTANCE 30
#define WALL_THRESHOLD 4

/************** Global Variables ******************************/

Phase phase = PhaseOne;
SubphaseOne subphase_one = PHASE_ONE_TO_BACK_WALL;

Accelerometer *accel;

// Ultrasonic sensor.
Ultra *ultra;
Servo pan_servo;

// Four-wheel drive system.
FWDrive *fwdrive;

// Infrared sensors.
Infrared *bottom_irs[2];
Infrared *front_irs[2];

int ir_left_prev = 0;
int ir_right_prev = 0;

/****************************** Functions ****************************/

// Terminate the program by detaching motors and sitting in endless loop.
void stop() {
  fwdrive->stop();
  while(1) {
    delay(60000); // 1 min
  }
}

void ir_ping_front() {
  front_irs[LEFT]->ping();
  front_irs[RIGHT]->ping();
}

int ir_avg_front_dist() {
  int front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
  int front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
  return (front_dist1 + front_dist2) / 2;
}

/******************* Setup ************************************/

void setup() {
#if TEST
  Serial.begin(9600); 
#endif 

  ultra = new Ultra(ULTRASONIC_PIN);
  pan_servo.attach(PAN_SERVO_PIN);
  pan_servo.write(ULTRA_RIGHT);
  
  accel = new Accelerometer(ACCEL_X_PIN, ACCEL_Y_PIN, ACCEL_Z_PIN);
  
  bottom_irs[LEFT] = new Infrared(BOTTOM_IR_LEFT_PIN);
  bottom_irs[RIGHT] = new Infrared(BOTTOM_IR_RIGHT_PIN);
  
  front_irs[LEFT] = new Infrared(FRONT_IR_LEFT_PIN);
  front_irs[RIGHT] = new Infrared(FRONT_IR_RIGHT_PIN);

  // Start the four-wheel drive system.
  fwdrive = new FWDrive();
  fwdrive->start();
}

/************************** Loop ********8*******************/

void loop() {
    
  if (phase == PhaseOne) {
   switch(subphase_one) {
    case PHASE_ONE_TO_BACK_WALL: {
      fwdrive->drive(20);
      
      ir_ping_front();
      int front_dist = ir_avg_front_dist();
      
      PRINTLN(front_dist);
      
      if (front_dist < WALL_DISTANCE - 5) {
        PRINTLN("Subphase wall following");
        fwdrive->pivot(-50);
        subphase_one = PHASE_ONE_WALL_FOLLOWING;
        pan_servo.write(ULTRA_RIGHT);
        delay(750);
      } else {
        break;
      }
    }
    case PHASE_ONE_WALL_FOLLOWING: {
      // Follow the wall until the robot is some distance from the side wall.

      ultra->ping();
      int wall_dist = ultra->distance();
      
      int error = abs(wall_dist - WALL_DISTANCE);
      
      bool too_close = wall_dist < WALL_DISTANCE - WALL_THRESHOLD;
      bool too_far = wall_dist > WALL_DISTANCE + WALL_THRESHOLD;
  
      // P-control
      // Baseline: left(15) right(20)
      if (too_close) {
        // turn right away from wall
        PRINTLN("wall too close");
        fwdrive->left(15)->right(20 * error / 4);
      } else if (too_far) {
        // turn left toward wall
        PRINTLN("wall too far");
        fwdrive->left(15 * error / 4)->right(20);
      } else {
        PRINTLN("wall just right");
        fwdrive->left(15)->right(20);
      }
      
      // Monitor front distance.
      ir_ping_front();
      int front_dist = ir_avg_front_dist();
      
      // Check if we proceed to next phase.
      if (front_dist <= 20) {
        subphase_one = PHASE_ONE_TURNING1;
      } else {
        break; 
      }
    }
    case PHASE_ONE_TURNING1: {

      // Turn until the front IR sensors are getting quite different values.
      // Then we can move into the next phase to start monitoring for the ramp.
      int front_dist1, front_dist2;
      do {
        ir_ping_front();
        front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
        front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
        fwdrive->pivot(-20);
        delay(50);
      } while (abs(front_dist1 - front_dist2) < 5);

      subphase_one = PHASE_ONE_TURNING2;
      break;
    }
    case PHASE_ONE_TURNING2: {

      ir_ping_front();
      int front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      int front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
      
      PRINT(front_dist1);
      PRINT(" ");
      PRINTLN(front_dist2);
      
      fwdrive->pivot(-20);
      
      // When do we stop:
      // The IRs are reading very similar values
      // OR
      // The IR values have actually missed the ramp and continued circling.
      if (abs(front_dist1 - front_dist2) < 2 || front_dist1 > front_dist2 && ir_left_prev < front_dist2) {
        subphase_one = PHASE_ONE_MOUNTING;
      } else {
        ir_left_prev = front_dist1;
        ir_right_prev = front_dist2;
        break;
      }
    }
    case PHASE_ONE_MOUNTING: {
      
      int front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      int front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
      
      // Do bang-bang control to keep the robot headed toward the ramp.
      if (front_dist1 > front_dist2 + 2) {
        // turn right
        fwdrive->left(15)->right(20 * 2 / 3);
      } else if (front_dist2 > front_dist1 + 2) {
        // turn left
        fwdrive->left(15 * 2 / 3)->right(30);
      } else {
        fwdrive->left(15)->right(20);
      }
      break;
    }
    default: ;
   } 
    
   delay(PHASE_ONE_DELAY);
    
  } else if (phase == PhaseTwo) {
    
    //accel->ping();
    //int z = accel->z();
    //if (accel_counter->countIf(z > 425 && z < 435)) {
      // we are no longer on the ramp
    //}
    //Serial.println(accel->z());
    
    bottom_irs[LEFT]->ping();
    bottom_irs[RIGHT]->ping();
  
    bool left_edge = BOTTOM_IR_LEFT_SCALE(bottom_irs[LEFT]->distance()) > BOTTOM_IR_RAMP_THRESHOLD;
    bool right_edge = BOTTOM_IR_RIGHT_SCALE(bottom_irs[RIGHT]->distance()) > BOTTOM_IR_RAMP_THRESHOLD;
    
    if (left_edge) {
      fwdrive->left(VELOCITY + LEFT_SERVO_CORR)->right((VELOCITY + RIGHT_SERVO_CORR) * 3 / 4);
    } else if (right_edge) {
      fwdrive->left((VELOCITY + LEFT_SERVO_CORR) * 3 / 4)->right(VELOCITY + RIGHT_SERVO_CORR);
    } else {
      fwdrive->left(VELOCITY + LEFT_SERVO_CORR)->right(VELOCITY + RIGHT_SERVO_CORR);
    }
    
    delay(PHASE_TWO_DELAY);
    
  } else {
    
    // TODO use the lfind algorithm

    delay(PHASE_THREE_DELAY);
  }
}
