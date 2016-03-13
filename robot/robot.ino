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
//#define RAMP_UP_LEFT_SPEED 15
//#define RAMP_UP_RIGHT_SPEED 23
#define RAMP_UP_LEFT_SPEED 20
#define RAMP_UP_RIGHT_SPEED 20

//#define RAMP_DOWN_LEFT_SPEED 10
//#define RAMP_DOWN_RIGHT_SPEED 18
#define RAMP_DOWN_LEFT_SPEED 5
#define RAMP_DOWN_RIGHT_SPEED 5

#define WALL_DISTANCE 30
#define WALL_THRESHOLD 4

#define TARGET_THRES 100
#define END_THRES 10

/************** Global Variables ******************************/

Phase phase = PhaseThree;
SubphaseOne subphaseOne = PHASE_ONE_WALL_FOLLOWING;
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

int ir_left_prev = 0;
int ir_right_prev = 0;

float i_error = 0;

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

void L_find()
{
  long dist;
  pan_servo.write(ULTRA_LEFT);
  delay(100);
  fwdrive->drive(20);
  
  // Delay a bit so we get past the ramp.
  delay(1000);          // tuned

  PRINTLN("finding target");
  // --------------------[O-O]-D -------|| --------// 
  while (true) {
    fwdrive->drive(20);
    
    ultra->ping();
    dist = ultra->distance();
    
    PRINTLN(dist);

    if (dist < TARGET_THRES) {
      PRINTLN("target found!");
      
      fwdrive->drive(20);
      delay(1250);
      
      pan_servo.write(90);
      fwdrive->pivot(-45);
      delay(850);
      
      // Drive until we're close to the second base.
      do {
        ultra->ping();
        fwdrive->drive(20);
        delay(100);
      } while((ultra->distance()) > END_THRES);
      
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
  
#if MOTORS_ON
  fwdrive->start();
#endif
}

/************************** Loop ****************************/

void loop() {
   
  if (phase == PhaseOne) {
   switch(subphaseOne) {
    case PHASE_ONE_TO_BACK_WALL: {
      fwdrive->drive(20);
      
      ir_ping_front();
      int front_dist = ir_avg_front_dist();
      
      PRINTLN(front_dist);
      
      if (front_dist < WALL_DISTANCE - 5) {
        PRINTLN("Subphase wall following");
        fwdrive->pivot(-50);
        subphaseOne = PHASE_ONE_WALL_FOLLOWING;
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
        fwdrive->left(20 * 2 / 3)->right(20);
      } else if (too_far) {
        // turn left toward wall
        PRINTLN("wall too far");
        fwdrive->left(20)->right(20 * 2 / 3);
      } else {
        PRINTLN("wall just right");
        fwdrive->left(20)->right(20);
      }
      
      // Monitor front distance.
      ir_ping_front();
      int front_dist = ir_avg_front_dist();
      
      // Check if we proceed to next phase.
      if (front_dist <= 20) {
        subphaseOne = PHASE_ONE_TURNING1;
      } else {
        break; 
      }
    }
    case PHASE_ONE_TURNING1: {

      // Turn until the front IR sensors are getting quite different values.
      // Then we can move into the next phase to start monitoring for the ramp.
      /*int front_dist1, front_dist2;
      do {
        ir_ping_front();
        front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
        front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
        fwdrive->pivot(-10);
        delay(50);
      } while (abs(front_dist1 - front_dist2) < 5);

      subphaseOne = PHASE_ONE_MOUNTING;
      ir_left_prev = front_dist1;
      ir_right_prev = front_dist2;*/
      fwdrive->pivot(-45);
      delay(850);
      subphaseOne = PHASE_ONE_MOUNTING;
      PRINTLN("phase 1 mounting");
    }
    case PHASE_ONE_TURNING2: {

      ir_ping_front();
      int front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      int front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
      
      PRINT(front_dist1);
      PRINT(" ");
      PRINTLN(front_dist2);
      
      fwdrive->pivot(-10);
      
      // When do we stop:
      // The IRs are reading very similar values
      // OR
      // The IR values have actually missed the ramp and continued circling.
      //if (abs(front_dist1 - front_dist2) < 2 front_dist1 > front_dist2 && ir_left_prev < front_dist2) {
      if (front_dist1 > front_dist2) {
        stop();
        subphaseOne = PHASE_ONE_MOUNTING;
        PRINTLN("phase 1 mounting");
      } else {
        ir_left_prev = front_dist1;
        ir_right_prev = front_dist2;
        break;
      }
    }
    case PHASE_ONE_MOUNTING: {
      
      ir_ping_front();
      int front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      int front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
      
      PRINT(front_dist1);
      PRINT(" ");
      PRINTLN(front_dist2);
      
      // Do bang-bang control to keep the robot headed toward the ramp.
      if (front_dist1 > front_dist2 + 2) {
        // turn right
        fwdrive->left(20)->right(20 * 2 / 3);
      } else if (front_dist2 > front_dist1 + 2) {
        // turn left
        fwdrive->left(20 * 2 / 3)->right(20);
      } else {
        fwdrive->left(20)->right(20);
      }
      break;
    }
    default: ;
   }
   
   // If the robot is on the ramp, go to phase two.
   accel->ping();
   if (accel->onSlope()) {
     phase = PhaseTwo;
     subphaseOne = PHASE_ONE_DONE;
     PRINTLN("Enter phase 2");
   } else {
     delay(PHASE_ONE_DELAY);
   }
    
  } else if (phase == PhaseTwo) {
    int left_speed, right_speed;
    
    accel->ping();
    
    switch(subphaseTwo) {
      case PHASE_TWO_ASCEND_RAMP:
        if (accel->onFlat()) {
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
    
    // PI control used to stay on the ramp.
    if (left_edge) {
      i_error -= 0.25;
      fwdrive->left(left_speed - i_error)->right(right_speed * 3 / 4 + i_error);
    } else if (right_edge) {
      i_error += 0.25;
      fwdrive->left(left_speed * 2 / 3 - i_error)->right(right_speed + i_error);
    } else {
      //i_error *= 0.5;
      i_error = 0;
      fwdrive->left(left_speed - i_error)->right(right_speed + i_error);
    }
    
    delay(PHASE_TWO_DELAY);
    
  } else {
    L_find();
    stop();
  }
}
