#include <PID_v1.h>

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

#define RAMP_DOWN_LEFT_SPEED 15
#define RAMP_DOWN_RIGHT_SPEED 15

#define WALL_DISTANCE 20
#define WALL_THRESHOLD 1

#define TARGET_THRES 200
#define END_THRES 10

#define TURN_90_SPEED 45
#define TURN_90_TIME 750

/******************* Global Variables ******************************/

Phase phase = PhaseTwo;
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
  int ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
  int ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
  return (ir_left_dist + ir_right_dist) / 2;
}

void ir_front_flush() {
  front_irs[LEFT]->flush();
  front_irs[RIGHT]->flush();
}



void turn90(Side side) {
  if (side == LEFT) {
    fwdrive->pivot(-TURN_90_SPEED);
  } else if (side == RIGHT) {
    fwdrive->pivot(TURN_90_SPEED);
  } 
  delay(TURN_90_TIME);
}

float pControl(float Kp, float ref, float meas) {
  return Kp * (ref - meas);
}

void L_find()
{
  long dist;
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
    
    PRINTLN(dist);

    if (dist < TARGET_THRES) {
      PRINTLN("target found!");
      
      pan_servo.write(90);
      fwdrive->drive(20);
      
      // Continue moving forward a bit to align.
      delay(2000);

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
        
      } while(ir_left_dist > END_THRES && ir_right_dist > END_THRES && us_dist > END_THRES);
      
      return;
    }
    
    delay(50); 
  }  
}


double pid_out;
double pid_in;
double pid_ref = WALL_DISTANCE;

double corr = 0;

// TODO make this able to return negative values from (-base_speed to +base_speed)
PID myPID(&pid_in, &pid_out, &pid_ref, 1, 0.1, 1, DIRECT);

double ramp_pid_in;
double ramp_pid_out;
double ramp_pid_ref = 0;

PID rampPID(&ramp_pid_in, &ramp_pid_out, &ramp_pid_ref, 5, 0.1, 0, DIRECT);

/******************* Setup ************************************/

void setup() {
#if TESTING
  Serial.begin(9600); 
#endif

  pinMode(BUTTON_PUSH, INPUT_PULLUP);
  pinMode(DIGITAL_GROUND, OUTPUT);
  digitalWrite(DIGITAL_GROUND, LOW);
  


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
  
   //myPID.SetMode(AUTOMATIC);
  rampPID.SetMode(AUTOMATIC);
  rampPID.SetOutputLimits(-10, 10);
}

bool run = false;



/************************** Loop ****************************/

void loop() {
  
  while (!run) {
     PRINTLN("waiting");
     run = !digitalRead(BUTTON_PUSH);
     if (run) {
       fwdrive->start();
     }
     delay(20);
  }
  
  
  if (phase == PhaseOne) {
   switch(subphaseOne) {
    case PHASE_ONE_TO_BACK_WALL: {
      fwdrive->drive(20);
      
      ir_ping_front();
      int front_dist = ir_avg_front_dist();
      
      PRINTLN(front_dist);
      
      // Next phase transition.
      if (front_dist < WALL_DISTANCE - 5) {
        PRINTLN("Subphase wall following");
        
        ir_front_flush();

        pan_servo.write(ULTRA_RIGHT);        
        turn90(LEFT);
        
        subphaseOne = PHASE_ONE_WALL_FOLLOWING;
      } else {
        break;
      }
    }
    case PHASE_ONE_WALL_FOLLOWING: {
      // Follow the wall until the robot is some distance from the side wall.

      ultra->ping();
      int wall_dist = ultra->distance();
      
      PRINTLN(wall_dist);
      
      // Negative error = we are too far from the wall - turn right
      // Positive error = we are too close to the wall - turn left
      //float Kp = 1.5;
      //float corr = pControl(Kp, WALL_DISTANCE, wall_dist);
      pid_in = wall_dist;
      if (myPID.Compute()) {
        corr = pid_out; 
      }  
      
      fwdrive->left(20 - corr)->right(20 + corr);
      
      /*
      bool too_close = wall_dist < WALL_DISTANCE - WALL_THRESHOLD;
      bool too_far = wall_dist > WALL_DISTANCE + WALL_THRESHOLD;
  
      // XXX probably should use P-control here...
      if (too_close) {
        // turn right away from wall
        PRINTLN("wall too close");
        fwdrive->left(20 - corr)->right(20 + corr);
      } else if (too_far) {
        // turn left toward wall
        PRINTLN("wall too far");
        fwdrive->left(20)->right(10);
      } else {
        PRINTLN("wall just right");
        fwdrive->left(20)->right(20);
      }*/
      
      // Monitor front distance.
      ir_ping_front();
      int front_dist = ir_avg_front_dist();
      
      // Check if we proceed to next subphase.
      //if (front_dist <= 25) {
      if (front_dist <= WALL_DISTANCE - 5) {
        stop();
        ir_front_flush();

        subphaseOne = PHASE_ONE_TURNING1;
      } else {
        break; 
      }
    }
    case PHASE_ONE_TURNING1: {
      
      // Turn left and hope the ramp is there.
      turn90(LEFT);
      
      // wall follow more bitches
      
      fwdrive->drive(20);
      
      //subphaseOne = PHASE_ONE_MOUNTING;
      subphaseOne = PHASE_ONE_WALL_FOLLOWING;
      PRINTLN("more wall following HELL YEAH");
      //PRINTLN("phase 1 mounting");
      //stop();
      break;
    }
    case PHASE_ONE_TURNING2: {
    /*
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
      if (front_dist1 > front_dist2) { // XXX this doesn't work
        stop();
        subphaseOne = PHASE_ONE_MOUNTING;
        PRINTLN("phase 1 mounting");
      } else {
        ir_left_prev = front_dist1;
        ir_right_prev = front_dist2;
        break;
      }*/
    }
    case PHASE_ONE_MOUNTING: {
      
      ir_ping_front();
      int front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      int front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
      
      PRINT(front_dist1);
      PRINT(" ");
      PRINTLN(front_dist2);
      
      // Head toward ramp.
      if (front_dist1 > front_dist2 + 20) {
        PRINTLN("turn right");
        fwdrive->left(20 - i_error)->right(10 + i_error);
      } else if (front_dist2 > front_dist1) {
        PRINTLN("turn left");
        fwdrive->left(10 - i_error)->right(20 + i_error);
      } else {
        PRINTLN("straight");
        fwdrive->left(20 - i_error)->right(20 + i_error);
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
     i_error = 0;
   }
   delay(100);
    
  } else if (phase == PhaseTwo) {
    int left_speed, right_speed;
    

    
    accel->ping();
    
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
    
    // PI control used to stay on the ramp.
    /*
    if (left_edge) {
      fwdrive->left(left_speed - i_error)->right(right_speed * 2 / 3);
    } else if (right_edge) {
      fwdrive->left(left_speed * 2 / 3)->right(right_speed + i_error);
    } else {
      fwdrive->left(left_speed - i_error)->right(right_speed + i_error);
    }*/
    
    delay(PHASE_TWO_DELAY);
    
  } else {
    // Turn left.
    fwdrive->pivot(-45);
    delay(850);
    
    // Find that damn base. Stranded mountaineers better be grateful.
    L_find();
    stop();
  }
}
