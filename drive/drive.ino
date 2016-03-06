#include <Infrared.h>
#include <Pins.h>
#include <Ultra.h>
#include <Duration.h>
#include <Drive.h>
#include <Phase.h>

#include <Servo.h>

/*
  // dank panning
  pan_angle += pan_inc;
  pan_servo.write(pan_angle);
  if (pan_angle >= 180) {
    pan_inc = -1;
  } else if (pan_angle <= 0) {
    pan_inc = 1;
  }
*/

#define LEFT_SERVO_CORR -5
#define RIGHT_SERVO_CORR 0
#define VELOCITY 15

#define WALL_DISTANCE 10
#define WALL_THRESHOLD 2

Phase phase = PhaseThree;

// Ultrasonic sensor.
Ultra *ultra;
Servo pan_servo;

// Four-wheel drive system.
FWDrive *fwdrive;

// Duration tracker.
Duration *dur;

// Infrared sensors.
Infrared *bottom_irs[2];
Infrared *front_irs[2];

// Terminate the program by detaching motors and sitting in endless loop.
void stop() {
  fwdrive->stop();
  while(1) {
    delay(60000); // 1 min
  }
}

void setup() {
  Serial.begin(9600);  

  ultra = new Ultra(ULTRASONIC_PIN);
  pan_servo.attach(PAN_SERVO_PIN);
  pan_servo.write(90);
  
  bottom_irs[LEFT] = new Infrared(BOTTOM_IR_LEFT_PIN);
  bottom_irs[RIGHT] = new Infrared(BOTTOM_IR_RIGHT_PIN);
  
  front_irs[LEFT] = new Infrared(FRONT_IR_LEFT_PIN);
  front_irs[RIGHT] = new Infrared(FRONT_IR_RIGHT_PIN);

  dur = new Duration(0);

  // Start the four-wheel drive system.
  fwdrive = new FWDrive();
  fwdrive->start();
}

// Phase 3 globals
bool found_base_2 = false;
int base2_dist = 0;
int past_turn = 1; // 1 is right, -1 is left

void loop() {
  
  if (phase == PhaseOne) {
    
    // Monitor wall distance.
    ultra->ping();
    int wall_dist = ultra->distance();
    
    bool too_close = wall_dist < WALL_DISTANCE - WALL_THRESHOLD;
    bool too_far = wall_dist > WALL_DISTANCE + WALL_THRESHOLD;

    if (too_close) {
      fwdrive->left(VELOCITY + LEFT_SERVO_CORR)->right(VELOCITY * 3 / 4 + RIGHT_SERVO_CORR);
    } else if (too_far) {
      fwdrive->left(VELOCITY * 3 / 4 + LEFT_SERVO_CORR)->right(VELOCITY + RIGHT_SERVO_CORR);
    } else {
      fwdrive->left(VELOCITY + LEFT_SERVO_CORR)->right(VELOCITY + RIGHT_SERVO_CORR);
    }
    
    // Monitor front distance.
    front_irs[LEFT]->ping();
    front_irs[RIGHT]->ping();
    
    int front_dist1 = front_irs[LEFT]->distance();
    int front_dist2 = front_irs[RIGHT]->distance();
    int front_dist = (front_dist1 + front_dist2) / 2;
    
    Serial.print(front_dist1);
    Serial.print("  ");
    Serial.print(front_dist2);
    Serial.print("  ");
    Serial.println(front_dist);
    
    delay(PHASE_ONE_DELAY);
    
  } else if (phase == PhaseTwo) {
    
    bottom_irs[LEFT]->ping();
    bottom_irs[RIGHT]->ping();
  
    bool left_edge = (7468.9 * pow(bottom_irs[LEFT]->distance(), -1.374) > BOTTOM_IR_RAMP_THRESHOLD);
    bool right_edge = (2797.1 * pow(bottom_irs[RIGHT]->distance(), -1.212) > BOTTOM_IR_RAMP_THRESHOLD);
    
    if (left_edge) {
      fwdrive->left(VELOCITY + LEFT_SERVO_CORR)->right(VELOCITY * 3 / 4 + RIGHT_SERVO_CORR);
    } else if (right_edge) {
      fwdrive->left(VELOCITY * 3 / 4 + LEFT_SERVO_CORR)->right(VELOCITY + RIGHT_SERVO_CORR);
    } else {
      fwdrive->left(VELOCITY + LEFT_SERVO_CORR)->right(VELOCITY + RIGHT_SERVO_CORR);
    }
  
    delay(PHASE_TWO_DELAY);
    
  } else {

    ultra->ping();
    int dist = ultra->distance();

    if (!found_base_2) {
      fwdrive->pivot(5);
      if (dist < 50) {
        EdgeSide edge = ultra->edge();
        if (edge != EdgeNone) {
           found_base_2 = true;
        }
        if (edge == EdgeRecent) {
          base2_dist = ultra->recent();
        } else if (edge == EdgeOld) {
          base2_dist = ultra->old(); 
        }
      }
    } else {
      // Need to "edge-follow"
      // This is tricky because the distance is constantly changing, and the edge itself is transient
      EdgeSide edge = ultra->edge();
      if (edge == EdgeRecent) {
        
      } else if (edge == EdgeOld) {
        
      } else {
        if (dist < base2_dist) {
          base2_dist = dist; 
        } else {
          Serial.println("ERROR"); 
        }
      }
      if (dist > base2_dist) {
        // turn toward base (right)
      } else {
        base2_dist = dist;
      }
      fwdrive->drive(10); // TODO need control to go in correct direction
    }

    // pivot robot
    // monitor US for edges
    // move forward when we find one
    delay(PHASE_THREE_DELAY);
  }

  if (dur->tick()) {
    stop();
  }
}
