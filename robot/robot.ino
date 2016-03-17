// PID
#include <PID_v1.h>

// IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Everest
#include <Infrared.h>
#include <Pins.h>
#include <Ultra.h>
#include <Duration.h>
#include <Drive.h>
#include <Phase.h>

#include <Servo.h>

/********************************** #defines **********************************/

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

// TODO get rid of this mess once we've done a little more testing with the
// servo correction factors.

// This makes us go pretty damn straight.
#define RAMP_UP_LEFT_SPEED 20
#define RAMP_UP_RIGHT_SPEED 20

#define RAMP_DOWN_LEFT_SPEED 10
#define RAMP_DOWN_RIGHT_SPEED 10

// XXX deprecated
#define WALL_DISTANCE 20
#define WALL_THRESHOLD 1

#define TARGET_THRES 200 // Keep this very tight
#define END_THRES_ULTRA 8
#define END_THRES_IR 8
#define END_THRES_FRONT_CASE 120

#define TURN_BAILOUT_TIME 4000
#define TURN_ERROR_THRESHOLD 3

/********************************** Globals ***********************************/

// Phase information.
Phase phase = PhaseThree;
SubphaseOne subphaseOne = PHASE_ONE_TO_BACK_WALL;
SubphaseTwo subphaseTwo = PHASE_TWO_ASCEND_RAMP;

// Ultrasonic sensor.
Ultra *ultra;
Servo pan_servo;

// Four-wheel drive system.
FWDrive *fwdrive;

// Infrared sensors.
Infrared *bottom_irs[2];
Infrared *front_irs[2];

// IMU.
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// True if the robot is running, false otherwise.
bool run = false;

// True if the robot is PID driving in a straigh line, false otherwise.
bool drive_pid_flag = false;

bool base_found_to_the_right = false;

// Stores an old heading orientation that we want to use later.
float old_heading = 0; // XXX deprecated

unsigned long on_base_time = 0;
bool on_base = false;

/************************************ PID *************************************/

double ramp_pid_in = 0;
double ramp_pid_out = 0;
double ramp_pid_ref = 0;

PID rampPID(&ramp_pid_in, &ramp_pid_out, &ramp_pid_ref, 5, 0.1, 0, DIRECT);

double turn_pid_in = 0;
double turn_pid_out = 0;
double turn_pid_ref = 0;

PID turnPID(&turn_pid_in, &turn_pid_out, &turn_pid_ref, 1, 0, 0.01, DIRECT);

double align_pid_in = 0;
double align_pid_out = 0;
double align_pid_ref = 0;

PID alignPID(&align_pid_in, &align_pid_out, &align_pid_ref, 1, 0.1, 0, DIRECT);

double drive_pid_in = 0;
double drive_pid_out = 0;
double drive_pid_ref = 0;

PID drivePID(&drive_pid_in, &drive_pid_out, &drive_pid_ref, 1, 1, 0.1, DIRECT);

/********************************* Functions **********************************/

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

void turn90(Side side) {
  sensors_event_t event;
  bno.getEvent(&event);

  // Set our reference 90 deg away.
  if (side == LEFT) {
    turn_pid_ref = ((int)event.orientation.x - 90) % 360;
  } else {
    turn_pid_ref = ((int)event.orientation.x + 90) % 360;
  }

  unsigned long start = millis();

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

    // Diagnostic information.
    PRINTLN("turn info");
    PRINT("ref = ");
    PRINTLN(turn_pid_ref);

    PRINT("angle = ");
    PRINTLN(turn_pid_in);

    PRINT("speed = ");
    PRINTLN(turn_pid_out);

    PRINT("error = ");
    PRINTLN(error);
    PRINTLN(" ");

    fwdrive->pivot(turn_pid_out);

    // Once the orientation error is small enough, we consider the turn
    // complete. Sometimes the turn never actually reaches this low of an
    // error, so we hit the failsafe, below.
    if (abs(error) < TURN_ERROR_THRESHOLD) {
      return;
    }

    // Bail out after some time - we fucked up, we but we ain't done yet.
    if (millis() - start > TURN_BAILOUT_TIME) {
      PRINT("Turn failsafe hit after: ");
      PRINT(millis() - start);
      PRINTLN(" ms.");
      return;
    }

    delay(20);
  }
}

void turnToHeading(float toHeading) {
  turn_pid_ref = toHeading;

  unsigned long start = millis();

  while (true) {
    sensors_event_t event;
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

    /*
    PRINT("ref = ");
    PRINTLN(turn_pid_ref);

    PRINT("angle = ");
    PRINTLN(turn_pid_in);

    PRINT("speed = ");
    PRINTLN(turn_pid_out);

    PRINT("error = ");
    PRINTLN(error);
    PRINTLN(" ");
    */

    fwdrive->pivot(turn_pid_out);

    // Once the orientation error is small enough, we consider the turn
    // complete. Sometimes the turn never actually reaches this low of an
    // error, so we hit the failsafe, below.
    if (abs(error) < TURN_ERROR_THRESHOLD) {
      return;
    }

    // Bail out after some time - we fucked up, we but we ain't done yet.
    if (millis() - start > TURN_BAILOUT_TIME) {
      PRINT("Turn failsafe hit after: ");
      PRINT(millis() - start);
      PRINTLN(" ms.");
      return;
    }

    delay(20);
  }
}

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

  PRINTLN("drive info");
  PRINT("ref =");
  PRINTLN(drive_pid_ref);
  PRINT("angle =");
  PRINTLN(drive_pid_in);
  PRINT("control = ");
  PRINTLN(drive_pid_out);
  PRINTLN(" ");

  fwdrive->left(vel + drive_pid_out)->right(vel - drive_pid_out);
}

// Must call after PID driving is done.
void PIDDriveDone() {
  drive_pid_flag = false;
}

// PID drive for a specific duration.
void PIDDriveDuration(int vel, int period, int duration) {
  int num_periods = duration / period;
  for (int i = 0; i < num_periods; i++) {
    PIDDrive(vel);
    delay(period);
  }
  PIDDriveDone();
}

void PIDDriveDurationNoStop(int vel, int period, int duration) {
  int num_periods = duration / period;
  for (int i = 0; i < num_periods; i++) {
    PIDDrive(vel);
    delay(period);
  }
}

bool check_on_base() {
  // We also want to stop after a short time if we're on the base but not
  // aligned to see the pole.
  sensors_event_t event;
  bno.getEvent(&event);

  // Detect if we've just mounted the base. This triggers a countdown to
  // stop the robot.
  if (!on_base && (abs(event.orientation.z) > 12 || abs(event.orientation.y) > 8)) {
    on_base = true;
    on_base_time = millis();
  }

  // We've been on the base for long enough. Stop.
  if (on_base && millis() > on_base_time + 400) {
    PRINTLN("We're done, we timed out on the base.");
    stop();
  }
}

// Phase 3
void L_find() {
  PIDDrive(20);

  ultra->flush();
  ir_front_flush();

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

  pan_servo.write(ULTRA_LEFT);

  // Delay a bit so we get past the ramp.
  //PIDDriveDuration(20, 50, 2000);
  for (int i = 0; i < 40; i++) {
    PIDDrive(20);
    delay(50);
  }

  ultra->flush();
  ir_front_flush();

  PRINTLN("Searching for Base 2.");
  while (true) {
    PIDDrive(20);

    ultra->ping();
    ir_ping_front();

    ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
    ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
    us_dist = ultra->distance();

    PRINTLN(us_dist);

    if (us_dist < TARGET_THRES) {
      PRINTLN("Base 2 found!");

      PIDDriveDone();

      //pan_servo.write(ULTRA_RIGHT);

      ultra->flush();
      ir_front_flush();

      // Continue moving forward a bit to align properly.
      PIDDriveDuration(20, 50, 1500);

      // Turn 90 deg toward the base. Assume we're either pointed right at the
      // base or are two far to the right of it, since the ultrasonic provides
      // a conical reading. Search to the right side to handle this case.
      turn90(LEFT);

      // Drive until we're close to the second base, or we see the base to our
      // right side.
      do {
        PIDDrive(20);
        delay(20);

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

        // We also want to stop after a short time if we're on the base but not
        // aligned to see the pole.
        check_on_base();

      } while(ir_left_dist > END_THRES_IR
              && ir_right_dist > END_THRES_IR);
      PIDDriveDone();

      return;
    } else if (ir_left_dist < 20 && ir_right_dist < 20) {
      // We've hit the wall and found nothing. Turn left to continue looking.
      PIDDriveDone();

      ultra->flush();
      ir_front_flush();

      PRINTLN("Turn left, keep searching.");
      turn90(LEFT);
    }

    check_on_base();

    delay(20);
  }
}

void waitForButton() {
  while (!run) {
    run = !digitalRead(BUTTON_PUSH_PIN);
     if (run) {
#if MOTORS_ON
       fwdrive->start();
#endif
     }
     delay(20);
  }
}

/*********************************** setup ************************************/

void setup() {
#if TESTING
  Serial.begin(9600);
#endif

  // Push-button start
  pinMode(BUTTON_PUSH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DIGITAL_GROUND_PIN, OUTPUT);
  digitalWrite(BUTTON_DIGITAL_GROUND_PIN, LOW);

  // IMU.
  pinMode(IMU_DIGITAL_HIGH_PIN, OUTPUT);
  digitalWrite(IMU_DIGITAL_HIGH_PIN, HIGH);
  if (!bno.begin()) {
    PRINTLN("IMU not detected. Reset to continue...");
    stop();
  }
  bno.setExtCrystalUse(true);

  // Ultrasonic.
  ultra = new Ultra(ULTRASONIC_PIN);
  pan_servo.attach(PAN_SERVO_PIN);
  pan_servo.write(ULTRA_RIGHT);

  // IRs.
  bottom_irs[LEFT] = new Infrared(BOTTOM_IR_LEFT_PIN);
  bottom_irs[RIGHT] = new Infrared(BOTTOM_IR_RIGHT_PIN);

  front_irs[LEFT] = new Infrared(FRONT_IR_LEFT_PIN);
  front_irs[RIGHT] = new Infrared(FRONT_IR_RIGHT_PIN);

  // Start the four-wheel drive system.
  fwdrive = new FWDrive();

  // PID systems.
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

/************************************ loop ************************************/

bool flag = false;

void test() {
  PIDDrive(20);
  delay(20);
}

void loop() {

  waitForButton();

/********************************* Phase One **********************************/

  if (phase == PhaseOne) {
   switch(subphaseOne) {
    case PHASE_ONE_TO_BACK_WALL: {
      /*
      PIDDrive(20);

      ir_ping_front();
      int front_dist = ir_avg_front_dist();

      PRINTLN(front_dist);

      // Next phase transition.
      if (front_dist < WALL_DISTANCE - 5) {
        ir_front_flush();

        PIDDriveDone();

        pan_servo.write(ULTRA_RIGHT);
        turn90(LEFT);

        subphaseOne = PHASE_ONE_WALL_FOLLOWING;

        PRINTLN("Phase 1: Wall Following");
      } else {
        break;
      }*/

      // Drive a little just to get off of Base 1.
      PIDDriveDurationNoStop(20, 50, 2000);
      subphaseOne = PHASE_ONE_WALL_FOLLOWING;
      PRINTLN("Past the first base.");
    }
    case PHASE_ONE_WALL_FOLLOWING: { // XXX we're not wall following anymore

      // Monitor front distance.
      ir_ping_front();
      int front_dist = ir_avg_front_dist();

      // Check if we proceed to next subphase.
      // NOTE that we need to compare against 0 in case the buffer is empty.
      if (front_dist <= 20 && front_dist > 0) {
        ir_front_flush();

        PIDDriveDone();

        // Turn left and hope the ramp is there.
        turn90(LEFT);

        // Store this heading for use later.
        sensors_event_t event;
        bno.getEvent(&event);
        old_heading = event.orientation.x;

        // Should be fine without PIDDrive since we just want to stop turning
        // and move toward the ramp. The PID loop in the next subphase should
        // deal with this.
        fwdrive->drive(20);

        subphaseOne = PHASE_ONE_MOUNTING;
        PRINTLN("Phase 1: Mounting");
      } else {
        break;
      }
    }
    case PHASE_ONE_MOUNTING: {
      ir_ping_front();
      long front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      long front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());

      align_pid_in = front_dist1 - front_dist2;

      alignPID.Compute();

      fwdrive->left(20 - align_pid_out)->right(20 + align_pid_out);

      break;
    }
    default: ;
   }

   // If the robot is on the ramp, go to phase two.
   sensors_event_t event;
   bno.getEvent(&event);
   PRINTLN(event.orientation.z);

   if (abs(event.orientation.z) > 25.0) {
     phase = PhaseTwo;
     subphaseOne = PHASE_ONE_DONE;
     PRINTLN("PHASE 2");
     PRINTLN("Ascending ramp.");
   } else {
     delay(20);
   }

/********************************* Phase Two **********************************/

  } else if (phase == PhaseTwo) {
    int left_speed, right_speed, ramp_thres;

    sensors_event_t event;
    bno.getEvent(&event);
    //PRINTLN(event.orientation.z);

    switch(subphaseTwo) {
      case PHASE_TWO_ASCEND_RAMP:
        if (abs(event.orientation.z) < 1.0) {
          subphaseTwo = PHASE_TWO_TOP_OF_RAMP;

          PRINTLN("At top of ramp.");
          //PRINT("z angle = ");
          //PRINTLN(event.orientation.z);

          // Tighten up these output limits for descent, which is at a slower pace.
          rampPID.SetOutputLimits(-5, 5);
          rampPID.SetTunings(2, 0.1, 0);

          left_speed = RAMP_DOWN_LEFT_SPEED;
          right_speed = RAMP_DOWN_RIGHT_SPEED;
        } else {
          left_speed = RAMP_UP_LEFT_SPEED;
          right_speed = RAMP_UP_RIGHT_SPEED;
          ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;

          break;
        }
      case PHASE_TWO_TOP_OF_RAMP:
        if (abs(event.orientation.z) > 25.0) {
          subphaseTwo = PHASE_TWO_DESCEND_RAMP;

          PRINTLN("Descending ramp.");
          //PRINT("z angle = ");
          //PRINTLN(event.orientation.z);
        } else {
          left_speed = RAMP_DOWN_LEFT_SPEED;
          right_speed = RAMP_DOWN_RIGHT_SPEED;
          ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;
          break;
        }
      case PHASE_TWO_DESCEND_RAMP:
        if (abs(event.orientation.z) < 2.0) {
          subphaseTwo = PHASE_TWO_DONE;
          ultra->flush();
          phase = PhaseThree;

          PRINTLN("PHASE 3");
          //PRINT("z angle = ");
          //PRINTLN(event.orientation.z);
          break;
        } else {
          left_speed = RAMP_DOWN_LEFT_SPEED;
          right_speed = RAMP_DOWN_RIGHT_SPEED;
          ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;
          break;
        }
        break;
      default: ;
    }

    bottom_irs[LEFT]->ping();
    bottom_irs[RIGHT]->ping();

    bool left_edge = BOTTOM_IR_LEFT_SCALE(bottom_irs[LEFT]->distance()) > ramp_thres;
    bool right_edge = BOTTOM_IR_RIGHT_SCALE(bottom_irs[RIGHT]->distance()) > ramp_thres;

    /*
    PRINT(BOTTOM_IR_LEFT_SCALE(bottom_irs[LEFT]->distance()));
    PRINT(" ");
    PRINTLN(BOTTOM_IR_RIGHT_SCALE(bottom_irs[RIGHT]->distance()));
    */

    ramp_pid_in = 0;
    if (left_edge) {
      ramp_pid_in = 1;
    } else if (right_edge) {
      ramp_pid_in = -1;
    }
    rampPID.Compute();

    PRINT("ramp pid out = ");
    PRINTLN(ramp_pid_out);

    fwdrive->left(left_speed - ramp_pid_out)->right(right_speed + ramp_pid_out);

    delay(20);

/******************************** Phase Three *********************************/

  } else {

    // Turn to the heading we had before attempting to mount the ramp. This
    // should be pretty straight, and we may not be super straight coming off
    // the ramp.
    //turnToHeading(old_heading);

    // XXX output limits may not be enough
    ir_ping_front();
    long front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
    long front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
    while (abs(front_dist1 - front_dist2) > 2) {
      ir_ping_front();
      front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());

      align_pid_in = front_dist1 - front_dist2;

      alignPID.Compute();

      fwdrive->left(-align_pid_out)->right(align_pid_out);
      delay(20);
    }

    pan_servo.write(ULTRA_CENTRE);

    // Drive forward from the ramp a bit.
    PIDDriveDuration(20, 50, 1000);

    PRINTLN("now turning left");
    turn90(LEFT);

    // Find that damn base. Stranded mountaineers better be grateful.
    L_find();
    PRINTLN("We've observed the base and stopped.");
    stop();
  }
}
