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

/****************************** Program Control *******************************/

#define SERIAL_ON 1
#define MOTORS_ON 1
#define TURN_DIAGNOSTICS 1
#define DRIVE_DIAGNOSTICS 1
#define HEADING_DIAGNOSTICS 1

#if SERIAL_ON
  #define PRINT(s) Serial.print(s);
  #define PRINTLN(s) Serial.println(s);
#else
  #define PRINT(s)
  #define PRINTLN(s)
#endif

/********************************** Constants *********************************/

#define RAMP_UP_SPEED 20
#define RAMP_DOWN_SPEED 10

#define TARGET_THRES 185
#define END_THRES_ULTRA 8
#define END_THRES_IR 8
#define END_THRES_FRONT_CASE 120

#define TURN_BAILOUT_TIME 4000
#define TURN_ERROR_THRESHOLD 1

/********************************** Globals ***********************************/

// Phase information.
Phase phase = PHASE_ONE;
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
float old_heading = 0;

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

// Terminate the program by detaching motors and sitting in endless loop until
// button is pressed.
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
    PRINTLN("Turning left...");
    turn_pid_ref = ((int)event.orientation.x - 90) % 360;
  } else {
    PRINTLN("Turning right...");
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

#if TURN_DIAGNOSTICS
    PRINTLN("Turn Diagnostics");
    PRINT("ref = ");
    PRINTLN(turn_pid_ref);

    PRINT("angle = ");
    PRINTLN(turn_pid_in);

    PRINT("speed = ");
    PRINTLN(turn_pid_out);

    PRINT("error = ");
    PRINTLN(error);
    PRINTLN(" ");
#endif

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
      fwdrive->stop();
      delay(500);
      fwdrive->start();
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

#if HEADING_DIAGNOSTICS
    PRINTLN("Heading Diagnostics");
    PRINT("ref = ");
    PRINTLN(turn_pid_ref);

    PRINT("angle = ");
    PRINTLN(turn_pid_in);

    PRINT("speed = ");
    PRINTLN(turn_pid_out);

    PRINT("error = ");
    PRINTLN(error);
    PRINTLN(" ");
#endif

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

void alignIRs(float aggression, long tolerance) {
  // How aggressively we want to align.
  alignPID.SetOutputLimits(-aggression, aggression);

  ir_ping_front();
  long front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
  long front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());

  while (abs(front_dist1 - front_dist2) > tolerance) {
    ir_ping_front();
    front_dist1 = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
    front_dist2 = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());

    align_pid_in = front_dist1 - front_dist2;

    alignPID.Compute();

    fwdrive->left(-align_pid_out)->right(align_pid_out);
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

#if DRIVE_DIAGNOSTICS
  PRINTLN("Drive diagnostics");
  PRINT("ref =");
  PRINTLN(drive_pid_ref);
  PRINT("angle =");
  PRINTLN(drive_pid_in);
  PRINT("control = ");
  PRINTLN(drive_pid_out);
  PRINTLN(" ");
#endif

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

// Check if the robot has mounted the base. Initiate a countdown to stop if so.
bool check_on_base() {
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
    PRINTLN("We're done; we timed out on the base.");
    stop();
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
#if SERIAL_ON
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

  // Four-wheel drive system.
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
  alignIRs(8, 2);
}

void loop() {

  waitForButton();

/********************************* Phase One **********************************/

  if (phase == PHASE_ONE) {
   switch(subphaseOne) {
    case PHASE_ONE_TO_BACK_WALL: {
      // Drive a little just to get off of Base 1.
      PIDDriveDurationNoStop(20, 50, 2000);
      subphaseOne = PHASE_ONE_WALL_FOLLOWING;
      PRINTLN("Past the first base.");
    }
    case PHASE_ONE_WALL_FOLLOWING: {

      // Monitor front distance.
      ir_ping_front();
      int front_dist = ir_avg_front_dist();
      PIDDrive(20);

      // Check if we proceed to next subphase.
      // NOTE that we need to compare against 0 in case the buffer is empty.
      if (front_dist <= 22 && front_dist > 0) {
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

      // Control loop to align with the ramp. We want to minimize the
      // difference between the two front IR sensor readings.
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

   if (abs(event.orientation.z) > 25.0) {
     phase = PHASE_TWO;
     subphaseOne = PHASE_ONE_DONE;
     PRINTLN("PHASE 2");
     PRINTLN("Ascending ramp.");
   } else {
     delay(20);
   }

/********************************* Phase Two **********************************/

  } else if (phase == PHASE_TWO) {
    int left_speed, right_speed, left_ramp_thres, right_ramp_thres;

    sensors_event_t event;
    bno.getEvent(&event);

    switch(subphaseTwo) {
      case PHASE_TWO_ASCEND_RAMP:
        if (abs(event.orientation.z) < 1.0) {
          subphaseTwo = PHASE_TWO_TOP_OF_RAMP;

          PRINTLN("At top of ramp.");

          // Tighten up these output limits for descent, which is at a slower
          // pace, and can be pretty scary.
          rampPID.SetOutputLimits(-5, 5);
          rampPID.SetTunings(3, 0.2, 0);

          left_speed = RAMP_DOWN_SPEED;
          right_speed = RAMP_DOWN_SPEED;
        } else {
          left_speed = RAMP_UP_SPEED;
          right_speed = RAMP_UP_SPEED;
          left_ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;
          right_ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;

          break;
        }
      case PHASE_TWO_TOP_OF_RAMP:
        if (abs(event.orientation.z) > 25.0) {
          subphaseTwo = PHASE_TWO_DESCEND_RAMP;

          PRINTLN("Descending ramp.");

        } else {
          left_speed = RAMP_DOWN_SPEED;
          right_speed = RAMP_DOWN_SPEED;
          left_ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;
          right_ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;
          break;
        }
      case PHASE_TWO_DESCEND_RAMP:
        if (abs(event.orientation.z) < 2.0) {
          subphaseTwo = PHASE_TWO_DONE;
          ultra->flush();
          phase = PHASE_THREE;

          PRINTLN("PHASE 3");

          break;
        } else {
          left_speed = RAMP_DOWN_SPEED;
          right_speed = RAMP_DOWN_SPEED;
          left_ramp_thres = BOTTOM_IR_RAMP_THRESHOLD;
          right_ramp_thres = 5;
          break;
        }
        break;
      default: ;
    }

    bottom_irs[LEFT]->ping();
    bottom_irs[RIGHT]->ping();

    bool left_edge = BOTTOM_IR_LEFT_SCALE(bottom_irs[LEFT]->distance()) > left_ramp_thres;
    bool right_edge = BOTTOM_IR_RIGHT_SCALE(bottom_irs[RIGHT]->distance()) > right_ramp_thres;

    ramp_pid_in = 0;
    if (left_edge) {
      ramp_pid_in = 1;
    } else if (right_edge) {
      ramp_pid_in = -1;
    }
    rampPID.Compute();

    fwdrive->left(left_speed - ramp_pid_out)->right(right_speed + ramp_pid_out);

    delay(20);

/******************************** Phase Three *********************************/

  } else {

    // Phase Three isn't broken up into subphases liek Phase One and Two.
    // That's because it was originally written by Rahul, and he didn't write
    // it in that style. Thanks, Rahul.

    // Attempt to align the robot parallel to the ramp, as it may have come off
    // of it at an angle.
    alignIRs(8, 2);

    pan_servo.write(ULTRA_CENTRE);

    // Drive forward from the ramp a bit.
    PIDDriveDuration(20, 50, 1000);

    turn90(LEFT);

    // Find that damn base. Stranded mountaineers better be grateful.
    pan_servo.write(ULTRA_LEFT);

    // Delay a bit so we get past the ramp.
    PIDDriveDurationNoStop(20, 50, 2000);

    ultra->flush();
    ir_front_flush();

    PRINTLN("Searching for Base 2.");
    while (true) {
      PIDDrive(20);

      ultra->ping();
      ir_ping_front();

      long ir_left_dist = FRONT_IR_LEFT_SCALE(front_irs[LEFT]->distance());
      long ir_right_dist = FRONT_IR_RIGHT_SCALE(front_irs[RIGHT]->distance());
      long us_dist = ultra->distance();

      PRINTLN(us_dist);

      if (us_dist < TARGET_THRES) {
        PRINTLN("Base 2 found!");

        ultra->flush();
        ir_front_flush();

        pan_servo.write(ULTRA_CENTRE);

        // Continue moving forward a bit to align properly.
        PIDDriveDuration(20, 50, 1500);

        // Turn toward the base.
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

          PRINT(ir_left_dist);
          PRINT(" ");
          PRINT(us_dist);
          PRINT(" ");
          PRINTLN(ir_right_dist);

          // We also want to stop after a short time if we're on the base but not
          // aligned to see the pole.
          check_on_base();

        } while(ir_left_dist > END_THRES_IR
                && ir_right_dist > END_THRES_IR);
        PIDDriveDone();

        return;
      } else if (ir_left_dist < 25 && ir_right_dist < 25) {
        // We've hit the wall and found nothing. Turn left to continue looking.
        PIDDriveDone();

        ultra->flush();
        ir_front_flush();

        PRINTLN("Turn left, keep searching.");

        turn90(LEFT);

        // Move forward a bit to avoid detecting the wall.
        PIDDriveDurationNoStop(20, 50, 3000);
      }

      check_on_base();

      delay(20);
    }
    PRINTLN("We've observed the base and stopped.");
    stop();
  }
}
