#include <Buffer.h>
#include <Pins.h>
#include <Counter.h>
#include <Infrared.h>
#include <Phase.h>
#include <Ultra.h>
#include <Duration.h>
#include <Accelerometer.h>
#include <Drive.h>

#include <Servo.h>

// Simple sketch to test ultrasonic sensor and if it is too high.

Ultra *ultra;
Servo pan_servo;

void setup() {
  Serial.begin(9600); 

  ultra = new Ultra(ULTRASONIC_PIN);
  pan_servo.attach(PAN_SERVO_PIN);
  pan_servo.write(0);
}

void loop() {
 ultra->ping();
 Serial.println(ultra->distance());
 delay(100); 
}
