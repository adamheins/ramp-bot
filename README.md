# Ramp Bot
This code was written for the MTE 380 Robotic Project course.

The objective of the course was to build a robot that could traverse an
obstacle course. The obstacle course consisted of two 8 foot by 8 foot sections
separated by a 3 foot high wall. The wall had a ramp leading up and down the
wall at an angle of 30 degrees. The goal was to get from a specified point on
one side of the wall to a another specified point on the other side of wall.
Since our robot climbed the ramp.

This is (very close to) the code that ran on the robot during the competition,
which the robot completed successfully.

## Environment
For this project, we were required to use an Arduino Mega with an Atmega 2560
chip. The Arduino IDE can be downloaded
[here](https://www.arduino.cc/en/Main/Software).

## Code
### Libraries
The main entry point of the code is the `robot/robot.ino` sketch. Libraries in
use are in the `libraries` directory. Libraries are present for some of the
different sensors in use, as well as the PID control algorithm. Our own custom
library, called `Everest`, is also included.

### Testing
Software tests are located in the `tests` directory.

### Calibration
Basic calibration code for the various sensors is present in the `calib`
directory.

### Style
This isn't a huge software project so we needn't be too stringent here, but
let's not make a mess. Keep lines to a length of 80 characters. When in doubt,
follow the
[Google Style Guide](https://google.github.io/styleguide/cppguide.html).
