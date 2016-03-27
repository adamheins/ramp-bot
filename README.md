# MTE 380
Software code for the MTE 380 Robot Project. This is (very close to) the code
that ran on the robot during the competition, which the robot completed
successfully.

## Environment
For this project, we are using an Arduino Mega with an Atmega 2560. The Arduino
IDE can be downloaded [here](https://www.arduino.cc/en/Main/Software).

## Structure
The main entry point of the code is the `robot` sketch. A large amount of code
for the robot can also be found in `libraries/Everest`.

### Constants
Constants are found in a couple of places, depending on the value in question.
* Pin number constants are found in `Pins.h`.
* Phase-dependent constants are found in `Phase.h`.
* Component constants are found in the header file of the component to which
  they pertain.

## Code Style
This isn't a huge software project so we needn't be too stringent here, but
let's not make a mess. Keep lines to a length of 80 characters. When in doubt,
follow the
[Google Style Guide](https://google.github.io/styleguide/cppguide.html).
