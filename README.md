# MTE 380
Software code for the MTE 380 Robot Project.

## Contributing
To contribute to the repository, do the following:

1. Clone the repository.
2. Make a new branch for your work. A descriptive branch name is preferred.
3. Push the branch containing your changes.
4. Submit a Pull Request, and get someone to review your changes. If everything
   looks good, merge it!

Avoid circumventing the above procedure and pushing directly to master. This
has a tendency to fuck things up.

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
