class DriveServo {
  public:
    DriveServo(int pin);
    void on();
    void off();
    void write(int vel);

  private:
    int pin;
    int vel;
    Servo servo;
}

DriveServo::DriveServo(int pin) {
  this->pin = pin;
}

void DriveServo::on() {
  this->servo.attach(this->pin);
}

void DriveServo::off() {
  this->servo.deattach();
}

void DriveServo::write(int vel) {
  this->vel = vel;
  this->servo.write(vel);
}
