

// this constant won't change.  It's the pin number
// of the sensor's output:
const int pingPin = 7;

// TEST CODE
// Don't use in production :)

#define BUF_SIZE 10
#define DIST_THRESHOLD 5

int buffer[BUF_SIZE];
int index = 0;

int vbuf[BUF_SIZE];
int pbuf[BUF_SIZE];

void buffer_insert(int value) {
  index = (index + 1) % BUF_SIZE;
  buffer[index] = value;
}

int buffer_avg(int *buf) {
  int sum = 0;
   for (int i = 0; i < BUF_SIZE; ++i) {
      sum += buf[i];
   }
  return sum / BUF_SIZE; 
}

int buffer_edge() {
   int avg1 = 0;
   int avg2 = 0;
   for (int i = 0; i < BUF_SIZE / 2; ++i) {
      avg1 += buffer[i];
   }
   for (int i = BUF_SIZE / 2; i < BUF_SIZE; ++i) {
      avg2 += buffer[i];
   }
   avg1 /= (BUF_SIZE / 2);
   avg2 /= (BUF_SIZE / 2);
   if (abs(avg1 - avg2) > DIST_THRESHOLD) {
     return 1;
   } else {
     return 0;
   }
}


void buffer_integrate(int *sbuf, int *dbuf) {
  int ilast = 0;
  int count = 0;
  for (int i = index; i < BUF_SIZE; i = (i + 1) % BUF_SIZE) {
    dbuf[i] = ilast + sbuf[i];
    ilast = dbuf[i];
    if (count++ > BUF_SIZE) {
       break; 
    }
  }
}

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
}

void loop() {
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  Serial.println("what the fuckkk");
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  buffer_insert(cm);
  
  buffer_integrate(buffer, vbuf);
  buffer_integrate(vbuf, pbuf);

  //Serial.print(cm);
  //Serial.print("cm");
  Serial.print(buffer_avg(buffer));
  Serial.print("  ");
  Serial.print(buffer_avg(pbuf));
  Serial.println();

  delay(1000);
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
