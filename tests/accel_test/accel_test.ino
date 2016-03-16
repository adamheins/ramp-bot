const int xInput = A0;
const int yInput = A1;
const int zInput = A2;
const int buttonPin = 2;

// Raw Ranges:
// initialize to mid-range and allow calibration to
// find the minimum and maximum for each axis
int xRawMin = 512;
int xRawMax = 512;

int yRawMin = 512;
int yRawMax = 512;

int zRawMin = 431;
int zRawMax = 640;

// Take multiple samples to reduce noise
const int sampleSize = 10;

// Raw Ranges: X: 404-609, Y: 405-613, Z: 431-640

// TEST CODE
// Don't use in production :)

#define BUF_SIZE 10
#define DIST_THRESHOLD 5

int buffer[BUF_SIZE];
int vbuf[BUF_SIZE];
int pbuf[BUF_SIZE];
int index = 0;

void buffer_init(int *buf) {
  for (int i = 0; i < BUF_SIZE; ++i) {
     buf[i] = 0; 
  }
}

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

// stupid don't use
void buffer_integrate(int *sbuf, int *dbuf) {
  int ilast = 0;
  int i = index;
  for (int count = 0; count < BUF_SIZE; ++count) {
    dbuf[i] = ilast + sbuf[i];
    ilast = dbuf[i];
    i = (i + 1) % BUF_SIZE;
  }
}

// on the fly calibration?
int buffer_range(int *buf) {
  int buf_min = buf[0];
  int buf_max = buf[0];
  for (int i = 1; i < BUF_SIZE; ++i) {
    if (buf[i] < buf_min) {
       buf_min = buf[i]; 
    }
    if (buf[i] > buf_max) {
       buf_max = buf[i]; 
    }
  } 
  return buf_max - buf_min;
}

void setup() 
{
  analogReference(EXTERNAL);
  Serial.begin(9600);

  buffer_init(buffer);
  buffer_init(vbuf);
  buffer_init(pbuf);
}

int offset = 510;
double a_total = 0;

void loop() 
{
  int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);
  
  int norm = xRaw - offset;
  
  buffer_insert(norm);
  
  if (buffer_range(buffer) == 0) {
      offset += buffer[0];
  }
  
  a_total += norm;
  a_total *= 0.9;
  
  Serial.print(norm);
  Serial.print("  ");
  Serial.println(a_total);
  /*
  if (digitalRead(buttonPin) == LOW)
  {
    AutoCalibrate(xRaw, yRaw, zRaw);
  }
  else
  {
    Serial.print("Raw Ranges: X: ");
    Serial.print(xRawMin);
    Serial.print("-");
    Serial.print(xRawMax);
    
    Serial.print(", Y: ");
    Serial.print(yRawMin);
    Serial.print("-");
    Serial.print(yRawMax);
    
    Serial.print(", Z: ");
    Serial.print(zRawMin);
    Serial.print("-");
    Serial.print(zRawMax);
    Serial.println();
    Serial.print(xRaw);
    Serial.print(", ");
    Serial.print(yRaw);
    Serial.print(", ");
    Serial.print(zRaw);
    
    // Convert raw values to 'milli-Gs"
    long xScaled = map(xRaw, xRawMin, xRawMax, -1000, 1000);
    long yScaled = map(yRaw, yRawMin, yRawMax, -1000, 1000);
    long zScaled = map(zRaw, zRawMin, zRawMax, -1000, 1000);
  
    // re-scale to fractional Gs
    float xAccel = xScaled / 1000.0;
    float yAccel = yScaled / 1000.0;
    float zAccel = zScaled / 1000.0;
  
    Serial.print(" :: ");
    Serial.print(xAccel);
    Serial.print("G, ");
    Serial.print(yAccel);
    Serial.print("G, ");
    Serial.print(zAccel);
    Serial.println("G");
  
  delay(500);
  }
  */
  delay(1000);
  
}

//
// Read "sampleSize" samples and report the average
//
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/sampleSize;
}

//
// Find the extreme raw readings from each axis
//
void AutoCalibrate(int xRaw, int yRaw, int zRaw)
{
  Serial.println("Calibrate");
  if (xRaw < xRawMin)
  {
    xRawMin = xRaw;
  }
  if (xRaw > xRawMax)
  {
    xRawMax = xRaw;
  }
  
  if (yRaw < yRawMin)
  {
    yRawMin = yRaw;
  }
  if (yRaw > yRawMax)
  {
    yRawMax = yRaw;
  }

  if (zRaw < zRawMin)
  {
    zRawMin = zRaw;
  }
  if (zRaw > zRawMax)
  {
    zRawMax = zRaw;
  }
}

