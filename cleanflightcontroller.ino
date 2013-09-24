#include <PID_v1.h> //for PID loops

//sensor boards
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <PinChangeInt.h> //interrupts for rx input

#include <Servo.h> //motor control

#include <Wire.h> //i2c bus communication

#include <LiquidCrystal.h> //in case we use the LCD

#define BMP085_ADDRESS 0x77  // I2C address of BMP085 barometer

//motor servos
Servo zero;
Servo one;
Servo two;
Servo three;

//array for values to push to motors
uint16_t motors[4] = {1000,1000,1000,1000}; //range b/w 1000 and 2000 uS

//for use in interrupts for radio input
volatile uint16_t aux1length;
volatile uint16_t gearlength;
volatile uint16_t ruddlength;
volatile uint16_t elevlength;
volatile uint16_t ailelength;
volatile uint16_t throlength;
uint16_t aux1start;
uint16_t gearstart;
uint16_t ruddstart;
uint16_t elevstart;
uint16_t ailestart;
uint16_t throstart;

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
//euler angles from imu
float angles[3]; //yaw pitch roll

//barometer
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  bmp085Calibration();
  delay(5);
  sixDOF.init();
  delay(5);
  
  zero.attach(22,1000,2000); // - pitch
  one.attach(24,1000,2000); // + roll
  two.attach(26,1000,2000); // + pitch
  three.attach(28,1000,2000); // - roll
  
  //ESCs need 1000uS pulses for a bit to turn on for whatever reason
  zero.writeMicroseconds(1000);
  one.writeMicroseconds(1000);
  two.writeMicroseconds(1000);
  three.writeMicroseconds(1000);
  delay(2000);
  
  // any time the logic level of these pins changes call the specified function
  PCintPort::attachInterrupt(A8, updateAux1, CHANGE);
  PCintPort::attachInterrupt(A9, updateGear, CHANGE);
  PCintPort::attachInterrupt(A10, updateRudd, CHANGE);
  PCintPort::attachInterrupt(A11, updateElev, CHANGE);
  PCintPort::attachInterrupt(A12, updateAile, CHANGE);
  PCintPort::attachInterrupt(A13, updateThro, CHANGE);
  //for quick reference, all of these are functionally identical,
  //returning pulse length in uS
}
float targetPitch;
float targetRoll;
void loop()
{
  /*
  //establish a 'home' position, put relevant euler angles into angles array for use
  float q[4];
  sixDOF.getQ(q);
  float _hq[4];
  quatConjugate(q, _hq);
  float hq[4];
  quatProd(_hq,q, hq);
  angles[0] = atan2(2 * hq[1] * hq[2] - 2 * hq[0] * hq[3], 2 * hq[0]*hq[0] + 2 * hq[1] * hq[1] - 1) * 180/M_PI; // psi
  angles[1] = -asin(2 * hq[1] * hq[3] + 2 * hq[0] * hq[2]) * 180/M_PI; // theta
  angles[2] = atan2(2 * hq[2] * hq[3] - 2 * hq[0] * hq[1], 2 * hq[0] * hq[0] + 2 * hq[3] * hq[3] - 1) * 180/M_PI; // phi
//*/
  sixDOF.getEuler(angles);
  


  float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());
  float atm = pressure / 101325; // "standard atmosphere"
  float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 

  
  byte sregRestore = SREG; //backup interruptable state
  cli(); // clear the global interrupt enable flag 
  //Serial printing uses iterrupts so don't print from in here
  
  // for safe printing to make sure my data isn't trashed
  uint16_t aux1 = aux1length;
  uint16_t gear = gearlength;
  uint16_t rudd = ruddlength;
  uint16_t elev = elevlength;
  uint16_t aile = ailelength;
  uint16_t thro = throlength;
  
  //process inputs, puts proper values in the motors array
  thinking();
  
  SREG = sregRestore; //restore previous interruptable state
  
  zero.writeMicroseconds(motors[0]);
  one.writeMicroseconds(motors[1]);
  two.writeMicroseconds(motors[2]);
  three.writeMicroseconds(motors[3]);
  
  
  Serial.print(aux1);
  Serial.print(" | ");
  Serial.print(gear);
  Serial.print(" | ");
  Serial.print(rudd);
  Serial.print(" | ");
  Serial.print(elev);
  Serial.print(" | ");
  Serial.print(aile);
  Serial.print(" | ");
  Serial.print(thro);
  Serial.print(" |||| ");//*/
  Serial.print(targetPitch);
  Serial.print(" | ");
  Serial.print(targetRoll);
  Serial.print(" |||| ");
  Serial.print(motors[0]);
  Serial.print(" | ");
  Serial.print(motors[1]);
  Serial.print(" | ");
  Serial.print(motors[2]);
  Serial.print(" | ");
  Serial.print(motors[3]);
  Serial.print(" |||| ");//*/
  Serial.print(angles[0]);
  Serial.print(" | ");
  Serial.print(angles[1]);
  Serial.print(" | ");
  Serial.print(angles[2]);
  Serial.print(" |||| ");
  //*/
  Serial.println();
}

/*
okay so basically every motor starts at the throttle speed.
desired in whichever respect is guessed based on rx input
eg, if (elevlength==1900) desiredpitch=20
motors[0] speeds up, motors[2] slows down
when the measured angle approaches the desired angle,
  a p loop kicks in. one loop per angle
pitch, roll, yaw should all play nicely
*/
void thinking()
{
//thro = throttle up and down
//rudd = rotating in yaw axis
//elev = tracking forward and backward
//aile = tracking left and right
  
  motors[0] = throlength;
  motors[1] = throlength;
  motors[2] = throlength;
  motors[3] = throlength;
  
  //float targetPitch;
  //float targetRoll;
  if (elevlength > 1600 || elevlength < 1400) //deadzone
    targetPitch = (.04 * elevlength) - 69.5;
  else targetPitch = -9.5;
  if (ailelength > 1600 || ailelength < 1400) //deadzone
    targetRoll = (.04 * ailelength) - 65.5;
  else targetRoll = -5.5;
  //float targetYaw; //yaw will be tough... gyro drifts
                     //also speed needs to be controlled, not angle itself
  
  //control yaw
  if (ruddlength > 1600 && ruddlength <= 2000)
  {
    int adj = (2000 - ruddlength) / 4;
    motors[0] += adj;
    motors[1] -= adj;
    motors[2] += adj;
    motors[3] -= adj;
  }
  else if (ruddlength < 1400 && ruddlength >= 1000)
  {
    int adj = (ruddlength - 1000) / 4;
    motors[0] -= adj;
    motors[1] += adj;
    motors[2] -= adj;
    motors[3] += adj;
  }
  
  //control pitch
  if (angles[1] > targetPitch + 2)
  {
    motors[0] += 100;
    motors[2] -= 100;
  }
  else if (angles[1] < targetPitch - 2)
  {
    motors[0] -= 100;
    motors[2] += 100;
  }
  else
  {
    pitchPID(targetPitch);
  }
  
  //control roll
  if (angles[2] > targetRoll + 4)
  {
    motors[1] -= 100;
    motors[3] += 100;
  }
  else if (angles[2] < targetRoll - 4)
  {
    motors[1] += 100;
    motors[3] -= 100;
  }
  else
  {
    rollPID(targetRoll);
  }
  
  //keep motor speeds in the correct range
  if (motors[0] < 1050) motors[0] = 1000;
  else if (motors[0] > 2000) motors[0] = 2000;
  if (motors[1] < 1050) motors[1] = 1000;
  else if (motors[1] > 2000) motors[1] = 2000;
  if (motors[2] < 1050) motors[2] = 1000;
  else if (motors[2] > 2000) motors[2] = 2000;
  if (motors[3] < 1050) motors[3] = 1000;
  else if (motors[3] > 2000) motors[3] = 2000;
}

//P loop for when close to desired pitch angle
void pitchPID(float target)
{
  float error = angles[1] - target;
  motors[0] += (error * 25);
  motors[2] -= (error * 25);  
}

//P loop for when close to desired roll angle
void rollPID(float target)
{
  float error = angles[2] - target;
  motors[1] += (error * 25);
  motors[3] -= (error * 25);
}

//time the pulse length from each channel of the transmitter
//called via pin change interrupts
void updateAux1()
{
  if (digitalRead(A8) == HIGH)
  {
    aux1start = micros();
  }
  else
  {
    aux1length = (uint16_t)(micros() - aux1start);
  }
}
void updateGear()
{
  if (digitalRead(A9) == HIGH)
  {
    gearstart = micros();
  }
  else
  {
    gearlength = (uint16_t)(micros() - gearstart);
  }
}
void updateRudd()
{
  if (digitalRead(A10) == HIGH)
  {
    ruddstart = micros();
  }
  else
  {
    ruddlength = (uint16_t)(micros() - ruddstart);
  }
}
void updateElev()
{
  if (digitalRead(A11) == HIGH)
  {
    elevstart = micros();
  }
  else
  {
    elevlength = (uint16_t)(micros() - elevstart);
  }
}
void updateAile()
{
  if (digitalRead(A12) == HIGH)
  {
    ailestart = micros();
  }
  else
  {
    ailelength = (uint16_t)(micros() - ailestart);
  }
}
void updateThro()
{
  if (digitalRead(A13) == HIGH)
  {
    throstart = micros();
  }
  else
  {
    throlength = (uint16_t)(micros() - throstart);
  }
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

float calcAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}

void quatProd(float * a, float * b, float * q) {

  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

}

// return the quaternion conjugate of quat
void quatConjugate(float * quat, float * conj) {
  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];
}
