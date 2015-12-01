#include "DualMC33926MotorShield.h"

#define pinA 2
#define pinB 5
#define numTicks 1216

// Create the motor shield object with the default I2C address
DualMC33926MotorShield md;

// Encoder stuff (counters and ticks)
volatile int mCounter;

// For converting from encoder ticks to angles
const float mDivider = numTicks / 360.;
const float pulleyMm;

// Angle measures
float mAngle;
float prevAngle;

// PySerial stuff
String incoming = "";
String panString = "";
String tiltString = "";

// Motor velocities
double mVelocity = 255;
double targetSpeed = mVelocity * .63;
double rpmVel;

// Timing
int time;
int prevTime;
int printTime;

void setup() {
  Serial.begin(9600);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode('A0', OUTPUT);
  // enable internal pullup resistors
  digitalWrite(pinA, 1);
  digitalWrite(pinB, 1);
  // using encoders with interrupts
  attachInterrupt(0, ISR_motor, CHANGE);
  // Start motor shield
  md.init();
  //tell PySerial you're ready to go
  Serial.print("Initialized!");
}

void loop() {
  // calculating angle from counters by scaling it
  mAngle = mCounter / mDivider;
  
  // hard code speed at 255
  md.setM1Speed(mVelocity);

  time = millis();
  rpmVel = (mAngle - prevAngle) * 1000 / 6 / (time - prevTime);
  prevTime = time;
  
  // Print stuff
  if (time-printTime > 100) {
    Serial.println(time);
    Serial.println(mAngle);
    Serial.println(mCounter);
    Serial.println(rpmVel);
    Serial.print('\n');
    printTime = time;
  }
  
  prevAngle = mAngle;
}

// Interrupt sequences to increment counters on CHANGE of encoder pins
void ISR_motor()
{
  if (digitalRead(pinA) == digitalRead(pinB)) {
    mCounter--;
  } else {
    mCounter++;
  }
}
