#include "DualMC33926MotorShield.h"

#define pinA 2
#define pinB 5
#define numTicks 608

// Create the motor shield object with the default I2C address
DualMC33926MotorShield md;

// Encoder stuff (counters and ticks)
volatile int mCounter;

// For converting from encoder ticks to angles
const float mDivider = numTicks / 360.;
const float pulleyMm = 200.12;

// Angle measures
float mAngle;
float pos;

// Motor velocities
double mVelocity = 200;

// Timing
int time;
int lastTime;

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
  pos = (mAngle / 360.)*pulleyMm;
  
  // hard code speed at 255
  md.setM1Speed(mVelocity);

  time = millis();
  // Print stuff
  if (time-lastTime > 250) {
    Serial.println(mAngle);
    Serial.println(mCounter);
    Serial.println(pos);
    Serial.println(time);
    Serial.print('\n');
    lastTime = time;
  }
  
  if (pos > 300) {
    mVelocity = -200;
  } else if (pos < -300) {
    mVelocity = 200;
  }
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
