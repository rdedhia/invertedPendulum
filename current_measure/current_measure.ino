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

// Angle measures
float mAngle;

// PySerial stuff
String incoming = "";
String panString = "";
String tiltString = "";

// Motor velocities
double mVelocity = 255;

// Timing
int time;
int lastTime;

// Current measurements
float current;
int current_len;

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
  mAngle = (mCounter % numTicks) / mDivider;
  current = current + analogRead('A0');
  current_len = current_len + 1;
  // hard code speed at 100
  md.setM1Speed(mVelocity);

  time = millis();
  // Print stuff every half second
  if (time-lastTime > 1000) {
    Serial.print(mAngle);
    Serial.print('\n');
    Serial.print(mCounter);
    Serial.print('\n');
    Serial.print(current/current_len);
    Serial.print('\n');
    Serial.print('\n');
    //mVelocity = mVelocity * -1;
    lastTime = time;
    current = 0;
    current_len = 0;
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
