#include "DualMC33926MotorShield.h"

#define pinA 2
#define pinB 5
#define numTicks 608

// Create the motor shield object with the default I2C address
DualMC33926MotorShield md;

int state = 1;
// Encoder stuff (counters and ticks)
volatile int mCounter;

// For converting from encoder ticks to angles
const float mDivider = numTicks / 360.;
const float pulleyMm = 200.12;

// Angle and position measures
float mAngle;
float pos;
float prevPos;

// gain constants
const float K = 20; // main gain of system, on integrator
const float kp = 0.2;
const float kv = 0.5;

// Pot readings
const float potToAngle = 3.09;
const float potCenter = 784.;
float pot;
float pendError;
float pendErrorPrev;

// Motor velocities
float interPrev;
float mVelocity;
float calcVel;

int counter;

void setup() {   
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(A2, INPUT);
  pinMode(13, OUTPUT);
  // enable internal pullup resistors
  digitalWrite(pinA, 1);
  digitalWrite(pinB, 1);
  // using encoders with interrupts
  attachInterrupt(0, ISR_motor, CHANGE);
  // Start motor shield
  md.init();
  
  Serial.begin(9600);
}

void loop() {
  // Stop program if cart has moved too much
  if ((pos > 500) || (pos < -500)) {
    cli();
    while(true) {
      digitalWrite(13, HIGH);
    };
  }
  
  // Take potentiometer readings and find error
  pot = analogRead(A2);
  pendError = (pot - potCenter) / potToAngle;
  
  Serial.println(pot);
  Serial.println(pendError);
  Serial.print('\n');
  
  // Actuate motor
  if (mVelocity > 150) {
    mVelocity = 150;
  } else if (mVelocity < -150) {
    mVelocity = -150;
  }
  md.setM1Speed(mVelocity);
  
  // For next time step
  pendErrorPrev = pendError;
  prevPos = pos;
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
