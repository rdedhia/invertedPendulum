#include "DualMC33926MotorShield.h"
#include <avr/interrupt.h>

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

// Angle and position measures
float mAngle;
float pos;
float prevPos;

// gain constants
const float K = 2000; // main gain of system, on integrator
const float kp = 0.2;
const float kv = 0.5;

// Pot readings
const float potToAngle = 3.09;
const float potCenter = 783.;
float pot;
float pendError;
float pendErrorPrev;

// Motor velocities
float interPrev;
float mVelocity;
float calcVel;

int counter;

void setup() { 
  Serial.begin(9600);
  
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode('A2', INPUT);
  // enable internal pullup resistors
  digitalWrite(pinA, 1);
  digitalWrite(pinB, 1);
  // using encoders with interrupts
  attachInterrupt(0, ISR_motor, CHANGE);
  // Start motor shield
  md.init();
  
  Serial.println(0);
  cli();
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 124;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS22 bit for 64 prescaler
  TCCR2B |= (1 << CS22);
  // enable timer compare interrupt (call ISR when TCNT2 = OCRA2)
  TIMSK2 |= (1 << OCIE2A);
  sei();
  Serial.println(1);
 }

void loop() {}

// Interrupt sequences to increment counters on CHANGE of encoder pins
void ISR_motor()
{
  if (digitalRead(pinA) == digitalRead(pinB)) {
    mCounter--;
  } else {
    mCounter++;
  }
}

// Interrupt for sampling at 2kHz using timer2
ISR(TIMER2_COMPA_vect) {
  // Stop program if cart has moved too much
  if ((pos > 500) || (pos < 500)) {
    cli();
    while(true) {};
  }
  
  // Take potentiometer readings and find error
  pot = analogRead('A2');
  pendError = (pot - potCenter) / potToAngle;
  
  // Add integrator
  mVelocity = K*(1.002*pendError - .9982*pendErrorPrev + interPrev);
  interPrev = mVelocity;
  
  // Add position and velocity loops
  mAngle = mCounter / mDivider;
  pos = (mAngle / 360.)*pulleyMm;
  calcVel = (pos - prevPos) * 2000;
  mVelocity = mVelocity + pos*kp - calcVel*kv;
  
  // Actuate motor
  md.setM1Speed(mVelocity);
  
  // For next time step
  pendErrorPrev = pendError;
  prevPos = pos;
}
