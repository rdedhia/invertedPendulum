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
  
  //cli();
  //set timer0 interrupt at 2kHz
  //TCCR0A = 0; //Set the TCCR0A register to 0
  //TCCR0B = 0;// same for TCCR0B
  //TCNT0  = 0;//initialize counter value to 0
  //set compare match register for 500Hz increments
  //OCR0A=124;
  // turn on CTC mode
  //TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  //TCCR0B |= (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  //TIMSK0 |= (1 << OCIE0A);
  //sei();
  
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

// Interrupt for sampling at 2kHz using timer0
ISR(TIMER0_COMPA_vect) {
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
  
  // Add integrator
  mVelocity = K*(1.002*pendError - .9982*pendErrorPrev + interPrev);
  interPrev = mVelocity;
  
  // Add position and velocity loops
  // mAngle = mCounter / mDivider;
  // pos = (mAngle / 360.)*pulleyMm;
  // calcVel = (pos - prevPos) * 2000;
  // mVelocity = mVelocity + pos*kp - calcVel*kv;
  
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
  
  counter++;
  if (((counter / 1000) % 2) == 0)
    digitalWrite(13, LOW);
  else {
    digitalWrite(13, HIGH);
  }
}
