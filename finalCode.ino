#include "DualMC33926MotorShield.h"

#define pinA 2
#define pinB 5
#define numTicks 608

// Create the motor shield object with the default I2C address
DualMC33926MotorShield md;

// Encoder stuff (counters and ticks)
volatile int mCounter;

// Sampling
const float samplingRate = 61.75;

// For converting from encoder ticks to angles
const float mDivider = numTicks / 360.;
const float rev_to_m = 0.20012;
const float pi = 3.1416;

// Angle and position measures
const float rad_to_m = 0.03185;
float mAngle;
float pos;
float prevPos;

// Gain constants
const float K = 240; // K ~ V/rad
const float kp = 150; // Kp ~ V/m
const float kv = 0.2; // Kv ~ V*s/rad

// Pot readings
const float potToAngle = 3.09;
float potCenter;
float pot;
float pendError;
float pendErrorPrev;

// Motor velocities and voltages
const float volts_to_vel = 400/12;
float interPrev;
float tempVolt;
float mVoltage;
float mVelocity;
float calcVel;

// Debugging
int counter;

void setup() {   
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(A2, INPUT);
  pinMode(13, OUTPUT);
  // average 5 measurements for pendulum measurement
  for (int i=0; i<5; i++) {
    potCenter = potCenter + analogRead(A2);
  }
  potCenter = potCenter / 5.;
  // enable internal pullup resistors
  digitalWrite(pinA, 1);
  digitalWrite(pinB, 1);
  // using encoders with interrupts
  attachInterrupt(0, ISR_motor, CHANGE);
  // Start motor shield
  md.init();
  
  cli();
  //set timer0 interrupt at 61.75Hz
  TCCR0A = 0; //Set the TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  //set compare match register for 61.75Hz increments
  OCR0A = 249; // 249 for 61.75Hz
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS02 bit for 1024 prescaler
  TCCR0B |= (1 << CS02) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();
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

ISR(TIMER0_COMPA_vect) {
  // Stop program if cart has moved too much
  if ((pos > .6) || (pos < -.6)) {
    cli();
    while(true) {
      md.setM1Speed(0);
      digitalWrite(13, HIGH);
    };
  }
  
  // Take potentiometer readings and find error
  pot = analogRead(A2);
  pendError = (pot - potCenter) / potToAngle * pi / 180; // rad
  
  // Stop program if pendulum arm error is greater than 30 degrees
  if (abs(pendError) > pi/6) {
    cli();
    while(true) {
      md.setM1Speed(0);
      digitalWrite(13, HIGH);
    };
  } 
  
  // Integrator compensation, (s+6)/(s+0.1) at 61.75Hz
  tempVolt = K*1.049*pendError - K*.9514*pendErrorPrev + .9984*interPrev; // V
  interPrev = tempVolt; // V
  
  // Update angles and positions
  mAngle = mCounter / mDivider; // degrees
  pos = mAngle / 360 * rev_to_m; // m
  
  // Calculate velocity based on change in position
  calcVel = (pos - prevPos) * samplingRate / rad_to_m; // rad/s
  
  // Combine integrator, velocity loop, and position loop
  mVoltage = tempVolt + pos*kp - calcVel*kv; // V
  
  // Convert voltage to motor controller velocity
  mVelocity = volts_to_vel * mVoltage; // MC units
  
  // Actuate motor
  if (mVelocity > 390) {
    mVelocity = 390;
  } else if (mVelocity < -390) {
    mVelocity = -390;
  }
  md.setM1Speed(mVelocity);
  
  // For next time step
  pendErrorPrev = pendError; // rad
  prevPos = pos; // m
  
  // Debugging LED to ensure interrupt is running at roughly correct speed
  counter++;
  if (((counter / 1000) % 2) == 0)
    digitalWrite(13, LOW);
  else {
    digitalWrite(13, HIGH);
  }
}
