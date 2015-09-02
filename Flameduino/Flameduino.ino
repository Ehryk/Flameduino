/* 
==============
= Flameduino =
==============
     v1.2
  Eric Menze

Flameduino Controls an ignition coil with a set dwell (compile time)
and variable spark frequency, specifically for purposes of flamethrowing.
*/
//#include <TimerOne.h>
#define TIMER1_RESOLUTION 65536UL  // Timer1 is 16 bit

// -------------------------
//     Pin Definitions
// -------------------------
#define ACTIVATION_PIN 2
#define TACH_PIN 3
#define IGNITION_PIN 4
#define EXTERNAL_LED_PIN 4
#define INTERNAL_LED_PIN 13
#define FREQUENCY_PIN A0

// -------------------------
//      Ignition Modes
// -------------------------
#define OFF 0
#define DWELL 1
#define FIRE 2
#define MF_WAIT 3
#define MF_DWELL 4
#define MULTIFIRE 5
#define WAIT 6

// -------------------------
//        Settings
// -------------------------
const bool debug = true;       //true = write to serial monitor, false = bypass
const int serialOutLoops = 000; //Number of loops that should execute before a serial write
//Ignition Settings
const int dwell = 2500;         //Ignition coil charging dwell in microseconds (2.5ms = 2500us)
const long periodMax = 2000000; //Maximum period between ignitions, in microseconds (min frequency)
const long periodMin = 5000;    //Minimum period between ignitions, in microseconds (max frequency)
const int periodCorrection = 0; //Variance to period due to loop code execution time (milliseconds)
const float linearity = 2.5;    //1.0 = linear, 0.5 = more resolution in the high end, 2.0 = more resolution in the low end (exponential)
const bool triggerHigh = true;  //true = HIGH to fire, false = LOW to fire
//Multifire Settings
const bool multiFire = false;    //true = fire [firingCount] times per period, false = fire once per period
const int multiCount = 3;       //Numeber of sparks to fire per event
const int multiDelay = 2000;    //Delay between multifires in microseconds (2ms = 2000us)
//Tachometer Settings
const int tachTrim = -30;        //Ticks to trim off the tach durations
const long tachStale = 100000;   //Ticks after which the tach signal is 'stale'
const byte activeState = LOW;    //State of activation pin that determines active

// -------------------------
//     Global Variables
// -------------------------
volatile bool active = false;
byte ignitionMode = OFF;
volatile bool updateTimer = true;
volatile unsigned long tachPrevious = 0;
volatile unsigned long tachCurrent = 0;
unsigned long tachDuration0 = 0;
unsigned long tachDuration1 = 0;
unsigned long tachDuration2 = 0;
unsigned int rpm = 0;
byte remainingCount = 0;
int loopsSinceSerialWrite = 0;
long period = 0;

int timerCount = 0;

// -------------------------
//      Initialization
// -------------------------
void setup() 
{
  pinMode(ACTIVATION_PIN, INPUT);
  pinMode(TACH_PIN, INPUT);
  pinMode(IGNITION_PIN, OUTPUT);
  pinMode(FREQUENCY_PIN, INPUT);
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  pinMode(EXTERNAL_LED_PIN, OUTPUT);
  
  //Use built in pull-up resistor
  digitalWrite(ACTIVATION_PIN, HIGH);
  
  active = isActive();
  dischargeCoil();

  attachInterrupt(TACH_PIN - 2, ISR_TACH, RISING);
  attachInterrupt(ACTIVATION_PIN - 2, ISR_ACTIVE, CHANGE);
  
  initializeTimer1();
  
  if (debug)
  {
    Serial.begin(9600); 
    Serial.println("Flameduino");
    Serial.println("v1.2 Eric Menze");
  }
}

// -------------------------
//         Main Loop
// -------------------------
void loop() 
{
  period = getPeriod();
  rpm = getRPM();

  if (debug && loopsSinceSerialWrite >= serialOutLoops)
  {
    Serial.print("Mode: ");
    Serial.print(ignitionMode);
    Serial.print(", ");
  }
  
  if (active && ignitionMode == OFF)
    enableCoil();
  else if (!active && ignitionMode != OFF)
    disableCoil();

  //Fire Ignition
  switch(ignitionMode)
  {
    case FIRE:
      dischargeCoil();
      if (multiFire) remainingCount = multiCount - 1;
      updateTimer = true;
      break;
    case MULTIFIRE:
      dischargeCoil();
      remainingCount--;
      updateTimer = true;
      break;
  }
  
  if (updateTimer == true) 
  {
    stopTimer1();
    updateTimer = false;
    timerCount++;
    
    if (debug && loopsSinceSerialWrite >= serialOutLoops)
    {
      Serial.print("Timer Reached - ");
      Serial.print(timerCount);
      Serial.print(", ");
    }
    
    ignitionMode = nextMode(ignitionMode);
    
    //Set the delay period (Head of Mode)
    switch(ignitionMode)
    {
      case OFF:
        dischargeCoil();
        break;
      case DWELL:
        //Charging Coil
        chargeCoil();
        startTimer1(dwell);
        break;
      case MF_WAIT:
        //Waiting (MultiFire)
        startTimer1(multiDelay);
        break;
      case MF_DWELL:
        //Charging Coil
        chargeCoil();
        startTimer1(dwell);
        break;
      case WAIT:
        //Waiting (Frequency)
        startTimer1(period);
        break;
    }
  }
  
  if (debug && loopsSinceSerialWrite >= serialOutLoops)
  {
    Serial.print("Mode: ");
    Serial.print(ignitionMode);
    Serial.print(", Period: ");
    Serial.print(period / 1000.0);
    //Serial.print("ms, dwell: ");
    //Serial.print(dwell / 1000.0);
    Serial.print("ms, RPM: ");
    Serial.print(rpm);
    Serial.println();
    loopsSinceSerialWrite = 0;
  }

  loopsSinceSerialWrite++;
}

// -------------------------
//          Methods
// -------------------------
bool isActive() 
{
  return digitalRead(ACTIVATION_PIN) == activeState;
}

long getPeriod()
{
  int frequency = analogRead(FREQUENCY_PIN);
  
  float percentage = frequency / 1023.0;
  
  if (debug && loopsSinceSerialWrite >= serialOutLoops)
  {
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.print("(");
    Serial.print(percentage * 100);
    Serial.print("%), ");
  }
  
  return periodMin + (periodMax - periodMin) * pow(percentage, linearity) + periodCorrection;
}

unsigned int getRPM()
{
  noInterrupts();
  unsigned long current = tachCurrent;
  unsigned long previous = tachPrevious;
  interrupts();
  
  if (isStale())
  {
    tachDuration0 = 0;
    tachDuration1 = 0;
    tachDuration2 = 0;
    return 0;
  }
  
  unsigned long duration;
  if (current >= previous)
  {
    duration = current - previous + tachTrim;
  }
  else
  {
    //Handle Rollover
    duration = 0xFFFFFFFFu - previous + current + tachTrim;
  }
  
  //Shift the values
  tachDuration2 = tachDuration1;
  tachDuration1 = tachDuration0;
  tachDuration0 = duration;
  
  if (false && debug && loopsSinceSerialWrite >= serialOutLoops)
  {
    Serial.print("t0: ");
    Serial.print(tachDuration0);
    Serial.print(", t1: ");
    Serial.print(tachDuration1);
    Serial.print(", t2: ");
    Serial.print(tachDuration2);
    Serial.print(", ");
  }
  
  if (tachDuration0 <= 0 || tachDuration1 <= 0 || tachDuration2 <= 0)
    return 0;
  
  //Calculate Weighted Average duration
  unsigned long average = (3*tachDuration0 + 2*tachDuration1 + 1*tachDuration2)/6;
  
  //Convert to RPM
  return 60.0 * 1000000 / average;
}

bool isStale()
{
  return (micros() - tachCurrent) > tachStale;
}

void enableCoil()
{
  //Enable Coil Firing
  ignitionMode = DWELL;
  updateTimer = true;
}

void disableCoil()
{
  //Disable Coil Firing
  ignitionMode = OFF;
  updateTimer = true;
}

void chargeCoil()
{
  digitalWrite(EXTERNAL_LED_PIN, HIGH);
  digitalWrite(INTERNAL_LED_PIN, HIGH);
  
  if (triggerHigh)
    digitalWrite(IGNITION_PIN, HIGH);
  else
    digitalWrite(IGNITION_PIN, LOW);
}

void dischargeCoil()
{
  if (triggerHigh)
    digitalWrite(IGNITION_PIN, LOW);
  else
    digitalWrite(IGNITION_PIN, HIGH);
    
  digitalWrite(EXTERNAL_LED_PIN, LOW);
  digitalWrite(INTERNAL_LED_PIN, LOW);
}

byte nextMode(byte mode)
{
  switch(mode) {
    case OFF:
      return OFF;
    case DWELL:
      return FIRE;
    case FIRE:
      if(multiFire && remainingCount > 0)
        return MF_WAIT;
      else
        return WAIT;
    case MF_WAIT:
      return MF_DWELL;
    case MF_DWELL:
      return MULTIFIRE;
    case MULTIFIRE:
      if (remainingCount > 0)
        return MF_WAIT;
      else
        return WAIT;
    case WAIT:
      return DWELL;
  }
}

void fireMultiple()
{
  fireCoil();
  for(int i = 1; i < multiCount; i++)
  {
    delayMicroseconds(multiDelay);
    fireCoil();
  }
}

void fireCoil()
{
  chargeCoil();
  delayMicroseconds(dwell);
  dischargeCoil();
}

// -------------------------
//    Interrupt Handlers
// -------------------------
void ISR_TACH()
{
  unsigned long current = micros();
  tachPrevious = tachCurrent;
  tachCurrent = current;
}

void ISR_ACTIVE()
{
  //Set the active global variable
  active = isActive();
}

void ISR_IGNITION()
{
  //Interrupt has fired
  updateTimer = true;
}

//Attach ISR to Timer1 Overflow
ISR(TIMER1_OVF_vect)
{
  ISR_IGNITION();
}

// -------------------------
// Generic Utility Functions
// -------------------------
void delayFixed(long milliseconds)
{
  if (milliseconds <= 0)
    return;
  else if (milliseconds < 16)
    delayMicroseconds(milliseconds * 1000);
  else
    delay(milliseconds);
}

void initializeTimer1()
{
  noInterrupts();
  //Set Mode: Phase and Frequency correct PWM (stops the timer)
  TCCR1A = 0;                 //Set Mode (bit 1A)
  TCCR1B = bit(WGM13);        //Set Mode (bit 1B)
  TIMSK1 = bit(TOIE1);        //Enable Overflow ISR
  interrupts();
}

void startTimer1(unsigned long microseconds)
{
  char clockSelectBits;
  short pwmPeriod;
  
  //Calculate clock select bits
  const unsigned long cycles = (F_CPU / 2000000) * microseconds;
  if (cycles < TIMER1_RESOLUTION) 
  {
    //No Prescaling
    clockSelectBits = bit(CS10);
    pwmPeriod = cycles;
  } 
  else if (cycles < TIMER1_RESOLUTION * 8) 
  {
    //Prescale by /8
    clockSelectBits = bit(CS11);
    pwmPeriod = cycles / 8;
  }
  else if (cycles < TIMER1_RESOLUTION * 64) 
  {
    //Prescale by /64
    clockSelectBits = bit(CS11) | bit(CS10);
    pwmPeriod = cycles / 64;
  } 
  else if (cycles < TIMER1_RESOLUTION * 256) 
  {
    //Prescale by /256
    clockSelectBits = bit(CS12);
    pwmPeriod = cycles / 256;
  }
  else if (cycles < TIMER1_RESOLUTION * 1024) 
  {
    //Prescale by /1024
    clockSelectBits = bit(CS12) | bit(CS10);
    pwmPeriod = cycles / 1024;
  } 
  else
  {
    //Out of bounds, use maximum prescaler
    clockSelectBits = bit(CS12) | bit(CS10);
    pwmPeriod = TIMER1_RESOLUTION - 1;
  }

  noInterrupts();
  TCNT1 = 1;                             //Resets the counter
  ICR1 = pwmPeriod;                      //Sets the Period
  //Set Mode: Phase and Frequency correct PWM (Starts the timer)
  TCCR1A = 0;                            //Set Mode (bit 1A)
  TCCR1B = bit(WGM13) | clockSelectBits; //Set Mode (bit 1B)
  //unsigned int tcnt1;
  //do {  // Nothing -- wait until timer moved on from zero - otherwise get a phantom interrupt
  //  tcnt1 = TCNT1;
  //} while (tcnt1==0);
  interrupts();
}

void stopTimer1()
{
  noInterrupts();
  //Set Mode: Phase and Frequency correct PWM (stops the timer)
  TCCR1A = 0;                 //Set Mode (bit 1A)
  TCCR1B = bit(WGM13);        //Set Mode (bit 1B)
  interrupts();
}

long readInternalTemp() 
{
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delayFixed(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = (result - 125) * 1075;
  return result;
}

long readInternalVcc() 
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayFixed(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

