/*
==============
= Flameduino =
==============
v1.0
Eric Menze

Flameduino Controls an ignition coil with a set dwell (compile time)
and variable spark frequency, specifically for purposes of flamethrowing.
*/

// -------------------------
//     Pin Definitions
// -------------------------
#define ACTIVATION_PIN 2
#define IGNITION_PIN 3
#define EXTERNAL_LED_PIN 4
#define INTERNAL_LED_PIN 13
#define FREQUENCY_PIN A0


// -------------------------
//        Settings
// -------------------------
const int dwell = 2500;         //Ignition coil charging dwell in microseconds (2.5ms = 2500us)
const long periodMax = 1000;    //Maximum period between ignitions, in milliseconds (min frequency)
const long periodMin = 5;       //Minimum period between ignitions, in milliseconds (max frequency)
const int periodCorrection = 0; //Variance to period due to loop code execution time (milliseconds)
const float linearity = 2.5;    //1.0 = linear, 0.5 = more resolution in the high end, 2.0 = more resolution in the low end (exponential)
const bool debug = false;       //true = write to serial monitor, false = bypass
const bool triggerHigh = true;  //true = HIGH to fire, false = LOW to fire
const bool multiFire = true;    //true = fire [firingCount] times per period, false = fire once per period
const int firingCount = 3;      //Numeber of sparks to fire per event
const int firingDwell = 2000;   //Ignition coil firing time in microseconds (2ms = 2000us)

// -------------------------
//     Global Variables
// -------------------------
long period = 0;

// -------------------------
//      Initialization
// -------------------------
void setup() 
{
  pinMode(ACTIVATION_PIN, INPUT);
  pinMode(IGNITION_PIN, OUTPUT);
  pinMode(FREQUENCY_PIN, INPUT);
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  pinMode(EXTERNAL_LED_PIN, OUTPUT);
  
  //Use built in pull-up resistor
  digitalWrite(ACTIVATION_PIN, HIGH);
  deactivate();
  
  if (debug)
  {
    Serial.begin(9600); 
    Serial.println("Flameduino");
    Serial.println("v1.0 Eric Menze");
  }
}

// -------------------------
//         Main Loop
// -------------------------
void loop() 
{
  period = getPeriod();
  
  if (isActive()) {
    if (multiFire)
      fireMultiple(); //Multiple Firing Events (firingCount)
    else
      fire();         //Single Firing Event
  }
  
  if (debug)
  {
    Serial.print("Period: ");
    Serial.print(period);
    Serial.print("ms, dwell: ");
    Serial.print(dwell / 1000.0);
    Serial.println("ms");
  }

  delayFixed(period);
}

// -------------------------
//          Methods
// -------------------------
bool isActive() 
{
  return digitalRead(ACTIVATION_PIN) == LOW;
}

long getPeriod()
{
  int frequency = analogRead(FREQUENCY_PIN);
  
  float percentage = frequency / 1023.0;
  
  if (debug)
  {
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.print("(");
    Serial.print(percentage * 100);
    Serial.print("%), ");
  }
  
  return periodMin + (periodMax - periodMin) * pow(percentage, linearity) + periodCorrection;
}

void activate()
{
  digitalWrite(EXTERNAL_LED_PIN, HIGH);
  digitalWrite(INTERNAL_LED_PIN, HIGH);
  
  if (triggerHigh)
    digitalWrite(IGNITION_PIN, HIGH);
  else
    digitalWrite(IGNITION_PIN, LOW);
}

void deactivate()
{
  if (triggerHigh)
    digitalWrite(IGNITION_PIN, LOW);
  else
    digitalWrite(IGNITION_PIN, HIGH);
    
  digitalWrite(EXTERNAL_LED_PIN, LOW);
  digitalWrite(INTERNAL_LED_PIN, LOW);
}

void fireMultiple()
{
  fire();
  for(int i = 1; i <= firingCount; i++)
  {
    delayMicroseconds(firingDwell);
    fire();
  }
}

void fire()
{
  activate();
  delayMicroseconds(dwell);
  deactivate();
}

// -------------------------
// Generic Utility Functions
// -------------------------
void delayFixed(long ms)
{
  if (ms <= 0)
    return;
  else if (ms < 16)
    delayMicroseconds(ms * 1000);
  else
    delay(ms);
}

long readInternalTemp() 
{
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delay(2); // Wait for Vref to settle
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
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

