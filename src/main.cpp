#include <Arduino.h>
#include <stdio.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <SmoothThermistor.h>
#include <EEPROM.h>
#include <PrintEx.h>
#include "RunningMedian.h"

#define MIN_T 20.0
#define MIN_RPM 500.0
#define MAX_T 35.0
#define MAX_PWM 1.0
#define STABILIZE_TIME 10000
#define SENSOR_PIN A1
#define REFERENCE_RESISTANCE 76000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950
#define SMOOTHING_FACTOR 5
#define eeAddress 10
#define PIN_SENSE 2            // where we connected the fan sense pin. Must be an interrupt capable pin (2 or 3 on Arduino Uno)
#define PIN_PWM 9              // where we connected the fan PWM pin
#define DEBOUNCE 20            // 0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
#define FANSTUCK_THRESHOLD 500 // if no interrupts were received for 500ms, consider the fan as stuck and report 0 RPM

RunningMedian rpmSamples = RunningMedian(7);
Thermistor *thermistor = NULL;

// static int serial_putc(const char c, FILE *stream) { return Serial.write(c); }
// static FILE *serial_stream = fdevopen(serial_putc, NULL);

// Interrupt handler. Stores the timestamps of the last 2 interrupts and handles debouncing
unsigned long volatile lastINT = 0, lastHit = 0;
static float min_pwm;
StreamEx mySerial = Serial;
enum opStates
{
  STABILIZE_FAN,
  CALIBRATE_FAN,
  SCALE_FAN,
  INVALID_NTC
};
enum opStates modeState;

void tachISR()
{
  unsigned long m = millis();
  if (m - lastINT > DEBOUNCE)
  {
    rpmSamples.add((60000.0 / (m - lastINT)) / 2.0);
    lastINT = m;
  }
  else
    rpmSamples.add(0);
}

// configure Timer 1 (pins 9,10) to output 25kHz PWM
void setupTimer1()
{
  // Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << CS10) | (1 << WGM13);
  ICR1 = 320;
  OCR1A = 0;
  OCR1B = 0;
}

// equivalent of analogWrite on a PWM pin
void setPWMpin(uint8_t pin, float dutyCycle)
{
  if (dutyCycle < 0)
  {
    dutyCycle = 0;
  }
  else if (dutyCycle > 1)
  {
    dutyCycle = 1;
  }
  uint16_t duty = 320.0 * dutyCycle;
  switch (pin)
  {
  case 9:
    OCR1A = duty;
    break;
  case 10:
    OCR1B = duty;
    break;
  default:
    break;
  }
}

double getPWMpin(uint8_t pin)
{
  uint16_t duty;
  switch (pin)
  {
  case 9:
    duty = OCR1A;
    break;
  case 10:
    duty = OCR1B;
    break;
  default:
    return 0.0f;
  }
  return duty / 320.0;
}

void setup()
{
  // stdout = serial_stream;
  Serial.begin(9600);
  pinMode(PIN_SENSE, INPUT_PULLUP);                                    // set the sense pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_SENSE), tachISR, FALLING); // set tachISR to be triggered when the signal on the sense pin goes low

  pinMode(9, OUTPUT); // 1A
  setupTimer1();

  min_pwm = 0.00f;
  EEPROM.get(eeAddress, min_pwm);
  setPWMpin(PIN_PWM, min_pwm);
  Thermistor *originThermistor = new NTC_Thermistor(
      SENSOR_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE);
  thermistor = new SmoothThermistor(originThermistor, SMOOTHING_FACTOR);
  modeState = STABILIZE_FAN;
  delay(2000);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{
  delay(200);
  if (millis() - lastINT > FANSTUCK_THRESHOLD)
    rpmSamples.add(0);

  auto pwm = getPWMpin(PIN_PWM);
  if (rpmSamples.getMedianAverage(3) < 100)
    modeState = STABILIZE_FAN;

  float celsius = thermistor->readCelsius();

  mySerial.printf("%d:RPM: %2.2f PWM: %2.2f  T:%2.2f\n", modeState, rpmSamples.getMedianAverage(3), pwm, celsius);

  switch (modeState)
  {
  case STABILIZE_FAN:
  {

    if (rpmSamples.getMedianAverage(3) < MIN_RPM - 200)
    {
      mySerial.printf("%d:stabilize PWM: %u\n", modeState, (uint16_t)(pwm * 100));
      setPWMpin(PIN_PWM, pwm + 0.05f);
      return;
    }
    else
    {
      modeState = CALIBRATE_FAN;
      return;
    }
    break;
  }

  case CALIBRATE_FAN:
  {
    auto rp = rpmSamples.getMedianAverage(3);
    if (abs(rp - MIN_RPM) > 10)
    {
      // mySerial.printf("%d:calibrate PWM: %u\n", modeState, (uint16_t)(pwm * 100));f
      mySerial.printf("factor: %.3f", (((MIN_RPM / rp - 1.0) / 100.0) + 1.0));
      setPWMpin(PIN_PWM, pwm * (((MIN_RPM / rp - 1.0) / 5.0) + 1.0));
      lastHit = 0;
      return;
    }
    // else if (rpmSamples.getMedianAverage(3) > MIN_RPM + 10)
    // {
    //   // mySerial.printf("%d:calibrate PWM: %u\n", modeState, (uint16_t)(pwm * 100));
    //   setPWMpin(PIN_PWM, pwm - 0.001f);
    //   lastHit=0;
    //   return;
    // }
    else if (lastHit == 0)
    {
      lastHit = millis();
      return;
    }
    else if (millis() - lastHit > STABILIZE_TIME)
    {
      mySerial.printf("%d:new calibrate point PWM: %u\n", modeState, (uint16_t)(pwm * 100));
      if (min_pwm != pwm)
      {
        min_pwm = pwm;
        EEPROM.put(eeAddress, min_pwm);
      }
      modeState = SCALE_FAN;
      return;
    }
    break;
  }

  case SCALE_FAN:
  {

    // if (celsius < 0 or celsius > 100)
    // {
    //   modeState = INVALID_NTC;
    //   return;
    // }

    pwm = mapfloat(celsius, MIN_T, MAX_T, min_pwm, MAX_PWM);
    pwm = constrain(pwm, min_pwm, MAX_PWM);
    setPWMpin(PIN_PWM, pwm);
    // mySerial.printf("%d:scale PWM: %2.2f\n", modeState, pwm);
    break;
  }

  default:
  {

    pwm = 0.5;
    mySerial.printf("%d:fallback PWM: %2.2f\n", modeState, pwm);
    setPWMpin(PIN_PWM, pwm);
    break;
  }
  }
}
