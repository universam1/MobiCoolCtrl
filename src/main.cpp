#include <Arduino.h>
#include <stdio.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <SmoothThermistor.h>
#include <PrintEx.h>
#include "RunningMedian.h"
#include <QuickPID.h>
#include <sTune.h>

#define MIN_T 20.0
#define MIN_RPM 500
#define MAX_T 35.0
#define MAX_RPM 1000
#define MAX_PWM_VALUE 320
#define MIN_RPM_AT_PWM MAX_PWM_VALUE * 0.65
#define SENSOR_PIN A1
#define REFERENCE_RESISTANCE 76000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950
#define SMOOTHING_FACTOR 5
#define PIN_SENSE 2            // where we connected the fan sense pin. Must be an interrupt capable pin (2 or 3 on Arduino Uno)
#define PIN_PWM 9              // where we connected the fan PWM pin
#define DEBOUNCE 20            // 0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
#define FANSTUCK_THRESHOLD 500 // if no interrupts were received for 500ms, consider the fan as stuck and report 0 RPM

PrintEx mySerial = Serial;
RunningMedian rpmSamples = RunningMedian(7);
Thermistor *thermistor = NULL;
unsigned long volatile lastINT = 0;

// start with sane defaults
float SetpointRPM = MIN_RPM,
      CurrentRPM = 0,
      PWM = MIN_RPM_AT_PWM,
      Kp=0.35, Ki=0.001, Kd=0.0;
QuickPID myPID(&CurrentRPM, &PWM, &SetpointRPM,
               Kp, Ki, Kd,                   // Kp, Ki, Kd,
               QuickPID::pMode::pOnError,        /* pOnError, pOnMeas, pOnErrorMeas */
               QuickPID::dMode::dOnError,        /* dOnError, dOnMeas */
               QuickPID::iAwMode::iAwCondition, /* iAwCondition, iAwClamp, iAwOff */
               QuickPID::Action::direct);       /* direct, reverse */

// sTune tuner = sTune(&CurrentRPM, &PWM, tuner.ZN_PID, tuner.directIP, tuner.printSUMMARY);

void tachISR()
{
  auto m = millis();
  auto rt = m - lastINT;
  if (rt > DEBOUNCE)
  {
    rpmSamples.add((60000.0f / rt) / 2.0f);
    lastINT = m;
  }
}

// configure Timer 1 (pins 9,10) to output 25kHz PWM
void setupTimer1()
{
  // Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << CS10) | (1 << WGM13);
  ICR1 = MAX_PWM_VALUE;
  OCR1A = 0;
  OCR1B = 0;
  pinMode(9, OUTPUT); // 1A
}

// equivalent of analogWrite on a PWM pin; 0-320
void setPWMpin(uint8_t pin, uint16_t dutyCycle)
{
  dutyCycle = constrain(dutyCycle, 0, MAX_PWM_VALUE);
  switch (pin)
  {
  case 9:
    OCR1A = dutyCycle;
    break;
  case 10:
    OCR1B = dutyCycle;
    break;
  default:
    break;
  }
}

float mapConstrainf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return constrain((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_SENSE, INPUT_PULLUP);                                    // set the sense pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_SENSE), tachISR, FALLING); // set tachISR to be triggered when the signal on the sense pin goes low

  setupTimer1();
  setPWMpin(PIN_PWM, MIN_RPM_AT_PWM);

  Thermistor *originThermistor = new NTC_Thermistor(SENSOR_PIN, REFERENCE_RESISTANCE, NOMINAL_RESISTANCE, NOMINAL_TEMPERATURE, B_VALUE);
  thermistor = new SmoothThermistor(originThermistor, SMOOTHING_FACTOR);

  myPID.SetOutputLimits(MIN_RPM_AT_PWM / 2, MAX_PWM_VALUE);
  // delay(500); // wait for the fan to spin up
  // CurrentRPM = rpmSamples.getMedianAverage(SMOOTHING_FACTOR);
  myPID.SetMode(QuickPID::Control::automatic);
// tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  // tuner.Configure(MAX_RPM, 150, 170, 290, 15, 3, 9);
}

void loop()
{
  delay(500);
  if (millis() - lastINT > FANSTUCK_THRESHOLD)
    rpmSamples.add(0);

  auto tempC = thermistor->readCelsius();
  CurrentRPM = rpmSamples.getMedianAverage(SMOOTHING_FACTOR);

  // switch (tuner.Run())
  // {
  // case tuner.sample: // active once per sample during test
  //   tuner.plotter(CurrentRPM, PWM, SetpointRPM, 1, 1);
  //   break;

  // case tuner.tunings:                            // active just once when sTune is done
  //   tuner.GetAutoTunings(&Kp, &Ki, &Kd);         // sketch variables updated by sTune
  //   myPID.SetTunings(Kp, Ki, Kd);                // update PID with the new tunings
  //   myPID.SetMode(QuickPID::Control::automatic); // the PID is turned on
  //   break;

  // case tuner.runPid: // active once per sample after tunings
    SetpointRPM = mapConstrainf(tempC, MIN_T, MAX_T, MIN_RPM, MAX_RPM);
    myPID.Compute();
  //   tuner.plotter(CurrentRPM, PWM, SetpointRPM, 1, 1);
  //   break;
  // }
  setPWMpin(PIN_PWM, PWM);
  mySerial.printf(">RPM:%f\n>SP:%f\n>PWM:%f\n>T:%2.2f\n", CurrentRPM, SetpointRPM, PWM, tempC);
  // mySerial.printf("RPM: %f\tSP: %f\tPWM: %f\tT:%2.2f\n", CurrentRPM, SetpointRPM, PWM, tempC);
}
