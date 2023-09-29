#include <Arduino.h>
#include <stdio.h>
static int serial_putc(const char c, FILE *stream) { return Serial.write(c); }
static FILE *serial_stream = fdevopen(serial_putc, NULL);

#define PIN_SENSE 2            // where we connected the fan sense pin. Must be an interrupt capable pin (2 or 3 on Arduino Uno)
#define PIN_PWM 9              // where we connected the fan PWM pin
#define DEBOUNCE 0             // 0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
#define FANSTUCK_THRESHOLD 500 // if no interrupts were received for 500ms, consider the fan as stuck and report 0 RPM
// Interrupt handler. Stores the timestamps of the last 2 interrupts and handles debouncing
unsigned long volatile ts1 = 0, ts2 = 0;
void tachISR()
{
  unsigned long m = millis();
  if ((m - ts2) > DEBOUNCE)
  {
    ts1 = ts2;
    ts2 = m;
  }
}
// Calculates the RPM based on the timestamps of the last 2 interrupts. Can be called at any time.
unsigned long calcRPM()
{
  if (millis() - ts2 < FANSTUCK_THRESHOLD && ts2 != 0)
  {
    return (60000 / (ts2 - ts1)) / 2;
  }
  else
    return 0;
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

float getPWMpin(uint8_t pin)
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
  stdout = serial_stream;
  pinMode(PIN_SENSE, INPUT_PULLUP);                                    // set the sense pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_SENSE), tachISR, FALLING); // set tachISR to be triggered when the signal on the sense pin goes low
  Serial.begin(9600);

  // enable serial so we can see the RPM in the serial monitor
  // enable outputs for Timer 1
  pinMode(9, OUTPUT);  // 1A
  pinMode(10, OUTPUT); // 1B
  setupTimer1();

  // note that pin 11 will be unavailable for output in this mode!
  // example...
  setPWMpin(PIN_PWM, 0.0f); // set duty to 50% on pin 9
}

void loop()
{
  delay(100);
  auto rpm = calcRPM();
  printf("RPM: %lu\n", rpm);
  // increase the duty cycle by 2% every until fan is spinning
  if (rpm == 0)
  {
    auto pwm = getPWMpin(PIN_PWM) + 0.02f;
    printf("increasing PWM: %f\n", pwm);
    setPWMpin(PIN_PWM, pwm);
    return;
  }
}