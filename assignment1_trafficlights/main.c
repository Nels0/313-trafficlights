/*
 * assignment1_trafficlights.c
 *
 * Created: 2/04/2019 12:14:12 PM
 * Author : ncoo162
 */

#define F_CPU 1000000

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

#define PRESCALE 8

//  --FUNCTION DECLARATIONS--
void basicLight(void);
void basicLight2(void);

void buttonUpdate(void);

void cameraCheck(void);
void speedCheck(void);
int lightUpdate(int lastUpdateTick);

typedef enum // light colour aliases
{ GREEN = PB2,
  YELLOW = PB1,
  RED = PB0 } lightColour;

// --HELPER FUNCTIONS--

lightColour nextLight(lightColour lightIn) // helper function to cycle through
                                           // lights in the right order
{
  switch (lightIn) {
  case GREEN:
    return YELLOW;
  case YELLOW:
    return RED;
  case RED:
    return GREEN;
  default:
    return GREEN;
  }
}

int tickToMS(int currentTick) {
  float ratio = 256.0 * (float)PRESCALE / (float)F_CPU;
  return (int)((float)currentTick * ratio);
}

// --GLOBAL VARIABLES--

/* Variables:
 - ----Button states: DONE
 - Light period
 - Bumper timestamps
 - red light flash time/state
 - Redlight car count for PWM
 - last car speed for PWM
 */

// Global current light color
lightColour currentLight = RED; // LEDs should always be updated when this is
                                // changed. Should always be accurate

// Whether traffic light system is in configuration mode
bool isConfiguring = false; // for lightUpdate

int lightPeriod = 1; // light period in s

// Switch states
bool SW0 = false;
bool LB1 = false;
bool LB2 = false;
bool LB3 = false;

// Global timestamp (overflows every 50 years or so)
uint32_t currentTick = 0;

int main(void) {

  TCCR0 = 0;
  TCCR0 |= (1 << CS01);  // Set prescaler for timer0 to 8, overflow of
                         // 8*256/1,000,000 = 2.048ms, or every 2048 cycles
  TIMSK |= (1 << TOIE0); // Enable overflow interrupt

  sei();

  // ADC Setup
  ADMUX = 0;                             // internal ref, ADC0 input
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // ADC input clock division factor of 8

  int lastLightChangeTick = 0;
  // Initialise  registers as output
  DDRB |= (1 << GREEN) | (1 << YELLOW) | (1 << RED);
  // Set outputs to 1 (off)
  PORTB |= (1 << GREEN) | (1 << YELLOW) | (1 << RED);

  while (1) // Loop time needs to be under 10ms for red light camera
  {
    // Camera check comes before lightUpdate so that it if a car drives through
    // on the same tick as the light changes from red, it will read what the
    // light was when it was triggered

    // cameraCheck();
    // speedCheck();
    lastLightChangeTick = lightUpdate(lastLightChangeTick);
  }
}

ISR(TIMER0_OVF_vect) { // fires every overflow of timer1 (every 2.048ms)
  buttonUpdate();
  currentTick += 1;
  // TODO: speed measuring
  /* button1 pressed:
      record timestamp
      reset button bool
     button2 pressed:
      record timestamp
      calculate speed
      output to PWM (use PWM setter function?)
      reset button bool
  */
}

void buttonUpdate(void) {

  // Switches are all connected to portD, respectively
  SW0 |= (PORTD & (1 << PD0)) >> PD0;

  // LB1, 2, 3 represent SW5, 6, 7 respectively
  LB1 |= (PORTD & (1 << PD5)) >> PD5;
  LB2 |= (PORTD & (1 << PD6)) >> PD6;
  LB3 |= (PORTD & (1 << PD7)) >> PD7;
}

void cameraCheck(void) {
  // check whether red light has been triggered

  // check current traffic light colour

  // record results
  //	set PWM and set light to flash

  // set light state as necessary using timestamp for flashes
}

void speedCheck(void) {
  // compare timestamps
  // if both are non-zero (assume that switch can't be hit within <1ms of system
  // boot) 	calculate speed 	record speed 	output speed to PWM
  // set timestamps back to zero when finished
}

// Currently undecided whether this should pass by reference, as a copy, or be
// global
int lightUpdate(int lastUpdateTick) { // Updates the traffic light, and
                                      // configuration of them

  if (SW0 && !isConfiguring) { // Enter config mode on next background cycle &
                               // red light
    isConfiguring = true;
    SW0 = false;
    ADCSRA |= (1 << ADFR) |
              (1 << ADSC); // Set ADC to free running and start conversions
  }

  if (isConfiguring && currentLight == RED) {
    if (SW0) { // Exit config mode on next background cycle
      isConfiguring = false;
      SW0 = false;
      ADCSRA &= ~(1 << ADFR); // Stop ADC free running mode thus stop conversions
    }

    // Read ADC
    if (ADCSRA & (1 << ADIF)) {

      uint16_t adcInput = ADC;

      lightPeriod = (int)((float)adcInput * 4.0 / 1024.0); // Find light period

      ADCSRA |= (1 << ADIF); // ADC conversion has been read
    }

    // TODO: Do light flash

  } else if (tickToMS(currentTick - lastUpdateTick) >= lightPeriod) {
    // if it's time to change the light
    PORTB |= (1 << currentLight);           // Turn current light off
    currentLight = nextLight(currentLight); // Move to next light colour
    PORTB &= ~(1 << currentLight);          // Turn current light on
    return currentTick;
  }
  return lastUpdateTick;
}
