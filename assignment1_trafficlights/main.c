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
#define LED3 PB3
#define LED4 PB4

//  --FUNCTION DECLARATIONS--
void basicLight(void);
void basicLight2(void);

void buttonUpdate(void);
void flashUpdate(void);

void cameraCheck(void);
void speedCheck(void);
void lightUpdate(void);

typedef enum // light colour aliases
{ GREEN = PB2,
  YELLOW = PB3,
  RED = PB0 } lightColour;


// --GLOBAL VARIABLES--

/* Variables:
 - ----Button states: DONE
 - Light period: DONE
 - Bumper timestamps
 - red light flash time/state
 - Redlight car count for PWM: DONE
 - last car speed for PWM
 */

// timestamps

volatile uint32_t start = 0;
uint32_t end = 0;

// Global current light color
lightColour currentLight = RED; // LEDs should always be updated when this is
                                // changed. Should always be accurate

int redLightCount = 0;

// Whether traffic light system is in configuration mode
bool isConfiguring = false; // for lightUpdate

int lightPeriod = 1;              // light period in s
uint32_t lastLightUpdateTick = 0; // Last time traffic lights changed colour

// Flash function state variables
uint32_t lastFlash_3 = 0;
bool flash_3 = false;
int flashCount_3 = 0;
uint32_t lastFlash_4 = 0;
int flashCount_4 = 0;

// Switch states
bool SW0 = false;
bool LB1 = false;
bool LB2 = false;
bool LB3 = false;

// Global timestamp (overflows every 50 years or so)
uint32_t currentTick = 0;

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

uint32_t tickToMS(uint32_t tick) {
  uint32_t largetick = tick * 256 * 8;
  return (largetick / 1000);
}

ISR(TIMER0_OVF_vect) { // fires every overflow of timer1 (every 2.048ms)

  buttonUpdate();
  currentTick = currentTick + 1;

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

int main(void) {

  TCCR0 = 0;
  TCCR0 |= (1 << CS01);  // Set prescaler for timer0 to 8, overflow of
                         // 8*256/1,000,000 = 2.048ms, or every 2048 cycles
  TIMSK |= (1 << TOIE0); // Enable overflow interrupt

  sei();

  // ADC Setup
  ADMUX = 0;                             // internal ref, ADC0 input
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // ADC input clock division factor of 8

  // Initialise  registers as output
  DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1<<DDB5);
  // Set outputs to 1 (off)
  PORTB |= (1 << GREEN) | (1 << YELLOW) | (1 << RED) | (1 << PB5);

  // Timer1 PWM Setup
  TCCR1A |= (1 << COM1A1) | (0 << COM1A0); // non-inverting mode
  TCCR1A |= (1 << WGM11) | (1 << WGM10);   // Fast PWM 10-bit mode
  TCCR1B |= (1 << WGM12);                  // Fast PWM 10-bit mode
  TCCR1B |=
      (1 << CS11) |
      (1 << CS10); // Prescale of 64 (PWM period of 1024*64/1000000 = 65.536ms)

  while (1) // Loop time needs to be under 10ms for red light camera
  {
    // Camera check comes before lightUpdate so that it if a car drives through
    // on the same tick as the light changes from red, it will read what the
    // light was when it was triggered

    cameraCheck();
    speedCheck();
    lightUpdate();
    flashUpdate();
  }
}



void buttonUpdate(void) {

  // Switches are all connected to portD, respectively
  SW0 |= (PORTD & (1 << PD0)) >> PD0;

  // LB1, 2, 3 represent SW5, 6, 7 respectively
  LB1 |= (PORTD & (1 << PD5)) >> PD5;
  LB2 |= (PORTD & (1 << PD6)) >> PD6;
  LB3 |= (PORTD & (1 << PD7)) >> PD7;
}

void flashUpdate(void) {
  // If red light camera is triggered while flashing is happening, then only 1
  // flash cycle will happen, uninterrupted
  if (flash_3 && tickToMS(currentTick - lastFlash_3) > 500) { // every 500ms
    if (flashCount_3 == 2 && (PORTB & (1 << LED3))) { // end of flash cycle
      // reset flash state
      PORTB &= ~(1 << LED3);
      flash_3 = false;
      flashCount_3 = 0;
    } else if (flashCount_3 < 2){
      
    }
  }

  // TODO: flashing LED4
}

void cameraCheck(void) {
  // check whether red light has been triggered
  if (LB3) {
    LB3 = false;
    // check current traffic light colour
    if (currentLight != RED) {
      return;
    } else {
      // record results
      redLightCount += 1;
      flash_3 = true;
    }
  }

  // No need to disable interrupts for 16-bit write as interrupts won't access
  // temp high reg

  // maybe

  // Disabling interrupts could make tick count inaccurate

  // Set TCNT1 for PWM compare mode
  TCNT1 = (uint16_t)(redLightCount * 1024 / 100);
}

void speedCheck(void) {
	if ((start != 0) && (end != 0)) {
		volatile uint32_t speed = 20/tickToMS(end - start)*3.6*1000;
		
	}
  // compare timestamps
  // if both are non-zero (assume that switch can't be hit within <1ms of system
  // boot) 	calculate speed 	record speed 	output speed to PWM
  // set timestamps back to zero when finished
}

void lightUpdate(void) { // Updates the traffic light, and
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
      // Stop ADC free running mode thus stop conversions
      ADCSRA &= ~(1 << ADFR);
    }

    // Read ADC
    if (ADCSRA & (1 << ADIF)) {

      uint16_t adcInput = ADC;

      lightPeriod = (int)((float)adcInput * 4.0 / 1024.0); // Find light period

      ADCSRA |= (1 << ADIF); // ADC conversion has been read
    }

    // TODO: Do light flash

  } else if (tickToMS(currentTick - lastLightUpdateTick) >= (lightPeriod * 1000)) {
    // if it's time to change the light
    PORTB |= (1 << currentLight);           // Turn current light off
    currentLight = nextLight(currentLight); // Move to next light colour
    PORTB &= ~(1 << currentLight);          // Turn current light on
    lastLightUpdateTick = currentTick;     //reset
  }
}
