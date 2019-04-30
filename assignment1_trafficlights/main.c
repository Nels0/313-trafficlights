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

// Flashing LEDs
#define LED3 PC2
#define LED4 PC3

//  --FUNCTION DECLARATIONS--
void flashUpdate(void);
void cameraCheck(void);
void speedCheck(void);
void lightUpdate(void);

typedef enum // light colour aliases
{ GREEN = PB5,
  YELLOW = PB4,
  RED = PB3 } lightColour;

// --PIN MAPPING
/*
  PB1 : PWM1 : Redlight count
  PB2 : PWM2 : Last speed
  PB3 : LED0 : Red light
  PB4 : LED1 : Yellow light
  PB5 : LED2 : Green light

  PC0 : POT1 : Period select
  PC2 : LED3 : Red light flash
  PC3 : LED4 : Configmode flash
  PC6 : speed timing pin

  PD2 : SW5 : LB1
  PD3 : SW6 : LB2
  PD4 : SW7 : LB3
  PD7 : SW0 : Config switch

*/

// --GLOBAL VARIABLES--

// Speed timestamps
uint32_t start = 0;
uint32_t end = 0;

int redLightCount = 0;

// Global current light color
lightColour currentLight = RED; // LEDs should always be updated when this is
                                // changed. Should always be accurate

bool isConfiguring = false; // for lightUpdate

int lightPeriod = 1;              // seconds
uint32_t lastLightUpdateTick = 0; // Last time traffic lights changed colour

// Flash function state variables
uint32_t lastFlash_3 = 0;
bool flash_3 = false;
int flashCount_3 = 0;
uint32_t lastFlash_4 = 0;
int flashCount_4 = 0;

// Switch States
bool SW0 = false;
bool SW0_last = false;
bool LB3 = false;
bool LB3_last = false;

// Global Timestamp (overflows every 50 years or so)
uint32_t currentTick = 0;

// --HELPER FUNCTIONS--

lightColour nextLight(lightColour lightIn) // helper function to cycle through lights in the right order
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

void buttonUpdate(void) {

  // detects (rising) edge of switch state change
  SW0 |= !((PIND & (1 << PD7)) >> PD7) & !SW0_last;
  SW0_last = !((PIND & (1 << PD7)) >> PD7);

  LB3 |= !((PIND & (1 << PD4)) >> PD4) & !LB3_last;
  LB3_last = !((PIND & (1 << PD4)) >> PD4);
}

// -- Interrupts --
ISR(TIMER0_OVF_vect) { // fires every overflow of timer1 (every 2.048ms)

  buttonUpdate();
  currentTick = currentTick + 1;
}

ISR(INT0_vect) {
  start = currentTick;
  PORTC &= ~(1 << PC4); // Toggles timing bit
}

ISR(INT1_vect) {
  end = currentTick;
  PORTC |= (1 << PC4);
}

int main(void) {

  GICR |= (1 << INT0) | (1 << INT1);    // Enable external interrupts
  MCUCR |= (1 << ISC11) | (1 << ISC01); // set INT1 falling edge interrupt

  TCCR0 = 0;
  TCCR0 |= (1 << CS01);  // Set prescaler for timer0 to 8, overflow of
                         // 8*256/1,000,000 = 2.048ms, or every 2048 cycles
  TIMSK |= (1 << TOIE0); // Enable overflow interrupt

  sei();

  // ADC Setup
  ADMUX = 0;                                           // internal ref, ADC0 input
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); // ADC input clock division factor of 8

  // Initialise  registers as output
  DDRB |= UINT8_MAX;
  DDRC |= (1 << DDC2) | (1 << DDC3) | (1 << DDC4);

  // Set outputs to 1 (off)
  PORTB |= (1 << GREEN) | (1 << YELLOW) | (1 << RED);
  PORTC |= (1 << LED3) | (1 << LED4);

  // Timer1 PWM Setup
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // non-inverting mode
  TCCR1A |= (1 << WGM10) | (1 << WGM11);   // Fast PWM 10-bit mode
  TCCR1B |= (1 << WGM12);                  // Fast PWM 10-bit mode
  // Prescale of 64 (PWM period of 1024*64/1000000 = 65.54ms)
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // Set PWM outputs to "zero"
  OCR1A = 0;
  OCR1B = 0;

  while (1) // Loop time needs to be under 10ms for red light camera
  {
    // Camera check comes before lightUpdate so that it if a car drives through
    // on the same tick as the light changes from red, it will read what the
    // light was when it was triggered

    cameraCheck();
    lightUpdate();
    flashUpdate();
    speedCheck(); // called outside interrupt so it won't interrupt 16-bit write
  }
}

void flashUpdate(void) {

  // Red light camera flash
  // If red light camera is triggered while flashing is happening, then only 1
  // flash cycle will happen, uninterrupted

  if (flash_3 && tickToMS(currentTick - lastFlash_3) > 500) {
    if (flashCount_3 == 4) { // end of flash cycle
      // reset flash state
      flash_3 = false;
      flashCount_3 = 0;
    } else if (flashCount_3 < 4) { // Continue flash cycle
      flashCount_3 += 1;
      PORTC ^= (1 << LED3);
      lastFlash_3 = currentTick;
    }
  }

  // Config mode flash
  if (isConfiguring && currentLight == RED) { // Flashed=s while in active config mode
    if (flashCount_4 < lightPeriod * 2 && tickToMS(currentTick - lastFlash_4) > 500) { // every 0.5s
      PORTC ^= (1 << LED4); // Toggle light
      lastFlash_4 = currentTick;
      flashCount_4 += 1;
    } else if (flashCount_4 >= lightPeriod * 2 && tickToMS(currentTick - lastFlash_4) > 3000) { // wait 3 seconds
      flashCount_4 = 1;
      PORTC &= ~(1 << LED4);
      lastFlash_4 = currentTick;
    }
  } else { // Make sure LED is off when out of active config
    PORTC |= (1 << LED4);
  }
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

  // No need to disable interrupts for 16-bit write as interrupts won't access temp high reg
  // Disabling interrupts could make tick count inaccurate

  OCR1A = (uint16_t)(redLightCount * 1024 / 100); // output to PWM1
}

void speedCheck(void) {

  if ((start != 0) && (end != 0)) { // check if both buttons have been triggered

    int32_t speed = 20 * 3.6 * 1000 / tickToMS(end - start); // calculate speed in km/h

    // TODO: make this more proper, i.e get to 100% duty cycle properly, handle
    // overflows
    if (speed > 100) {
      speed = 99;
    } else if (speed < 0) { // Occurs when int overflows because speed is too high
      speed = 99;
    }

    OCR1B = (uint16_t)(speed * 1024 / 100); // output to PWM2

    //reset values
    start = 0;
    end = 0;
  }
}

void lightUpdate(void) { // Updates the traffic light, and configuration of them

  if (SW0) {
    if (isConfiguring && currentLight == RED) { // Exit config mode
      isConfiguring = false;
      ADCSRA &= ~(1 << ADFR);// Stop ADC free running mode thus stop conversions
    } else {   // Enter config mode
      isConfiguring = true;
      ADCSRA |= (1 << ADFR) | (1 << ADSC); // Set ADC to free running and start conversions
    }
    SW0 = false; // Switch has been read
  }

  if (isConfiguring && currentLight == RED) {

    // Read ADC if conversion finished
    if (ADCSRA & (1 << ADIF)) {

      uint16_t adcInput = ADC;

      lightPeriod = (int)((float)adcInput * 4.0 / 1024.0) + 1; // Find light period

      ADCSRA |= (1 << ADIF); // ADC conversion has been read
    }

  } else if (tickToMS(currentTick - lastLightUpdateTick) >= (lightPeriod * 1000)) {
    // if it's time to change the light
    PORTB |= (1 << currentLight);           // Turn current light off
    currentLight = nextLight(currentLight); // Move to next light colour
    PORTB &= ~(1 << currentLight);          // Turn current light on
    lastLightUpdateTick = currentTick;      // reset
  }
}
