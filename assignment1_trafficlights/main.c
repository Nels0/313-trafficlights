/*
 * assignment1_trafficlights.c
 *
 * Created: 2/04/2019 12:14:12 PM
 * Author : ncoo162
 */ 

#define F_CPU 1000000

#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>


//Function Declarations
void basicLight(void);
void basicLight2(void);

void buttonUpdate(void);

void cameraCheck(void);
void speedCheck(void);
void lightUpdate(void);


typedef enum { //light colour aliases
	GREEN = PB2,
	YELLOW = PB1,
	RED = PB0
	} lightColour;


lightColour nextLight(lightColour lightIn){ //helper function to cycle through lights in the right order
	switch(lightIn) {
		case GREEN	:	return YELLOW;
		case YELLOW :	return RED;
		case RED	:	return GREEN;
		default		:	return GREEN;
	}
}

//Global current light color
lightColour currentLight = RED; //LEDs should always be updated when this is changed. Should always be accurate

//Whether traffic light system is in configuration mode
bool isConfiguring = false; // for lightUpdate

bool SW0 = false;
bool LB1 = false;
bool LB2 = false;
bool LB3 = false;

void buttonUpdate(void){
	
	//Switches are all connected to portD, respectively
	SW0 |= (PORTD & (1<<PD0)) >> PD0;
	
	//LB1, 2, 3 represent SW5, 6, 7 respectively
	LB1 |= (PORTD & (1<<PD5)) >> PD5;
	LB2 |= (PORTD & (1<<PD6)) >> PD6;
	LB3 |= (PORTD & (1<<PD7)) >> PD7;
	
	return;
}

//1ms timer interrupt
/*
void timer() 
{
	time +=1;
	
	//Check switches
	if a bumper switch is pressed{
		// <1.5ms response time key
		record timestamp
		set output pin high as proof on oscilloscope
	}
	if red light switch pressed
	{
		set switch boolean
	}
	if config switch pressed 
	{
		set boolean
	}
	
	
		
}
*/

ISR(TIMER0_OVF_vect){
	buttonUpdate();
}

int main(void)
{
	

	
	TCCR0 = 0;
	TCCR0 |= (1 << CS00); // Set prescaler for timer0 to 0, overflow of 256/1,000,000 = 0.256ms, or every 256 cycles
	TIMSK |= (1 << TOIE0); //Enable overflow interrupt
	
	sei();
	
    while (1) //Loop time needs to be under 10ms for red light camera
    {
		// Camera check comes before lightUpdate so that it if a car drives through on the same tick as the light changes from red, it will read what the light was when it was triggered
		
		
		//cameraCheck
		//speedCheck
		//lightUpdate 
		


    }
}


void cameraCheck(void)
{
	//check whether red light has been triggered 
	
	//check current traffic light colour
	
	//record results
	//	set PWM and set light to flash
	
	//set light state as necessary using timestamp for flashes
}

void speedCheck(void)
{
	//compare timestamps
	//if both are non-zero (assume that switch can't be hit within <1ms of system boot)
	//	calculate speed
	//	record speed
	//	output speed to PWM
	//	set timestamps back to zero when finished
	
}

void lightUpdate(void)
{
	//TODO: check if config switch pressed and handle that
	
	//check config mode
	//if not config
	//	check timestamps and cycle light
	//if config
	//	read ADC
	//	update light flash state
	
}


/* Variables:
 - Button states
 - Light period
 - Bumper timestamps
 - red light flash time/state
 - Redlight car count for PWM
 - last car speed for PWM
 */

void basicLight(void) //Task 1 done naively
{
	//Initialise 3 registers as output
	DDRB |= (1<<GREEN) | (1<<YELLOW) | (1<<RED);
	//Set outputs to 1 (off)
	PORTB |= (1<<GREEN) | (1<<YELLOW) | (1<<RED);
	
	while(1){
		PORTB &= ~(1<<PB2);
		_delay_ms(1000);
		PORTB |= (1<<PB2);
		PORTB &= ~(1<<PB1);
		_delay_ms(1000);
		PORTB |= (1<<PB1);
		PORTB &= ~(1<<PB0);
		_delay_ms(1000);
		PORTB |= (1<<PB0);
		
	}
	
}

void basicLight2(void) //Task 1 done with a timer and loops
{
	
	//Initialise  registers as output
	DDRB |= (1<<GREEN) | (1<<YELLOW) | (1<<RED);
	//Set outputs to 1 (off)
	PORTB |= (1<<GREEN) | (1<<YELLOW) | (1<<RED);
	
	
	//Timer Setup
	
	uint8_t cTCCR1B = TCCR1B;	
	//Set normal operation timer mode
	TCCR1A = 0;
	cTCCR1B &= ~(1<<WGM12);
	cTCCR1B &= ~(1<<WGM13);	
	//Set pre-scaler to 1/64
	cTCCR1B |= (1<<CS10) | (1<<CS11);
	cTCCR1B &= ~(1<<CS12);	
	//Commit register settings
	//TCCR1B should = 0000011
	TCCR1B = cTCCR1B;
	
	int s_tcnt = F_CPU/64; //Number of ticks in a second
	
	while(1){
		if(TCNT1 > s_tcnt){ //every second
			PORTB |= (1<<currentLight); //Turn current light off
			currentLight = nextLight(currentLight); //Move to next light to switch
			PORTB &= ~(1<<currentLight); //Turn current light on
			TCNT1 = 0; //Reset timer
		}
	}
}

