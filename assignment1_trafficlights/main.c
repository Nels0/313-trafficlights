/*
 * assignment1_trafficlights.c
 *
 * Created: 2/04/2019 12:14:12 PM
 * Author : ncoo162
 */ 

#include <avr/io.h>

#define F_CPU 1000000
#include <util/delay.h>


//Function Declarations
void basicLight(void);
void basicLight2(void);

typedef enum {
	GREEN = PB2,
	YELLOW = PB1,
	RED = PB0
	} lightColour;


lightColour nextLight(lightColour lightIn){
	switch(lightIn) {
		case GREEN	:	return YELLOW;
		case YELLOW :	return RED;
		case RED	:	return GREEN;
		default		:	return GREEN;
	}
}

//Global current light color
lightColour currentLight = RED;




int main(void)
{
	
	basicLight2();
	
    while (1) 
    {
    }
}

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

void basicLight2(void){ //Task 1 done with a timer and loops
	
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
	
	int s_tcnt = 1000000/64; //Number of ticks in a second
	
	while(1){
		if(TCNT1 > s_tcnt){ //every second
			PORTB |= (1<<currentLight); //Turn current light off
			currentLight = nextLight(currentLight); //Move to next light to switch
			PORTB &= ~(1<<currentLight); //Turn current light on
			TCNT1 = 0; //Reset timer
		}
	}
}

