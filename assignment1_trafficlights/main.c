/*
 * assignment1_trafficlights.c
 *
 * Created: 2/04/2019 12:14:12 PM
 * Author : ncoo162
 */ 

#include <avr/io.h>
#include <util/delay.h>
#define F_CPU 1000000

void basicLight(void);


int main(void)
{
	
	basicLight();
	
    while (1) 
    {
    }
}

void basicLight(void)
{
	//Initialise 3 registers as output
	DDRD |= (1<<PD0) | (1<<PD1) | (1<<PD2);
	//Set outputs to 1 (off)
	PORTD |= (1<<PD0) | (1<<PD1) | (1<<PD2);
	
	while(1){
		PORTD &= ~(1<<PD2);
		_delay_ms(1000);
		PORTD |= (1<<PD2);
		PORTD &= ~(1<<PD1);
		_delay_ms(1000);
		PORTD |= (1<<PD1);
		PORTD &= ~(1<<PD0);
		_delay_ms(1000);
		PORTD |= (1<<PD0);
		
	}
}

