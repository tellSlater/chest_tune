/*
 * chest_tune.cpp
 * 
 *
 * Chip used: ATTiny13A
 * The internal oscillator and no prescaling is used for this project.
 * The state of the chip's fuses should be (E:FF, H:FF, L:6A) when not locked.
 *
 *								 _________
 * PIN1 - N/C       		   _|	 O    |_		PIN8 - VCC
 * PIN2	- Note 2     		   _|		  |_		PIN7 - Note 1
 * PIN3	- Note 3        	   _|ATTiny13A|_		PIN6 - Light sensor LDR
 * PIN4	- Ground			   _|		  |_		PIN5 - Virtual GND
 *							    |_________|
 */ 


#define F_CPU   9600000
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


volatile uint16_t duration = 0;


void setup_pins(){
	DDRB |= 1 << PINB0;	    // OUTPUT - Sets virtual ground
	DDRB &= ~(1 << PINB1);  // INPUT  - Interrupt sensing LDR for wakeup
	PORTB |= (1 << PINB1);  // Enable pull-up resistor on PB1
	DDRB |= 1 << PINB2;	    // OUTPUT - Note 1
	DDRB |= 1 << PINB3;	    // OUTPUT - Note 2
	DDRB |= 1 << PINB4;	    // OUTPUT - Note 3
}


void setup_sleep() {
	MCUCR |= (1 << SM1) | (1 << SE);
}


void setup_timer0() {
	TCCR0B = (1 << CS00);   // Prescaler = 1 (Timer runs at full 9.6 MHz)
	TIMSK0 = (1 << TOIE0);  // Enable Timer0 Overflow Interrupt
}


void setup_int0() {
    GIMSK |= (1 << INT0);    // Enable INT0 interrupt
}


void sleep() {
	TIMSK0 &= ~(1 << TOIE0);  // Disable Timer0 Overflow Interrupt
	PORTB &= ~(1 << PINB0);   // Virtual ground off
	GIMSK |= (1 << INT0);   // Enable INT0 interrupt

	sleep_mode();
	_delay_ms(800);

	GIMSK &= ~(1 << INT0);  // Disable INT0 interrupt
	PORTB |= 1 << PINB0;    // Virtual ground on
	TIMSK0 = (1 << TOIE0);  // Enable Timer0 Overflow Interrupt
	duration = 0;
}


void quarter_square(const uint16_t period_samples, const uint16_t duration_samples, const uint8_t pin) {
	if ((duration_samples % period_samples) < (period_samples >> 2)) {
		PORTB |= 1 << pin;
	}
	else {
		PORTB &= ~(1 << pin);
	}
}


int main() {
	//_delay_ms(3000);
	setup_sleep();
	setup_pins();
	setup_int0();
	setup_timer0();

	sei();  // Enable global interrupts

	while (1) {
		if (duration < 6244){
			quarter_square(42, duration, PINB2);
			quarter_square(71, duration, PINB3);
			quarter_square(106, duration, PINB4);
		}
		else if (duration < 2*6244){
			quarter_square(40, duration - 6244, PINB2);
			quarter_square(67, duration, PINB3);
			quarter_square(100, duration, PINB4);
		}
		else if (duration < 3*6244){
			quarter_square(38, duration - 2*6244, PINB2);
			quarter_square(63, duration, PINB3);
			quarter_square(94, duration, PINB4);
		}
		else if (duration < (8*6244U)){
			quarter_square(35, duration - 3*6244, PINB2);
			quarter_square(60, duration, PINB3);
			quarter_square(89, duration, PINB4);
		}
		else {
			sleep();
		}
	}
}


ISR(TIM0_OVF_vect) {
	duration++;
}


ISR(PCINT0_vect) {
	duration = 0;
}
