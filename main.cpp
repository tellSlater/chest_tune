/*
 * GccApplication1.cpp
 *
 * Created: 2025-02-15 19:46:40
 * Author : ilcha
 */ 


/*

#define F_CPU   9600000
#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#define C6_OCR 71  // OCR0A value for C6 (1046.5 Hz)
#define DURATION_MS 3000  // 3 seconds duration

volatile uint16_t timer_count = 0;
volatile uint8_t play_note = 0;

ISR(TIM0_COMPA_vect) {
	if (play_note) {
		if (timer_count < (DURATION_MS * 2)) { // 2 toggles per cycle
			if (timer_count % 4 == 0) PORTB |= (1 << PINB2);  // 25% duty ON
			else if (timer_count % 4 == 1) PORTB &= ~(1 << PINB2); // 25% duty OFF
			timer_count++;
			} else {
			play_note = 0;  // Stop playing after 3 seconds
			TIMSK0 &= ~(1 << OCIE0A); // Disable interrupt
			PORTB &= ~(1 << PINB2); // Ensure output is LOW
		}
	}
}

void setup_timer0() {
	TCCR0A = (1 << WGM01);  // CTC Mode
	TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
	OCR0A = C6_OCR;
	TIMSK0 |= (1 << OCIE0A);  // Enable CTC interrupt
}

int main() {
	DDRB |= (1 << PINB2);  // Set PINB2 as output
	DDRB &= ~(1 << PINB1); // PINB1 as input
	PORTB |= (1 << PINB1); // Enable pull-up on PINB1
	sei(); // Enable global interrupts

	while (1) {
		if (!(PINB & (1 << PINB1))) { // Button press detected (active low)
			_delay_ms(50); // Debounce
			if (!(PINB & (1 << PINB1))) { // Confirm press
				play_note = 1;
				timer_count = 0;
				setup_timer0();
			}
			while (!(PINB & (1 << PINB1))); // Wait for button release
		}
	}
}

*/


/*

#define F_CPU   9600000
#include <avr/io.h>
#include <util/delay.h>

int main() {
	DDRB |= (1 << PINB2);  // Set PB2 as output

	while (1) {

		
		PORTB = (1 << PINB2);  // Toggle PB2
		_delay_us(236);         // Adjust delay for different frequencies
		PORTB = 0x00;  // Toggle PB2
		_delay_us(707); 
		
		//PORTB = (1 << PINB2);  // Toggle PB2
		//_delay_us(280);         // Adjust delay for different frequencies
		//PORTB = 0x00;  // Toggle PB2
		//_delay_us(842); 
         // Adjust delay for different frequencies
	}
}

*/


#define F_CPU   9600000
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint16_t duration = 0;


void setup_timer0() {
	TCCR0B = (1 << CS00);  // Prescaler = 1 (Timer runs at full 9.6 MHz)
	TIMSK0 = (1 << TOIE0); // Enable Timer0 Overflow Interrupt
	sei();                 // Enable global interrupts
}


void quarter_square(const uint16_t period_samples, const uint16_t duration_samples, const uint8_t pin){
	if ((duration_samples % period_samples) < (period_samples >> 2)) {
		PORTB |= 1 << pin;
	}
	else {
		PORTB &= ~(1 << pin);
	}
}


int main() {
	DDRB |= (1 << PINB2) | (1 << PINB3) | (1 << PINB4);  // Set PB2 PB3 PB4 as output
	_delay_ms(500);
	setup_timer0();
	
	
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
			TIMSK0 &= ~(1 << TOIE0); // Disable Timer0 Overflow Interrupt
		}
	}
}


ISR(TIM0_OVF_vect) {
	duration++;
}
