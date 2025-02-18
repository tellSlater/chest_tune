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


volatile uint16_t duration = 0xffff;
volatile uint8_t darkness = 4;


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
	TCCR0B = (1 << CS00);     // Prescaler = 1 (Timer runs at full 9.6 MHz)
	//TIMSK0 = (1 << TOIE0);  // Enable Timer0 Overflow Interrupt
}


void setup_pcint() {
	GIFR |= (1 << PCIF);     // Clear pci flag
    GIMSK |= (1 << PCIE);    // Enable pc interrupt
    PCMSK |= (1 << PCINT1);  // Enable PCINT1 (PB1)
}


void setup_watchdog() {
	MCUSR |= 0x00;					                   //Watchdog settings
	//WDTCR |= (1 << WDCE);				   //Enable changing watchdog settings (lasts 4 clock cycles)
	//WDTCR &= ~(1 << WDE);
	//WDTCR = (1 << WDTIE) | (1 << WDP0) | (1 << WDP3);  //Once every 8", interrupt only mode
	
    MCUSR &= ~(1 << WDRF);  // Clear watchdog reset flag (prevents unintended resets)

    // Enable watchdog change sequence
    WDTCR = (1 << WDCE) | (1 << WDE);
    
    // Set watchdog to **interrupt-only mode** (fires ISR every 8s, no reset)
    WDTCR = (1 << WDTIE) | (1 << WDP3) | (1 << WDP0);
}


void sleep() {
	TIMSK0 &= ~(1 << TOIE0);  // Disable Timer0 Overflow Interrupt
	PORTB &= ~(1 << PINB0);   // Virtual ground off
	GIMSK |= (1 << INT0);     // Enable INT0 interrupt
	PORTB &= ~(1 << PINB0);   // Disconnect virtual ground

	while (duration) sleep_mode();
	_delay_ms(800);

	PORTB |= 1 << PINB0;    // Connect virtual ground
	GIMSK &= ~(1 << INT0);  // Disable INT0 interrupt
	PORTB |= 1 << PINB0;    // Virtual ground on
	TIMSK0 = (1 << TOIE0);  // Enable Timer0 Overflow Interrupt
}


void quarter_square(const uint16_t period_samples, const uint16_t duration_samples, const uint8_t pin) {
	if ((duration_samples % period_samples) < (period_samples >> 2)) {
		PORTB |= 1 << pin;
	}
	else {
		PORTB &= ~(1 << pin);
	}
}


void dbg(uint8_t x) {
	PORTB &= ~(1 << PINB0);
	_delay_ms(1000);
	while (x) {
		PORTB |= 1 << PINB0;
		_delay_ms(200);
		PORTB &= ~(1 << PINB0);
		_delay_ms(200);
		x--;
	}
	_delay_ms(1000);
}


int main() {
	cli();  // Disable interrupts golbaly
	setup_watchdog();
	setup_sleep();
	setup_pins();
	setup_pcint();
	setup_timer0();
	dbg(5);
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
	dbg(2);
	//PORTB ^= 1 << PINB0;
	if (darkness >= 3) {
		duration = 0;
		darkness = 0;
	}
}


ISR (WDT_vect) {  //Wake from sleep and check darkness once every 8sec
	if (PINB & 1 << PINB1) {
		if (darkness < 0xff) darkness++;
	}
	else darkness = 0;	
}