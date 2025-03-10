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
volatile uint8_t note1_samples = 0;
volatile uint8_t note2_samples = 0;
volatile uint8_t note3_samples = 0;
volatile uint8_t darkness = 6;
uint8_t note1 = 0;
uint8_t note2 = 0;
uint8_t note3 = 0;
uint8_t pwm = 0;

void setup_pins(){
	DDRB &= ~(1 << PINB2);				  // INPUT  - Interrupt sensing LDR for wakeup
	PORTB |= (1 << PINB2);				  // Enable pull-up resistor on PB2
	DDRB |= (1 << PINB0) | (1 << PINB1);  // Set PB0 (OCR0A) and PB1 (OCR0B) as outputs
	DDRB |= 1 << PINB4;
}


void setup_sleep() {
	MCUCR |= (1 << SM1) | (1 << SE);
}


void setup_timer0_pwm() {

	// Set Fast PWM mode (WGM02:WGM00 = 3)
	TCCR0A |= (1 << WGM00) | (1 << WGM01);  // Fast PWM
	TCCR0B |= (1 << CS00);                  // No prescaler

	// Set non-inverted mode on OCR0A (Clear on Compare, Set at TOP)
	TCCR0A |= (1 << COM0A1);  // Non-inverted PWM on OCR0A

	// Set inverted mode on OCR0B (Set on Compare, Clear at TOP)
	TCCR0A |= (1 << COM0B1);  // Non-inverted PWM on OCR0B

	// Set initial duty cycle (e.g., 50%)
	OCR0A = 128;  // 50% duty cycle
	OCR0B = 128;  // Same duty cycle, but inverted
}


void setup_pcint() {
	GIFR |= (1 << PCIF);     // Clear pci flag
    GIMSK |= (1 << PCIE);    // Enable pc interrupt
    PCMSK |= (1 << PCINT2);  // Enable PCINT2 (PB2)
}


void setup_watchdog() {
	MCUSR |= 0x00;	
	
    MCUSR &= ~(1 << WDRF);                             // Clear watchdog reset flag (prevents unintended resets)
    WDTCR = (1 << WDCE) | (1 << WDE);                  // Enable watchdog change sequence
    WDTCR = (1 << WDTIE) | (1 << WDP3) | (1 << WDP0);  // Interrupt-only every 8s, no reset
}


void dbg(uint8_t x) {
	PORTB &= ~(1 << PINB4);
	_delay_ms(1000);
	while (x) {
		PORTB |= 1 << PINB4;
		_delay_ms(200);
		PORTB &= ~(1 << PINB4);
		_delay_ms(200);
		x--;
	}
	_delay_ms(1000);
}


void sleep() {
	TIMSK0 &= ~(1 << TOIE0);  // Disable Timer0 Overflow Interrupt
	DDRB &= ~((1 << PINB0) | (1 << PINB1));  // Disable PWM

	//dbg(3);
	while (duration) sleep_mode();
	//dbg(4);
	_delay_ms(800);

	DDRB |= (1 << PINB0) | (1 << PINB1);  // Set PB0 (OCR0A) and PB1 (OCR0B) as outputs
	GIMSK &= ~(1 << PCIE);  // Disable pin change interrupt
	TIMSK0 = (1 << TOIE0);  // Enable Timer0 Overflow Interrupt
}


void quarter_square(const uint16_t note_period_samples, volatile uint8_t* sample_counter_ptr, uint8_t* note) {
	if (*sample_counter_ptr < (note_period_samples >> 2)) {
		*note = 1;
	}
	else if (*sample_counter_ptr < (note_period_samples)) {
		*note = 0;
	}
	else {
		*sample_counter_ptr = 0;
	}
}


void output() {
	pwm = note1 * 85 + note2 * 85 + note3 * 85;
	OCR0A = pwm;
	OCR0B = 255 - pwm;
}


int main() {
	cli();  // Disable interrupts golbaly
	setup_watchdog();
	setup_sleep();
	setup_pins();
	setup_pcint();
	setup_timer0_pwm();
	//dbg(7);
	sei();  // Enable global interrupts


	while (1) {		
		if (duration < 6244){
			quarter_square(42, &note1_samples, &note1);
			quarter_square(71, &note2_samples, &note2);
			quarter_square(106, &note3_samples, &note3);
			output();
		}
		else if (duration < 2*6244){
			quarter_square(40, &note1_samples, &note1);
			quarter_square(67, &note2_samples, &note2);
			quarter_square(100, &note3_samples, &note3);
			output();
		}
		else if (duration < 3*6244){
			quarter_square(38, &note1_samples, &note1);
			quarter_square(63, &note2_samples, &note2);
			quarter_square(94, &note3_samples, &note3);
			output();
		}
		else if (duration < (8*6244U)){
			quarter_square(35, &note1_samples, &note1);
			quarter_square(60, &note2_samples, &note2);
			quarter_square(89, &note3_samples, &note3);
			output();	
		}
		else {
			sleep();
		}
	}
}


ISR(TIM0_OVF_vect) {  // Executes at 37500Hz when chip is not sleeping
	duration++;
	note1_samples++;
	note2_samples++;
	note3_samples++;
}


ISR(PCINT0_vect) {         // Executes on light change
	//dbg(6);
	duration = 0;
	darkness = 0;
}


ISR (WDT_vect) {							     //Wake from sleep and check darkness once every 8sec
	if (PINB & 1 << PINB2) {
		//dbg(5);
		if (darkness < 0xff) darkness++;
		if (darkness >=0) GIMSK |= (1 << PCIE);  // Enable pin change interrupt
	}
	else {
		darkness = 0;
		GIMSK &= ~(1 << PCIE);                   // Disable pin change interrupt
    }
}

