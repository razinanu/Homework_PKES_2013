#include <avr/io.h>
#define F_CPU 16000000    // Systemtakt in Hz
#define BAUD 9600      // Baudrate

#include <inttypes.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/setbaud.h>

#include <stdint.h>

volatile int8_t counter_trigger;
volatile int8_t counter_dir;
volatile int8_t counter_value;

#include "def.h"

extern void counter_func(void);
void init();
int uart_putc(char);
void uart_puts(const char*);
void writetoDisplay(char, char, char);
char s[30];
enum {
	Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8, Z9, MINUS_1, MINUS, SPACE
};

void init() {
	// -------------------------------------------------------------
	// Single Onboard LED configuration
	// -------------------------------------------------------------
	// set leds LED_X as output
	// alternative
	//     - intermediate
	//       DDRA=1+2+4+8;
	//     - using processor makros
	//       DDRA=((1<<DDA0) | (1<<DDA1) | (1<<DDA2) | (1<<DDA3));
	//     - using hardware specific makros
	DDRB |= (1 << DDA7);
	// disable leds
	PORTB &= ~(1 << 7);
	// -------------------------------------------------------------
	// Serial bus lines
	// -------------------------------------------------------------
	// Pin 5 = PORT E 3 = clock
	DDRE |= (1 << DDE3);
	// Pin 6 = PORT H 3 = data
	DDRH |= (1 << DDH3);
	// Pin 7 = PORT H 4 = enable
	DDRH |= (1 << DDH4);
	// -------------------------------------------------------------
	// Button configuration
	// -------------------------------------------------------------
	// not necessary but for completion
	// S1 as input
	DDRF &= ~(1 << DDG4);
	// S2 as input
	DDRG &= ~(1 << DDG5);

	// --------------------------------------------------------------
	// UART 0 - Debug
	// --------------------------------------------------------------
	UBRR0H = UBRRH_VALUE;
	;    //Define Baud rate
	UBRR0L = UBRRL_VALUE;
#if USE_2X
	UCSR0A |= (1 << U2X0);/* U2X-Modus erforderlich */
#else
	UCSR0A &= ~(1 << U2X0);/* U2X-Modus nicht erforderlich */
#endif
	UCSR0B = (1 << TXEN0); //Enable Recceiver and Transmitter
	UCSR0C = (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);
}

int main() {

	init();
	//char s[20];
	int delay = 10; // main clock [ms]
	float version = 0.2;
	char disp[] = { 0b10011111, 0b11111101, 0b10110111 };

	uart_puts("----------------------------------\r\n");
	uart_puts("PKES Wintersemester 2013/14\r\n");
	uart_puts("Vorlage 1. Aufgabe - Version ");
	uart_puts(dtostrf(version, 1, 2, s));
	uart_puts("\r\n----------------------------------\r\n");

	writetoDisplay(disp[0], disp[1], disp[2]);

	counter_trigger = COUNTER_CYCLE;
	counter_value = 0;
	counter_dir = 0;

	uart_puts("Taste druecken \r\n");
	while (1) {
		if ((PINF & (1 << 4)) || (PING & (1 << 5)))
			break;
	}

	while (1) {
		_delay_ms(delay);
		PORTB &= ~(1 << 7);
		_delay_ms(delay);
		PORTB |= (1 << 7);

		counter_func();

		sprintf(s, "trigger: %3d", counter_trigger);
		uart_puts(s);
		sprintf(s, "| count: %2d | ", counter_value);
		uart_puts(s);
		if (counter_dir == 1)
			uart_puts(" up ");
		else
			uart_puts("down");
		uart_puts(" | \r\n");

		convertToDisplay();

	}
	return 0;
}

void convertToDisplay() {

	char display[] = { 0b11111100, 0b01100000, 0b11011010, 0b11110010,
			0b01100110, 0b10110110, 0b10111110, 0b11100000, 0b11111110,
			0b11110110, 0b01100010, 0b00000010, 0b00000000 };
	int8_t rest = counter_value;
	int minus = 0;
	int i = 0;

	int digit[3] = { SPACE, SPACE, SPACE };

	if (rest == -128) {
		sprintf(s, "COUNTER VALUE IS 128: %3d", abs(rest));
		uart_puts(s);
	}

	if (rest < 0) {
		rest = abs(rest);
		minus = 1;
	}

	if (rest == 0) {
		digit[0] = 0;
	}

	while (rest > 0) {
		digit[i] = rest % 10;
		rest = rest - digit[i];
		rest = rest / 10;
		i++;
	}

	if (minus) {
		if (digit[2] == SPACE)
			digit[2] = MINUS;
		else
			digit[2] = MINUS_1;
	}

	writetoDisplay(display[digit[2]], display[digit[1]], display[digit[0]]);
}

void writetoDisplay(char digit1, char digit2, char digit3) {

	char stream[36];
	stream[0] = 1;
	int i;
	for (i = 1; i < 36; i++) {
		stream[i] = 0;
	}

	for (i = 0; i < 8; i++) {
		if (digit1 & (1 << (7 - i)))
			stream[i + 1] = 1;
		if (digit2 & (1 << (7 - i)))
			stream[i + 9] = 1;
		if (digit3 & (1 << (7 - i)))
			stream[i + 17] = 1;
	}

	// Pin 5 = PORT E 3 = clock
	// Pin 6 = PORT H 3 = data
	// Pin 7 = PORT H 4 = enable

	for (i = 0; i < 36; i++) {
		// clock low
		PORTE &= ~(1 << 3);
		// data enable low
		PORTH &= ~(1 << 4);
		_delay_us(1);
		// data
		if (stream[i] == 1)
			PORTH |= (1 << 3);
		else
			PORTH &= ~(1 << 3);
		_delay_us(1);
		// clock high - Transmission finished
		PORTE |= (1 << 3);
		_delay_us(1);
		// data enable high - ready for next cycle
		PORTH |= (1 << 4);
	}
}

int uart_putc(char c) {
	while (!(UCSR0A & (1 << UDRE0)))
		;
	UDR0 = c;
	return 0;
}

void uart_puts(const char *s) {
	while (*s) {
		uart_putc(*s);
		s++;
	}
}

