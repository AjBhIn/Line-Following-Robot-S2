/*
 * Motor Coding.c
 *
 * Created: 8/10/2025 11:53:13 am
 * Author : oldaj
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Functions.
void setUp();
int centreSen();
int senPos();
int leftSen();
int rightSen();
int leftErrorRec(int sensorL, int sensorR, int preLeftValue);
int rightErrorRec(int sensorL, int sensorR, int preRightValue);
void OneSensorON(int sensorL, int sensorR, int error);
void centreON(int sensorL, int sensorR, int error);
void HardTuring(int rightErrors, int leftErrors, int err);
int lastError(int sensorL, int sensorR);
int sw1();
int sw2();
void backing();

// Timing
void timer1_init(void);
unsigned long millis(void);
void wait_ms(unsigned long ms);


// Sensor Following Equation
int position = 0;
#define leftWei -1
#define rightWei 1
#define kp 70
#define kd 40
#define ki 1.3
#define alpha 0.55
#define limitsErros 8

int integral = 0;
int derva = 0;
int correction = 0;

// Error
int preError = 0;
int preLeftValue = 0;
int preRightValue = 0;
#define dataSample 20
int instantLastError = 0;

// Speed
#define baseSpeed 220
#define turningSpeed 255
#define turningTime 48
#define runAbleSpeed 50
#define backwardsTime 20
int countdown = 0;
int backingCount = 0;

// Timing
volatile unsigned long millis_count = 0;


// Reversing:
#define motABack 0b00001000	
#define motBBack 0b00000001

// Forward
#define motfor	0

// Sensors
#define L ((PINA >> PINA7) & 1)
#define C ((PINA >> PINA6) & 1)
#define R ((PINA >> PINA3) & 1)
int switch1;
int switch2;

int main(void)
{
	setUp();
	timer1_init();
	
	// The center Sensor
	while (1)
	{	
		preLeftValue = 0;
		preRightValue = 0;
		derva = 0;
		integral = 0;
		countdown = 0;
		preError = 0;
		backingCount = 0;
		
		// When center is ON
		while (C) {
			// Sensor Values
			int leftS = L;
			int rightS = R;
			// Action Taker
			centreON(leftS, rightS, senPos(leftS, rightS));

			// Recording Previous Errors
			preRightValue = rightErrorRec(leftS, rightS, preRightValue);
			preLeftValue = leftErrorRec(leftS, rightS, preLeftValue);
			preError = lastError(leftS, rightS);
			
			// backing if switch pressed
			if (!sw1() || !sw2()) {
				while (1) {
					backingCount += 1;
					
					backing();
					if ((backingCount > backwardsTime)) {
						PORTB = 0;
						break;
					}
					
					wait_ms(8);
				}
			}

			wait_ms(1);

		}
	
		backingCount = 0;

		// When at least one Sensors is ON
		while (!C && (L || R)) {
			// Sensor Values
			int leftS = L;
			int rightS = R;

			// Action Taker
			OneSensorON(leftS, rightS, senPos(leftS, rightS));

			// Recording Previous Errors
			preRightValue = rightErrorRec(leftS, rightS, preRightValue);
			preLeftValue = leftErrorRec(leftS, rightS, preLeftValue);
			preError = lastError(leftS, rightS);
			
			// backing
			if (!sw1() || !sw2()) {
				while (1) {
					backingCount += 1;
					
					backing();
					if ((backingCount > backwardsTime)) {
						PORTB = 0;
						break;
					}
					
					wait_ms(8);
				}
			}

			wait_ms(1);
		}
		
		backingCount = 0;

		// Harding Turning:
		while (!C && !L && !R)	{
			int centS = C;
			int rightS = R;
			int leftS = L;
			countdown += 1;

			// Action Taker
			HardTuring(preRightValue, preLeftValue, preError);

			// Checking if it the track or should stop
			if (leftS || centS || rightS) {
				break;
			}
			else if (countdown > turningTime) {
				OCR0B = 0;
				OCR0A = 0;
				PORTA &= ~(1 << PORTA1);
				PORTC &= ~(1 << PORTC4);
				preLeftValue = 0;
				preRightValue = 0;
				preError = 0;
			}
			
			// Backing
			if (!sw1() || !sw2()) {
				while (1) {
					backingCount += 1;
					
					backing();
					if ((backingCount > backwardsTime)) {
						PORTB = 0;
						break;
					}
					
					wait_ms(8);
				}
			}
			wait_ms(10);
		}
			
		// If nothing keep it stopped.
		OCR0A = 0;
		OCR0B = 0;
		PORTA &= ~(1 << PORTA1);
		PORTC &= ~(1 << PORTC4);
		PORTB = 0;
		

	}
}


void setUp() {
	// Motor Set up
	DDRA =			0b00100010;
	PORTA |=		0b00100000;
	DDRC =			0b00010001;
	PORTC |=		0b00000001;
	DDRB =			0b00001001;
	PORTB |=		0b00000000;


	TCCR0A = (1 << WGM00); // Setting it to fast PWM
	TCCR0B = (1 << CS01) | (1 << CS00); // Speed of the Clock (Prescaling)

	// Both Motors Mode
	TCCR0A |= (1 << COM0B1);
	TCCR0A &= ~(1 << COM0B0); // Non inverting mode
	TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0); // Non inverting mode
	
}

// Timing setup
void timer1_init(void) {
	// Normal mode, CTC with OCR1A
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  // CTC mode, prescaler = 64
	OCR1A = 125;    // (8 MHz / 64) / 1000 Hz = 125 ? 1 ms period
	TIMSK |= (1 << OCIE1A);   // enable compare match interrupt
	sei();  // enable global interrupts
}

// increasing the timer
ISR(TIMER1_COMPA_vect) {
	millis_count++;
}

// Reading the current value
unsigned long millis(void) {
	unsigned long m;
	cli();
	m = millis_count;
	sei();
	return m;
}

// Specific Time Setter
void wait_ms(unsigned long ms) {
	unsigned long start = millis();
	while ((millis() - start) < ms) {
		// do nothing — this still allows interrupts to run
	}
}


int senPos(int l, int r) {
	return ((leftWei*l) + (rightWei*r));
}

// Error Recording
int leftErrorRec(int sensorL, int sensorR, int preLeftValue) {
	if (sensorL && !sensorR) {
		preLeftValue += 1;        // line leaning left
	}
	if (preLeftValue > dataSample) preLeftValue = dataSample;
	return preLeftValue;
}

int rightErrorRec(int sensorL, int sensorR, int preRightValue) {
	if (sensorR && !sensorL) {
		preRightValue += 1;       // line leaning right
	}
	if (preRightValue > dataSample) preRightValue = dataSample;
	return preRightValue;
}


int lastError(int sensorL, int sensorR) {
	if (sensorR) {
		return 1;
	}
	else if (sensorL) {
		return -1;
	}
	return 0;
}

int sw1() { // active low
	return ((PINA) & (1 << PINA0));
}

int sw2() { // active low
	return ((PINA) & (1 << PINA2));
}

void backing() {
	PORTB = 0b00001001;
	OCR0B = 0;
	OCR0A = 0;
}



// PDI
int PDI(int error) {
	int delta = error - instantLastError;
	derva = (int)((alpha*derva) + (1 - alpha) * delta);
	integral += error;
	if (integral > limitsErros) {
		integral = limitsErros;
	}
	else if (integral <  -(limitsErros)) {
		integral = -(limitsErros);
	}
	
	instantLastError = error;
	correction = (int)((kp * error) + (kd * derva) + (ki * integral));
	
	return correction;
}


// When one sensor is on the line
void OneSensorON(int sensorL, int sensorR, int error) {
	
	int corre = PDI(error);
	
	if (sensorL) { // left ON
		OCR0A = baseSpeed + corre;
		if (OCR0A < runAbleSpeed) {
			OCR0A = (runAbleSpeed - 20);
		}
		else if (OCR0A > 255) {
			OCR0A = 255;
		}
		OCR0B = baseSpeed - corre;
		if (OCR0B > 255) {
			OCR0B = 255;
		}
		else if (OCR0B < runAbleSpeed) {
			OCR0B = (runAbleSpeed - 20);
		}
		PORTA |= (1 << PORTA1);
		PORTC &= ~(1 << PORTC4);
	}
	else if (sensorR) { // right ON
		OCR0A = baseSpeed + corre;
		if (OCR0A > 255) {
			OCR0A = 255;
		}
		else if (OCR0A < runAbleSpeed) {
			OCR0A = (runAbleSpeed - 20);
		}
		OCR0B = baseSpeed - corre;
		if (OCR0B < runAbleSpeed) {
			OCR0B = (runAbleSpeed - 20);
		}
		else if (OCR0B > 255) {
			OCR0B = 255;
		}
		PORTC |= (1 << PORTC4);
		PORTA &= ~(1 << PORTA1);
	}
}


void centreON(int sensorL, int sensorR, int error) {
	
	OCR0A = 255;
	OCR0B = 255;
	
	int corre = PDI(error);

	if (sensorL) { // left ON
		OCR0A = baseSpeed + corre; // baseSpeed + (-correction)
		if (OCR0A < runAbleSpeed) {
			OCR0A = runAbleSpeed;
		}
		else if (OCR0A > 255) {
			OCR0A = 255;
		}
		OCR0B = baseSpeed - corre; // baseSpeed - (-correction)
		if (OCR0B > 255) {
			OCR0B = 255;
		}
		else if (OCR0B < runAbleSpeed) {
			OCR0B = runAbleSpeed;
		}
		PORTA |= (1 << PORTA1);
		PORTC &= ~(1 << PORTC4);
	}
	else if (sensorR) { // right ON
		OCR0A = baseSpeed + corre;
		if (OCR0A < runAbleSpeed) {
			OCR0A = runAbleSpeed;
		}
		else if (OCR0A > 255) {
			OCR0A = 255;
		}
		OCR0B = baseSpeed - corre;
		if (OCR0B > 255) {
			OCR0B = 255;
		}
		else if (OCR0B < runAbleSpeed) {
			OCR0B = runAbleSpeed;
		}
		PORTC |= (1 << PORTC4);
		PORTA &= ~(1 << PORTA1);
	}
}


void HardTuring(int rightErrors, int leftErrors, int err) {
	if (leftErrors > rightErrors) {
		OCR0A = turningSpeed;
		OCR0B = 10;
	}
	else if (rightErrors > leftErrors) {
		OCR0A = 10;
		OCR0B = turningSpeed;
	}
	else { // makes a little nudge to see where it needs to go
		if (err == -1) {
			OCR0A = turningSpeed;
			OCR0B = 20;
		}
		else if (err) {
			OCR0A = 20;
			OCR0B = turningSpeed;
		}
	}
		
		
}
