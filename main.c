/*
 * irtx v1 beta
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>

#define BIT_1 {TCCR0A |= (1 << COM0A0); _delay_us(420); TCCR0A &= ~(1 << COM0A0); _delay_us(115);} //bit 1(long) ~38khz pulse duration 420us
#define BIT_0 {TCCR0A |= (1 << COM0A0); _delay_us(210); TCCR0A &= ~(1 << COM0A0); _delay_us(115);} //bit 0(short) ~38khz pulse duration 210us

uint8_t i, counter, transmitAccel, transmitActuator;
uint16_t accel, actuator;



uint16_t adc_read(int channel) {
	
	if (channel == 2) {
		ADMUX &= 0xF8;
		ADMUX |= (1 << MUX0); //pb2 accel
	}
	if (channel == 3) {
		ADMUX &= 0xF8;
		ADMUX |= (1 << MUX1) | (1 << MUX0); //pb3 actuator(left/right terns)
	}
	
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADSC); //start conversion
	while (ADCSRA & (1 << ADSC)); //Ожидание завершения преобразования
	return ADC; 
}



void IRdata(uint16_t data) { //generate data
	
	TCCR0A |= (1 << COM0A0);  //start bit
	_delay_us(525);
	TCCR0A &= ~(1 << COM0A0);
	_delay_us(525);
	
	for (i = 0; i < 8; i++) { //8bit data
		if ((data >> (7 - i)) & 1) {
			BIT_1
		}
		else {
			BIT_0
		}
	}
}



ISR(TIMER1_OVF_vect) {
	counter++;
	if (counter % 2 == 0) {
		accel = adc_read(2); //pb2
		transmitAccel = accel / 8; //0-127
		IRdata(transmitAccel);
	}
	else {
		actuator = adc_read(3); //pb3
		transmitActuator = (actuator / 8) + 128; //127-255
		IRdata(transmitActuator);
	}	
}



int main(void) {
	
	DDRB |= (1 << PB0); //IRled signal out
	
	//ADC settings
	ADMUX = (1 << REFS1) | (1 << REFS2);
	ADCSRA = (1 << ADPS2) | (1 << ADPS1); //set prescaler at 64 (8 mhz / 64 = 125 khz)
	
	//Timer0 PWM freq set on ~38khz
	TCCR0A |= (1 << WGM00) | (1 << WGM01); //задаем мод ШИМ
 	TCCR0B |= (1 << WGM02);
	TCCR0B |= (1 << CS00); //настраиваем предделитель частоты
	OCR0A = 210; //значение регистра сравнения(задающее напряжение)

	//timer1 interrupts	
	TCCR1 |= (1 << CS10) | (1 << CS13);	
	TIMSK |= (1 << TOIE1);
	TCNT1 = 0;
	sei();
	
	
    while (1) {
		/////
	}
	
}
