/*
 * irtx v1 beta
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>


#define BIT_1 {TCCR0A |= (1<<COM0A0); _delay_us(420); TCCR0A &= ~(1<<COM0A0); _delay_us(105);}
#define BIT_0 {TCCR0A |= (1<<COM0A0); _delay_us(210); TCCR0A &= ~(1<<COM0A0); _delay_us(105);}

uint8_t i, data, counter, accelValue, actuatorlValue;



uint8_t adc_read(int channel) {
	if(channel == 2) ADMUX = (1<<MUX0); //pb2 accel
	if(channel == 3) ADMUX = (1<<MUX1) | (1<<MUX0); //pb3 actuator

	ADCSRA |= (1<<ADSC); //������ ��������������
	while (ADCSRA & (1<<ADSC)); //�������� ���������� ��������������
	return ADC >> 2; //������ ���������� � ���������� � 8 ����� (data)
}



void IRdata(uint8_t data) { //generate data
	
	TCCR0A |= (1<<COM0A0);  //start bit
	_delay_us(525);
	TCCR0A &= ~(1<<COM0A0);
	_delay_us(525);
	
	for(i=0; i<8; i++) {
		if((data >> (7-i)) & 1) {
			BIT_1
		}
		else {
			BIT_0
		}
	}
	_delay_ms(8);
}



ISR(TIMER1_OVF_vect) {
	counter++;
	if(counter % 2 == 0) { //accel

		adc_read(2); //pb2
		accelValue = adc_read(2);
		IRdata(accelValue);
		
	} 
	else { //actuator
		
		adc_read(3); //pb3
		accelValue = adc_read(3);
		IRdata(actuatorlValue);
		
	}
}



int main(void) {
	
	DDRB |= (1<<PB0); //IRled signal out
	
	//ADC settings
	ADMUX = (1<<REFS0) | (1<<REFS1); //������� ����� PB3 (ADC3) ����� � ������� adc_read()
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); //��������� ��� � ��������� �������� ������� � 64 (8 ��� / 64 = 125 ���)
	
	//Timer0 PWM freq set ~38khz
	TCCR0A |= (1<<WGM00) | (1<<WGM01); //������ ��� ���
	TCCR0B |= (1<<WGM02);
	TCCR0B |= (1<<CS00); //����������� ������������ �������
	OCR0A = 210; //�������� �������� ���������(�������� ����������)

	//interrupts	
	TCCR1 |= (1<<CS00) | (1<<CS01) | (1<<CS02);	
	TIMSK |= (1<<TOIE1);
	TCNT1 = 0;
	sei();
	
	
    while(1) {
		
    }
}
