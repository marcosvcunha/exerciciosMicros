/*
 * SensorTemp.cpp
 *
 * Created: 18/11/2019 16:50:28
 * Author : CLIENTE
 */ 


#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>
#include "lcd_i2c.h"
#include "uart.h"


int adc0() {
	int high;
	int low;
	DIDR0 = _BV(ADC0D);
	ADMUX = _BV(REFS0);
	ADCSRB = 0; // free running
	ADCSRA = 0;
	ADCSRA = _BV(ADEN) | _BV(ADIF) | (7 << ADPS0);
	
	ADCSRA |= _BV( ADSC);
	
	while((ADCSRA & _BV(ADSC)));
	low = ADCL;
	high = ADCH;
	return ((high<<8) | (low));

}

int adc1() {
	int high;
	int low;
	DIDR0 = _BV(ADC0D);
	ADMUX = _BV(REFS0);
	ADCSRB = 0; // free running
	ADCSRA = 0;
	ADCSRA = _BV(ADEN) | _BV(ADIF) | (7 << ADPS0);
	
	ADCSRA |= _BV( ADSC);
	
	while((ADCSRA & _BV(ADSC)));
	low = ADCL;
	high = ADCH;

	return ((high<<8) | (low));

}

int main(void)
{
	int a;
	float r;
	float temp;
	i2c_master::init();
	char valor[20];
	char valor2[20];
	
	lcd_i2c lcd;
	lcd.clear();	
	lcd.setColor(0xff, 0x00, 0x00);

	lcd.setCursor(0, 0);
	lcd.write((const char *)"Hello !!");
    /* Replace with your application code */
    while (1) 
    {
			lcd.setCursor(1,0);
			a = adc0();
			itoa(a, valor2, 10);
			r = 1023.0/a - 1.0;
			temp = 1.0/(log(r)/4275+1.0/298.15) - 273.15;
			dtostrf(temp, 8, 3, valor);
			lcd.write(valor);
			lcd.write((const char *)" ");
			lcd.write(valor2);
			_delay_ms(200);			
    }
}

