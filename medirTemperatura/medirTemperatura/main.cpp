/*
 * medirTemperatura.cpp
 *
 * Created: 15/11/2019 17:54:30
 * Author : Marcos Cunha
 */ 
#define  F_CPU 16000000UL
#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

using namespace std;

void USART_Init( unsigned int ubrr);
void sendMsg(char text[]);
void settingInterrupt();
int startADC();
int convertTemp(int read);

ISR(TIMER1_COMPA_vect){
	if(PORTD == 0){
		PORTD = (1<<2);
	}else{
		PORTD = 0;
	}
}

int main(void)
{
	DDRD = (1<<2);
	PORTD = (1<<2);
	//DDRC = 1;
	//PORTC = 1;
    USART_Init(MYUBRR);
	//settingInterrupt();
	/* Replace with your application code */
	while (1) 
    {
		char resul[40];
		int read = startADC()*4;
		int temp = convertTemp(read);
		sprintf(resul, "%d", temp);
		sendMsg(resul);
		sendMsg("\n");
		
    }
}

int startADC(){
	// REFS0 define tensão de referencia
	// ADLAR alinha o resultado a esquerda
	// MUX0 seleciona a porta que sera lida
	ADMUX = (1<<REFS0)|(1<<ADLAR)|(0<<MUX0);
	// ADEN habilita a conversão
	// ADSC inicia a conversão
	// ADIE habilita interrupção
	// ADIF não sei
	// ADPS seleciona a divisão do clock
	ADCSRA =  (1<<ADEN)|(1<<ADIF)|(0<<ADIE)|(7<<ADPS0);
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADIF)|(0<<ADIE)|(7<<ADPS0);
	
	//ADCSRB = 1;
	
	while((ADCSRA) & (1<<ADIF) == 0){}
	
	return ADCH;
}


void sendMsg(char text[]){
	for(int i = 0; text[i] != '\0'; i++){
		while (!(UCSR0A & (1 << UDRE0)));
			UDR0 = text[i];
	}
}

void USART_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	// Enable receiver and transmitter
	UCSR0B = (1<<TXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	sei();
}

void settingInterrupt(){
	// WGM seta o modo para CTC
	// CS seta o prescaler para 2014
	// OCR é o valor da comparação: 15625
	// TIMSK habilita a interrupção
	
	TCCR1B = (1<<WGM12)|(5<<CS10);
	TCCR1A = (0<<WGM10);
	OCR1AH = 0x3D;
	OCR1AL = 0x09;
	TIMSK1 = (1<<OCIE1A);
	sei();
}

int convertTemp(int read){
	int B = 4275;
	float R = 1023/read - 1;
	float temperature = 1.0/(log(R)/B+1/298.15)-273.15; 
	return (int)temperature;
}