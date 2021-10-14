/*
 * Sprint_10_escravo.c
 *
 * Created: 08/10/2021 19:49:17
 * Author : bruno
 */ 
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>


//Protótiopo das funções
void anima_semaforo(uint8_t i);
void USART_Transmit(unsigned char data);


//Interrupção gerada pelo recebimento via USART
ISR(USART_RX_vect)
{
	anima_semaforo(UDR0 - '0');
}
//Função que determina o funcionamento do semáforo
void anima_semaforo(uint8_t i)
{
	//Variáveis que auxiliam na operação, cada posição de "estados"
	//representa quantos leds acessos codificados em 4 bits
	const uint16_t estados[9] = { 0b00010001, 0b00011000, 0b00011001, 0b00100000,0b00000000, 0b00000001, 0b00001000, 0b00001001 ,0b0001000};
	//LEDS serão acessos de acordo com essa função
	PORTB = estados[i] & 0b01111111;

}

//Função para envio de um frame de 5 a 8 bits
void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0))); //Espera limpeza do registrador de transmissão
	UDR0 = data;
}
int main(void)
{
	//Configurando entradas e saídas
	DDRB = 0b11111111;  //Todos os pinos B como saída
	DDRD |= 0b10001000; //Pinos D3 e D7 como saída
	PORTC &= 0b11111110; //Desabilitando o pull-up de C0
	PORTC |= 0b01000000; //Habilitando o pull-up de C6
	PORTD |= 0b01111111; //Definindo os pinos de pull-up D4-D6
	
	//Cofigurando as interrupções
	EICRA  |= 0b00000010;//Interrupção por borda de subida de INT0
	EIMSK  |= 0b00000001;//Habilita interrupção do INT0
	PCICR  = 0b00000100;//Habilita interrupção pelos pinos D
	PCMSK2 = 0b01111111;//Habilita as interrupções dos pinos PD2,PD4, PD5 e PD6
	
	//Configurando timer 1ms
	TCCR0A = 0b00000010; //habilitando o modo CTC do TC0
	TCCR0B = 0b00000011; //ligando com prescaler = 64
	OCR0A = 249; //Ajustando a contagem até 250
	TIMSK0 = 0b00000010; //Habilitando a interrupção na igualdade de comparação com OCR0A
	
	//Configurando o ADC
	ADMUX = 0b01000000; //Vcc como referência, canal 0
	ADCSRA = 0b11100111; //Habilita o AD, modo conversão contínua, prescaler = 128
	ADCSRB = 0b00000000; //Modo de conversão contínua
	DIDR0 = 0b00000000; //Habilita o PC0 como entrada do ADC0
	
	//Configurando o PWM
	OCR2A = 255; //Ajustando periodo do PWM, TOP-1 = 16M/(244*256) = 256
	TCCR2A = 0b00100011; //PWM rápido, ativa PWM no OC2B não invertido
	TCCR2B = 0b00011010; //Prescale 256
	OCR2B = 76; //Duty inical de 30%
	
	//Configurando o USART
	UBRR0H = (unsigned char)(MYUBRR>>8); //Ajuste da taxa de transmissão
	UBRR0L = (unsigned char)MYUBRR;  //Ajuste da taxa de transmissão
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita a transmissão e o recepção
	UCSR0C = (3<<UCSZ00);  //Ajusta o formato do frame: 8 bits de dados e 1 de parada, sem paridade
	

	sei(); //Habilitando as interrupções
	while (1)
	{

	}
}

