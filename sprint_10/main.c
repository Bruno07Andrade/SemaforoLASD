 /*
 * sprint_9.c
 *
 * Created: 07/12/2021 14:56:07
 * Author : Bruno Andrade
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

#include "Nokia5110\nokia5110.h"

typedef enum enum_parametros {Modo,Tempo_verde, Tempo_vermelho, Tempo_amarelo, Size_enum_parametros} enum_parametros;

//Struct para armazenar os tempos dos estados do sem�foro
typedef struct stc_semaforo
{
	uint8_t modo_am;
	uint16_t tempo_verde_ms;
	uint16_t tempo_vermelho_ms;
	uint16_t tempo_amarelo_ms;
	uint16_t carros_min;
	uint16_t LUX;
	uint8_t temp;
} stc_semaforo;

//Vari�veis globais necess�rias
stc_semaforo semaforo = {.modo_am = 0, .tempo_verde_ms = 5000, .tempo_vermelho_ms = 3000, .tempo_amarelo_ms = 1000, .carros_min = 0, .LUX = 0,.temp = 25};
enum_parametros selecao_parametro = Modo;
uint32_t tempo_ms = 0;
uint8_t flag_5000ms = 0, flag_500ms = 0; //Flags para disparar com 5 segundos e 5000ms
uint8_t flag_trem = 0,flag_fecha = 0, flag_abre = 0; //Flags para o trem
uint16_t carros = 0; // Contagem dos carros
//Prot�tipo das fun��es
void anima_semaforo(stc_semaforo Semaforo, uint32_t Tempo_ms, uint8_t *flag_disparo);
void anima_LCD(stc_semaforo Semaforo);
void conta_carro(uint8_t *flag_disparo);
void leitura_ADC(uint8_t *flag_disparo);
void USART_Transmit(unsigned char data);
void passagem_trem(uint8_t *flag_fecha, uint8_t *flag_abre);
//Interrup��o para contabilizar o tempo
ISR(TIMER0_COMPA_vect)
{
	tempo_ms++;
	if((tempo_ms % 5000) == 0)
	    flag_5000ms = 1; //Flag de 5 segundos para a contagem de carros
	if((tempo_ms % 500) == 0)
	   flag_500ms = 1;  //Flag de 500ms para ilumina��o
	if((tempo_ms % 60000) == 0)
	   flag_fecha = 1; //Flag para fechar cancela
	if((tempo_ms % 70000) == 0)
	   flag_abre = 1; //Flag para abrir cancela
	
}
//Contagem de carros que passam o sinal
ISR(INT0_vect)
{
	carros++;
}

//Interrup��o do funcionamento dos LEDs
ISR(PCINT2_vect)
{
	//Interrup��o que altera qual dado est� selecionado
	if((PIND & 0b01000000) == 0)
	{
		if(selecao_parametro < (Size_enum_parametros-1))
		selecao_parametro++;
		else
		selecao_parametro = Modo;
	}
	//Interrup��o que diminui em 1 segundo o sinal ou altera o modo
	else if((PIND & 0b00100000) == 0)
	{
		switch(selecao_parametro)
		{
			case Modo:
			semaforo.modo_am = !semaforo.modo_am;
			break;
			case Tempo_verde:
			if(semaforo.modo_am == 1)
			{
				if(semaforo.tempo_verde_ms >= 2000)
				semaforo.tempo_verde_ms -= 1000;
			}
			break;
			case Tempo_vermelho:
			if(semaforo.modo_am == 1)
			{
				if(semaforo.tempo_vermelho_ms >= 2000)
				semaforo.tempo_vermelho_ms -= 1000;
			}
			break;
			case Tempo_amarelo:
			if(semaforo.modo_am == 1)
			{
				if(semaforo.tempo_amarelo_ms >= 2000)
				semaforo.tempo_amarelo_ms -= 1000;
			}
			break;
		}
		
	}
	//Interrup��o que aumenta em 1 segundo o sinal ou altera o modo
	else if((PIND & 0b00010000) == 0)
	{
		switch(selecao_parametro)
		{
			case Modo:
			semaforo.modo_am = !semaforo.modo_am;
			break;
			case Tempo_verde:
			if(semaforo.modo_am == 1)
			{
				if(semaforo.tempo_verde_ms <= 8000)
				semaforo.tempo_verde_ms += 1000;
			}
			break;
			case Tempo_vermelho:
			if(semaforo.modo_am == 1)
			{
				if(semaforo.tempo_vermelho_ms <= 8000)
				semaforo.tempo_vermelho_ms += 1000;
			}
			break;
			case Tempo_amarelo:
			if(semaforo.modo_am == 1)
			{
				if(semaforo.tempo_amarelo_ms <= 8000)
				semaforo.tempo_amarelo_ms += 1000;
			}
			break;
		}
		
	}

}

//Fun��o que determina o funcionamento do sem�foro
void anima_semaforo(stc_semaforo Semaforo,uint32_t tempo_ms,uint8_t *flag_disparo)
{
	//Vari�veis que auxiliam na opera��o, cada posi��o de "estados"
	//representa quantos leds acessos codificados em apenas 4 bits 
	const uint16_t estados[9] = {0b00000000, 0b00000001, 0b00001000, 0b00001001, 0b00010000, 0b00010001, 0b00011000, 0b00011001, 0b00100000};
	static uint8_t i_M = 0, i_E = 0, i_S = 0;
	static uint32_t Tempo_Ant_M = 0, Tempo_Ant_E = 0; //Vari�vel auxiliar para o timer
	
	if(*flag_disparo)
	{
		PORTB = 0b00010001;
	}
	else if((PIND & 0b10000000)== 0) //Se a chave que representa oscila��o for acionada o sem�foro passa a piscar amarelo
	{
		//A vari�vel i_S representa os 2 estados do sem�foro, apagado ou o led amarelo aceso
		if(i_S <= 0)
		    //Quando i_S � 0, o circuito ir� ficar aceso e a vari�vel ir� passar a ser 1
		    if((tempo_ms - Tempo_Ant_M) >= 500) 
		    {
			    PORTB = 0b00010000;
			    Tempo_Ant_M = tempo_ms;
				i_S++;		
		    }	
		if(i_S == 1)
		{
			//Quando i_S � 1, o sem�foro apaga e fica assim por meio segundo, at� ser aceso de novo
			if((tempo_ms-Tempo_Ant_M) >= 2000)
			{
				PORTB = 0b00111001;
				Tempo_Ant_M = tempo_ms;
				i_S = 0;
			}
		}
	}
	else
	{
			//Opera��o para determinar o tempo de cada led aceso
	        PORTB = estados[i_M] & 0b01111111;
		
		//Teste para determinar o tempo que cada led deve ficar aceso
		if(i_M <=3)
		{
			if((tempo_ms - Tempo_Ant_M) >= (Semaforo.tempo_verde_ms/4))
			{
				i_M++;
				Tempo_Ant_M = tempo_ms;
			}
		}
		//Teste para o led amarelo
		else if(i_M <= 4)
		{
			if((tempo_ms - Tempo_Ant_M) >= Semaforo.tempo_amarelo_ms)
			{
				i_M++;
				Tempo_Ant_M = tempo_ms;
			}
		}
		//Teste para os leds vermelhos
		else if(i_M<=8)
		{
			if((tempo_ms - Tempo_Ant_M) >=(Semaforo.tempo_vermelho_ms/4))
			{
				i_M++;
				Tempo_Ant_M = tempo_ms;
			}
		}
		//Reiniciando a contagem de estados, voltando a ligar o sinal verde
		else
		{
			i_M = 0;
			Tempo_Ant_M = tempo_ms;
			Tempo_Ant_E = tempo_ms;  //Sincroniza os 2 sem�foros
		}
		
		//Avan�o de estados do Escravo
		if(i_E <=3)
		{
			if((tempo_ms - Tempo_Ant_E) >= ((Semaforo.tempo_verde_ms + Semaforo.tempo_amarelo_ms)/4))
			{
				i_E++;
				Tempo_Ant_E += ((Semaforo.tempo_verde_ms + Semaforo.tempo_amarelo_ms)/4);
				USART_Transmit('0'+ i_E);
			}
		}
		//Teste para os leds vermelhos
		else
		{
			if(i_E<=7)
			{
				if((tempo_ms - Tempo_Ant_E) >=(Semaforo.tempo_vermelho_ms - Semaforo.tempo_amarelo_ms)/4)
				{
					i_E++;
					Tempo_Ant_E += (Semaforo.tempo_vermelho_ms - Semaforo.tempo_amarelo_ms)/4;
					USART_Transmit('0' + i_E);
				}
			}
			else
			{
				if(i_E <= 8)
				{
					if((tempo_ms - Tempo_Ant_E) >= (Semaforo.tempo_amarelo_ms))
					{
						i_E++;
						Tempo_Ant_E += Semaforo.tempo_amarelo_ms;
						USART_Transmit('0' + i_E);
					}
				}
				else
				{
					i_E = 0;
					USART_Transmit('0' + i_E);
				}
			}
	    }

     } 
}

//Fun��o que mostra na tela as informa��es de tempo
void anima_LCD(stc_semaforo Semaforo)
{
	
	//Configurando as vari�veis que representerar�o o tempo no display
	unsigned char tempo_verde_s_strig[2];
	unsigned char tempo_vermelho_s_strig[2];
	unsigned char tempo_amarelo_s_strig[2];
	unsigned char modo_s_string [2];
	unsigned char carros_min_string [5];
	unsigned char lux_string [8];
	
	sprintf(tempo_verde_s_strig,"%u", Semaforo.tempo_verde_ms/1000);
	sprintf(tempo_vermelho_s_strig,"%u", Semaforo.tempo_vermelho_ms/1000);
	sprintf(tempo_amarelo_s_strig,"%u", Semaforo.tempo_amarelo_ms/1000);
	sprintf(carros_min_string,"%u",Semaforo.carros_min);
	sprintf(lux_string,"%u",Semaforo.LUX);
	//Opera��es utilizando a biblioteca do display para mostrar na tela
	nokia_lcd_clear();
	nokia_lcd_set_cursor(0,5);
	nokia_lcd_write_string("Modo",1);
	nokia_lcd_set_cursor(35,5);
	if(semaforo.modo_am == 0)
	nokia_lcd_write_string("A",1);
	else
	nokia_lcd_write_string("M",1);
	nokia_lcd_set_cursor(0,15);
	nokia_lcd_write_string("T. Vd",1);
	nokia_lcd_set_cursor(35,15);
	nokia_lcd_write_string(tempo_verde_s_strig,1);
	nokia_lcd_set_cursor(0,25);
	nokia_lcd_write_string("T. Vm",1);
	nokia_lcd_set_cursor(35,25);
	nokia_lcd_write_string(tempo_vermelho_s_strig,1);
	nokia_lcd_set_cursor(0,35);
	nokia_lcd_write_string("T. Am",1);
	nokia_lcd_set_cursor(35,35);
	nokia_lcd_write_string(tempo_amarelo_s_strig,1);
	nokia_lcd_set_cursor(45, 5+selecao_parametro*10);
	nokia_lcd_write_string("<",1);
	nokia_lcd_set_cursor(52,25);
	nokia_lcd_write_string(carros_min_string, 2);
	nokia_lcd_set_cursor(55,40);
	nokia_lcd_write_string("c/min",1);
	nokia_lcd_set_cursor(52,0);
	nokia_lcd_write_string(lux_string, 2);
	nokia_lcd_set_cursor(55,15);
	nokia_lcd_write_string("lux",1);
	nokia_lcd_render();
	
}
//Fun��o que conta o n�mero de carros
void conta_carro(uint8_t *flag_disparo)
{

	if(*flag_disparo) //Teste de 5 segundos
	{
		*flag_disparo = 0;
		semaforo.carros_min = carros * 12; //Contagem de carros por minuto
		carros = 0;
		if(semaforo.modo_am == 0)
		{
			semaforo.tempo_verde_ms = 1000 + ((uint16_t)(semaforo.carros_min*16.7)/1000)*1000; //Equa��o que determina o tempo do led verde aceso no modo autom�tico
			if(semaforo.tempo_verde_ms > 9000) //Limite de tempo para o led verde
			    semaforo.tempo_verde_ms = 9000;
			semaforo.tempo_vermelho_ms = 9000 -	((uint16_t)(semaforo.carros_min*16.7)/1000)*1000;//Equa��o que determina o tempo do led vermelho aceso no modo autom�tico
			if(semaforo.tempo_vermelho_ms > 32000) //Limite de tempo para o led vermelho
			    semaforo.tempo_vermelho_ms = 1000;
			
		}
	}
}
//Fun��o que contabiliza o LUX e determina o acendimento da Lumin�ria
void leitura_ADC (uint8_t *flag_disparo)
{
	
	if(*flag_disparo)  //Teste de 500 milisegundos
	{
		*flag_disparo = 0;
		semaforo.LUX = 1023000/ADC - 1000; //Equa��o que determina o valor de lux em fun��o da tens�o de entrada
		if(semaforo.LUX >= 300)
		    OCR2B = 0;  //Se LUX estiver igual ou acima de 300, a lumin�ria apaga
		else
		{
			if(((PINC & 0b01000000) == 0) || (semaforo.carros_min > 0) )
			    OCR2B = 255; //Se LUX < 300 e tenha pessoas ou carros passando, lumin�ria ligada 100%
			else
			    OCR2B = 76;//Se LUX < 300 sem que tenha pessoas ou carros passando, lumin�ria ligada 30%
		}
	}
	anima_LCD(semaforo);
}
//Fun��o de transmiss�o para sem�foro escravo
//Fun��o para envio de um frame de 5 a 8 bits
void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0))); //Espera limpeza do registrador de transmiss�o
	UDR0 = data;
}
//Fun��o que controla cancela do trem
void passagem_trem(uint8_t *flag_fecha,uint8_t *flag_abre)
{
	//Quando flag para fechar a cancela � 1, a cancela fecha e para o sem�foro
	if(*flag_fecha)
	{
		flag_trem = 1;
		OCR1A = 3000; //Duty Cycle de 10% para o servo estar em 90�
		*flag_fecha = 0;
	}
	//Quando a flag para abrir � 1, a cancela abre e o sem�foro come�a do vermelho 
	if(*flag_abre)
	{
		flag_trem = 0;
		OCR1A = 2000; //Duty Cycle de 5% para o servo estar em 0�
		*flag_abre = 0;
	}
}

int main(void){
	//Configurando entradas e sa�das
	DDRB = 0b00111111;  //Pinos B0-B5 como sa�da
	DDRD |= 0b00001000; //Pino D3 como entrada
	DDRC |= 0b00000010; //Pino C1 como entrada
	PORTC &= 0b10111110; //Desabilitando o pull-up de C0
	PORTC |= 0b01000000; //Habilitando o pull-up de C6
	PORTD |= 0b11110000; //Definindo os pinos de pull-up D4-D7
	
	
	//Cofigurando as interrup��es
	EICRA  |= 0b00001010;//Interrup��o por borda de subida de INT0
	EIMSK  |= 0b00000001;//Habilita interrup��o do INT0
	PCICR  = 0b00000100;//Habilita interrup��o pelos pinos D
	PCMSK2 = 0b01110000;//Habilita as interrup��es dos pinos PD4, PD5 e PD6
	
	//Configurando timer 1ms
	TCCR0A = 0b00000010; //habilitando o modo CTC do TC0
	TCCR0B = 0b00000011; //ligando com prescaler = 64
	OCR0A = 249; //Ajustando a contagem at� 250
	TIMSK0 = 0b00000010; //Habilitando a interrup��o na igualdade de compara��o com OCR0A
	
	//Configurando o ADC
	ADMUX = 0b01000000; //Vcc como refer�ncia, canal 0
	ADCSRA = 0b11100111; //Habilita o AD, modo convers�o cont�nua, prescaler = 128
	ADCSRB = 0b00000000; //Modo de convers�o cont�nua
	DIDR0 = 0b00111000; //Habilita o PC0 como entrada do ADC
	
	//Configurando o PWM da lumin�ria
	TCCR2A = 0b00100011; //PWM r�pido, ativa PWM no OC2B n�o invertido
	TCCR2B = 0b00000110; //Prescale 256, fpm = 16MHZ/(256*256)= 244Hz
	OCR2B = 77; //Duty inical de 30%, 256 * 30% = 76.8
	
	//Configurando PWM do trem
    ICR1 = 39999; // Configura per�odo do PWM
	TCCR1A = 0b10100010; //PWM r�pido, ativa PWM no OC1A n�o invertido
	TCCR1B = 0b00011010; // Prescale 8, fpm = 50 hz => TOP = 39999
	OCR1A = 2000; //Duty Cycle de 5%

	//Configurando o USART
	UBRR0H = (unsigned char)(MYUBRR>>8); //Ajuste da taxa de transmiss�o
	UBRR0L = (unsigned char)MYUBRR;  //Ajuste da taxa de transmiss�o
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita interrup��o do receptor, o transmissor e o receptor
	UCSR0C = (3<<UCSZ00);  //Ajusta o formato do frame: 8 bits de dados e 1 de parada, sem paridade
	
	//Iniciando o LCD
	nokia_lcd_init();
	if(semaforo.modo_am == 0) //Valor inicial de sem�foro autom�tico sem carros passando
	{
		semaforo.tempo_verde_ms =   1000;
		semaforo.tempo_vermelho_ms = 9000;
		semaforo.tempo_amarelo_ms = 1000;
	}
	
	anima_LCD(semaforo);

	sei(); //Habilitando as interrup��es
	while (1)
	{

		leitura_ADC(&flag_500ms);
		conta_carro(&flag_5000ms);
		//Chamando a fun��o que realiza a opera��o do sem�foro
		anima_semaforo(semaforo,tempo_ms,&flag_trem);
		passagem_trem(&flag_fecha, &flag_abre);
	}
}

