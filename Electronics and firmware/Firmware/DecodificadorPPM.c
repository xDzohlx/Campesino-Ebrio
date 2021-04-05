/*
 * Decodificador ppm.c
 *
 * Created: 02/12/2020 21:44:54
 * Author : Usuario
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

volatile static uint16_t canal[6];//valor de 11 bits del canal
int cont = 0;//contador de canal incrementa con cada pulso

void setup(void){
	DDRA |= (1<<PORTA1)|(1<<PORTA2);//pines de salida del decodificador
	PCICR |= (1<<PCIE0);//Habilita las interrupciones por cambio de estado en este caso la 0
	PCMSK0 |= (1<<PCINT0);//Habilita el pin 0 de la interrupcion que es el PORTA0
	//Configuracion de timer de 16 bits para lectura de ppm con microsegundos
	TCCR1B = (1<<WGM12)|(1<<CS11);//Seleccion de reloj y forma de donda en este caso sirve para captura
	
	OCR1A = 0xFA0;//valor de sincronizacion
	//Interrupciones
	TIMSK1 = (1<<OCIE1A);//Habilita interrupciones del timer
	
	TCNT1 = 0x0000;//Reinicia el contador
	sei();//Activa interrupciones globales
}
//Interrupcion por cambio de estado para lectura
ISR(PCINT0_vect){
	if (!(PINA & 0x01)){//checa el cambio en el pin si es bajo
		if (cont == 0){//flanco de subida canal 1
			PORTA |= (1<<PORTA1);
		}
		if (cont > 0){// lectura del canal no es necesario para decodificador
			canal[cont -1]=TCNT1;//lectura del canal no necesario para salida decodificada
			PORTA = (PORTA<<1);
		}
		TCNT1 = 0x00;//reinicio del timer del canal
		cont++;//siguiente canal
	}
}
ISR(TIMER1_COMPA_vect){//Segundo vector de interrupcion, sincronizacion
	cont = 0;//reinicia la posicion de los canales
}
int main(void){
	setup();//inizializacion
	while(1){
	}
}
