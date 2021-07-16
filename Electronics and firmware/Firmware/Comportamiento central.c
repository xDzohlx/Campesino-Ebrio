/*
 * Comportamiento central.c
 *
 * Created: 06/07/2021 19:38:21
 * Author : xDzohlx
 */ 
#define F_CPU 16000000UL //Frecuencia del cpu 16 MHz
#define P 1
#define D 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <stdbool.h>
int cont = 0;//Contador de canal
int contadc = 0;//Contador de sensores
volatile uint16_t Controlador_D = 0x0000;
volatile uint16_t Controlador_P = 0x0000;
volatile uint16_t VALORADC = 0x0000;
volatile uint16_t canal[6];//valor de canal de 4000 hasta 8000
volatile uint16_t sensor[16];
volatile uint16_t motor[1];
bool lectura = false;

void setup(void){
	//Configuracion de CLKPER para fuente de reloj de varios perifericos
	ccp_write_io((void *) & (CLKCTRL.MCLKCTRLB), (CLKCTRL_PDIV_2X_gc|CLKCTRL_PEN_bm));//Maxima frecuencia de lectura de pwm
	
	//configuracion de puertos
	PORTA.DIRSET = PIN0_bm|PIN2_bm|PIN3_bm|PIN4_bm;//|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm;//dirección de entrada en este caso es de salida
	
	//Configuracion de TCA para salida de motores
	TCB0.CTRLA |= TCB_ENABLE_bm|TCB_CLKSEL_CLKDIV2_gc;//ACTIVACION DE EVENTOS RELOJ PROVIENE DE TCA A 1MHZ //clkper 
	TCB0.CTRLB |= TCB_CNTMODE_FRQ_gc;//modo de lectura de frecuencia
	TCB0.EVCTRL |= TCB_CAPTEI_bm|TCB_EDGE_bm;//ACTIVA CAPTURA DE EVENTOS, EDGE es el sentido del pulso en este caso esta inverso
	TCB0.INTCTRL |= TCB_CAPT_bm;
	//CONFIGURACION DE RED DE EVENTOS
	EVSYS.CHANNEL2 |= EVSYS_GENERATOR_PORT0_PIN0_gc;//PORTC3 CONECTADO A CANAL 2 para lectura de ppm
	EVSYS.USERTCB0 |= EVSYS_CHANNEL_CHANNEL2_gc;//usuario TCB0 conectado al canal 2

	//adc
	ADC0.CTRLA |= ADC_RESSEL_bm|ADC_FREERUN_bm;//RESOLUCION Y MODO DE MUESTRAS CONSECUTIVAS
	ADC0.CTRLB |= ADC_SAMPNUM_ACC8_gc;//MUESTRAS
	ADC0.CTRLC |= ADC_REFSEL_VDDREF_gc|ADC_PRESC_DIV2_gc;//VOLTAJE DE REFERENCIA
	ADC0.CTRLD |= ADC_INITDLY_DLY16_gc;//CONFIGURACION DEL RELOJ DEL ADC
	//Canales de 0 al 7 despues a partir del 12 o 0x0C
	ADC0.MUXPOS = 0x00;//SELECCION DE CANAL DE ADC PD1
	ADC0.INTCTRL |= ADC_RESRDY_bm;//Habilitar interrupciones de resultado completo
	//ADC0.INTCTRL |= ADC_WCMP_bm;
	ADC0.CTRLA |= ADC_ENABLE_bm;//ENCENDIDO DE ADC
	ADC0.COMMAND |= ADC_STCONV_bm;//INICIO DE MUESTRAS

	//timer TCA
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1_gc;//fuente de reloj,
	TCA0.SINGLE.PER = 0x07EF;//Selección de resolucion de pwm y periodo total de pwm
	TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2_bm|TCA_SINGLE_CMP0EN_bm|TCA_SINGLE_WGMODE_SINGLESLOPE_gc;//Habilitar comparador y seleccion de modo de de generacion de onda con modo de rampa sensilla
	TCA0.SINGLE.CMP0 = 0x3E0;//registro de 16 bits para comparacion y pediodo de pwm
	TCA0.SINGLE.CMP2 = 0x3E0;
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;//Habilitar pwm

	//configuracion de vector de interrupcion
	CPUINT.LVL0PRI = ADC0_RESRDY_vect_num;
	CPUINT.LVL1VEC = TCB0_INT_vect_num;	
	//Habilitar interrupciones generales
	sei();
}
ISR(TCB0_INT_vect){//Interrupcion de lecutra y decodificacion de ppm
if (cont > 0)// lectura del canal no es necesario para decodificador
	canal[cont -1]=TCB0.CCMP;//lectura del canal no necesario para salida decodificada
	if (cont==2){
		PORTA.OUTSET = PIN4_bm;
	}
	if (cont==3){
		PORTA.OUTCLR = PIN4_bm;
	}
cont++;//siguiente canal
if (canal[cont-2]>16000)
	cont = 0;
}
//Interrupción de lectura de ADC para sensores
ISR(ADC0_RESRDY_vect){
//Canales de 0 al 7 despues a partir del 12 hasta sensor 14
	sensor[contadc-1] = ADC0.RES;
	contadc++;
	if (contadc== 9){
		contadc = 12;
	}
if (contadc>= 16){
	contadc = 0;
	lectura = true;
	}
ADC0.MUXPOS = contadc;
}
int main(void){
	setup();
	while (1){
//valor de canal de 4000 a 8000
//motores aceptan valores de 1000 a 2000 punto medio de 1500
//ajuste de señal de lectura para señal de motores
//salida de motores
if(canal[4]<6000&&canal[0]>7000&&canal[1]>7000&&canal[0]<5000&&canal[1]<5000){//Controlador encendido y seguros de funcionamiento
	motor[0] = (canal[0]>>2);//division entre 4 para ajustar la resolucion
	motor[1] = (canal[1]>>2);
	//validaciones
	//Lado izquierdo
	if ((sensor[1]-sensor[6])>250){
		Controlador_P = sensor[1];
	}
	if ((sensor[2]-sensor[13])>250){
		Controlador_P = sensor[2];
	}
	if ((sensor[0]-sensor[7])>250){
	}
	//Lado derecho
	if ((sensor[12]-sensor[3])>250){
		Controlador_P = sensor[12];
	}
	if ((sensor[5]-sensor[3])>350){
		Controlador_P = sensor[5];
	}
	if ((sensor[4]-sensor[14])>250){
		Controlador_P = sensor[4];
	}
TCA0.SINGLE.CMP0 = motor[0] + Controlador_P*P + Controlador_D*D;
TCA0.SINGLE.CMP2 = motor[1] - Controlador_P*P - Controlador_D*D;
}else{//Controlador apagado
	motor[0] = (canal[0]>>2);
	motor[1] = (canal[1]>>2);
	TCA0.SINGLE.CMP0 = motor[0];
	TCA0.SINGLE.CMP2 = motor[1];
}
}//while(1)
}//main
