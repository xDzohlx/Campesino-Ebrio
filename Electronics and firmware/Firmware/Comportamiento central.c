/*
 * Comportamiento central.c
 *
 * Created: 06/07/2021 19:38:21
 * Author : xDzohlx
 */ 

#define F_CPU 16000000UL //Frecuencia del cpu 16 MHz
#define P 0//Controladores apagados
#define D 0
#define acelerador 1
#define volante 0
#define arma 3
#define izquierdo 0
#define derecho 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <stdbool.h>
#include <util/delay.h>
int cont = 0;//Contador de canal
int contadc = 0;//Contador de sensores
int contcal = 0;//contador de calibracion

volatile uint16_t Controlador_D = 0x0000;
volatile uint16_t Controlador_P = 0x0000;
volatile uint16_t VALORADC = 0x0000;
volatile uint16_t canal[7];//valor de canal de 4000 hasta 8000
uint16_t canalcal[3][7];//vectores de calibracion
volatile uint16_t canalcontrol[2];
volatile uint16_t sensor[16];
volatile uint16_t motor[2];//motor 0 izquid, motor 1 derecho
volatile uint16_t canaloffset[2];
bool lectura = false;
bool lectura_canal = false;
bool calibracion = false;
bool frente = true;

void offsetsignals(){//etapa de autocalibracion por promedio, pequeño filtro digital

while (!lectura_canal&&canal[acelerador]<6500&&canal[acelerador]>5500&&canal[volante]<6500&&canal[volante]>5500){//Seguridad
}
_delay_ms(1000);
	for (int i = 0;i<8;i++){
		_delay_ms(20);
		canalcal[volante][i] = canal[volante];
		canalcal[acelerador][i] = canal[acelerador];
	}
	for(int i = 0;i<8;i++){
		canaloffset[volante] += canalcal[volante][i];
		canaloffset[acelerador] += canalcal[acelerador][i];	
	}
	canaloffset[volante] = (canaloffset[volante]>>3);//division entre 8
	canaloffset[acelerador] = (canaloffset[acelerador]>>3);

}
void setup(void){
	
	//Configuracion de CLKPER para fuente de reloj de varios perifericos
	ccp_write_io((void *) & (CLKCTRL.MCLKCTRLB), (CLKCTRL_PDIV_2X_gc|CLKCTRL_PEN_bm));//Maxima frecuencia de lectura de pwm en señal de reloj per
	
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
	//_delay_ms(2000);
//	_delay_ms(2000);
	offsetsignals();// se obtiene el offset por promedio, de 8 valores por comodidad
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
	lectura_canal = true;
}
//Interrupción de lectura de ADC para sensores
ISR(ADC0_RESRDY_vect){//solo 4 sensores para empezar
//Canales de 0 al 7 despues a partir del 12 hasta sensor 14
	sensor[contadc-1] = ADC0.RES;
	contadc++;
if (contadc>= 16){
	contadc = 0;
	lectura = true;
	}
ADC0.MUXPOS = contadc;
}
int main(void){
	setup();
	while (1){
//valor de canal de 4000 a 8000 con 6000 de punto medio
//motores aceptan valores de 1000 a 2000 punto medio de 1500
//ajuste de señal de lectura para señal de motores
//salida de motores

//if(canal[4]>1500){//Encendido del sistema autonomo y asistido

//if (canal[volante]>=canaloffset[volante]){
//canalcontrol[volante] = ((canal[volante]-canaloffset[volante])>>1);
//}else{
//canalcontrol[volante] = (canaloffset[volante]-canal[volante])>>1;
//}//((canal[volante]-canaloffset[volante])>>1)
//
//

if(canal[4]>6000){//mezcladora con el sentido del robot
	PORTA.OUTCLR = PIN3_bm;// led prendido
if (canal[volante]>=canaloffset[volante]){
canalcontrol[volante] = ((canal[volante]-canaloffset[volante])>>1);
	motor[0] = (canal[acelerador]-canalcontrol[volante] )>>2;//reversa, solo al acelerador
	motor[1] =  (canal[acelerador]+canalcontrol[volante] )>>2;
}else{
canalcontrol[volante] = (canaloffset[volante]-canal[volante])>>1;
		motor[0] = (canal[acelerador]+canalcontrol[volante] )>>2;//reversa, solo al acelerador
	motor[1] =  (canal[acelerador]-canalcontrol[volante] )>>2;
}//((canal[volante]-canaloffset[volante])>>1)
	}else{
	PORTA.OUTSET = PIN3_bm;//led apagado
	if (canal[volante]>=canaloffset[volante]){
	canalcontrol[volante] = ((canal[volante]-canaloffset[volante])>>1);
	motor[0] = (((canaloffset[acelerador]<<1)-canal[acelerador])-canalcontrol[volante] )>>2;//reversa, solo al acelerador
	motor[1] =  (((canaloffset[acelerador]<<1)-canal[acelerador])+canalcontrol[volante] )>>2;
	}else{
	canalcontrol[volante] = (canaloffset[volante]-canal[volante])>>1;
		motor[0] = (((canaloffset[acelerador]<<1)-canal[acelerador])+canalcontrol[volante] )>>2;//reversa, solo al acelerador
	motor[1] =  (((canaloffset[acelerador]<<1)-canal[acelerador])-canalcontrol[volante] )>>2;
	}//((canal[volante]-canaloffset[volante])>>1)

}
//motor[izquierdo] = (motor[izquierdo]>>2);// resolucion de 10 bits
//motor[derecho] = (motor[derecho]>>2);


	TCA0.SINGLE.CMP0 = motor[0];
	TCA0.SINGLE.CMP2 = motor[derecho];
	
	
//TCA0.SINGLE.CMP0 = motor[izquierdo] + Controlador_P*P + Controlador_D*D;//Controlador
//TCA0.SINGLE.CMP2 = motor[derecho] - Controlador_P*P - Controlador_D*D;
//}else{//Controlador apagado
	//motor[izquierdo] = (canal[0]>>2);// resolucion de 10 bits
	//motor[derecho] = (canal[1]>>2);
	//TCA0.SINGLE.CMP0 = motor[izquierdo];
	//TCA0.SINGLE.CMP2 = motor[derecho];
//}
}//while(1)
}//main
