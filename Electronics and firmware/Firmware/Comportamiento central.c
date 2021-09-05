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
	#define sensor_reversa 4

	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <avr/cpufunc.h>
	#include <stdbool.h>
	#include <util/delay.h>

	int cont = 0;//Contador de canal
	int contadc = 0;//Contador de sensores
	int contcal = 0;//contador de calibracion

	uint16_t Controlador_D[2] ;
	uint16_t Controlador_P[2] ;
	volatile uint16_t VALORADC = 0x0000;
	volatile uint16_t canal[7];//valor de canal de 4000 hasta 8000
	//uint16_t canalcal[3][7];//vectores de calibracion
	//uint16_t sensorcal[3][7];
	volatile uint16_t canalcontrol[2];
	volatile uint16_t sensor[16];
	volatile uint16_t motor[2];//motor 0 izquid, motor 1 derecho
	volatile uint16_t automatico[2];
	
	uint16_t controlautomatico[2];
	uint16_t controlautomaticoprevio = 0x00;
	uint16_t canaloffset[2];
	uint16_t sensoroffset[8];
	uint16_t millis = 0;
	uint16_t previousmillis = 0;
	uint16_t currentmillis = 0;
	uint16_t nuevomillis = 0;
	uint8_t contfiltro = 0x00;
	uint8_t numsensor = 0;
	volatile bool manual = true;//0 sin asistencia, 1 asistencia, 2 autonomo
	volatile bool asistido = false;
	volatile bool autonomo = false;
	//0 manual
	//1 asistido,semiautonomo
	//2 autonomo
	bool lectura = false;
	bool lectura_canal = false;
	bool calibracion = false;
	bool primero = true;
	bool segundo = false;
	
	volatile bool reversa = false;

	void offsetsignals(){//etapa de autocalibracion por promedio, pequeño filtro digital

		while (!lectura_canal&&canal[acelerador]<6500&&canal[acelerador]>5500&&canal[volante]<6500&&canal[volante]>5500){//Seguridad
		}
		_delay_ms(1000);
		for (int i = 0;i<8;i++){//adquisición de señales, 8 valores en total
			_delay_ms(20);
			canaloffset[volante] += canal[volante];
			canaloffset[acelerador] += canal[acelerador];
		}
		canaloffset[volante] = (canaloffset[volante]>>3);//division del promedio
		canaloffset[acelerador] = (canaloffset[acelerador]>>3);
	}
	//generador de trayectorias, falta el parametro de velocidad mientras tanto sera fija
	//valores de 4000 a 2000
	uint16_t adelante(uint16_t distancia){
		if (primero){
		millis = 0;// reiniciar temporizador
		primero = false;
		}
		if (millis>=(distancia)){
			if (controlautomatico[acelerador]<= canaloffset[acelerador]){//saturacion
				controlautomatico[acelerador] = canaloffset[acelerador];
			}else{
				
				controlautomatico[acelerador] = (canaloffset[acelerador]<<1)-millis;//rampa inversa
			}
		}else{
			if (millis<=2000){//rampa positiva
				controlautomatico[acelerador] = canaloffset[acelerador]+millis;	
			}
		}
		return  controlautomatico[acelerador];
	}
	void atras(uint8_t distancia){
		
	}
	void giro(uint8_t angulo){
		
	}
	void arco(uint8_t distancia){
			
	}
	
	void setup(void){
	
		//Configuracion de CLKPER para fuente de reloj de varios perifericos
		ccp_write_io((void *) & (CLKCTRL.MCLKCTRLB), (CLKCTRL_PDIV_2X_gc|CLKCTRL_PEN_bm));//Maxima frecuencia de lectura de pwm en señal de reloj per
			//RTC Reloj de tiempo real
		while((RTC.STATUS > 0x00 )){}//se checa que no se este uitilizando	el RTC
		RTC.PER = 0x01A4;//Timer para apagar sistemas autonomos en 7 minutos
		RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
		RTC.DBGCTRL |= RTC_DBGRUN_bm;
		RTC.CTRLA = RTC_PRESCALER_DIV32768_gc| RTC_RTCEN_bm| RTC_RUNSTDBY_bm;
		RTC.INTCTRL |= RTC_OVF_bm;//RTC_CMP_bm;
		//configuracion de puertos
		PORTA.DIRSET = PIN0_bm|PIN1_bm|PIN2_bm|PIN3_bm;//|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm;//dirección de entrada en este caso es de salida
		//Configuracion de TCA para salida de motores
		TCB0.CTRLA |= TCB_ENABLE_bm|TCB_CLKSEL_CLKDIV2_gc;//ACTIVACION DE EVENTOS RELOJ PROVIENE DE TCA A 1MHZ //clkper 
		TCB0.CTRLB |= TCB_CNTMODE_FRQ_gc;//modo de lectura de frecuencia
		TCB0.EVCTRL |= TCB_CAPTEI_bm|TCB_EDGE_bm;//ACTIVA CAPTURA DE EVENTOS, EDGE es el sentido del pulso en este caso esta inverso
		TCB0.INTCTRL |= TCB_CAPT_bm;
		//TCB1.CTRLB |= 0;//modo de tiempo fuera
		TCB1.CTRLA |= TCB_ENABLE_bm;
		TCB1.CCMP = 0xFA0;//Valor top 4000 interrupcion cada milisegundo
		TCB1.INTCTRL |= TCB_CAPT_bm;//habilita la interrupcion	
		//Interrupcion para perifericos de baja velocidad
		TCB2.CTRLA |= TCB_ENABLE_bm|TCB_CLKSEL_CLKDIV2_gc;
		TCB2.CCMP = 0xFFFE;//Valor top 4000 interrupcion cada 10 milisegundos
		TCB2.INTCTRL |= TCB_CAPT_bm;//habilita la interrupcion
		//CONFIGURACION DE RED DE EVENTOS
		//Configuracion de sistema de eventos para el radio
		EVSYS.CHANNEL2 |= EVSYS_GENERATOR_PORT0_PIN0_gc;//PORTC3 CONECTADO A CANAL 2 para lectura de ppm
		EVSYS.USERTCB0 |= EVSYS_CHANNEL_CHANNEL2_gc;//usuario TCB0 conectado al canal 2
		//Configuracion de sistema de eventos para sensor de reversa con filtro
		EVSYS.CHANNEL0 |= EVSYS_GENERATOR_PORT0_PIN3_gc;//PORTA3 conectado a canal 0 para sensor de reversa
		EVSYS.USERTCB1 |= EVSYS_CHANNEL_CHANNEL0_gc;//TCB1 coneectado al canal 0
		//ADC
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
		TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2_bm|TCA_SINGLE_CMP1EN_bm|TCA_SINGLE_WGMODE_SINGLESLOPE_gc;//Habilitar comparador y seleccion de modo de de generacion de onda con modo de rampa sensilla
		TCA0.SINGLE.CMP1 = 0x3E0;//registro de 16 bits para comparacion y pediodo de pwm
		TCA0.SINGLE.CMP2 = 0x3E0;
		TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;//Habilitar pwm

		//configuracion de vector de interrupcion
		CPUINT.LVL0PRI = ADC0_RESRDY_vect_num;
		CPUINT.LVL1VEC = TCB0_INT_vect_num;	
		//Habilitar interrupciones generales
		sei();
		offsetsignals();// se obtiene el offset por promedio, de 8 valores por comodidad
		controlautomatico[volante] = canaloffset[volante];
		controlautomatico[acelerador] = canaloffset[acelerador];
		PORTA.OUTSET = PIN0_bm;//Termino la configuracion
		_delay_ms(500);//parpadeo para ver el encendido
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
	ISR(TCB1_INT_vect){//contador de milisegundos, para generador de trayectorias
		millis++;
		TCB1.INTFLAGS &= ~TCB_CAPT_bp;
	}
	ISR(TCB2_INT_vect){//Interrupcion para checar canales cada 10 ms
		//currentMillis++;
			if (sensor[sensor_reversa]>900){
			reversa = true;
			//PORTA.OUTSET = PIN3_bm;// led apagado
			}else{
			//PORTA.OUTCLR = PIN3_bm;// led prendido
			reversa = false;
			}
		if (canal[4]<5000){
		manual = true;
		asistido = false;
		autonomo = false;
		PORTA.OUTCLR= PIN0_bm;//led de aviso
		}
		
		if (canal[4]<6500&&canal[4]>5500){
			manual = false;
			asistido = true;
			autonomo = false;
			currentmillis = millis;
			if (currentmillis - previousmillis >= 250) {
				previousmillis = currentmillis;
				PORTA.OUTTGL = PIN0_bm;
			}
		}
		
		if (canal[4]>7000){
		manual = false;
		asistido = false;
		autonomo = true;
			PORTA.OUTSET = PIN0_bm;
		}
		
		
		TCB2.INTFLAGS &= ~TCB_CAPT_bp;
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
	ISR(RTC_CNT_vect){//Interrupcion por tiempo seguridad
		//Apagar motores
		TCA0.SINGLE.CMP1 = canaloffset[acelerador];
		TCA0.SINGLE.CMP2 = canaloffset[acelerador];
		while(1){
		PORTA.OUTTGL = PIN0_bm;//led de aviso
		_delay_ms(1000);
		}//se detiene el programa
	}
	int main(void){
		setup();
		while (1){
	//valor de canal de 4000 a 8000 con 6000 de punto medio
	//motores aceptan valores de 1000 a 2000 punto medio de 1500
	//ajuste de señal de lectura para señal de motores
	//salida de motores
	
		//PORTA.OUTCLR = PIN3_bm;// led prendido
		//Controlador_P[0] = (sensor[1]-sensoroffset[1])>>2;//derecho
		//Controlador_P[1] = (sensor[0]-sensoroffset[0])>>2;//izquierda
		//*((uint8_t*)&(value)+1);
		
		Controlador_P[0] = 0;//derecho
		Controlador_P[1] = 0;//izquierda

		if (manual){
			canalcontrol[volante] = canal[volante];// + Controlador_P[0] - Controlador_P [1];
			canalcontrol[acelerador] = canal[acelerador];
		}
		if (asistido){
			canalcontrol[volante] = canal[volante] + Controlador_P[0] - Controlador_P [1];
			canalcontrol[acelerador] = canal[acelerador];
		}
		if (autonomo){
			canalcontrol[volante] = canaloffset[volante];
			canalcontrol[acelerador] = adelante(4000);
		}
		
	if(!reversa){//reversa con sensor
		if (canalcontrol[volante]>=canaloffset[volante]){
			canalcontrol[volante] = ((canalcontrol[volante]-canaloffset[volante])>>1);//mezcladora
			motor[izquierdo] = (canalcontrol[acelerador]-canalcontrol[volante] )>>2;
			motor[derecho] = (canalcontrol[acelerador]+canalcontrol[volante] )>>2;
			}else{
			canalcontrol[volante] = (canaloffset[volante]-canal[volante])>>1;//mezcladora
			motor[izquierdo] = (canalcontrol[acelerador]+canalcontrol[volante])>>2;
			motor[derecho] = (canalcontrol[acelerador]-canalcontrol[volante])>>2;
			}}else{//Normal
			if (canalcontrol[volante]>=canaloffset[volante]){
				canalcontrol[volante] = ((canalcontrol[volante]-canaloffset[volante])>>1);//mezcladora
				motor[izquierdo] = (((canaloffset[acelerador]<<1)-canalcontrol[acelerador])-canalcontrol[volante])>>2;//reversa, solo al acelerador
				motor[derecho] = (((canaloffset[acelerador]<<1)-canalcontrol[acelerador])+canalcontrol[volante])>>2;
				}else{
				canalcontrol[volante] = (canaloffset[volante]-canalcontrol[volante])>>1;//mezcladora
				motor[izquierdo] = (((canaloffset[acelerador]<<1)-canalcontrol[acelerador])+canalcontrol[volante])>>2;//reversa, solo al acelerador
				motor[derecho] = (((canaloffset[acelerador]<<1)-canalcontrol[acelerador])-canalcontrol[volante])>>2;
			}}
			if (motor[izquierdo]>0x07E0){//SATURACION DE MOTORES
				motor[izquierdo] = 0x07E0;
			}
			if (motor[derecho]>0x07E0){
				motor[derecho] = 0x07E0;
			}
			TCA0.SINGLE.CMP1 = motor[izquierdo];
			TCA0.SINGLE.CMP2 = motor[derecho];
	}//while(1)
	}//main
