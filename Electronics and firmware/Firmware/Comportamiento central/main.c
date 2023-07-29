/*
	 * Comportamiento central.c
	 *
	 * Created: 06/07/2021 19:38:21
	 * Author : xDzohlx
	 */ 

	#define F_CPU 16000000UL //Frecuencia del cpu 16 MHz
	#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 32 / (16 * (float)BAUD_RATE)) + 0.5)
	#define Kp 32
	#define D 10
	#define acelerador 1
	#define volante 0
	#define arma 3
	#define seleccion 4
	#define izquierdo 0
	#define derecho 1
	#define sensor_reversa 4
	#define sensor_acelerometro 5
	#define controlador 3

	#define CPPM false
	#define calibracion true
	
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <avr/cpufunc.h>
	#include <stdbool.h>
	#include <util/delay.h>
	
	void USART0_init(void);
	char USART0_readChar(void);
	//uint8_t softwareid  __attribute__ ((.noinit)) = 0x01;


	uint8_t cont = 0;//Contador de canal
	uint8_t contadc = 0;//Contador de sensores
	uint8_t contador = 0;
	uint8_t programa = 0;
	//uint8_t contfiltro=0;
	
	uint16_t Controlador_D[2] ;
	uint16_t Controlador_P[2] ;
	uint16_t error_anterior = 0;
	volatile uint16_t VALORADC = 0x0000;
	volatile uint16_t canal[7];//valor de canal de 4000 hasta 8000
	//uint16_t canalcal[3][7];//vectores de calibracion
	//uint16_t sensorcal[3][7];
	volatile static uint16_t canalcontrol[2];
	volatile uint16_t sensor[16];
	volatile uint16_t motor[2];//motor 0 izquid, motor 1 derecho
	volatile uint16_t automatico[2];
	
	//RC canales SBUS
	
	volatile static uint8_t sbuschannel[25];
	volatile static uint8_t sbuschannelindex = 0;
	volatile static uint8_t buffer_sbus = 0x00;
	volatile static uint8_t sbus_anterior = 0x00;
	volatile static uint16_t nuevo_canal[25];
	volatile static bool fin_recepcion = false;
	
	
	uint16_t controlautomatico[2];
	uint16_t controlautomaticoprevio[2];
	uint16_t canaloffset[2];
	uint16_t sensoroffset[8];
	uint16_t millis[3];
	uint16_t previousmillis = 0;
	uint16_t currentmillis = 0;
	uint16_t nuevomillis = 0;
	uint8_t contfiltro = 0x00;
	uint8_t numsensor = 0;
	volatile uint8_t sensorcontrol[2];
	volatile bool manual = true;//0 sin asistencia, 1 asistencia, 2 autonomo
	volatile bool asistido = false;
	volatile bool autonomo = false;
	bool lectura = false;
	bool lectura_canal = false;
	bool sentido = false;
	bool finadelante = false;
	bool fingiro = false;
	volatile bool primero = true;
	volatile bool segundo = true;
	volatile bool tercero = true;
	bool cuarto = false;
	volatile bool reversa = false;
	volatile bool control_reversa = false;

	void offsetsignals(){//etapa de autocalibracion por promedio, peque�o filtro digital
	PORTA.DIRCLR = PIN0_bm;
	PORTF.OUTSET = PIN0_bm|PIN1_bm|PIN2_bm|PIN3_bm;
		while (!lectura_canal&&canal[acelerador]<6500&&canal[acelerador]>5500&&canal[volante]<6500&&canal[volante]>5500){//Seguridad
		}
		_delay_ms(1500);
		for (int i = 0;i<8;i++){//adquisici�n de se�ales, 8 valores en total
			_delay_ms(20);
			canaloffset[volante] += canal[volante];
			canaloffset[acelerador] += canal[acelerador];
			sensoroffset[sensor_acelerometro] += sensor[sensor_acelerometro];
		}
		sensoroffset[sensor_acelerometro] = (sensoroffset[sensor_acelerometro]>>3);
		canaloffset[volante] = (canaloffset[volante]>>3);//division del promedio
		canaloffset[acelerador] = (canaloffset[acelerador]>>3);
	PORTF.OUTCLR = PIN0_bm|PIN1_bm|PIN2_bm|PIN3_bm;
	PORTA.DIRSET = PIN0_bm;
	}
	//generador de trayectorias, falta el parametro de velocidad mientras tanto sera fija
	//valores de 4000 a 2000
	uint16_t adelante(uint16_t distancia, uint16_t velocidad){//control de aceleracion de 2000, 2000 max
		if (segundo){
			millis[acelerador] = 0;// reiniciar temporizador
			segundo = false;
		}
			if (millis[acelerador]<=velocidad){//rampa positiva
				controlautomatico[acelerador] = canaloffset[acelerador]+millis[acelerador];
				//controlautomaticoprevio[acelerador] = controlautomatico[acelerador];
			}
		if (millis[acelerador]>=(distancia<<1)){
				contador++;
				segundo = true;
		}
		return  controlautomatico[acelerador];
	}
	uint16_t giro(uint16_t distancia,uint16_t velocidad,bool sentido){//true derecha, false izquierda,alrededor de 900 son 90 grados con 550 de velcodiad tangencial y 500 de giro
				if (primero){
					millis[volante] = 0;// reiniciar temporizador
					primero = false;
				}
				if (millis[volante]>=(distancia)){
					if (controlautomatico[volante]<= canaloffset[volante]){//saturacion
						controlautomatico[volante] = canaloffset[volante];
						}else{
							if(sentido){
						controlautomatico[volante] = (controlautomaticoprevio[volante]<<1)-canaloffset[volante]-millis[volante];//rampa inversa
							}else{
						controlautomatico[volante] = (controlautomaticoprevio[volante]<<1)-canaloffset[volante]+millis[volante];//rampa inversa		
							}
					}
					}else{
					if (millis[volante]<=velocidad){//rampa positiva
						if (sentido){
						controlautomatico[volante] = canaloffset[volante]+millis[volante];
						} 
						else{
						controlautomatico[volante] = canaloffset[volante]-millis[volante];
						}
						controlautomaticoprevio[volante] = controlautomatico[volante];
					}
				}
				if (millis[volante]>=(distancia<<1)){
				controlautomatico[volante] = canaloffset[volante];
				contador++;
				primero = true;
				}
				return  controlautomatico[volante];
	}	
	void ocho(void){
		if (contador==0){
			canalcontrol[volante] = canaloffset[volante];
			canalcontrol[acelerador] = adelante(2000,500);
		}
		if (contador==1){
			canalcontrol[volante] = giro(910,500,false);
		}
		//if (contador==2){
			//canalcontrol[volante] = canaloffset[volante];
			//canalcontrol[acelerador] = adelante(450,450);
		//}
		//if (contador==3){
			//canalcontrol[volante] = giro(2000,500,false);
		//}
		if (contador>=2){
			contador = 0;
		}
	}
	void busqueda(void){
		if (contador==0){
			canalcontrol[volante] = canaloffset[volante];
			canalcontrol[acelerador] = adelante(1000,700);
		}
		if (contador==1){
			canalcontrol[volante] = giro(1200,100,true);
		}
		if (contador>=2){
			contador = 0;
		}
	}
	
	char USART0_readChar(void)
	{
		while (!(USART0.STATUS & USART_RXCIF_bm))
		{
			;
		}
		return USART0.RXDATAL;
	}
	
	void USART0_init(void)
	{
		//PORTA.DIR &= ~PIN1_bm;
		//PORTA.DIR |= PIN0_bm;
		
		USART1.BAUD = (uint16_t)USART0_BAUD_RATE(100000);

		USART1.CTRLA |= USART_RXCIE_bm;

		USART1.CTRLB |= USART_RXEN_bm;
		
		USART1.CTRLC |= USART_PMODE_EVEN_gc|USART_SBMODE_2BIT_gc;
	}
	
	void setup(void){
	
		//Configuracion de CLKPER para fuente de reloj de varios perifericos
		ccp_write_io((void *) & (CLKCTRL.MCLKCTRLB), (CLKCTRL_PDIV_2X_gc|CLKCTRL_PEN_bm));//Maxima frecuencia de lectura de pwm en se�al de reloj per
			//RTC Reloj de tiempo real
		//while((RTC.STATUS > 0x00 )){}//se checa que no se este uitilizando	el RTC
		//RTC.PER = 0x01A4;//Timer para apagar sistemas autonomos en 7 minutos
		//RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
		//RTC.DBGCTRL |= RTC_DBGRUN_bm;
		//RTC.CTRLA = RTC_PRESCALER_DIV32768_gc| RTC_RTCEN_bm| RTC_RUNSTDBY_bm;
		//RTC.INTCTRL |= RTC_OVF_bm;//RTC_CMP_bm;
		//configuracion de puertos
		
		PORTA.DIRSET = PIN1_bm|PIN0_bm|PIN2_bm|PIN3_bm;//PIN1_bm|PIN0_bm|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm;//direcci�n de entrada en este caso es de salida
		//PORTA.DIRCLR = PIN1_bm;
		PORTC.DIRCLR = PIN1_bm;
		PORTF.DIRSET = PIN0_bm|PIN1_bm|PIN2_bm|PIN3_bm;
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
		//USART SBUS PARA RC
		USART0_init();
		//CONFIGURACION DE RED DE EVENTOS
		//Configuracion de sistema de eventos para el radio
		EVSYS.CHANNEL2 |= EVSYS_GENERATOR_PORT0_PIN0_gc;//PORTC0 CONECTADO A CANAL 2 para lectura de ppm
		
		EVSYS.USERTCB0 |= EVSYS_CHANNEL_CHANNEL2_gc;//usuario TCB0 conectado al canal 2
		//Configuracion de sistema de eventos para sensor de reversa con filtro
		EVSYS.CHANNEL0 |= EVSYS_GENERATOR_PORT0_PIN3_gc;//PORTA3 conectado a canal 0 para sensor de reversa
		EVSYS.USERTCB1 |= EVSYS_CHANNEL_CHANNEL0_gc;//TCB1 coneectado al canal 0
		//ADC y Vref
		VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;// Se selecciona la referencia a 2.5 volts par mejorar la resolucion de los sensores
		//ADC con referencia interna
		ADC0.CTRLA |= ADC_RESSEL_bm|ADC_FREERUN_bm;//RESOLUCION Y MODO DE MUESTRAS CONSECUTIVAS
		ADC0.CTRLB |= ADC_SAMPNUM_ACC32_gc;//MUESTRAS
		ADC0.CTRLC |= ADC_REFSEL_INTREF_gc|ADC_PRESC_DIV8_gc;//VOLTAJE DE REFERENCIA
		ADC0.CTRLD |= ADC_INITDLY_DLY16_gc;//CONFIGURACION DEL RELOJ DEL ADC
		//Canales de 0 al 7 despues a partir del 12 o 0x0C
		ADC0.MUXPOS = 0x00;//SELECCION DE CANAL DE ADC PD1
		ADC0.INTCTRL |= ADC_RESRDY_bm;//Habilitar interrupciones de resultado completo
		//ADC0.INTCTRL |= ADC_WCMP_bm;
		ADC0.CTRLA |= ADC_ENABLE_bm;//ENCENDIDO DE ADC
		ADC0.COMMAND |= ADC_STCONV_bm;//INICIO DE MUESTRAS

		//timer TCA
		TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV8_gc;//fuente de reloj,
		TCA0.SINGLE.PER = 0x4E20;//Selecci�n de resolucion de pwm y periodo total de pwm
		TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP1_bm|TCA_SINGLE_CMP2_bm|TCA_SINGLE_WGMODE_SINGLESLOPE_gc;//Habilitar comparador y seleccion de modo de de generacion de onda con modo de rampa sensilla|TCA_SINGLE_CMP1EN_bm
		//TCA0.SINGLE.CMP1 = 0x3E0;//registro de 16 bits para comparacion y pediodo de pwm
		TCA0.SINGLE.CMP2 = 0x5DC;
		TCA0.SINGLE.CMP1 = 0x5DC;
		TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;//Habilitar pwm

		//configuracion de vector de interrupcion
		CPUINT.LVL0PRI = ADC0_RESRDY_vect_num;
		CPUINT.LVL1VEC = TCB0_INT_vect_num;	
		//Habilitar interrupciones generales
		sei();
		#if calibracion
		
		_delay_ms(1500);//parpadeo para ver el encendido
		
		offsetsignals();// se obtiene el offset por promedio, de 8 valores por comodidad
		controlautomatico[volante] = canaloffset[volante];
		controlautomatico[acelerador] = canaloffset[acelerador];
		
		#endif
		
		
		PORTA.OUTSET = PIN0_bm;//Termino la configuracion
	}
	//Interrupci�n de lectura de RC	
#if CPPM== true
	ISR(TCB0_INT_vect){//Interrupcion de lecutra y decodificacion de ppm
	if (cont > 0)// lectura del canal no es necesario para decodificador
		canal[cont -1]=TCB0.CCMP;//lectura del canal no necesario para salida decodificada
		//if (cont==2){
			//PORTA.OUTSET = PIN3_bm;
		//}
		//if (cont==3){
			//PORTA.OUTCLR = PIN3_bm;
		//}
	cont++;//siguiente canal
	if (canal[cont-2]>16000)
		cont = 0;
		lectura_canal = true;
	}	
#endif
	//Interrupci�n de contador de milisegundos
	ISR(TCB1_INT_vect){//contador de milisegundos, para generador de trayectorias
		millis[acelerador]++;
		millis[volante]++;
		millis[controlador]++;
		TCB1.INTFLAGS &= ~TCB_CAPT_bp;
	}
	//Interrupci�n periodica de 16 milisegundos para lecturas lentas
	ISR(TCB2_INT_vect){//Interrupcion para checar canales cada 10 ms
	
		//Sensor de reversa con filtro
			if (sensor[sensor_reversa]>180){
			if (contfiltro<15){
			contfiltro++;
			}
			}
			if (sensor[sensor_reversa]<140){
			if (contfiltro>0){
			contfiltro--;
			}
			}
			if (contfiltro>10){	
				control_reversa = false;
				//PORTA.OUTSET = PIN0_bm;
			}
			if (contfiltro<5){	
				control_reversa = true;
				//PORTA.OUTCLR = PIN0_bm;
			}
		if (canal[4]<5000){
		manual = true;
		asistido = false;
		autonomo = false;
		primero = true;
		segundo = true;
		tercero = true;
		cuarto = false;
		//PORTA.OUTCLR= PIN0_bm;//led de aviso//descomentar despues
		}
		
		if (canal[4]<6500&&canal[4]>5500){
			manual = false;
			asistido = true;
			autonomo = false;
			currentmillis = millis[acelerador];
			if (currentmillis - previousmillis >= 250) {
				previousmillis = currentmillis;
				//PORTA.OUTTGL = PIN0_bm;
			}
		}
		
		if (canal[4]>7000){
		manual = false;
		asistido = false;
		autonomo = true;
			//PORTA.OUTSET = PIN0_bm;
		}
		TCB2.INTFLAGS &= ~TCB_CAPT_bp;
	}
	//Interrupci�n de lectura de ADC para sensores
	ISR(ADC0_RESRDY_vect){//solo 4 sensores para empezar
	//Canales de 0 al 7 despues a partir del 12 hasta sensor 14
		sensor[contadc-1] = ADC0.RES>>5;//Promedio de 16 muestras
		contadc++;
	if (contadc>= 8){
		contadc = 0;
		lectura = true;
		}
	ADC0.MUXPOS = contadc;
	}
	//Interrupcion USART
	ISR(USART1_RXC_vect){
		
		buffer_sbus = (uint8_t)USART1.RXDATAL;
		sbuschannelindex++;
		if((buffer_sbus == 0x0F)&&(sbus_anterior== 0x00)){
			sbuschannelindex = 0x00;
		}
		sbuschannel[sbuschannelindex] = buffer_sbus;
		if (sbuschannelindex==2)
		{	
			canal[volante] = sbuschannel[1] + ((sbuschannel[2]&0x07)<<8);
		}
		if (sbuschannelindex==3)
		{
			canal[acelerador] = ((sbuschannel[3]&0x3F)<<5) + ((sbuschannel[2]&0xF8)>>3);
		}
		if (sbuschannelindex==4)
		{
			canal[arma] = ((sbuschannel[3]&0x3F)<<5) + ((sbuschannel[2]&0xF8)>>3);
		}
		sbus_anterior = buffer_sbus;
		//nuevo_canal[sbuschannelindex] = sbuschannel[sbuschannelindex];
	}
	//Interrupci�n de reloj de tiempo real de seguridad
	//ISR(RTC_CNT_vect){//Interrupcion por tiempo seguridad
		////Apagar motores
		//TCA0.SINGLE.CMP1 = canaloffset[acelerador];
		//TCA0.SINGLE.CMP2 = canaloffset[acelerador];
		//while(1){
		//PORTA.OUTTGL = PIN0_bm;//led de aviso
		//_delay_ms(1000);
		//}//se detiene el programa
	//}
	
	int main(void){
		setup();
		while (1){
	//valor de canal de 4000 a 8000 con 6000 de punto medio
	//motores aceptan valores de 1000 a 2000 punto medio de 1500
	//ajuste de se�al de lectura para se�al de motores
	//salida de motore
	
	#ifdef decodificador 
		while(1){
			////if (nuevo_canal[3]>1000){
			//////PORTA.OUTSET = PIN0_bm;
			////}else{
			//////PORTA.OUTCLR = PIN0_bm;
			////}
			TCA0.SINGLE.CMP1 = nuevo_canal[3]+500;
			TCA0.SINGLE.CMP2 = nuevo_canal[4]+500;
		}
	#endif

	
		if (1){//manual
			canalcontrol[volante] = canal[volante];// + Controlador_P[0] - Controlador_P [1];
			canalcontrol[acelerador] = canal[acelerador];
			reversa = false;
		}
		//if (autonomo){
			////millis[controlador] = 0;
			//reversa = control_reversa;
			//if (sensoroffset[sensor_acelerometro]<=sensor[sensor_acelerometro]){//valores negativos
				//Controlador_P[volante]=sensor[sensor_acelerometro]-sensoroffset[sensor_acelerometro];//Error del controlador
				//canalcontrol[volante] = canal[volante]+(Controlador_P[volante]<<4);//canal[volante] + Controlador_P[0] - Controlador_P [1];	
				//canalcontrol[acelerador] = canal[acelerador]-(Controlador_P[volante]<<2);	
			//}else{// valores positivos
				//Controlador_P[volante]=sensoroffset[sensor_acelerometro]-sensor[sensor_acelerometro];//Error del controlador
				//canalcontrol[volante] = canal[volante]-(Controlador_P[volante]<<4);//canal[volante] + Controlador_P[0] - Controlador_P [1];	
				//error_anterior = Controlador_P[volante];
				//canalcontrol[acelerador] = canal[acelerador]-(Controlador_P[volante]<<2);
			//}
		//}
		//if (asistido){
			////canalcontrol[volante] = giro(2800,500);
			////canalcontrol[acelerador] = canaloffset[volante];//adelante(3000,500);//adelante(2000,1000);
			////funcion de trayectoria
			////ocho();
			//canalcontrol[volante] = canal[volante];// + Controlador_P[0] - Controlador_P [1];
			//canalcontrol[acelerador] = canal[acelerador];
			//reversa = true;
		//}
	//Cinematica del robot, valores de 4000 an 8000 con dos variables de control canalcontrol volante y acelerador
	if(!reversa){//reversa con sensor
		if (canalcontrol[volante]>=canaloffset[volante]){
			canalcontrol[volante] = ((canalcontrol[volante]-canaloffset[volante])>>1);//mezcladora
			motor[izquierdo] = (canalcontrol[acelerador]-canalcontrol[volante] )>>2;
			motor[derecho] = (canalcontrol[acelerador]+canalcontrol[volante] )>>2;
			}else{
			canalcontrol[volante] = (canaloffset[volante]-canalcontrol[volante])>>1;//mezcladora
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