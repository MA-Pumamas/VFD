/*
* Variador de Frecuencia.c
*
* Created: 8/05/2020 20:17:34
* Author : MA_Pumamas
*/

/*
* control_spwm_con_atmega8.c
*/
/***********************************************************************************
En este primer bloque incluiremos las librer�as necesarias para el resto de la programaci�n.
*/
#define F_CPU 16000000UL
#define BAUD 9600
#include <util/setbaud.h>		//Libreria encargada de calcular el valor del UBBR a partir de la Frecuencia del micro y el Baud deseado

//#define myubrr F_CPU/16/baud-1	

#include <stdbool.h>			//Libreria que permite manejar los tipo de dato Boolean, operadores booleanos y comparaciones		
#include <stdlib.h>				//Libreria que incluye funciones malloc,free (Para separar zonas de memoria)
#include <avr/io.h> 			//para utilizar las entradas y salidas del microcontrolador
#include <avr/interrupt.h>		//para utilizar las interrupciones internas generadas por el TIMER
#include <math.h>				//para generar la funci�n senoidal
#include <util/delay.h>			//Libreria para generar retardos
#include <stdio.h>				//Libreria para usar la funcion Printf y sus complementos para enviar el texto por comunicacion serial

/***********************************************************************************/
static int mf = 20;							// Indice de modulacion en Frecuencia
static int SinDivisions_1;					// Sub divisiones de la onda senoidal
static int SinDivisions_2;
static int microMHz = 16; 					// Frecuencia del CPU en microherz
static int freq_seno = 2000;     			// Frecuencia senoidal
static long int Amplitud_Triangular;   		// Periodo de la PWM
				
//static unsigned int lookUp_1[200];							// Tabla de muestras de la senoidal Actual
//static unsigned int lookUP_2[200];							// Tabla de muestras de la senoidal Memorizada
static unsigned int *lookUp_1;				// Se declara el puntero de la 1� tabla de muestras de la senoidal (Actual)
static unsigned int *lookUP_2;				// Se declara el puntero de la 2� tabla de muestras de la senoidal (Memorizada)

//0b10000010: PWM, Phase Correct
//0b10000000: PWM, Phase & Frequency Correct

static char theTCCR1A = 0b10000000; 		// variable para TCCR1A
long int TOP;								// Amplitud de la senoidal
volatile int num;							// Indice de la table de muestras (Se utiliza el especificador VOLATILE ya que la variable esta siendo usada en una funcion de interrupci�n o en cualquier otra funcion)

long int amplitud_senoidal,n;
static int freq_seno_variable;				// variable usada en la 2 fase para la modulacion por amplitud
static int delay1;							// variable est�tica

//static bool temp_1;						// variable temporal

/***********************************************************************************/

void table_block(int TOP);			//Prototipo de la tabla de valores muestreados
void setup_TIMER(void);				//Prototipo para configurar el TIMER
void setup_ADC(void);				//Prototipo para configurar el ADC
unsigned int read_ADC(void);		//Prototipo para leer el ADC

//void pins_setup(void);			// funcion para configurar los pines de entradas y salidas
void usart_setup();					// funcion para configurar al usart
char usart_rx(void);				// funcion para para recibir dato
void usart_tx(unsigned char data);	// funcion para transmitir dato
void tx_string (char *data);		// funcion para transmitir una cadena de caracteres
int USART_printChart(char character, FILE *stream);

static FILE USART_0_stream = FDEV_SETUP_STREAM(USART_printChart,NULL,_FDEV_SETUP_WRITE);

/***********************************************************************************/

int main(void)		// Funcion principal
{
	/*
	//Valores iniciales (opcionales) de la frecuencia de la senoidal y de la triangular
	//freq_seno = 2000 Hz
	//freq_triangular = 80 kHz
	
	SinDivisions_1 = mf;										// Numero de divisiones para la senoidal			
	Amplitud_Triangular = (microMHz*1e6)/(freq_seno)/(4*mf);	// amplitud de la se�al triangular
	table_block(Amplitud_Triangular);							// Funcion de valores muestreados de la senoidal
	*/
	
	/*
	Definiciones Malloc, Calloc y Free
	
	Malloc: Funcion que reserva un bloque de memorias alineado con el numero de bytes especificado en su argumento
	Calloc: Funcion que cumple la misma funcion que Malloc, pero adicionalmente inicializa los valores a 0
	Free: Libera el espacio de memoria reservada previamente por Malloc o Calloc
	Realloc: Redimensiona el espacio de memoria previamente designado por Malloc o Calloc y adicionalmente este no debe haber sido liberado por la funci�n Free 
	*/
	
	lookUp_1 = (unsigned int*)calloc(200,sizeof(int));		//Asigna un espacio de memoria de 200 datos del tipo entero, inicializando todos a 0 	
	lookUP_2 = (unsigned int*)calloc(200,sizeof(int));		//(Se asigna 200 datos porque es el numero maximo de divisiones para la senoidal)
	
	setup_ADC();		// Funcion para configura el ADC
	setup_TIMER();		// Funcion para configurar el TIMER
	usart_setup();		// Funcion para configurar la comunicacion serial por USART
	
	sei();             	// Habilitar interruciones globales.
	
	while(1)			// Bucle infinito
	{
		n = read_ADC();		//Lectura del valor registrado en el ADC finalizado su conversi�n de analogico a digital
		
		/****************************************************************************************************************************
		Se calcula la frecuencia de la senoidal a partir del valor leido del registro ADC
		La frecuencia de la senoidal se encuentra en el rango de:
		
		Min: 50 Hz
		Max: 2000 Hz
		*/
		
		freq_seno_variable = 50 + (n*(freq_seno-50))/1023;
		printf("freq_seno: %d\n\r",freq_seno_variable);		//Se envia por comunicaci�n serial el valor de la frecuencia senoidal
		
		/****************************************************************************************************************************
		Se calcula el valor del indice de modulaci�n en frecuencia (mf), de acuerdo a los valores limites obtenidos experimentalmente
		
		1 Observaci�n:
			La frecuencia de la se�al triangular obtenida por (freq_triangular = 2 * mf * freq_senoidal), no puede ser mayor a 160 kHz
			pero con este valor no se logra una buena simulaci�n siendo el valor minimo aceptable de 80 kHz.
										�� Realizar pruebas con valores m�s altos de 80 kHz!!
			
			Es por ello que se desarrollo la siguiente formula para calcular mf.
			mf * freq_senoidal = 40 * 1e3 --------(1)
			
		2 Observaci�n:
			El valor de mf no debe ser mayor a 200, de acuerdo a pruebas experimentales
			Es por ello que se agreg� la condici�n en la que, si la freq_senoidal >= 200 Hz el valor del mf ser� calculado por la
			ecuaci�n (1), de lo contrario el valor de mf se mantendr� constante igual a 200.
		
		*/
		
		if (freq_seno_variable >= 200)
		{
			mf = 40*1e3/freq_seno_variable;		//Maxima valor probado 85 | Minimo valor aceptable probado 40
		}
		else
		{
			mf = 200;
		}
		
		printf("mf: %d\n\r\r",mf);		//Se envia por comunicaci�n serial el valor del indice de modulaci�n en frecuencia (mf)
		
		/****************************************************************************************************************************/
		
		SinDivisions_1 = mf;				// Numero de divisiones para la se�al senoidal
		
		Amplitud_Triangular = (microMHz*1e6)/(freq_seno_variable)/(4*mf);	// amplitud de la se�al triangular
		
		table_block(Amplitud_Triangular);	//Funci�n para calcular todos los valores de comparaci�n de la se�al senoidal 
		ICR1 = Amplitud_Triangular;			//ICR1 es el TOP Value del Timer1 durante su ejecuci�n
		
		/****************************************************************************************************************************/
	}
}
/************************************************************************************/
//Configuraci�n y activaci�n de la comunicacion USART
void usart_setup()
{
	UBRRH = UBRRH_VALUE;			// configura la velocidad de transmision
	UBRRL = UBRRL_VALUE;
	UCSRB |= (1<<TXEN| 1<<RXEN);	// habilita la transmision y recepcion de la usart
	UCSRC |= (1<<URSEL| 0<<UMSEL| 1<<USBS| 0<<UPM1| 0<<UPM0| 1<<UCSZ1| 1<<UCSZ0);	// habilita la escritura en la UCSRC, operacion asincrona, 2 bit de parada, sin paridad, 8 bits de datos

	stdout = &USART_0_stream;
}
char usart_rx(void)
{
	while(!(UCSRA & (1<<RXC)));
	return UDR;
}
void usart_tx(unsigned char data)
{
	while (!( UCSRA & (1<<UDRE)));
	UDR = data;
}
void tx_string (char *data)
{
	while(*data)
	{
		usart_tx(*data);
		data++;
	}
}
int USART_printChart(char character, FILE *stream)
{
	usart_tx(character);
	return 0;
}

/************************************************************************************/
//Configuraci�n y activaci�n Timer1
void setup_TIMER(void)
{
	/************************************************************************************************************************************
	Recomendaci�n del manual del ATMEGA 8 (Pag.91):
	Se utiliza la forma de onda PWM, Phase & Frequency Correct, ya que el valor del TOP Value cambia mientras el Timer1 esta ejecutandose.
	Si el valor del TOP Value fuera estatico no exitiria diferencia entre la forma de PWM, Phase Correct y PWM, Phase & Frequency Correct.
	
	Periodo de la se�al triangular, condiciones:
	- Frecuencia de Reloj: 16 MHz
	- Frecuencia Senoidal: Variable
	- mf: Variable
	
	freq_triangular = 2 * mf * freq_seno --------(1)
	freq_triangular = (freq_reloj) / (2*TOP)-----(2)
	
	Igualando ecucanci�n 1 y 2, Calculamos el TOP Value:
	TOP = (freq_reloj)/(freq_seno)/(4*mf) 
	*/
	/************************************************************************************************************************************/	
	TCCR1A = theTCCR1A; // 0b10000010 (Phase Correct) or 0b10000000 (Phase & Frequency Correct);
	/*10 Se limpia a cero con una comparacion, set at BOTTOM for compA.
	00 compB incialmente desconectado, luego se conmuta por el compA.
	00
	10 WGM1 1:0 para la forma de onda PWM, Phase Correct | 00 WGM1 1:0 para la forma de onda PWM, Phase & Frequency Correct
	*/
	TCCR1B = 0b00010001;
	/*000
	10 WGM1 3:2 para la forma de onda PWM, Phase Correct o PWM, Phase & Frequency Correct
	001 sin prescalador para el conteo | 000 El conteo es parado.
	*/

	//TIMSK = 0b00100100;
	TIMSK = 0b00000100;
	/*
	0 (TICIE1):	ICF1 Interrupci�n por input capture habilitado.
	1 (TOIE1):	TOV1 Interrupci�n por sobre flujo habilitado.
	*/
	ICR1   = Amplitud_Triangular;   
	
	DDRB = 0b00000110;				// Configurar PB1 y PB2 como salidas.
	/************************************************************************************************************************************/
}

/************************************************************************************/
//Configuraci�n y activaci�n del ADC
void setup_ADC(void)
{
	ADCSRA|= (1<<ADEN|0<<ADIE|1<<ADPS2|1<<ADPS1|1<<ADPS0);
	/*
	1(ADEN): Habilita el ADC
	1(ADIE): Habilita la interrupci�n, cuando una conversi�n a sido completada
	111(ADPS2|ADPS1|ADPS0): Selecciona el Prescalador a utilizar (ej: 111 - Prescaler 128)
	*/
	
	ADMUX |= 1<<REFS0;
	/*
	1(REFS0): Selecciona al AVcc como voltaje de referencia, 
			y sobre el pin AREF se de colocar un capacitor externo para evitar el ruido
	*/
}
unsigned int read_ADC(void)
{
	/*
	Al colocar el bit ADSC en 1 se inicia la conversi�n de la se�al de entrada de analogica a digital
	el bit ADSC cambia a 0 al concluir la conversi�n, este dato se utiliza para avisar el registro
	ADC (ADCH y ADCL) ya se encuentra listo con el dato a leer.
	*/
	
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	return ADC;
}

/************************************************************************************/
// Generando tabla de muestras para la onda senoidal.
void table_block(int TOP)
{
	static int m = -1;		//Variable usada como memoria del estado anterior del valor registrado en el ADC
	double temp;			//Variable doble para funciones <math.h>.
	
	if (n!=m)				//Condici�n para evaluar si el valor registrado en el ADC ha cambiado
	{
		/*****************************************************************************************
		Bucle para calcular todos los valores de comparaci�n de la senoidal con la onda triangular
		*/
		
		//lookUp_1 = realloc(lookUp_1, SinDivisions_1 * sizeof(int));		//Se redimensiona el espacio de memoria de acuerdo a la cantidad de divisiones para la onda senoidal
		
		for(int i = 0; i < SinDivisions_1; i++)
		{
			temp = sin((i)*M_PI/(SinDivisions_1-1))*TOP;
			lookUp_1[i] = (int) (temp);						// Redondeado a un entero.
		}
		/*****************************************************************************************/
		while (num < SinDivisions_2)
		{
			//Tiempo de espera hasta que se haya graficado por completo la ultima semionda iniciada antes
			//de ingresar al While 
		}
		/******************************************************************************************
		Bucle que memoriza todos los datos del array "lookUP_1" antes llenado en el array "lookUP_2"
		Siendo los datos del array "lookUP_2" los comparados constantemente con los datos de la onda triangular
		*/
		
		//lookUP_2 = realloc(lookUP_2, SinDivisions_1 * sizeof(int));		//Se redimensiona el espacio de memoria de acuerdo a la cantidad de divisiones para la onda senoidal
		
		for(int i=0; i < SinDivisions_1; i++)	
		{
			lookUP_2[i] = lookUp_1[i];
		}
		
		SinDivisions_2 = SinDivisions_1;		//Se actualiza el numero de divisiones para la se�al senoidal
		/******************************************************************************************/
		m = n;	//Se memoriza el valor registrado por el ADC para una siguiente comparaci�n
	}
}

/***********************************************************************************/
//Func��n de interrupci�n por sobreflujo del timer1
ISR(TIMER1_OVF_vect)
{
	/************************************************************************************************************************************/
	//Codigo para generar un retardo en la primera comparci�n entre la se�al triangular y la senoidal
	//De esta manera se asegura que los mosfet's de ramas distintas no se encuentren conmutadas al mismo tiempo en ningun momento
	
	if (delay1 == 1) {
	theTCCR1A ^= 0b10100000; 	// Toggle connect and disconnect of compareoutput A and B // TCCR1A = 0b10000010
	TCCR1A = theTCCR1A; 		//TCCR1A = 0b10000010
	delay1 = 0;
	}	
	//*/
	
	/****************************************************************************************************************************************************
	Codigo que actualiza el valor del OCR1A/B cada vez que ocurra la interrupci�n por sobreflujo (Cuando el timer alcance el BOTTOM Value que es de 0x00) 
	Adem�s al cumplir medio ciclo de la onda senoidal conmuta las salidas OC1A y OC1B, desactivandose una de ellas y activandose la otra
	*/
	
	if (num >= SinDivisions_2){
		num = 0;                	// Reset num
		//theTCCR1A ^= 0b10100000; 	// Toggle connect and disconnect of compareoutput A and B // TCCR1A = 0b10000010
		//TCCR1A = theTCCR1A; 		//TCCR1A = 0b10000010
		
		delay1++;					//delay relaciona con el codigo anterior para generar el retardo
	}
	
	OCR1A = OCR1B = lookUP_2[num]; 	// Se actualiza el valor del OCR1A/B, cambiando el ciclo de trabajo (Cada interrpci�n por sobreflujo)
	num++;	
	
	/************************************************************************************************************************************/
}

/***********************************************************************************/
//Funci�n de interrupci�n por InputCapture en el Timer1
/*La interrupci�n se encuentra deshabilitada, ya que el timer1 solo tiene 2 posibilidades para actualizar el valor OCR1A/B ya sea en el BOTTOM Value o el TOP Value en un periodo de la se�al
Esto depende del tipo de onda seleccionada, en este caso se esta escogiendo la forma de onda PWM, Phase & Frequency correct, por lo que el OCR1A/B se actualiza en el BOTTOM Value,
es por ello que el valor OCR1A/B solo se actualizar� en el flanco de subida tomando importancia de esta manera el valor que se escriba en el OCR1A/B en la interrupci�n por sobreflujo
ya que esta interrupci�n se ejecuta en el BOTTOM Value.
*/
ISR(TIMER1_CAPT_vect)
{
	OCR1A = OCR1B = lookUP_2[num];
	num++;
}

/***********************************************************************************/
//Funci�n de interrupci�n por fin de conversi�n del ADC
//Interrupci�n deshabilitada en la funci�n de configuraci�n Setup_ADC
ISR(ADC_vect)
{
	//temp_1 = true;	
}