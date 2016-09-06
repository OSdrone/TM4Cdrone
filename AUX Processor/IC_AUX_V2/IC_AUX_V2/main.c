/*
 * IC_AUX.c
 *
 * Created: 20/11/2015 17:42:57
 * Author : Ruben
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL

#define Numero_canales 8
#define Numero_celdas 3

#define Rango_PWM_MAX 255

#define Ticks_min 1000
#define Ticks_max 2000


#define Conversor_Rango(Ticks_pulso) ((Ticks_pulso - Ticks_min) * Rango_PWM_MAX) / (Ticks_max - Ticks_min)
#define Pulso_256us (TCCR0 = _BV(CS01) | _BV(CS00))

enum {Normal, Conexion_Perdida}Estado = Normal;

/*
struct { uint8_t PWM_canal[Numero_canales];
	uint8_t Voltaje_Celda[Numero_celdas];
	uint8_t Error;
	}Datos = {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0}, 0};


uint16_t TickSubida = 0;
*/

static uint8_t PWM_canal[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int main(void){	   
	//Configuracion pines
	DDRD = 0x00; //Entradas
	DDRC = _BV(PINC3);
	DDRB = _BV(PINB0) | _BV(PINB1) | _BV(PINB2) | _BV(PINB3); //Pin 0 PPM, el resto PWM a implemetar
	//Configuramos Timer 1 (16 bits ) a unos 50ms
	TCCR1B = 0x01; // No captura, clock/8, Normal mode
	TIMSK = _BV(TOIE1); // Interrupcion de desborde activada
	
	uint8_t Entrada_anterior;
	uint8_t Entrada_actual;
	uint8_t Entradas_cambiadas;

	uint16_t Tiempo_Inicio_Flanco[Numero_canales];
	uint8_t Comunicacion_activa = 0x00;
	
    while (1){
		
		Entrada_anterior = Entrada_actual;
		Entrada_actual = PIND;
		Entradas_cambiadas = (Entrada_actual ^ Entrada_anterior);
		
		//...Canal..0..//
		if(Entradas_cambiadas & _BV(0)){
			if(Entrada_actual & __BV(0)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[0] = TCNT1; //Tick
				Comunicacion_activa = 1;
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}else{
				PWM_canal[0] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[0]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
		
		//...Canal..1..//
		if(Entradas_cambiadas & _BV(1)){
			if(Entrada_actual & __BV(1)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[1] = TCNT1; //Tick
			}else{
				PWM_canal[1] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[1]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
		
		//...Canal..2..//
		if(Entradas_cambiadas & _BV(2)){
			if(Entrada_actual & __BV(2)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[2] = TCNT1; //Tick
			}else{
				PWM_canal[2] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[2]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
		
		//...Canal..3..//
		if(Entradas_cambiadas & _BV(3)){
			if(Entrada_actual & __BV(3)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[3] = TCNT1; //Tick
			}else{
				PWM_canal[3] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[3]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
		
		//...Canal..4..//
		if(Entradas_cambiadas & _BV(4)){
			if(Entrada_actual & __BV(4)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[4] = TCNT1; //Tick
			}else{
				PWM_canal[4] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[4]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
		
		//...Canal..5..//
		if(Entradas_cambiadas & _BV(5)){
			if(Entrada_actual & __BV(5){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[5] = TCNT1; //Tick
			}else{
				PWM_canal[5] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[5]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
		
		//...Canal..6..//
		if(Entradas_cambiadas & _BV(6)){
			if(Entrada_actual & __BV(6)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[6] = TCNT1; //Tick
			}else{
				PWM_canal[6] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[6]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
		
		//...Canal..7..//
		if(Entradas_cambiadas & _BV(7)){
			if(Entrada_actual & __BV(7)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[7] = TCNT1; //Tick
			}else{
				PWM_canal[7] = (uint8_t)Conversor_Rango(TCNT1 - Tiempo_Inicio_Flanco[7]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				Pulso_256us;
			}
		}
	/*
		}
		//Canal 0
		while(DDRD & _BV(PIND0)); //Esperamos 0 en canal 0;
		while(!(DDRD & _BV(PIND0))); //Esperamos al 1;
		TickSubida = TCNT1;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		while(DDRD & _BV(PIND0)); //Esperamos 0 en canal 0;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[0] = Conversor_Rango(TCNT1 - TickSubida);
		
		//Canal 1
		TickSubida = TCNT1;
		while(DDRD & _BV(PIND1)); //Esperamos 0 en canal 1;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[1] = Conversor_Rango(TCNT1 - TickSubida);		

		//Canal 2
		TickSubida = TCNT1;
		while(DDRD & _BV(PIND2)); //Esperamos 0 en canal 2;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[2] = Conversor_Rango(TCNT1 - TickSubida);

		//Canal 3
		TickSubida = TCNT1;
		while(DDRD & _BV(PIND3)); //Esperamos 0 en canal 3;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[3] = Conversor_Rango(TCNT1 - TickSubida);

		//Canal 4
		TickSubida = TCNT1;
		while(DDRD & _BV(PIND4)); //Esperamos 0 en canal 4;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[4] = Conversor_Rango(TCNT1 - TickSubida);

		//Canal 5
		TickSubida = TCNT1;
		while(DDRD & _BV(PIND5)); //Esperamos 0 en canal 5;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[5] = Conversor_Rango(TCNT1 - TickSubida);

		//Canal 6
		TickSubida = TCNT1;
		while(DDRD & _BV(PIND6)); //Esperamos 0 en canal 6;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[6] = Conversor_Rango(TCNT1 - TickSubida);

		//Canal 7
		TickSubida = TCNT1;
		while(DDRD & _BV(PIND7)); //Esperamos 0 en canal 7;
		PORTB |= _BV(PINB0);  //Subimos la señal PPM
		Pulso_256us;
		PWM_canal[7] = Conversor_Rango(TCNT1 - TickSubida);
		if (PWM_canal[7] > 128){
			PORTC |= _BV(PINC3);
		}else{
			PORTC &= ~_BV(PINC3);
		}

		//Reset Timer 0
		TCNT1 = 0x00;
*/
	}
}
   

ISR(TIMER1_OVF_vect){
	if(Comunicacion_Activa){
		Comunicacion_Activa = 0;
	else{
		Estado = Conexion_Perdida;
		//Activar Protocolo Perdida Transmision
		}
}

ISR(TIMER0_OVF_vect){
	PORTB |= ~_BV(PINB0);  //Bajamos la señal PPM
	TCCR0 = 0x00;
	TCNT0 = 0;				//Paramos y reiniciamos el contador;
}

ISR(TWI_vect){
	static uint8_t Registro = 0;
	
	switch(TWSR & 0xF4){
		case 0x60: break;
		case 0x80://Recibir
		case 0x88: Registro = TWDR;
		
		case 0xA8:
		case 0xB8: TWDR = *((uint8_t *)&Datos + Registro++);
		if(Registro >= (Numero_canales + Numero_celdas + 1)) Registro=0;
		case 0xC0: break;
	}
}