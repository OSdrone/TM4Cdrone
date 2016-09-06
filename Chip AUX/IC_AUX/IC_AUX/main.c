/*/*
 * IC_AUX.c
 *
 * Created: 20/11/2015 17:42:57
 * Author : Ruben
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL

#define Numero_canales 8
#define Numero_celdas 4

enum {Normal=0, Conexion_Perdida}Estado = Normal;

struct {
	 uint16_t PWM_canal[8];
	 uint8_t Voltaje_Bateria[Numero_celdas];
	 uint8_t Comunicacion_activa;
}Datos = {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0}, 0};

int main(void){	   
	//Configuracion pines
	DDRD = 0x00; //Entradas
	DDRC = _BV(PINC3);
	PORTC |= _BV(PINC3);
	DDRB = _BV(PINB0) | _BV(PINB1) | _BV(PINB2) | _BV(PINB3); //Pin 0 PPM, el resto PWM a implemetar
		
	//Configuramos Timer 1 (16 bits ) a unos 65ms	
	TCCR1A = 0;
	TCCR1B = _BV(CS11); // No captura, clock/8, Normal mode
	TIMSK = _BV(TOIE1); // Interrupcion de desborde activada
	
	//Configurar Timer 0

	TCCR0 = _BV(CS01);
	TIMSK |= _BV(TOIE0);
	
	//Configurar I2C
	TWAR = 0x03; //Direccion esclavo 0x1, llamada al 0 aceptada tambien
	
	TWSR = 0x00;
	TWBR = 2;    // 400Khz de velocidad en I2C
	
	TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
	
	uint8_t Entrada_anterior = 0;
	uint8_t Entrada_actual = 0;
	uint8_t Entradas_cambiadas = 0;

	uint16_t Tiempo_Inicio_Flanco[Numero_canales] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	sei();
    
	while (1){
		
		Entrada_anterior = Entrada_actual;
		Entrada_actual = PIND;
		Entradas_cambiadas = (Entrada_actual ^ Entrada_anterior);
		
		//...Canal..0..//
		if(Entradas_cambiadas & _BV(0)){
			if(Entrada_actual & _BV(0)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[0] = TCNT1; //Tick
				
				Estado = Normal;
				
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);				
			}else{
				Datos.PWM_canal[0] = (TCNT1 - Tiempo_Inicio_Flanco[0]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
			}
		}
		
		//...Canal..1..//
		if(Entradas_cambiadas & _BV(1)){
			if(Entrada_actual & _BV(1)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[1] = TCNT1; //Tick
			}else{
				Datos.PWM_canal[1] = (TCNT1 - Tiempo_Inicio_Flanco[1]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
			}
		}
		
		//...Canal..2..//
		if(Entradas_cambiadas & _BV(2)){
			if(Entrada_actual & _BV(2)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[2] = TCNT1; //Tick
			}else{
				Datos.PWM_canal[2] = (TCNT1 - Tiempo_Inicio_Flanco[2]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
			}
		}
		
		//...Canal..3..//
		if(Entradas_cambiadas & _BV(3)){
			if(Entrada_actual & _BV(3)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[3] = TCNT1; //Tick
			}else{
				Datos.PWM_canal[3] = (TCNT1 - Tiempo_Inicio_Flanco[3]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
			}
		}
		
		//...Canal..4..//
		if(Entradas_cambiadas & _BV(4)){
			if(Entrada_actual & _BV(4)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[4] = TCNT1; //Tick
			}else{
				Datos.PWM_canal[4] = (TCNT1 - Tiempo_Inicio_Flanco[4]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
			}
		}
		
		//...Canal..5..//
		if(Entradas_cambiadas & _BV(5)){
			if(Entrada_actual & _BV(5)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[5] = TCNT1; //Tick
			}else{
				Datos.PWM_canal[5] = (TCNT1 - Tiempo_Inicio_Flanco[5]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
			}
		}
		
		//...Canal..6..//
		if(Entradas_cambiadas & _BV(6)){
			if(Entrada_actual & _BV(6)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[6] = TCNT1; //Tick
			}else{
				Datos.PWM_canal[6] = (TCNT1 - Tiempo_Inicio_Flanco[6]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
			}
		}
				
		
		//...Canal..7..//
		if(Entradas_cambiadas & _BV(7)){
			if(Entrada_actual & _BV(7)){ //Estado actual 1 (flanco ascendente)
				Tiempo_Inicio_Flanco[7] = TCNT1; //Tick
			}else{
				Datos.PWM_canal[7] = (TCNT1 - Tiempo_Inicio_Flanco[7]);
				PORTB |= _BV(PINB0);  //Subimos la señal PPM
				TCCR0 = _BV(CS01);
				/*
				if ((Datos.PWM_canal[7] > 0 && Datos.PWM_canal[7] <83)){	
//					PORTC |= _BV(PINC3);				
				}else{
//					PORTC &= ~_BV(PINC3);					
				}
				*/
			}
		}					
	}
}
   

ISR(TIMER1_OVF_vect){
	if(Estado == Normal){
		Datos.Comunicacion_activa = 0x11;
		PORTC &= ~_BV(PINC3);
		Estado = Conexion_Perdida;
	}else{
		Datos.Comunicacion_activa = 0;
		PORTC |= _BV(PINC3);
		//Activar Protocolo Perdida Transmision
		}
}

ISR(TIMER0_OVF_vect){
	PORTB &= ~_BV(PINB0);;  //Bajamos la señal PPM
	TCCR0 = 0x00;
	TCNT0 = 0;				//Paramos y reiniciamos el contador;
}

ISR(TWI_vect){
	static uint8_t Registro = 0;
	
	switch((TWSR & 0xF8)){
		case 0x60: break;
		case 0x80: break;
		case 0x88: break;
		
		case 0xA8: Registro = 0; 
				   TWDR = *((uint8_t *)&Datos + Registro++);
//		case 0xA8: TWDR = 0x11;
				   break;
		case 0xB8: TWDR = *((uint8_t *)&Datos + Registro++);
//		case 0xB8: TWDR =  0x12;
		case 0xC0: break;
	}
	TWCR |= _BV(TWINT); 
}