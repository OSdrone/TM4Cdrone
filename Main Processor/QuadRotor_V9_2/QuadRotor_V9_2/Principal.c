/*
AHRS como tarea
Todos Sensores IMU en una tarea.
Telemetria sensores sin filtrar. paso de dato con buzon
Añadida correcccion de posicion mediante rotacion de la medidas, con calculo de matriz al inicio del ciclo




Sincronizacion de arranaque cambiada, las tareas inician su propio clock excepto el IMU
IMU->Se calibra, Guarda la primera lectura, Inicio AHRS , Tarea ciclico.
											*Reset de los parametros, Guarda la primera lectura, Inicio Coordinador , Tarea ciclico.
																								 Arranca WD, SYNCRO, Tarea ciclico.


*/

/* PARAMETROS CONFIGURACION */
#define Estimador_Parcial
//#define Filtro_Perturbaciones
#define IMU_MPU6050
//#define GYRO_L3G4200
#define COMPASS_HMC5883L
//#define Sensor_RPM
#define Filtrado_Vel_IMU
//#define Sensor_RPM


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
//#include <xdc/runtime/IHeap.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/Knl/Task.h>
#include <ti/sysbios/Knl/Clock.h>
#include <ti/sysbios/Knl/Semaphore.h>
#include <ti/sysbios/Knl/Mailbox.h>
#include <ti/sysbios/Knl/Swi.h>
#include <ti/sysbios/gates/GateMutexPri.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Watchdog.h>

/* Periphals libraries */
#include <driverlib/eeprom.h>

/* Board Header file */
#include "Quad_Board.h"
#include "arm_math.h"

#include "Parametros.h"
#include "Funciones_Transferencia.h"

#include "AHRS.h"
#include "Sensores.h"
#include "Servidores.h"
//#include "Transmisores.h"

//................Variables.....................................................//
//...Sistema....//
Ptr Datos;

tpCalibracion_Receptor Calibracion_Receptor = { //POENR PROTEGIDO???
		{.Rango_Salida = {-1, 1},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-1, 1},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {0, 1},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-1, 1},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-1, 1},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-1, 1},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-1, 1},
		 .Rango_Entrada = {966, 1973}
		},
		{.Rango_Salida = {-100, 100},
		 .Rango_Entrada = {966, 1973}
		}
};
tpCalibracion_IMU Calibracion_IMU = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		.Correccion_Alineamiento_matriz = {1, 0, 0, 0, 1, 0, 0, 0, 1},
		.Correccion_Alineamiento.numCols = 3,
		.Correccion_Alineamiento.numRows = 3,
		.Correccion_Alineamiento.pData = Calibracion_IMU.Correccion_Alineamiento_matriz,
		.Giro = {47.5, 0, 0}
};
tpCalibracion_Brujula Calibracion_Brujula = {0, 0, 0, 0, 0, 0};
tpModoCalibracionIMU ModoCalibracionIMU = CALIBRACION_COMPLETA_IMU; //CALIBRACION_COMPLETA_IMU;

tpEstado_Sistema Estado_Sistema  = ESPERA;
tpEstado_Sistema Estado_Sistema_Anterior = ESPERA;

tpModoTelemetria ModoTelemetria = TELEMETRIA_CONTROL;
tpInfoTelemetria InfoTelemetria = TELE_0;
const tpOrden START_FRAME = START;
const tpOrden FINAL_FRAME = FINAL;

tpModo_Control Modo_Control = ANGULOS_3;
tpModo_Control Modo_Control_Anterior = ANGULOS_3;
tpModoPerturbaciones ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; //NO_CORREGIR_PERTURBACIONES;
//tpModoPerturbaciones Perturbaciones_Seleccionada = CORREGIR_PERTURBACIONES;

tpModoWatchdog ModoWatchdog = PARADA_EMER;

tpModeloGiroscopo ModeloGiroscopo = GYRO_IMU6050;

//....SENSORES....//
tpIMU6050 IMU6050 = {
	.Direccion = Dir_0_IMU_MPU6050,
	.SMPLRT_DIV = 0,
	.DLPF_CFG = DLPF_CFG_1,
	.Ganancia_Gyro = Gain_Gyro_500,
	.Ganancia_Acel = Gain_Acel_16G,
	.Sensibilidad_Giroscopo = 32768.0 / 500.0,
	.Sensibilidad_Acel = 32768.0 /16.0
};

tpGiroscopo_L3G4200 Giroscopo_L3G4200 = {
		.Direccion = Dir_1_L3G4200,
		.Ganancia = dps_2000,
		.Sensibilidad_Giroscopo = 32768.0 / 2000.0,
		.ODR = ODR_800_Hz,
		.BDU = BDU_Continuo,
		.Modo = Bypass,
		.BLE = BLE_Big_Endian,
		.BW_LPF = LPF1_1,
		.BW_HPF = HPF_0,
		.HPF_activar = HPF_No_Filtro,
		.HPF_modo = HPF_Normal,
		.Modo_Filtro = Filtrado_LPF2
};

tpHMC5883L Brujula_HMC5883L = {
	.Angulo_Rotacion = 0,
	.Ganancia = Gauss_1_3,
	.Modo_Operacion = Continuo,
	.Modo_Medida = Normal,
	.Muestras_Media = MEDIA_4,
	.ODR = ODR_75_Hz,
	.Velocidad_I2C = I2C_400_Khz,
	.Sensibilidad = S_1
};

//..Variables del sistema...//
float32_t Posicion_inicial = 0;
uint16_t Altura_US_mm = 0;
uint16_t Altura_Presion_mm = 0;
float32_t Gravedad;

//......Identificacion..........//
uint32_t nDatos_Identifiacion = 0;
uint32_t PuntoTrabajo_motor = 0;
uint16_t nDatos_leidos = 0;
uint16_t Ticks_por_RPS = 0;

//................Instancias....................................................//
//.......PWM............//
PWM_Handle PWM0;
PWM_Handle PWM1;
PWM_Handle PWM2;
PWM_Handle PWM3;

PWM_Params PARAMS_PWM0;
PWM_Params PARAMS_PWM1;
PWM_Params PARAMS_PWM2;
PWM_Params PARAMS_PWM3;

//.......I2C............//
I2C_Handle I2C_PRINCIPAL;
I2C_Handle I2C_AUX;

I2C_Params PARAMS_I2C;

//I2C_Params PARAMS_I2C_AUX;

//.......UART............//
UART_Handle UART_USB;
//UART_Handle UART_BT_MANDO;
UART_Handle UART_BT_TELEMETRIA;
UART_Handle UART_AUX;

UART_Params PARAMS_UART_USB;
UART_Params PARAMS_UART_BT_TELEMETRIA;
UART_Params PARAMS_UART_AUX;

//......SPI...nRF24L01......//
//tp_nRF24L01 nRF24L01;
//SPI_Params PARAMS_SPI_0;

//.....Timer..............//
Timer_Handle US_Timer;
Timer_Params PARAMS_US_Timer;

//....WATCHDOG...........//
Watchdog_Handle WatchDog_0;
Watchdog_Params PARAMS_WatchDog_0;

//...BUZON............//
Mailbox_Handle Buzon_Lecturas_IMU;
Mailbox_Params PARAMS_Buzon;

/*
Mailbox_Handle Buzon_Calibracion_IMU;
Mailbox_Handle Buzon_Calibracion_Brujula;
Mailbox_Params PARAMS_Buzon_Calibracion;
*/


//.....................TASK.......SEMAPHORES.............CLOCK........................//

Task_Params Parametros_Tarea;
Semaphore_Params Parametros_Semaforo;
Clock_Params Parametos_Clock;

//....CONTROL....//
Task_Handle TASK_Control;
Semaphore_Handle SEMAPHORE_Control;
Clock_Handle CLOCK_Control;

//....ALTURA_US..//
Task_Handle TASK_Calculo_Altura;
Semaphore_Handle SEMAPHORE_Calculo_Altura;
Clock_Handle CLOCK_Calculo_Altura;

//....Leer_IMU...//
Task_Handle TASK_Leer_IMU;
Semaphore_Handle SEMAPHORE_Leer_IMU;
Clock_Handle CLOCK_Leer_IMU;

//...Calculo_AHRS....//
Task_Handle TASK_Calculo_AHRS;
Semaphore_Handle SEMAPHORE_Calculo_AHRS;
Clock_Handle CLOCK_Calculo_AHRS;

//....Identificacion......//
Task_Handle TASK_Identificacion;
Semaphore_Handle SEMAPHORE_Identificacion;
Clock_Handle CLOCK_Identificacion;

//....Coordinador......//
Task_Handle TASK_Coordinador;
Semaphore_Handle SEMAPHORE_Coordinador;
Clock_Handle CLOCK_Coordinador;

//............ERROR..........................//
Error_Block eb;

//...................FUNCIONES......................................................//
void Rotacion_X(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes);
void Rotacion_Y(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes);
void Rotacion_Z(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes);
void Rotacion_ZYZp(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes);
void Rotacion_ZXY(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes);
//void Rotacion_XYZ(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes);

//.........WATCHDOG.................................//
void FuncionWatchDog();
void Reestablecer_Conexion();
//...Control.....//
void Control(UArg arg0, UArg arg1);
void CLK_Control();
//...US..........//
void ISR_Timer_US();
void ISR_GPIO_US();
void Calculo_Altura(UArg arg0, UArg arg1);
void CLK_Calculo_Altura();
//....Leer_IMU...//
void Lectura_Datos_IMU(UArg arg0, UArg arg1);
void CLK_Lectura_Datos_IMU();
//....Calculo_AHRS...//
void Calculo_AHRS(UArg arg0, UArg arg1);
void CLK_Calculo_AHRS();
//....Identificacion......//
void Identificacion(UArg arg0, UArg arg1);
void CLK_Identificacion();
#ifdef Sensor_RPM
	void ISR_GPIO_RPM();
#endif

//....Coordinador......//
void Coordinador(UArg arg0, UArg arg1);
void CLK_Coordinador();


/*
 *  ======== main ========
 */
int main(void){
    Error_init(&eb);
//...........................INICIALIZACION...DRIVERS...................................................//
    /* Call board init functions */
	QUAD_BOARD_initGeneral();
    QUAD_BOARD_initGPIO();
    QUAD_BOARD_initPWM();
    QUAD_BOARD_initI2C();
    QUAD_BOARD_initSPI();
    QUAD_BOARD_initUART();
    QUAD_BOARD_initWatchdog();

//......SERVIDORES..........//
    Iniciar_Servidores();

    GPIO_write(QUAD_BOARD_LED_RED, 1);
    GPIO_write(QUAD_BOARD_LED_GREEN, 0);
    GPIO_write(QUAD_BOARD_LED_BLUE, 0);

//......PWM........//
    PWM_Params_init(&PARAMS_PWM0);
    PARAMS_PWM0.period = 5000;
    PARAMS_PWM0.dutyMode = PWM_DUTY_TIME;
    PWM_Params_init(&PARAMS_PWM1);
    PARAMS_PWM1.period = 5000;
    PARAMS_PWM1.dutyMode = PWM_DUTY_TIME;
    PWM_Params_init(&PARAMS_PWM2);
    PARAMS_PWM2.period = 5000;
    PARAMS_PWM2.dutyMode = PWM_DUTY_TIME;
    PWM_Params_init(&PARAMS_PWM3);
    PARAMS_PWM3.period = 5000;
    PARAMS_PWM3.dutyMode = PWM_DUTY_TIME;

    PWM0 = PWM_open(QUAD_BOARD_PWM0, &PARAMS_PWM0);
    PWM1 = PWM_open(QUAD_BOARD_PWM1, &PARAMS_PWM1);
    PWM2 = PWM_open(QUAD_BOARD_PWM2, &PARAMS_PWM2);
    PWM3 = PWM_open(QUAD_BOARD_PWM3, &PARAMS_PWM3);
/*
    PWM_setDuty(PWM0, Pulso_maximo_PWM_motor);
    PWM_setDuty(PWM1, Pulso_maximo_PWM_motor);
    PWM_setDuty(PWM2, Pulso_maximo_PWM_motor);
    PWM_setDuty(PWM3, Pulso_maximo_PWM_motor);
*/
    PWM_setDuty(PWM0, 0);
    PWM_setDuty(PWM1, 0);
    PWM_setDuty(PWM2, 0);
    PWM_setDuty(PWM3, 0);
//.......I2C............//
    I2C_Params_init(&PARAMS_I2C);
    PARAMS_I2C.bitRate = I2C_400kHz;
    PARAMS_I2C.transferMode = I2C_MODE_BLOCKING;
    PARAMS_I2C.transferCallbackFxn = NULL;

    I2C_PRINCIPAL = I2C_open(QUAD_BOARD_I2C0, &PARAMS_I2C);

//    I2C_Params_init(&PARAMS_I2C_AUX);
//    PARAMS_I2C.bitRate = I2C_400kHz;
//    PARAMS_I2C.transferMode = I2C_MODE_BLOCKING;
//    PARAMS_I2C.transferCallbackFxn = NULL;

    I2C_AUX = I2C_open(QUAD_BOARD_I2C2, &PARAMS_I2C);

//........UART..........//
    UART_Params_init(&PARAMS_UART_BT_TELEMETRIA);
    PARAMS_UART_BT_TELEMETRIA.baudRate = BAUD_RATE_460800;
    PARAMS_UART_BT_TELEMETRIA.dataLength = UART_LEN_8;
    PARAMS_UART_BT_TELEMETRIA.parityType = UART_PAR_NONE;
    PARAMS_UART_BT_TELEMETRIA.stopBits = UART_STOP_ONE;
    PARAMS_UART_BT_TELEMETRIA.readEcho = UART_ECHO_OFF;

    PARAMS_UART_BT_TELEMETRIA.readReturnMode = UART_RETURN_FULL;
    PARAMS_UART_BT_TELEMETRIA.writeDataMode = UART_DATA_BINARY;
    PARAMS_UART_BT_TELEMETRIA.readDataMode = UART_DATA_BINARY;
    PARAMS_UART_BT_TELEMETRIA.readMode = UART_MODE_BLOCKING;
    PARAMS_UART_BT_TELEMETRIA.writeMode = UART_MODE_BLOCKING;
    PARAMS_UART_BT_TELEMETRIA.writeTimeout = BIOS_WAIT_FOREVER;
    //PARAMS_UART_BT_TELEMETRIA.readTimeout = BIOS_NO_WAIT;
    PARAMS_UART_BT_TELEMETRIA.readTimeout = 180;

    UART_BT_TELEMETRIA = UART_open(QUAD_BOARD_UART5_BT_TELEMETRIA, &PARAMS_UART_BT_TELEMETRIA);

//.......SPI...........//

/*
	SPI_Params_init(&PARAMS_SPI_0);

	PARAMS_SPI_0.transferMode = SPI_MODE_BLOCKING;
	PARAMS_SPI_0.transferTimeout = BIOS_WAIT_FOREVER;
	PARAMS_SPI_0.mode = SPI_MASTER;
	PARAMS_SPI_0.bitRate = BITRATE_SPI;
	PARAMS_SPI_0.dataSize = 8;
	PARAMS_SPI_0.frameFormat = SPI_POL0_PHA0;

	nRF24L01.SPI = SPI_open(QUAD_BOARD_SPI0, &PARAMS_SPI_0);
	nRF24L01.PIN_CE = QUAD_BOARD_SPI_CE;
	nRF24L01.PIN_CSN = QUAD_BOARD_SPI_CSN;
	nRF24L01.PIN_IRQ = NULL;
*/

//.....WATCHDOG.....................//
	Watchdog_Params_init(&PARAMS_WatchDog_0);
	PARAMS_WatchDog_0.callbackFxn = FuncionWatchDog;
	PARAMS_WatchDog_0.debugStallMode = Watchdog_DEBUG_STALL_ON;
	PARAMS_WatchDog_0.resetMode = Watchdog_RESET_OFF;

//.....BUZON...............//
/*
	Mailbox_Params_init(&PARAMS_Buzon_Calibracion);
	Buzon_Calibracion_IMU = Mailbox_create(sizeof(tpLecturas_IMU), 1, &PARAMS_Buzon_Calibracion, &eb);
	Buzon_Calibracion_Brujula = Mailbox_create(sizeof(tpLecturas_Brujula), 1, &PARAMS_Buzon_Calibracion, &eb);
*/
	Mailbox_Params_init(&PARAMS_Buzon);
	Buzon_Lecturas_IMU = Mailbox_create(sizeof(tpLecturas_IMU), 1, &PARAMS_Buzon, &eb);

//.........TIMER...............//
	Timer_Params_init(&PARAMS_US_Timer);
	PARAMS_US_Timer.period = Pulso_arranque_us;  //Para una distancia MAX de 4 metros (aprox)
   	PARAMS_US_Timer.periodType = Timer_PeriodType_MICROSECS;
    PARAMS_US_Timer.runMode = Timer_RunMode_ONESHOT;
    PARAMS_US_Timer.startMode = Timer_StartMode_USER;

    US_Timer = Timer_create(5, ISR_Timer_US, &PARAMS_US_Timer, &eb );

//................INTERRUPCIONES.....................................//
    GPIO_setCallback(QUAD_BOARD_ECHO, ISR_GPIO_US);
    GPIO_enableInt(QUAD_BOARD_ECHO);
#ifdef Sensor_RPM
    GPIO_setCallback(QUAD_BOARD_RPM, ISR_GPIO_RPM);
    GPIO_enableInt(QUAD_BOARD_RPM);
#endif

//...........................INICIALIZACION...TAREAS...................................................//
//....Control....//
	Task_Params_init(&Parametros_Tarea);
	Parametros_Tarea.priority = PRIORIDAD_Control;
	Parametros_Tarea.stackSize = 2304;
	TASK_Control = Task_create(Control, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
	SEMAPHORE_Control = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

	Clock_Params_init(&Parametos_Clock);
	Parametos_Clock.period = PERIODO_Control;
	Parametos_Clock.startFlag = false;
    CLOCK_Control = Clock_create(CLK_Control, Timeout_Clk_Control, &Parametos_Clock, &eb);


//....Altura..//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Calculo_Altura;
    Parametros_Tarea.stackSize = 768;
	TASK_Calculo_Altura = Task_create(Calculo_Altura, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Calculo_Altura = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Calculo_Altura;
    Parametos_Clock.startFlag = false;
    CLOCK_Calculo_Altura = Clock_create(CLK_Calculo_Altura, Timeout_Clk_Calculo_Altura, &Parametos_Clock, &eb);

//....Leer_IMU...//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Leer_IMU;
    Parametros_Tarea.stackSize = 1256;
	TASK_Leer_IMU = Task_create(Lectura_Datos_IMU, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Leer_IMU = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Leer_IMU;
    Parametos_Clock.startFlag = true;
    CLOCK_Leer_IMU = Clock_create(CLK_Lectura_Datos_IMU, Timeout_Clk_Leer_IMU, &Parametos_Clock, &eb);

//...Calculo AHRS....//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Calculo_AHRS;
	TASK_Calculo_AHRS = Task_create(Calculo_AHRS, &Parametros_Tarea, &eb);

	Semaphore_Params_init(&Parametros_Semaforo);
	Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Calculo_AHRS = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Calculo_AHRS;
    Parametos_Clock.startFlag = false;
    CLOCK_Calculo_AHRS = Clock_create(CLK_Calculo_AHRS, Timeout_Clk_Calculo_AHRS, &Parametos_Clock, &eb);

//....Identificacion......//
    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Identificacion;
    Parametros_Tarea.stackSize = 512;
    TASK_Identificacion = Task_create(Identificacion, &Parametros_Tarea, &eb);

    Semaphore_Params_init(&Parametros_Semaforo);
    Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Identificacion = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Identificacion;
    Parametos_Clock.startFlag = false;
    CLOCK_Identificacion = Clock_create(CLK_Identificacion, Timeout_Clk_Identificacion, &Parametos_Clock, &eb);

//......Coordinador.......//

    Task_Params_init(&Parametros_Tarea);
    Parametros_Tarea.priority = PRIORIDAD_Coordinador;
    TASK_Coordinador = Task_create(Coordinador, &Parametros_Tarea, &eb);

    Semaphore_Params_init(&Parametros_Semaforo);
    Parametros_Semaforo.mode = Semaphore_Mode_BINARY;
    SEMAPHORE_Coordinador = Semaphore_create(nTokensIniciales_0, &Parametros_Semaforo, &eb);

    Clock_Params_init(&Parametos_Clock);
    Parametos_Clock.period = PERIODO_Coordinador;
    Parametos_Clock.startFlag = false;
    CLOCK_Coordinador = Clock_create(CLK_Coordinador, Timeout_Clk_Coordinador, &Parametos_Clock, &eb);

/*
	PWM_setDuty(PWM0, Pulso_minimo_PWM_motor + Accion_Maxima);
	PWM_setDuty(PWM1, Pulso_minimo_PWM_motor + Accion_Maxima);
	PWM_setDuty(PWM2, Pulso_minimo_PWM_motor + Accion_Maxima);
	PWM_setDuty(PWM3, Pulso_minimo_PWM_motor + Accion_Maxima);
    while(GPIO_read(QUAD_BOARD_SW2));

	PWM_setDuty(PWM0, Pulso_minimo_PWM_motor + Accion_Minima);
	PWM_setDuty(PWM1, Pulso_minimo_PWM_motor + Accion_Minima);
	PWM_setDuty(PWM2, Pulso_minimo_PWM_motor + Accion_Minima);
	PWM_setDuty(PWM3, Pulso_minimo_PWM_motor + Accion_Minima);
    while(!GPIO_read(QUAD_BOARD_SW2));
*/

/*
    	PWM_setDuty(PWM0, Accion_Maxima + Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM1, Accion_Maxima + Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM2, Accion_Maxima + Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM3, Accion_Maxima + Pulso_minimo_PWM_motor);
    while(GPIO_read(QUAD_BOARD_SW2));
    	PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
    	PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);
    while(!GPIO_read(QUAD_BOARD_SW2));
*/

    //Iniciamos la matriz de correccion
    Rotacion_ZXY(&Calibracion_IMU.Correccion_Alineamiento, Calibracion_IMU.Giro, false);

    while(GPIO_read(QUAD_BOARD_SW2));

   /* Start BIOS */
    BIOS_start();

    return (0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void FuncionWatchDog(){
	float32_t Referencia[4] = {0, 0, 0, 0};
	UInt Key, Key2, Key3;

	Key = Task_disable();
	Key2 = Hwi_disable();
	Key3 = Swi_disable();

	Watchdog_clear(WatchDog_0);

	switch(Estado_Sistema){
		case VUELO:
		case ESPERA:
		default:
			Estado_Sistema_Anterior = Estado_Sistema;
			Modo_Control_Anterior = Modo_Control;

			Estado_Sistema = ERROR_CONEXION;

			GPIO_write(QUAD_BOARD_LED_RED, 0);
			GPIO_write(QUAD_BOARD_LED_GREEN, 0);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);
		break;
	}

	switch(ModoWatchdog){
		case PARADA_EMER:
			GPIO_write(QUAD_BOARD_LED_RED, 1);
			GPIO_write(QUAD_BOARD_LED_GREEN, 0);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);

			//Stop a todos los clocks y tareas
			Clock_stop(CLOCK_Control);
			Semaphore_reset(SEMAPHORE_Control, 0);
			Clock_stop(CLOCK_Identificacion);
			Semaphore_reset(SEMAPHORE_Identificacion, 0);

			PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);

			while(1);
			//System_exit(0);
		//break;
		case ESTABILIZACION_EMER:
			GPIO_toggle(QUAD_BOARD_LED_RED);
			GPIO_toggle(QUAD_BOARD_LED_GREEN);
			GPIO_toggle(QUAD_BOARD_LED_BLUE);

			Modo_Control = ANGULOS_4;
			memcpy(Direccion_servidor_Referencia(), Referencia, sizeof(Referencia));
		break;
	}

	Swi_restore(Key3);
	Hwi_restore(Key2);
	Task_restore(Key);
}

void Reestablecer_Conexion(){

	Modo_Control = Modo_Control_Anterior;
	Estado_Sistema = Estado_Sistema_Anterior;

	switch(Estado_Sistema){
		case VUELO:
			GPIO_write(QUAD_BOARD_LED_RED, 0);
			GPIO_write(QUAD_BOARD_LED_GREEN, 1);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);
			break;
		case ESPERA:
			GPIO_write(QUAD_BOARD_LED_RED, 1);
			GPIO_write(QUAD_BOARD_LED_GREEN, 1);
			GPIO_write(QUAD_BOARD_LED_BLUE, 0);
			break;
	}
}
void Rotacion_X(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes){
	float32_t Seno = 0;
	float32_t Cos = 0;

	float32_t Aux_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};

	float32_t Matriz_Rotacion_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Matriz_Rotacion = {3, 3, Matriz_Rotacion_Matriz};

	if(Radianes){ Giro = Giro * 180.0/PI; }
	arm_sin_cos_f32(Giro, &Seno, &Cos);
	Matriz_Rotacion.pData[0] = 1;
	Matriz_Rotacion.pData[4] = Cos;
	Matriz_Rotacion.pData[5] = -Seno;
	Matriz_Rotacion.pData[7] = Seno;
	Matriz_Rotacion.pData[8] = Cos;

	arm_copy_f32(Matriz->pData, Aux.pData, 3*Matriz->numCols);
	Aux.numCols = Matriz->numCols;
	Aux.numRows = Matriz->numRows;

	arm_mat_mult_f32(&Aux, &Matriz_Rotacion, Matriz);

}
void Rotacion_Y(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes){
	float32_t Seno = 0;
	float32_t Cos = 0;

	float32_t Aux_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};

	float32_t Matriz_Rotacion_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Matriz_Rotacion = {3, 3, Matriz_Rotacion_Matriz};

	if(Radianes){ Giro = Giro * 180.0/PI; }
	arm_sin_cos_f32(Giro, &Seno, &Cos);
	Matriz_Rotacion.pData[0] = Cos;
	Matriz_Rotacion.pData[2] = Seno;
	Matriz_Rotacion.pData[4] = 1;
	Matriz_Rotacion.pData[6] = -Seno;
	Matriz_Rotacion.pData[8] = Cos;

	arm_copy_f32(Matriz->pData, Aux.pData, 3*Matriz->numCols);
	Aux.numCols = Matriz->numCols;
	Aux.numRows = Matriz->numRows;

	arm_mat_mult_f32(&Aux, &Matriz_Rotacion, Matriz);
}
void Rotacion_Z(arm_matrix_instance_f32 *Matriz, float32_t Giro, bool Radianes){
	float32_t Seno = 0;
	float32_t Cos = 0;

	float32_t Aux_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};

	float32_t Matriz_Rotacion_Matriz[9] = {0,0,0,0,0,0,0,0,0};
	arm_matrix_instance_f32 Matriz_Rotacion = {3, 3, Matriz_Rotacion_Matriz};

	if(Radianes){ Giro = Giro * 180.0/PI; }
	arm_sin_cos_f32(Giro, &Seno, &Cos);
	Matriz_Rotacion.pData[0] = Cos;
	Matriz_Rotacion.pData[1] = -Seno;
	Matriz_Rotacion.pData[3] = Seno;
	Matriz_Rotacion.pData[4] = Cos;
	Matriz_Rotacion.pData[8] = 1;

	arm_copy_f32(Matriz->pData, Aux.pData, 3*Matriz->numCols);
	Aux.numCols = Matriz->numCols;
	Aux.numRows = Matriz->numRows;

	arm_mat_mult_f32(&Aux, &Matriz_Rotacion, Matriz);
}
/*
void Rotacion_ZYZp(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes){
	float32_t Aux_Matriz[9] = {1,0,0,0,1,0,0,0,1};
	arm_matrix_instance_f32 Aux = {Matriz->numRows, Matriz->numCols, Aux_Matriz};
	float32_t Aux_Matriz2[9] = {1,0,0,0,1,0,0,0,1};
	arm_matrix_instance_f32 Aux2 = {Matriz->numRows, Matriz->numCols, Aux_Matriz2};
	float32_t Aux_Matriz3[9] = {1,0,0,0,1,0,0,0,1};
	arm_matrix_instance_f32 Aux3 = {Matriz->numRows, Matriz->numCols, Aux_Matriz3};

	Rotacion_Z(&Aux, Giro[0], Radianes);
	Rotacion_Y(&Aux2, Giro[1], Radianes);
	arm_mat_mult_f32(&Aux, &Aux2, &Aux3);
	Rotacion_Z(&Aux, Giro[2], Radianes);
	arm_mat_mult_f32(&Aux3, &Aux, Matriz);
}
*/
void Rotacion_ZXY(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes){

	Matriz->pData[0] = 1;
	Matriz->pData[1] = 0;
	Matriz->pData[2] = 0;
	Matriz->pData[3] = 0;
	Matriz->pData[4] = 1;
	Matriz->pData[5] = 0;
	Matriz->pData[6] = 0;
	Matriz->pData[7] = 0;
	Matriz->pData[8] = 1;

	Rotacion_Z(Matriz, Giro[0], Radianes);
	Rotacion_X(Matriz, Giro[1], Radianes);
	Rotacion_Y(Matriz, Giro[2], Radianes);
}

void Rotacion_ZYZp(arm_matrix_instance_f32 *Matriz, float32_t Giro[3], bool Radianes){

	Matriz->pData[0] = 1;
	Matriz->pData[1] = 0;
	Matriz->pData[2] = 0;
	Matriz->pData[3] = 0;
	Matriz->pData[4] = 1;
	Matriz->pData[5] = 0;
	Matriz->pData[6] = 0;
	Matriz->pData[7] = 0;
	Matriz->pData[8] = 1;

	Rotacion_Z(Matriz, Giro[0], Radianes);
	Rotacion_Y(Matriz, Giro[1], Radianes);
	Rotacion_Z(Matriz, Giro[2], Radianes);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//....Leer_IMU...//
void Lectura_Datos_IMU(UArg arg0, UArg arg1){
	tpLecturas_IMU Lecturas_IMU = {0, 0, 0, 0, 0, 0, 0};
	tpLecturas_Giroscopo Lecturas_Giroscopo = {0, 0, 0, 0};
	tpLecturas_Brujula Lecturas_Brujula;

	float32_t Lecturas_matriz[3];
	float32_t Lecturas_Corregidas_matriz[3];

	arm_matrix_instance_f32 Lecturas = {3, 1, Lecturas_matriz};
	arm_matrix_instance_f32 Lecturas_Corregidas = {3, 1, Lecturas_Corregidas_matriz};

	float32_t aux;
	uint16_t nMuestras = 0;

	tpTelemetria_IMU Telemetria_IMU = {
			.Inicio = START_FRAME,
			.Final = FINAL_FRAME
	};

#ifdef Filtrado_Vel_IMU
	float32_t Estado_filtro_Vel_X[4*num_etapas_Filtro_Vel];
	float32_t Estado_filtro_Vel_Y[4*num_etapas_Filtro_Vel];
	float32_t Estado_filtro_Vel_Z[4*num_etapas_Filtro_Vel];
#endif

	float32_t Estado_filtro_Acel_X[4*num_etapas_Filtro_Acel];
	float32_t Estado_filtro_Acel_Y[4*num_etapas_Filtro_Acel];
	float32_t Estado_filtro_Acel_Z[4*num_etapas_Filtro_Acel];

#ifdef Filtrado_Vel_IMU
	arm_biquad_casd_df1_inst_f32 Filtro_Vel_X = {num_etapas_Filtro_Vel, Estado_filtro_Vel_X, (float32_t *)Coeficientes_Filtro_Vel_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Vel_Y = {num_etapas_Filtro_Vel, Estado_filtro_Vel_Y, (float32_t *)Coeficientes_Filtro_Vel_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Vel_Z = {num_etapas_Filtro_Vel, Estado_filtro_Vel_Z, (float32_t *)Coeficientes_Filtro_Vel_Valores};
#endif

	arm_biquad_casd_df1_inst_f32 Filtro_Acel_X = {num_etapas_Filtro_Acel, Estado_filtro_Acel_X, (float32_t *)Coeficientes_Filtro_Acel_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Acel_Y = {num_etapas_Filtro_Acel, Estado_filtro_Acel_Y, (float32_t *)Coeficientes_Filtro_Acel_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Acel_Z = {num_etapas_Filtro_Acel, Estado_filtro_Acel_Z, (float32_t *)Coeficientes_Filtro_Acel_Valores};


	Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);

//Inicializamos el filtro
#ifdef Filtrado_Vel_IMU
	arm_fill_f32(0.0, Estado_filtro_Vel_X, 4*num_etapas_Filtro_Vel);
	arm_fill_f32(0.0, Estado_filtro_Vel_Y, 4*num_etapas_Filtro_Vel);
	arm_fill_f32(0.0, Estado_filtro_Vel_Z, 4*num_etapas_Filtro_Vel);
#endif

	arm_fill_f32(0.0, Estado_filtro_Acel_X, 4*num_etapas_Filtro_Acel);
	arm_fill_f32(0.0, Estado_filtro_Acel_Y, 4*num_etapas_Filtro_Acel);
	arm_fill_f32(0.0, Estado_filtro_Acel_Z, 4*num_etapas_Filtro_Acel);

#ifdef GYRO_L3G4200
	Iniciar_Giroscopo_L3G4200(I2C_PRINCIPAL, Giroscopo_L3G4200);
#endif

	//Iniciar_Giroscopio_ITG3200(I2C_PRINCIPAL, Dir_Gir_0, 8000, 0);

#ifdef IMU_MPU6050
	Iniciar_IMU_MPU6050(I2C_PRINCIPAL, IMU6050);
#endif

#ifdef COMPASS_HMC5883L
	Iniciar_Brujula_HMC5883L(I2C_PRINCIPAL, Brujula_HMC5883L);
#endif

//........Calibracion....media.........//
    GPIO_write(QUAD_BOARD_LED_RED, 0);
    GPIO_write(QUAD_BOARD_LED_GREEN, 1);
    GPIO_write(QUAD_BOARD_LED_BLUE, 1);

    Datos = Memory_alloc(NULL, Numero_Muetras_calibracion_IMU*4, 0, &eb);

#ifdef IMU_MPU6050

   	if(ModoCalibracionIMU == CALIBRACION_ACELEROMETRO || ModoCalibracionIMU == CALIBRACION_COMPLETA_IMU){
   		for(nMuestras=0; nMuestras<Numero_Muetras_calibracion_IMU; nMuestras++){
   			Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);
   			Leer_IMU_MPU6050(I2C_PRINCIPAL, IMU6050, &Lecturas_IMU);
   			*((float32_t*)Datos + nMuestras) = Lecturas_IMU.Valor.x_acel;
   			Task_sleep(PERIODO_Leer_IMU);
   		}
   		arm_mean_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Acel_x);
   		Calibracion_IMU.Media_Acel_x = (int16_t)Calibracion_IMU.Des_est_Acel_x;
   		arm_std_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Acel_x);

   		for(nMuestras=0; nMuestras<Numero_Muetras_calibracion_IMU; nMuestras++){
   			Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);
   			Leer_IMU_MPU6050(I2C_PRINCIPAL, IMU6050, &Lecturas_IMU);
   			*((float32_t*)Datos + nMuestras) = Lecturas_IMU.Valor.y_acel;
   			Task_sleep(PERIODO_Leer_IMU);
    	}
    	arm_mean_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Acel_y);
    	Calibracion_IMU.Media_Acel_y = (int16_t)Calibracion_IMU.Des_est_Acel_y;
    	arm_std_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Acel_y);

    	for(nMuestras=0; nMuestras<Numero_Muetras_calibracion_IMU; nMuestras++){
    		Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);
   			Leer_IMU_MPU6050(I2C_PRINCIPAL, IMU6050, &Lecturas_IMU);
    		*((float32_t*)Datos + nMuestras) = Lecturas_IMU.Valor.z_acel;
    		Task_sleep(PERIODO_Leer_IMU);
    	}
    	arm_mean_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Acel_z);
    	Calibracion_IMU.Media_Acel_z = (int16_t)Calibracion_IMU.Des_est_Acel_z;
    	Gravedad  = sqrt(pow(Calibracion_IMU.Media_Acel_x,2) +	pow(Calibracion_IMU.Media_Acel_y,2) + pow(Calibracion_IMU.Media_Acel_z,2));
    	Calibracion_IMU.Media_Acel_z -= (int16_t)sqrt(pow(Calibracion_IMU.Media_Acel_x,2) + pow(Calibracion_IMU.Media_Acel_y,2) + pow(Calibracion_IMU.Media_Acel_z,2));
    	arm_std_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Acel_z);
    }

	if(ModoCalibracionIMU == CALIBRACION_GIROSCOPO || ModoCalibracionIMU == CALIBRACION_COMPLETA_IMU){
		for(nMuestras=0; nMuestras<Numero_Muetras_calibracion_IMU; nMuestras++){
			Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);
   			Leer_IMU_MPU6050(I2C_PRINCIPAL, IMU6050, &Lecturas_IMU);
			*((float32_t*)Datos + nMuestras) = Lecturas_IMU.Valor.x_vel;
			Task_sleep(PERIODO_Leer_IMU);
		}
		arm_mean_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Vel_x);
		Calibracion_IMU.Media_Vel_x = (int16_t)Calibracion_IMU.Des_est_Vel_x;
		arm_std_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Vel_x);

		for(nMuestras=0; nMuestras<Numero_Muetras_calibracion_IMU; nMuestras++){
			Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);
   			Leer_IMU_MPU6050(I2C_PRINCIPAL, IMU6050, &Lecturas_IMU);
			*((float32_t*)Datos + nMuestras) = Lecturas_IMU.Valor.y_vel;
			Task_sleep(PERIODO_Leer_IMU);
		}
		arm_mean_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Vel_y);
		Calibracion_IMU.Media_Vel_y = (int16_t)Calibracion_IMU.Des_est_Vel_y;
		arm_std_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Vel_y);

		for(nMuestras=0; nMuestras<Numero_Muetras_calibracion_IMU; nMuestras++){
			Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);
   			Leer_IMU_MPU6050(I2C_PRINCIPAL, IMU6050, &Lecturas_IMU);
			*((float32_t*)Datos + nMuestras) = Lecturas_IMU.Valor.z_vel;
			Task_sleep(PERIODO_Leer_IMU);
 		}
		arm_mean_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Vel_z);
		Calibracion_IMU.Media_Vel_z = (int16_t)Calibracion_IMU.Des_est_Vel_z;
		arm_std_f32((float32_t*)Datos, Numero_Muetras_calibracion_IMU, &Calibracion_IMU.Des_est_Vel_z);
	}

#endif

	Memory_free(NULL, Datos, Numero_Muetras_calibracion_IMU*4);
	Escribir_servidor_Lecturas_IMU(&Lecturas_IMU);

	PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
	PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
	PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
	PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);

//ARRANCA LA TAREA DE AHRS
	Semaphore_post(SEMAPHORE_Calculo_AHRS);

    GPIO_write(QUAD_BOARD_LED_RED, 1);
    GPIO_write(QUAD_BOARD_LED_GREEN, 1);
    GPIO_write(QUAD_BOARD_LED_BLUE, 0);

	while(1){
		Semaphore_pend(SEMAPHORE_Leer_IMU, BIOS_WAIT_FOREVER);

		//..IMU................//
#ifdef IMU_MPU6050
		Leer_IMU_MPU6050(I2C_PRINCIPAL, IMU6050, &Lecturas_IMU);

		//..Offset..//
		Lecturas_IMU.Valor.x_vel -= Calibracion_IMU.Media_Vel_x;
		Lecturas_IMU.Valor.y_vel -= Calibracion_IMU.Media_Vel_y;
		Lecturas_IMU.Valor.z_vel -= Calibracion_IMU.Media_Vel_z;

		Mailbox_pend(Buzon_Lecturas_IMU, NULL, BIOS_NO_WAIT);
		Mailbox_post(Buzon_Lecturas_IMU, &Lecturas_IMU, BIOS_NO_WAIT);

		//..Filtrado...Lectura...IMU..........//
		aux = (float32_t)Lecturas_IMU.Valor.x_acel;
		arm_biquad_cascade_df1_f32(&Filtro_Acel_X, &aux, &Lecturas_matriz[0], 1);
		aux = (float32_t)Lecturas_IMU.Valor.y_acel;
		arm_biquad_cascade_df1_f32(&Filtro_Acel_Y, &aux, &Lecturas_matriz[1], 1);
		aux = (float32_t)Lecturas_IMU.Valor.z_acel;
		arm_biquad_cascade_df1_f32(&Filtro_Acel_Z, &aux, &Lecturas_matriz[2], 1);

		arm_mat_mult_f32(&Calibracion_IMU.Correccion_Alineamiento, &Lecturas, &Lecturas_Corregidas);

		Lecturas_IMU.Valor.x_acel = (int16_t)Lecturas_Corregidas_matriz[0];
		Lecturas_IMU.Valor.y_acel = (int16_t)Lecturas_Corregidas_matriz[1];
		Lecturas_IMU.Valor.z_acel = (int16_t)Lecturas_Corregidas_matriz[2];

	#ifdef Filtrado_Vel_IMU
		aux = (float32_t)Lecturas_IMU.Valor.x_vel;
		arm_biquad_cascade_df1_f32(&Filtro_Vel_X, &aux, &Lecturas_matriz[0], 1);
		aux = (float32_t)Lecturas_IMU.Valor.y_vel;
		arm_biquad_cascade_df1_f32(&Filtro_Vel_Y, &aux, &Lecturas_matriz[1], 1);
		aux = (float32_t)Lecturas_IMU.Valor.z_vel;
		arm_biquad_cascade_df1_f32(&Filtro_Vel_Z, &aux, &Lecturas_matriz[2], 1);

		arm_mat_mult_f32(&Calibracion_IMU.Correccion_Alineamiento, &Lecturas, &Lecturas_Corregidas);

		Lecturas_IMU.Valor.x_vel = (int16_t)Lecturas_Corregidas_matriz[0];
		Lecturas_IMU.Valor.y_vel = (int16_t)Lecturas_Corregidas_matriz[1];
		Lecturas_IMU.Valor.z_vel = (int16_t)Lecturas_Corregidas_matriz[2];

	#endif
#endif
		Escribir_servidor_Lecturas_IMU(&Lecturas_IMU);

#ifdef GYRO_L3G4200
		Leer_Giroscopo_L3G4200(I2C_PRINCIPAL, Giroscopo_L3G4200, &Lecturas_Giroscopo);
#endif

#ifdef ROT_GYRO
		Lecturas_Giroscopo_Rotadas.Valor.x_vel = -Lecturas_Giroscopo.Valor.y_vel * Rot_sin_giro + Lecturas_Giroscopo.Valor.x_vel * Rot_cos_giro;
		Lecturas_Giroscopo_Rotadas.Valor.y_vel =  Lecturas_Giroscopo.Valor.x_vel * Rot_sin_giro + Lecturas_Giroscopo.Valor.y_vel * Rot_cos_giro;
		Lecturas_Giroscopo_Rotadas.Valor.z_vel =  Lecturas_Giroscopo.Valor.z_vel;
#endif

		Escribir_servidor_Lecturas_Giroscopo(&Lecturas_Giroscopo);

#ifdef COMPASS_HMC5883L
		Leer_Brujula_HMC5883L(I2C_PRINCIPAL, Brujula_HMC5883L, &Lecturas_Brujula);
		//Mailbox_post(Buzon_Calibracion_Brujula, &Lecturas_Brujula, BIOS_NO_WAIT);
		Escribir_servidor_Lecturas_Brujula(&Lecturas_Brujula);
#endif

	switch(ModoTelemetria){

		case(TELEMETRIA_IMU):
				Telemetria_IMU.Acel[0] = Lecturas_IMU.Valor.x_acel;
				Telemetria_IMU.Acel[1] = Lecturas_IMU.Valor.y_acel;
				Telemetria_IMU.Acel[2] = Lecturas_IMU.Valor.z_acel;

				switch(ModeloGiroscopo){
					case GYRO_IMU6050:
						Telemetria_IMU.Gyro[0] = Lecturas_IMU.Valor.x_vel;
						Telemetria_IMU.Gyro[1] = Lecturas_IMU.Valor.y_vel;
						Telemetria_IMU.Gyro[2] = Lecturas_IMU.Valor.z_vel;
					break;
//					case GYRO_L3G4200:
//					break;
					case GYRO_ITG_3200:
					break;
				}
				UART_write(UART_BT_TELEMETRIA, &Telemetria_IMU, sizeof(Telemetria_IMU));
		break;
		}
	}

}
void CLK_Lectura_Datos_IMU(){
	Semaphore_post(SEMAPHORE_Leer_IMU);
}

//....Calculo_AHRS...........//
void Calculo_AHRS(UArg arg0, UArg arg1){
	tpLecturas_IMU Lecturas_IMU_Control;
	tpLecturas_Giroscopo Lecturas_Giroscopo_Control;
	tpLecturas_Brujula Lecturas_Brujula_control;

	static tpAHRS AHRS = {
	    .DCM_matriz = {1, 0, 0, 0, 1, 0, 0, 0, 1},
		.DCM = {3, 3, (float32_t *)AHRS.DCM_matriz},
		.Kp_Roll_Pitch = Kp_ROLLPITCH,
		.Ki_Roll_Pitch = Ki_ROLLPITCH,
		.Kp_Yaw = Kp_YAW,
		.Ki_Yaw = Ki_YAW,
		.Periodo_Muestreo = PERIODO_Calculo_AHRS / 1000.0
	};


	Semaphore_pend(SEMAPHORE_Calculo_AHRS, BIOS_WAIT_FOREVER);

	Clock_start(CLOCK_Calculo_AHRS);
	Semaphore_pend(SEMAPHORE_Calculo_AHRS, BIOS_WAIT_FOREVER);

	ResetDCM();
	Leer_servidor_DCM((float32_t*)AHRS.DCM_matriz);
	Leer_servidor_RPY(&AHRS.Roll, &AHRS.Pitch, &AHRS.Yaw);

	Semaphore_post(SEMAPHORE_Coordinador);
	while(1){
		Semaphore_pend(SEMAPHORE_Calculo_AHRS, BIOS_WAIT_FOREVER);

		Leer_servidor_Lecturas_IMU(&Lecturas_IMU_Control);
		Leer_servidor_Lecturas_Giroscopo(&Lecturas_Giroscopo_Control);
		Leer_servidor_Lecturas_Brujula(&Lecturas_Brujula_control);

		//.................AHRS...................//
				//..ACEL..//
#ifdef IMU_MPU6050

		AHRS.Vector_Aceleracion_lineal[0] = Lecturas_IMU_Control.Valor.x_acel;
		AHRS.Vector_Aceleracion_lineal[1] = Lecturas_IMU_Control.Valor.y_acel;
		AHRS.Vector_Aceleracion_lineal[2] = Lecturas_IMU_Control.Valor.z_acel;
#endif

				//..GYRO..//
#ifdef IMU_MPU6050
		AHRS.Vector_Velocidad_Angular[0] = CONVERTIR_A_RADIANES((float32_t)Lecturas_IMU_Control.Valor.x_vel / IMU6050.Sensibilidad_Giroscopo);
		AHRS.Vector_Velocidad_Angular[1] = CONVERTIR_A_RADIANES((float32_t)Lecturas_IMU_Control.Valor.y_vel / IMU6050.Sensibilidad_Giroscopo);
		AHRS.Vector_Velocidad_Angular[2] = CONVERTIR_A_RADIANES((float32_t)Lecturas_IMU_Control.Valor.z_vel / IMU6050.Sensibilidad_Giroscopo);
#endif

#ifdef GYRO_L3G4200
		AHRS.Vector_Velocidad_Angular[0] = CONVERTIR_A_RADIANES((float32_t)Lecturas_Giroscopo_Control.Valor.x_vel / Giroscopo_L3G4200.Sensibilidad_Giroscopo);
		AHRS.Vector_Velocidad_Angular[1] = CONVERTIR_A_RADIANES((float32_t)Lecturas_Giroscopo_Control.Valor.y_vel / Giroscopo_L3G4200.Sensibilidad_Giroscopo);
		AHRS.Vector_Velocidad_Angular[2] = CONVERTIR_A_RADIANES((float32_t)Lecturas_Giroscopo_Control.Valor.z_vel / Giroscopo_L3G4200.Sensibilidad_Giroscopo);
#endif

				//...BRUJULA...//
#ifdef COMPASS_HMC5883L
		AHRS.Vector_Magnetico[0] = Lecturas_Brujula_control.Valor.Magnetismo_x;
		AHRS.Vector_Magnetico[1] = Lecturas_Brujula_control.Valor.Magnetismo_y;
		AHRS.Vector_Magnetico[2] = Lecturas_Brujula_control.Valor.Magnetismo_z;
#endif
		Algortimo_DCM(&AHRS);
		Escribir_servidor_DCM((float32_t*)AHRS.DCM_matriz);
		Escribir_servidor_RPY(&AHRS.Roll, &AHRS.Pitch, &AHRS.Yaw);
	}
}

void CLK_Calculo_AHRS(){
	Semaphore_post(SEMAPHORE_Calculo_AHRS);
}
//....Coordinador......//

void Coordinador(UArg arg0, UArg arg1){
//..Ref.............................

	tpOrden Orden = DATO_ANTERIOR;
	uint16_t i = 0;

	tpLectura_Radio Lectura_Radio;
	I2C_Transaction I2C_Transmision;
	bool estado_Transmision = false;
	I2C_Transmision.slaveAddress = Dir_AUX;
	I2C_Transmision.writeBuf = NULL;
	I2C_Transmision.writeCount = 0;
	I2C_Transmision.readBuf = &Lectura_Radio;
	I2C_Transmision.readCount = sizeof(Lectura_Radio)-1;

	static uint8_t Temporizador_Ticks = ticks_arranque_vuelo; //3seg

	float32_t Canal[8];
	float32_t Referencia[4] = {0, 0, 0, 0};
	float32_t Angulos[3] = {0, 0, 0};


	Semaphore_pend(SEMAPHORE_Coordinador, BIOS_WAIT_FOREVER);

	WatchDog_0 = Watchdog_open(QUAD_BOARD_WATCHDOG0 , &PARAMS_WatchDog_0);
	Watchdog_clear(WatchDog_0);

	Clock_start(CLOCK_Coordinador);

	estado_Transmision = I2C_transfer(I2C_AUX, &I2C_Transmision);


	while(!(estado_Transmision && Lectura_Radio.Canal_PWM[2] < 1100)){
		Semaphore_pend(SEMAPHORE_Coordinador, BIOS_WAIT_FOREVER);
		estado_Transmision = I2C_transfer(I2C_AUX, &I2C_Transmision);
  		if(Lectura_Radio.Error_conexion != 0){
			Watchdog_clear(WatchDog_0);
		}
	}

	while(1){
		Semaphore_pend(SEMAPHORE_Coordinador, BIOS_WAIT_FOREVER);

		estado_Transmision = I2C_transfer(I2C_AUX, &I2C_Transmision);
//		Lectura_Radio.Canal_PWM[7] = Lectura_Radio.Canal_PWM[6];////////////////////////////////////////////////////////////////////


		if(estado_Transmision && Lectura_Radio.Error_conexion != 0){
			Watchdog_clear(WatchDog_0);

			Canal[0] = ( ( Lectura_Radio.Canal_PWM[0] - Calibracion_Receptor[0].Rango_Entrada[0] ) * ( Calibracion_Receptor[0].Rango_Salida[1] - Calibracion_Receptor[0].Rango_Salida[0] ) / ( Calibracion_Receptor[0].Rango_Entrada[1] - Calibracion_Receptor[0].Rango_Entrada[0] ) + Calibracion_Receptor[0].Rango_Salida[0] );
			Canal[1] = ( ( Lectura_Radio.Canal_PWM[1] - Calibracion_Receptor[1].Rango_Entrada[0] ) * ( Calibracion_Receptor[1].Rango_Salida[1] - Calibracion_Receptor[1].Rango_Salida[0] ) / ( Calibracion_Receptor[1].Rango_Entrada[1] - Calibracion_Receptor[1].Rango_Entrada[0] ) + Calibracion_Receptor[1].Rango_Salida[0] );
			Canal[2] = ( ( Lectura_Radio.Canal_PWM[2] - Calibracion_Receptor[2].Rango_Entrada[0] ) * ( Calibracion_Receptor[2].Rango_Salida[1] - Calibracion_Receptor[2].Rango_Salida[0] ) / ( Calibracion_Receptor[2].Rango_Entrada[1] - Calibracion_Receptor[2].Rango_Entrada[0] ) + Calibracion_Receptor[2].Rango_Salida[0] );
			Canal[3] = ( ( Lectura_Radio.Canal_PWM[3] - Calibracion_Receptor[3].Rango_Entrada[0] ) * ( Calibracion_Receptor[3].Rango_Salida[1] - Calibracion_Receptor[3].Rango_Salida[0] ) / ( Calibracion_Receptor[3].Rango_Entrada[1] - Calibracion_Receptor[3].Rango_Entrada[0] ) + Calibracion_Receptor[3].Rango_Salida[0] );
			Canal[4] = ( ( Lectura_Radio.Canal_PWM[4] - Calibracion_Receptor[4].Rango_Entrada[0] ) * ( Calibracion_Receptor[4].Rango_Salida[1] - Calibracion_Receptor[4].Rango_Salida[0] ) / ( Calibracion_Receptor[4].Rango_Entrada[1] - Calibracion_Receptor[4].Rango_Entrada[0] ) + Calibracion_Receptor[4].Rango_Salida[0] );
			Canal[5] = ( ( Lectura_Radio.Canal_PWM[5] - Calibracion_Receptor[5].Rango_Entrada[0] ) * ( Calibracion_Receptor[5].Rango_Salida[1] - Calibracion_Receptor[5].Rango_Salida[0] ) / ( Calibracion_Receptor[5].Rango_Entrada[1] - Calibracion_Receptor[5].Rango_Entrada[0] ) + Calibracion_Receptor[5].Rango_Salida[0] );
			Canal[6] = ( ( Lectura_Radio.Canal_PWM[6] - Calibracion_Receptor[6].Rango_Entrada[0] ) * ( Calibracion_Receptor[6].Rango_Salida[1] - Calibracion_Receptor[6].Rango_Salida[0] ) / ( Calibracion_Receptor[6].Rango_Entrada[1] - Calibracion_Receptor[6].Rango_Entrada[0] ) + Calibracion_Receptor[6].Rango_Salida[0] );
			Canal[7] = ( ( Lectura_Radio.Canal_PWM[7] - Calibracion_Receptor[7].Rango_Entrada[0] ) * ( Calibracion_Receptor[7].Rango_Salida[1] - Calibracion_Receptor[7].Rango_Salida[0] ) / ( Calibracion_Receptor[7].Rango_Entrada[1] - Calibracion_Receptor[7].Rango_Entrada[0] ) + Calibracion_Receptor[7].Rango_Salida[0] );

			//......PULSADORES...................................//
			if(!GPIO_read(QUAD_BOARD_SW2)){
				while(!GPIO_read(QUAD_BOARD_SW2));

				switch(Estado_Sistema){
					case ESPERA:

					break;
					case DEBUG:
						Estado_Sistema = CALIBRACION;

					    GPIO_write(QUAD_BOARD_LED_RED, 0);
						GPIO_write(QUAD_BOARD_LED_GREEN, 1);
						GPIO_write(QUAD_BOARD_LED_BLUE, 1);

						Calibracion_Receptor[0].Rango_Entrada[0] = 1500; Calibracion_Receptor[0].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[1].Rango_Entrada[0] = 1500; Calibracion_Receptor[1].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[2].Rango_Entrada[0] = 1500; Calibracion_Receptor[2].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[3].Rango_Entrada[0] = 1500; Calibracion_Receptor[3].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[4].Rango_Entrada[0] = 1500; Calibracion_Receptor[4].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[5].Rango_Entrada[0] = 1500; Calibracion_Receptor[5].Rango_Entrada[1] = 1500;
						Calibracion_Receptor[6].Rango_Entrada[0] = 1500; Calibracion_Receptor[6].Rango_Entrada[1] = 1500;
//						Calibracion_Receptor[7].Rango_Entrada[0] = 1500; Calibracion_Receptor[7].Rango_Entrada[1] = 1500;

					break;
					case CALIBRACION:
						Estado_Sistema = ESPERA;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 1);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

					break;
				}

			//...........STICK_IZQUIERDA NEGATIVO.............................................................//
			}else if((Canal[0] <= -0.9) && (Canal[2] < 0.05)){
				if (Temporizador_Ticks-- == 0){

					switch(Estado_Sistema){
						case DEBUG:
							//resetear variables
						break;
						case ESPERA:
							Estado_Sistema = DEBUG;

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 1);
						break;
						case VUELO:
							Estado_Sistema = ESPERA;

							Clock_stop(CLOCK_Control);
							Semaphore_reset(SEMAPHORE_Control, 0);

							PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
							PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
							PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
							PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);
						break;
					}
				}
			//...........STICK_IZQUIERDA POSITIVO.............................................................//
			}else if((Canal[0] >= 0.9) && (Canal[2]) < 0.05) {
				if (--Temporizador_Ticks == 0){

					switch(Estado_Sistema){
						case DEBUG:
							Estado_Sistema = ESPERA;

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);

							Leer_servidor_RPY(NULL, NULL, &Posicion_inicial);
							Posicion_inicial = CONVERTIR_A_GRADOS(Posicion_inicial);

						break;
						case ESPERA:
							Estado_Sistema = VUELO;

							GPIO_write(QUAD_BOARD_LED_RED, 0);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);

							Clock_start(CLOCK_Control);
							Resetear_servidor_Perturbaciones_Estimadas();

						break;
					}
				}
			//...........STICK DERECHA NEGATIVO.............................................................//
			}else if((Canal[3] <= -0.9) && (Canal[2]) < 0.05) {
				if (--Temporizador_Ticks == 0){

					switch(Estado_Sistema){
						case DEBUG:
						break;
						case ESPERA:
							//RESET

							Leer_servidor_RPY(&Angulos[1], &Angulos[0], &Angulos[2]);

							//Iniciamos la matriz de correccion
							Calibracion_IMU.Giro[1] -= CONVERTIR_A_GRADOS(Angulos[0]);
							Calibracion_IMU.Giro[2] -= CONVERTIR_A_GRADOS(Angulos[1]);

						    Rotacion_ZXY(&Calibracion_IMU.Correccion_Alineamiento, Calibracion_IMU.Giro, false);

							GPIO_write(QUAD_BOARD_LED_RED, 0);
							GPIO_write(QUAD_BOARD_LED_GREEN, 0);
							GPIO_write(QUAD_BOARD_LED_BLUE, 1);

							Task_sleep(250);

							ResetDCM();

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 1);
							GPIO_write(QUAD_BOARD_LED_BLUE, 0);
						break;
					}
					Temporizador_Ticks = ticks_arranque_vuelo;
				}
			//...........STICK DERECHA POSITIVO.............................................................//
			}else if((Canal[3] >= 0.9) && (Canal[2]) < 0.05) {
				if (--Temporizador_Ticks == 0){

					switch(Estado_Sistema){
						case DEBUG:
							Estado_Sistema = IDENTIFICACION;

							GPIO_write(QUAD_BOARD_LED_RED, 1);
							GPIO_write(QUAD_BOARD_LED_GREEN, 0);
							GPIO_write(QUAD_BOARD_LED_BLUE, 1);

							i = 0;

							do{
								Orden = START;
								UART_write(UART_BT_TELEMETRIA, &Orden, 1);
								Orden = IDENTIFICAR;
								UART_write(UART_BT_TELEMETRIA, &Orden, 1);
								Orden = FINAL;
								UART_write(UART_BT_TELEMETRIA, &Orden, 1);

								UART_read(UART_BT_TELEMETRIA, &Orden, 1);

								Watchdog_clear(WatchDog_0);
							}while(Orden != IDENTIFICAR && ++i < Num_intentos_conexion_Identificacion);

							if(Orden == IDENTIFICAR){
								UART_read(UART_BT_TELEMETRIA, &nDatos_Identifiacion, 4);
								UART_read(UART_BT_TELEMETRIA, &PuntoTrabajo_motor ,4);

								Datos = Memory_alloc(NULL, nDatos_Identifiacion*2, 0, &eb);

								for(i=0; i<nDatos_Identifiacion; i++){
									UART_read(UART_BT_TELEMETRIA, (int16_t *)Datos + i, 2);
									Watchdog_clear(WatchDog_0);
								}

								UART_read(UART_BT_TELEMETRIA, &Orden, 1);

								if(Orden == FINAL){

									nDatos_leidos = 0;
									Clock_start(CLOCK_Identificacion);
								}else{
									Estado_Sistema = ESPERA;

									GPIO_write(QUAD_BOARD_LED_RED, 1);
									GPIO_write(QUAD_BOARD_LED_GREEN, 1);
									GPIO_write(QUAD_BOARD_LED_BLUE, 0);
								}
							}else{

								Estado_Sistema = ESPERA;

								GPIO_write(QUAD_BOARD_LED_RED, 1);
								GPIO_write(QUAD_BOARD_LED_GREEN, 1);
								GPIO_write(QUAD_BOARD_LED_BLUE, 0);
							}
					}
				}
			}else{
				Temporizador_Ticks = ticks_arranque_vuelo;
			}

			//.............Accion...............................//
			switch(Estado_Sistema){
				case VUELO:
					if(Canal[7] < -83.2){	//...ERROR....//
		//				Estado_Sistema_Anterior = Estado_Sistema;
						Estado_Sistema = ERROR;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 0);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

						//Stop a todos los clocks y tareas
						Clock_stop(CLOCK_Control);
						Semaphore_reset(SEMAPHORE_Control, 0);
						Clock_stop(CLOCK_Identificacion);
						Semaphore_reset(SEMAPHORE_Identificacion, 0);

						//Parada_motores
						PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);
					}
					else if((Canal[7] > -83.2) && (Canal[7] <= -50.0)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_1; }
					else if((Canal[7] > -50.0) && (Canal[7] <= -16.6)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = INTEGRAR_PERTURBACIONES_ESTIMADAS; InfoTelemetria = TELE_2;}
					else if((Canal[7] > -16.6) && (Canal[7] <=  16.6)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = INTEGRAR_PERTURBACIONES; InfoTelemetria = TELE_3;}
					else if((Canal[7] >  16.6) && (Canal[7] <=  50.0)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_0;}
					else if((Canal[7] >  50.0) && (Canal[7] <=  83.2)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_0;}
					else if((Canal[7] >  83.2))						  { Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_0;}

					switch(Modo_Control){
						case ANGULOS_3:
							Referencia[0] = Canal[1] * Angulo_Maximo;
							Referencia[1] = Canal[3] * Angulo_Maximo;
							Referencia[2] = Canal[0] * Angulo_Maximo;
							Referencia[3] = Canal[2] * Valor_Empuje_Maximo;
						break;
						case ANGULOS_4:
							Referencia[0] = Canal[1] * Angulo_Maximo;
							Referencia[1] = Canal[3] * Angulo_Maximo;
							Referencia[2] = Canal[0] * Angulo_Maximo;
							Referencia[3] = Canal[2] * Valor_Fuerza_Maximo;
						break;
						case EMPUJE:
							Referencia[0] = Canal[1] * Angulo_Maximo;
							Referencia[1] = Canal[3] * Angulo_Maximo;
							Referencia[2] = Canal[0] * Angulo_Maximo;
							Referencia[3] = Canal[2] * Valor_Empuje_Maximo;
						break;
					}
					Escribir_servidor_Referencia(Referencia, NULL);
				break;
				case CALIBRACION:

					Calibracion_Receptor[0].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[0] < Calibracion_Receptor[0].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[0] :  Calibracion_Receptor[0].Rango_Entrada[0];
					Calibracion_Receptor[0].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[0] > Calibracion_Receptor[0].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[0] :  Calibracion_Receptor[0].Rango_Entrada[1];
					Calibracion_Receptor[1].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[1] < Calibracion_Receptor[1].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[1] :  Calibracion_Receptor[1].Rango_Entrada[0];
					Calibracion_Receptor[1].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[1] > Calibracion_Receptor[1].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[1] :  Calibracion_Receptor[1].Rango_Entrada[1];
					Calibracion_Receptor[2].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[2] < Calibracion_Receptor[2].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[2] :  Calibracion_Receptor[2].Rango_Entrada[0];
					Calibracion_Receptor[2].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[2] > Calibracion_Receptor[2].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[2] :  Calibracion_Receptor[2].Rango_Entrada[1];
					Calibracion_Receptor[3].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[3] < Calibracion_Receptor[3].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[3] :  Calibracion_Receptor[3].Rango_Entrada[0];
					Calibracion_Receptor[3].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[3] > Calibracion_Receptor[3].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[3] :  Calibracion_Receptor[3].Rango_Entrada[1];
					Calibracion_Receptor[4].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[4] < Calibracion_Receptor[4].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[4] :  Calibracion_Receptor[4].Rango_Entrada[0];
					Calibracion_Receptor[4].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[4] > Calibracion_Receptor[4].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[4] :  Calibracion_Receptor[4].Rango_Entrada[1];
					Calibracion_Receptor[5].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[5] < Calibracion_Receptor[5].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[5] :  Calibracion_Receptor[5].Rango_Entrada[0];
					Calibracion_Receptor[5].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[5] > Calibracion_Receptor[5].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[5] :  Calibracion_Receptor[5].Rango_Entrada[1];
					Calibracion_Receptor[6].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[6] < Calibracion_Receptor[6].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[6] :  Calibracion_Receptor[6].Rango_Entrada[0];
					Calibracion_Receptor[6].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[6] > Calibracion_Receptor[6].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[6] :  Calibracion_Receptor[6].Rango_Entrada[1];
//					Calibracion_Receptor[7].Rango_Entrada[0] = Lectura_Radio.Canal_PWM[7] < Calibracion_Receptor[7].Rango_Entrada[0] ? Lectura_Radio.Canal_PWM[7] :  Calibracion_Receptor[7].Rango_Entrada[0];
//					Calibracion_Receptor[7].Rango_Entrada[1] = Lectura_Radio.Canal_PWM[7] > Calibracion_Receptor[7].Rango_Entrada[1] ? Lectura_Radio.Canal_PWM[7] :  Calibracion_Receptor[7].Rango_Entrada[1];
				break;
				case ERROR:
					if (Canal[7] > -83.2){ //..FIN_ERROR..//
						Estado_Sistema = ESPERA;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 1);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

					}
/*					if((Canal[7] > -83.2) && (Canal[7] < -50.0)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = Perturbaciones_Seleccionada; InfoTelemetria = TELE_0; }
					else if((Canal[7] > -50.0) && (Canal[7] < -16.6)){ Modo_Control = ANGULOS_4; ModoPerturbaciones = Perturbaciones_Seleccionada; InfoTelemetria = TELE_1;}
					else if((Canal[7] > -16.6) && (Canal[7] <  16.6)){ Modo_Control =    EMPUJE; ModoPerturbaciones = Perturbaciones_Seleccionada; InfoTelemetria = TELE_0;}
					else if((Canal[7] >  16.6) && (Canal[7] <  50.0)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_2;}
					else if((Canal[7] >  50.0) && (Canal[7] <  83.2)){ Modo_Control = ANGULOS_4; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_3;}
					else if((Canal[7] > -83.2)){ Modo_Control = EMPUJE; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_4;}
*/					break;
				case ESPERA:
				case DEBUG:
					if(Canal[7] < -83.2){	//...ERROR....//
						Estado_Sistema = ERROR;

						GPIO_write(QUAD_BOARD_LED_RED, 1);
						GPIO_write(QUAD_BOARD_LED_GREEN, 0);
						GPIO_write(QUAD_BOARD_LED_BLUE, 0);

						//Stop a todos los clocks y tareas
						Clock_stop(CLOCK_Control);
						Semaphore_reset(SEMAPHORE_Control, 0);
						Clock_stop(CLOCK_Identificacion);
						Semaphore_reset(SEMAPHORE_Identificacion, 0);

						//Parada_motores
						PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
						PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);
					}
/*					else if((Canal[7] > -83.2) && (Canal[7] < -50.0)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = Perturbaciones_Seleccionada; InfoTelemetria = TELE_0; }
					else if((Canal[7] > -50.0) && (Canal[7] < -16.6)){ Modo_Control = ANGULOS_4; ModoPerturbaciones = Perturbaciones_Seleccionada; InfoTelemetria = TELE_1;}
					else if((Canal[7] > -16.6) && (Canal[7] <  16.6)){ Modo_Control =    EMPUJE; ModoPerturbaciones = Perturbaciones_Seleccionada; InfoTelemetria = TELE_0;}
					else if((Canal[7] >  16.6) && (Canal[7] <  50.0)){ Modo_Control = ANGULOS_3; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_2;}
					else if((Canal[7] >  50.0) && (Canal[7] <  83.2)){ Modo_Control = ANGULOS_4; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_3;}
					else if((Canal[7] >  83.2)){ Modo_Control = EMPUJE; ModoPerturbaciones = NO_CORREGIR_PERTURBACIONES; InfoTelemetria = TELE_0;}
*/					break;
			}
		}
	}
}

void CLK_Coordinador(){
	Semaphore_post(SEMAPHORE_Coordinador);
}

//....Identificacion......//
void Identificacion(UArg arg0, UArg arg1){
	UInt Key;
	tpLecturas_IMU Lecturas_IMU;
	int16_t Aux;
//	uint32_t Rpm;

    GPIO_enableInt(QUAD_BOARD_RPM);

	while(1){
		Semaphore_pend(SEMAPHORE_Identificacion, BIOS_WAIT_FOREVER);

		Aux = *((int16_t *)Datos + nDatos_leidos);
		PWM_setDuty(PWM0, PuntoTrabajo_motor + Aux/2 + Pulso_minimo_PWM_motor);
		PWM_setDuty(PWM2, PuntoTrabajo_motor - Aux/2 + Pulso_minimo_PWM_motor);
		nDatos_leidos++;

		Leer_servidor_Lecturas_IMU(&Lecturas_IMU);

		Key = Hwi_disable();
//		Rpm = (uint32_t)(Frecuencia_CPU / Ticks_por_RPS *60);
		Hwi_restore(Key);

		UART_write(UART_BT_TELEMETRIA, &Lecturas_IMU, 14);
//		UART_write(UART_BT_TELEMETRIA, &Rpm, sizeof(Rpm));

		if((nDatos_leidos == nDatos_Identifiacion) || (Estado_Sistema == ESPERA)){
			Clock_stop(CLOCK_Identificacion);
			Semaphore_reset(SEMAPHORE_Identificacion, 0);
			nDatos_leidos = 0;

			Memory_free(NULL, Datos, nDatos_Identifiacion*2);
			PWM_setDuty(PWM0, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM1, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM2, Pulso_minimo_PWM_motor);
			PWM_setDuty(PWM3, Pulso_minimo_PWM_motor);

			Estado_Sistema = ESPERA;

		    GPIO_write(QUAD_BOARD_LED_RED, 1);
		    GPIO_write(QUAD_BOARD_LED_GREEN, 1);
		    GPIO_write(QUAD_BOARD_LED_BLUE, 0);
		}
	}
}

void CLK_Identificacion(){
	Semaphore_post(SEMAPHORE_Identificacion);
}
#ifdef Sensor_RPM
	void ISR_GPIO_RPM(UArg arg0){
		static uint32_t Tick_anterior = 0;

		Ticks_por_RPS =  Clock_getTicks() - Tick_anterior;
	}
#endif
//....Altura.............//

void ISR_GPIO_US(UArg arg0){
	UInt Key;

	GPIO_clearInt(QUAD_BOARD_ECHO);

	if(GPIO_read(QUAD_BOARD_ECHO) == 0){
		Key = Hwi_disable();
		Timer_stop(US_Timer);
		Altura_US_mm = (Timer_getPeriod(US_Timer) - Timer_getCount(US_Timer))/80*0.340/2 ;  //microsegondos
		Hwi_restore(Key);
	}else{
		Key = Hwi_disable();
		Timer_setPeriodMicroSecs(US_Timer, (uint32_t)Max_pulso_us);
		Timer_start(US_Timer);
		Hwi_restore(Key);
	}
}

void ISR_Timer_US(){

	if(Timer_getPeriod(US_Timer) == 800){
		GPIO_write(QUAD_BOARD_TRIGG, 0);
	}
	else{
		Altura_US_mm = 0xFF;
	}
}

void Calculo_Altura(UArg arg0, UArg arg1){
	UInt Key;

	uint32_t Presion_Inicial = 0;
	tpLecturasBarometro LecturasBarometro;

	Iniciar_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);

	Iniciar_Medida_Temp_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);
	Task_sleep(5);
	Leer_Temp_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);
	Iniciar_Medida_Temp_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);
	Task_sleep(26);
	Leer_Presion_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);

	Presion_Inicial = LecturasBarometro.Presion;

	Clock_start(CLOCK_Calculo_Altura);
	while(1){
		Semaphore_pend(SEMAPHORE_Calculo_Altura, BIOS_WAIT_FOREVER);
		//....US....//
		GPIO_write(QUAD_BOARD_TRIGG, 1);
		Timer_setPeriodMicroSecs(US_Timer, (uint32_t)Pulso_arranque_us);
		Key = Hwi_disable();
		Timer_start(US_Timer);
		Hwi_restore(Key);

		//...Barometro....//
		Iniciar_Medida_Temp_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);
		Task_sleep(5);
		Leer_Temp_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);
		Iniciar_Medida_Temp_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);
		Task_sleep(26);
		Leer_Presion_Barometro(I2C_PRINCIPAL, Direccion_Barometro, &LecturasBarometro);

		Altura_Presion_mm = 4433000 * ( 1 - pow(LecturasBarometro.Presion / Presion_Inicial , 1/5.255));

	}
}

void CLK_Calculo_Altura(){
	Semaphore_post(SEMAPHORE_Calculo_Altura);
}

void Control(UArg arg0, UArg arg1){

	tpLecturas_IMU Lecturas_IMU_Control;
	tpLecturas_Giroscopo Lecturas_Giroscopo_control;
	tpLecturas_Brujula Lecturas_Brujula_control;

	tpAHRS AHRS = {
	    .DCM_matriz = {1, 0, 0, 0, 1, 0, 0, 0, 1},
		.DCM = {3, 3, (float32_t *)AHRS.DCM_matriz},
		.Kp_Roll_Pitch = Kp_ROLLPITCH,
		.Ki_Roll_Pitch = Ki_ROLLPITCH,
		.Kp_Yaw = Kp_YAW,
		.Ki_Yaw = Ki_YAW,
		.Periodo_Muestreo = PERIODO_Control / 1000.0
	};

	tpTelemetria_YPR Telemetria_YPR = {
			.Inicio = START_FRAME,
			.Final = FINAL_FRAME
		};

	tpTelemetria_Control Telemetria_Control = {
		.Inicio = START_FRAME,
		.Final = FINAL_FRAME
	};

	float32_t Ref_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Ref = {4, 1, Ref_matriz};

	float32_t Accion_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Accion = {4, 1, Accion_matriz};

	float32_t Variables_medidas_matriz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Variables_medidas = {10, 1, Variables_medidas_matriz};

	float32_t Variables_predichas_matriz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Variables_predichas = {10, 1, Variables_predichas_matriz};

	float32_t Variables_estimadas_matriz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Variables_estimadas = {10, 1, Variables_estimadas_matriz};

	float32_t Perturbaciones_estimadas_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Perturbaciones_estimadas = {4, 1, Perturbaciones_estimadas_matriz};

	float32_t Perturbaciones_calculadas_matriz[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 Perturbaciones_calculadas = {4, 1, Perturbaciones_calculadas_matriz};

	float32_t Aux_Matriz[100];
	arm_matrix_instance_f32 Aux = {10, 10, Aux_Matriz};

	float32_t Aux2_Matriz[100];
	arm_matrix_instance_f32 Aux2 = {10, 10, Aux2_Matriz};

	float32_t Var_Est_Aux[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#ifdef Filtro_Perturbaciones

	float32_t Estado_filtro_Per_0[4*num_etapas_Filtro_Per];
	float32_t Estado_filtro_Per_1[4*num_etapas_Filtro_Per];
	float32_t Estado_filtro_Per_2[4*num_etapas_Filtro_Per];
	float32_t Estado_filtro_Per_3[4*num_etapas_Filtro_Per];

	arm_biquad_casd_df1_inst_f32 Filtro_Per_0 = {num_etapas_Filtro_Per, Estado_filtro_Per_0, (float32_t *)Coeficientes_Filtro_Pre_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Per_1 = {num_etapas_Filtro_Per, Estado_filtro_Per_1, (float32_t *)Coeficientes_Filtro_Pre_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Per_2 = {num_etapas_Filtro_Per, Estado_filtro_Per_2, (float32_t *)Coeficientes_Filtro_Pre_Valores};
	arm_biquad_casd_df1_inst_f32 Filtro_Per_3 = {num_etapas_Filtro_Per, Estado_filtro_Per_3, (float32_t *)Coeficientes_Filtro_Pre_Valores};

	//Inicializamos el filtro
	arm_fill_f32(0.0, Estado_filtro_Per_0, 4*num_etapas_Filtro_Per);
	arm_fill_f32(0.0, Estado_filtro_Per_1, 4*num_etapas_Filtro_Per);
	arm_fill_f32(0.0, Estado_filtro_Per_2, 4*num_etapas_Filtro_Per);
	arm_fill_f32(0.0, Estado_filtro_Per_3, 4*num_etapas_Filtro_Per);
#endif

	while(1){
		Semaphore_pend(SEMAPHORE_Control, BIOS_WAIT_FOREVER);

		Leer_servidor_Lecturas_IMU(&Lecturas_IMU_Control);
		Leer_servidor_Lecturas_Giroscopo(&Lecturas_Giroscopo_control);
		Leer_servidor_Lecturas_Brujula(&Lecturas_Brujula_control);
		Leer_servidor_Referencia(Ref_matriz, NULL);
		Ref_matriz[2] = Posicion_inicial + Ref_matriz[2]; //Giro no absoluto +- posicion inicial;

		Leer_servidor_DCM((float32_t*)AHRS.DCM_matriz);
		Leer_servidor_RPY(&AHRS.Roll, &AHRS.Pitch, &AHRS.Yaw);

		Leer_servidor_Perturbaciones_Estimadas(Perturbaciones_estimadas.pData);

//.....Sensado..Variables.....//
		//Ajuste posicion inicial//
		Variables_medidas.pData[1] = Normalizar_Grados(CONVERTIR_A_GRADOS(AHRS.Pitch));
		Variables_medidas.pData[3] = Normalizar_Grados(CONVERTIR_A_GRADOS(AHRS.Roll));
		Variables_medidas.pData[5] = Normalizar_Grados(CONVERTIR_A_GRADOS(AHRS.Yaw));

#ifdef IMU_MPU6050
		Variables_medidas.pData[0] = (float32_t)Lecturas_IMU_Control.Valor.y_vel / IMU6050.Sensibilidad_Giroscopo;
		Variables_medidas.pData[2] = (float32_t)Lecturas_IMU_Control.Valor.x_vel / IMU6050.Sensibilidad_Giroscopo;
		Variables_medidas.pData[4] = (float32_t)Lecturas_IMU_Control.Valor.z_vel / IMU6050.Sensibilidad_Giroscopo;
#endif

#ifdef GYRO_L3G4200
		Variables_medidas.pData[0] = (float32_t)Lecturas_Giroscopo_control.Valor.y_vel / Giroscopo_L3G4200.Sensibilidad_Giroscopo;
		Variables_medidas.pData[2] = (float32_t)Lecturas_Giroscopo_control.Valor.x_vel / Giroscopo_L3G4200.Sensibilidad_Giroscopo;
		Variables_medidas.pData[4] = (float32_t)Lecturas_Giroscopo_control.Valor.z_vel / Giroscopo_L3G4200.Sensibilidad_Giroscopo;
#endif

		if(Modo_Control == ANGULOS_3){
			switch(ModoPerturbaciones){
				case NO_CORREGIR_PERTURBACIONES:
				case CORREGIR_PERTURBACIONES:
				case INTEGRAR_PERTURBACIONES_ESTIMADAS:
					Variables_medidas.pData[9] = Variables_predichas.pData[9];
				break;
				case INTEGRAR_PERTURBACIONES:
					Variables_medidas.pData[9] = Ref.pData[3];
			}
		}
		Escribir_servidor_Variables_Estado_Medidas(Variables_medidas.pData);

//Estimar variables estado

		Aux.numRows = Variables_medidas.numRows;
		Aux.numCols = Variables_predichas.numCols;
		arm_mat_sub_f32(&Variables_medidas, &Variables_predichas, &Aux);

		Aux2.numRows = 10;
		Aux2.numCols = 1;
		arm_mat_mult_f32(&Lo_per, &Aux, &Aux2);

		arm_mat_add_f32(&Variables_predichas, &Aux2, &Variables_estimadas);

#ifdef Estimador_Parcial //Optimizar multiplicando solo los estimados
		Variables_estimadas.pData[0] = Variables_medidas.pData[0];
		Variables_estimadas.pData[1] = Variables_medidas.pData[1];
		Variables_estimadas.pData[2] = Variables_medidas.pData[2];
		Variables_estimadas.pData[3] = Variables_medidas.pData[3];
		Variables_estimadas.pData[4] = Variables_medidas.pData[4];
		Variables_estimadas.pData[5] = Variables_medidas.pData[5];

#endif
		Escribir_servidor_Variables_Estado_Estimadas(Variables_estimadas.pData);

//Estimar perturbacion
		switch(ModoPerturbaciones){
			case NO_CORREGIR_PERTURBACIONES:
			case CORREGIR_PERTURBACIONES:
/*
				Aux.numRows = Variables_medidas.numRows;
				Aux.numCols = Variables_predichas.numCols;
				arm_mat_sub_f32(&Variables_medidas, &Variables_predichas, &Aux);
*/
				Aux2.numRows = Aux2.numRows;
				Aux2.numCols = Aux2.numCols;
				arm_mat_mult_f32(&Lp, &Aux, &Aux2);

				Aux.numRows = Aux.numRows;
				Aux.numCols = Aux.numCols;
				arm_mat_add_f32(&Perturbaciones_estimadas, &Aux2, &Aux);

				//A 0 la estimacion de Empuje
//				Aux.pData[3] = 0;

#ifndef Filtro_Perturbaciones
				arm_copy_f32(Aux.pData, Perturbaciones_estimadas.pData, sizeof(Perturbaciones_estimadas_matriz)/sizeof(float32_t));
#else
				//		Filtar_Perturbacion
				arm_biquad_cascade_df1_f32(&Filtro_Per_0, &Aux.pData[0], &Perturbaciones_estimadas.pData[0], 1);
				arm_biquad_cascade_df1_f32(&Filtro_Per_1, &Aux.pData[1], &Perturbaciones_estimadas.pData[1], 1);
				arm_biquad_cascade_df1_f32(&Filtro_Per_2, &Aux.pData[2], &Perturbaciones_estimadas.pData[2], 1);
				arm_biquad_cascade_df1_f32(&Filtro_Per_3, &Aux.pData[3], &Perturbaciones_estimadas.pData[3], 1);
#endif

			break;
			case INTEGRAR_PERTURBACIONES:

				Perturbaciones_estimadas.pData[0] += Ki * (Ref.pData[0] - Variables_medidas.pData[1]);
				Perturbaciones_estimadas.pData[1] += Ki * (Ref.pData[1] - Variables_medidas.pData[3]);
				Perturbaciones_estimadas.pData[2] += Ki * (Ref.pData[2] - Variables_medidas.pData[5]);
				Perturbaciones_estimadas.pData[3] += Ki * (Ref.pData[3] - Variables_medidas.pData[9]);
			break;

			case INTEGRAR_PERTURBACIONES_ESTIMADAS:

				Perturbaciones_estimadas.pData[0] += Ki_EST * (Variables_predichas.pData[1] - Variables_medidas.pData[1]);
				Perturbaciones_estimadas.pData[1] += Ki_EST * (Variables_predichas.pData[3] - Variables_medidas.pData[3]);
				Perturbaciones_estimadas.pData[2] += Ki_EST * (Variables_predichas.pData[5] - Variables_medidas.pData[5]);
				Perturbaciones_estimadas.pData[3] += Ki_EST * (Variables_predichas.pData[9] - Variables_medidas.pData[9]);
			break;
		}

				//Prealimentar perturbaciones conocidas
				Perturbaciones_calculadas.pData[0] = Perturbaciones_estimadas.pData[0];// + Pert_Fuerza_Bateria*sin(Variables_estimadas.pData[1]*PI/180.0);
				Perturbaciones_calculadas.pData[1] = Perturbaciones_estimadas.pData[1];// + Pert_Fuerza_Bateria*sin(Variables_estimadas.pData[3]*PI/180.0);
				Perturbaciones_calculadas.pData[2] = Perturbaciones_estimadas.pData[2];
				Perturbaciones_calculadas.pData[3] = Perturbaciones_estimadas.pData[3];

		 Escribir_servidor_Perturbaciones_Estimadas(Perturbaciones_estimadas.pData);

//Accion
		switch(ModoPerturbaciones){
			case NO_CORREGIR_PERTURBACIONES:
				Aux.numRows = K_pre_4.numRows;
				Aux.numCols = Ref.numCols;

				Aux2.numRows = K_4.numRows;
				Aux2.numCols = Variables_estimadas.numCols;

				switch(Modo_Control){
					case ANGULOS_4:
						arm_mat_mult_f32(&K_pre_4, &Ref, &Aux);
						arm_mat_mult_f32(&K_4, &Variables_estimadas, &Aux2);
						arm_mat_sub_f32(&Aux, &Aux2, &Accion);
					break;

					case ANGULOS_3:
						arm_mat_mult_f32(&K_pre_3, &Ref, &Aux);
						arm_mat_mult_f32(&K_3, &Variables_estimadas, &Aux2);
						arm_mat_sub_f32(&Aux, &Aux2, &Accion);
					break;

					case EMPUJE:
						Accion_matriz[0] =  Ref.pData[3]; //
						Accion_matriz[1] =  Ref.pData[3]; //
						Accion_matriz[2] =  Ref.pData[3]; //
						Accion_matriz[3] =  Ref.pData[3]; //
					break;
				}
			break;

			case CORREGIR_PERTURBACIONES:
			case INTEGRAR_PERTURBACIONES:
			case INTEGRAR_PERTURBACIONES_ESTIMADAS:

				Aux.numRows = K_4.numRows;
				Aux.numCols = Variables_estimadas.numCols;

				Aux2.numRows = Accion.numRows;
				Aux2.numCols = Aux.numCols;

				switch(Modo_Control){
					case ANGULOS_4:
						arm_mat_mult_f32(&K_pre_4, &Ref, &Accion);
						arm_mat_mult_f32(&K_4, &Variables_estimadas, &Aux);

						arm_mat_sub_f32(&Accion, &Aux, &Aux2);

						Aux.numRows = La.numRows;
						Aux.numCols = Perturbaciones_calculadas.numCols;
						arm_mat_mult_f32(&La, &Perturbaciones_calculadas, &Aux);

						arm_mat_sub_f32(&Aux2, &Aux, &Accion);
					break;

					case ANGULOS_3:
						arm_mat_mult_f32(&K_pre_3, &Ref, &Accion);
						arm_mat_mult_f32(&K_3, &Variables_estimadas, &Aux);

						arm_mat_sub_f32(&Accion, &Aux, &Aux2);

						Aux.numRows = La.numRows;
						Aux.numCols = Perturbaciones_calculadas.numCols;
						arm_mat_mult_f32(&La, &Perturbaciones_calculadas, &Aux);

						arm_mat_sub_f32(&Aux2, &Aux, &Accion);
					break;
					case EMPUJE:
						Accion_matriz[0] =  Ref.pData[3]; //
						Accion_matriz[1] =  Ref.pData[3]; //
						Accion_matriz[2] =  Ref.pData[3]; //
						Accion_matriz[3] =  Ref.pData[3]; //
					break;
				}
			break;

		}

		if(Accion.pData[0] < Accion_Minima )
			Accion.pData[0] = Accion_Minima;
		if(Accion.pData[1] < Accion_Minima )
			Accion.pData[1] = Accion_Minima;
		if(Accion.pData[2] < Accion_Minima )
			Accion.pData[2] = Accion_Minima;
		if(Accion.pData[3] < Accion_Minima )
			Accion.pData[3] = Accion_Minima;

		if(Accion.pData[0] > Accion_Maxima )
			Accion.pData[0] = Accion_Maxima;
		if(Accion.pData[1] > Accion_Maxima )
			Accion.pData[1] = Accion_Maxima;
		if(Accion.pData[2] > Accion_Maxima )
			Accion.pData[2] = Accion_Maxima;
		if(Accion.pData[3] > Accion_Maxima )
			Accion.pData[3] = Accion_Maxima;

//Aplicar U
		PWM_setDuty(PWM0, (uint32_t)Accion.pData[0] + Pulso_minimo_PWM_motor);
		PWM_setDuty(PWM1, (uint32_t)Accion.pData[1] + Pulso_minimo_PWM_motor);
		PWM_setDuty(PWM2, (uint32_t)Accion.pData[2] + Pulso_minimo_PWM_motor);
		PWM_setDuty(PWM3, (uint32_t)Accion.pData[3] + Pulso_minimo_PWM_motor);

//Predecir estado
		arm_mat_mult_f32(&F, &Variables_estimadas, &Variables_predichas);

		Aux.numRows = G.numRows;
		Aux.numCols = Accion.numCols;
		arm_mat_mult_f32(&G, &Accion, &Aux);

		Aux2.numRows = Variables_predichas.numRows;
		Aux2.numCols = Aux.numCols;
		arm_mat_add_f32(&Variables_predichas, &Aux, &Aux2);

		Aux.numRows = Gp.numRows;
		Aux.numCols = Perturbaciones_calculadas.numCols;
		arm_mat_mult_f32(&Gp, &Perturbaciones_calculadas, &Aux);

		arm_mat_add_f32(&Aux2, &Aux, &Variables_predichas);


//Telemetria
		switch(ModoTelemetria){
			case TELEMETRIA_YPR:
				Telemetria_YPR.Yaw = (int16_t)(AHRS.Yaw * 10.0);
				Telemetria_YPR.Pitch = (int16_t)(AHRS.Pitch * 10.0);
				Telemetria_YPR.Roll = (int16_t)(AHRS.Roll * 10.0);

				UART_write(UART_BT_TELEMETRIA, &Telemetria_YPR, sizeof(Telemetria_YPR)-1);
			break;
			case TELEMETRIA_CONTROL:
				Telemetria_Control.InfoTelemetria = InfoTelemetria;
				//Referencia
				Var_Est_Aux[0] = Ref_matriz[0] / Angulo_Max_Q16;
				Var_Est_Aux[1] = Ref_matriz[1] / Angulo_Max_Q16;
				Var_Est_Aux[2] = Ref_matriz[2] / Angulo_Max_Q16;
				Var_Est_Aux[3] = Ref_matriz[3] / 1000.0;
				arm_float_to_q15( Var_Est_Aux, (q15_t *)Telemetria_Control.Referencia, 4);

				//Accion
				Telemetria_Control.Accion[0] = Accion_matriz[0];
				Telemetria_Control.Accion[1] = Accion_matriz[1];
				Telemetria_Control.Accion[2] = Accion_matriz[2];
				Telemetria_Control.Accion[3] = Accion_matriz[3];

				//Var_Est
				Var_Est_Aux[0] = Variables_estimadas.pData[0] / Velocidad_Max_Q16;
				Var_Est_Aux[1] = Variables_estimadas.pData[1] / Angulo_Max_Q16;
				Var_Est_Aux[2] = Variables_estimadas.pData[2] / Velocidad_Max_Q16;
				Var_Est_Aux[3] = Variables_estimadas.pData[3] / Angulo_Max_Q16;
				Var_Est_Aux[4] = Variables_estimadas.pData[4] / Velocidad_Max_Q16;
				Var_Est_Aux[5] = Variables_estimadas.pData[5] / Angulo_Max_Q16;
				Var_Est_Aux[6] = Variables_estimadas.pData[6] / F_Max_Q16;
				Var_Est_Aux[7] = Variables_estimadas.pData[7] / F_Max_Q16;
				Var_Est_Aux[8] = Variables_estimadas.pData[8] / F_Max_Q16;
				Var_Est_Aux[9] = Variables_estimadas.pData[9] / F_Max_Q16;
				arm_float_to_q15( Var_Est_Aux, (q15_t *)Telemetria_Control.Variables_Estado, 10);

				//Perturbaciones
				Var_Est_Aux[0] = Perturbaciones_calculadas_matriz[0] / F_Max_Q16;
				Var_Est_Aux[1] = Perturbaciones_calculadas_matriz[1] / F_Max_Q16;
				Var_Est_Aux[2] = Perturbaciones_calculadas_matriz[2] / F_Max_Q16;
				Var_Est_Aux[3] = Perturbaciones_calculadas_matriz[3] / F_Max_Q16;
				arm_float_to_q15( Var_Est_Aux, (q15_t *)Telemetria_Control.Perturbaciones, 4);

				//Altura
				Telemetria_Control.Altura_Barometrica = Altura_Presion_mm;
				Telemetria_Control.Altura_US = Altura_US_mm;

				Mailbox_pend(Buzon_Lecturas_IMU, &Lecturas_IMU_Control, BIOS_NO_WAIT);

				//Lectura ACCEL
				Telemetria_Control.Acel[0] = Lecturas_IMU_Control.Valor.x_acel;
				Telemetria_Control.Acel[1] = Lecturas_IMU_Control.Valor.y_acel;
				Telemetria_Control.Acel[2] = Lecturas_IMU_Control.Valor.z_acel;

				//Lectura GYRO
				Telemetria_Control.Gyro[0] = Lecturas_IMU_Control.Valor.x_vel;
				Telemetria_Control.Gyro[1] = Lecturas_IMU_Control.Valor.y_vel;
				Telemetria_Control.Gyro[2] = Lecturas_IMU_Control.Valor.z_vel;

#ifdef GYRO_L3G4200
				Telemetria_Control.Gyro[0] = Lecturas_Giroscopo_control.Valor.x_vel;
				Telemetria_Control.Gyro[1] = Lecturas_Giroscopo_control.Valor.y_vel;
				Telemetria_Control.Gyro[2] = Lecturas_Giroscopo_control.Valor.z_vel;
#endif
				//Lectura Brujula
				Telemetria_Control.Magnetics[0] = Lecturas_Brujula_control.Valor.Magnetismo_x;
				Telemetria_Control.Magnetics[1] = Lecturas_Brujula_control.Valor.Magnetismo_y;
				Telemetria_Control.Magnetics[2] = Lecturas_Brujula_control.Valor.Magnetismo_z;

				UART_write(UART_BT_TELEMETRIA, &Telemetria_Control, sizeof(Telemetria_Control)-1);
			break;
		}

	}
}
void CLK_Control(){
	Semaphore_post(SEMAPHORE_Control);
}
