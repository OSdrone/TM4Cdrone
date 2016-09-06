/*
 * Sensores.h
 *
 *  Created on: 11/5/2015
 *      Author: Ruben
 */

#ifndef QUADROTOR_V1_3_1_SENSORES_H_
#define QUADROTOR_V1_3_1_SENSORES_H_

#include <ti/drivers/I2C.h>

#include "arm_math.h"
#include "Parametros.h"
#include "math.h"

#define CONVERTIR_A_RADIANES(DEGS) PI/180.0*DEGS
#define CONVERTIR_A_GRADOS(RADS) 180.0/PI*RADS

float32_t Normalizar_Grados(float32_t Grados);

//............RECEPTOR............................//
typedef struct{
	uint16_t Canal_PWM[8];
	uint8_t Voltaje_Bat[4];
	uint8_t Error_conexion;
}tpLectura_Radio;

typedef struct {
	float32_t Rango_Salida[2];   //min max
	uint16_t Rango_Entrada[2]; //min max
}tpCalibracion_canal_PWM;

typedef tpCalibracion_canal_PWM tpCalibracion_Receptor[numCanales];

//..........IMU...................................//
typedef union{ //tpLecturas_IMU
	struct{
    uint8_t x_acel_l;	//Los ponemos al reves (LOW y HIGH) dado que
	uint8_t x_acel_h;	//La arquitectura usa LITTLE Endian
	uint8_t y_acel_l;
	uint8_t y_acel_h;
    uint8_t z_acel_l;
    uint8_t z_acel_h;

	uint8_t temp_l;
    uint8_t temp_h;

    uint8_t x_vel_l;
    uint8_t x_vel_h;
    uint8_t y_vel_l;
	uint8_t y_vel_h;
    uint8_t z_vel_l;
    uint8_t z_vel_h;

  }Reg;

  struct{
		int16_t x_acel;
		int16_t y_acel;
		int16_t z_acel;

		int16_t temp;

		int16_t x_vel;
		int16_t y_vel;
		int16_t z_vel;
  }Valor;
}tpLecturas_IMU;

typedef struct{ //tpCalibracion_IMU
	int16_t Rango_Acel_x[2]; // Min Max
	int16_t Media_Acel_x;
	float32_t Des_est_Acel_x;
	int16_t Rango_Acel_y[2];
	int16_t Media_Acel_y;
	float32_t Des_est_Acel_y;
	int16_t Rango_Acel_z[2];
	int16_t Media_Acel_z;
	float32_t Des_est_Acel_z;

	int16_t Media_Temp;
	float32_t Des_est_Temp;

	int16_t Rango_Vel_x[2];
	int16_t Media_Vel_x;
	float32_t Des_est_Vel_x;
	int16_t Rango_Vel_y[2];
	int16_t Media_Vel_y;
	float32_t Des_est_Vel_y;
	int16_t Rango_Vel_z[2];
	int16_t Media_Vel_z;
	float32_t Des_est_Vel_z;

	float32_t Correccion_Alineamiento_matriz[9];
	arm_matrix_instance_f32 Correccion_Alineamiento;

	float32_t Giro[3];

}tpCalibracion_IMU;


//..........BRUJULA......................//

typedef union{ //tpLecturas_Brujula
	struct{
		uint8_t Magnetismo_x_l; //tpLecturas_Brujula
		uint8_t Magnetismo_x_h; //La arquitectura usa LITTLE Endian
		uint8_t Magnetismo_y_l;
		uint8_t Magnetismo_y_h;
		uint8_t Magnetismo_z_l;
		uint8_t Magnetismo_z_h;
	}Reg;
	struct{
		int16_t Magnetismo_x;
		int16_t Magnetismo_y;
		int16_t Magnetismo_z;
	}Valor;
}tpLecturas_Brujula;

typedef struct{
	int16_t Media_Magnetismo_x;
	float32_t Des_est_Mag_x;

	int16_t Media_Magnetismo_y;
	float32_t Des_est_Mag_y;

	int16_t Media_Magnetismo_z;
	float32_t Des_est_Mag_z;

	float32_t Offset_matriz[3];
	arm_matrix_instance_f32 Offset;
	float32_t Transformada_matriz[9];
	arm_matrix_instance_f32 Transformada;

}tpCalibracion_Brujula;

//.......BAROMETRO.................//

typedef struct{//tpLecturasBarometro
		uint32_t UP;
		uint16_t UT;

		uint32_t Presion;
		float Temperatura;

		//Parametros de calibracion
		int16_t AC1;
		int16_t AC2;
		int16_t AC3;
		uint16_t AC4;
		uint16_t AC5;
		uint16_t AC6;
		int16_t B1;
		int16_t B2;
		int16_t MB;
		int16_t MC;
		int16_t MD;
}tpLecturasBarometro;

//...........GIROSCOPO..................//

typedef union{ //tpLecturas_Giroscopo
	struct{

	    uint8_t x_vel_l;
	    uint8_t x_vel_h;
	    uint8_t y_vel_l;
		uint8_t y_vel_h;
	    uint8_t z_vel_l;
	    uint8_t z_vel_h;
	}Reg;
	struct{

		int16_t x_vel;
		int16_t y_vel;
		int16_t z_vel;
	}Valor;
}tpLecturas_Giroscopo;


//..............................IMU_6050........................................//
typedef struct{
	uint8_t Direccion;
	uint8_t SMPLRT_DIV;
	float32_t Sensibilidad_Giroscopo;
	float32_t Sensibilidad_Acel;
	enum {DLPF_CFG_0, DLPF_CFG_1, DLPF_CFG_2, DLPF_CFG_3, DLPF_CFG_4, DLPF_CFG_5, DLPF_CFG_6, DLPF_CFG_7}DLPF_CFG;
	enum {Gain_Gyro_250, Gain_Gyro_500, Gain_Gyro_1000, Gain_Gyro_2000}Ganancia_Gyro;
	enum {Gain_Acel_2G, Gain_Acel_4G, Gain_Acel_8G, Gain_Acel_16G}Ganancia_Acel;

}tpIMU6050;


#define Dir_0_IMU_MPU6050 0b1101000
#define Dir_1_IMU_MPU6050 0b1101001

#define IMU_MPU6050_SMPLRT_DIV 0x19
#define IMU_MPU6050_CONFIG 0x1A
#define IMU_MPU6050_GYRO_CONFIG 0x1B
#define IMU_MPU6050_ACCEL_CONFIG 0x1C

#define IMU_MPU6050_PWR_MGMT_1 0x6B
#define IMU_MPU6050_ACCEL_XOUT_H 0x3B

bool Iniciar_IMU_MPU6050(I2C_Handle I2C, tpIMU6050 IMU6050);
bool Leer_IMU_MPU6050(I2C_Handle I2C, tpIMU6050 IMU6050, tpLecturas_IMU *Lecturas_IMU);

//............................GYRO_ITG3200...................................//
#define Dir_Gir_0 0b01101000
#define Dir_Gir_1 0b01101001
#define Reg_DLPF_FS 0x16
#define Reg_TEMP_OUT_H 0x1B

bool Iniciar_Giroscopio_ITG3200(I2C_Handle I2C, uint8_t Dir_Giroscopo, uint16_t Frecuencia_muestreo, uint8_t Filtro);
bool Leer_Giroscopio_ITG3200(I2C_Handle I2C, uint8_t Dir_Giroscopo, tpLecturas_Giroscopo *Lecturas_Giroscopo);

//...........................GYRO_L3G4200........................................//
#define Dir_0_L3G4200 0x68
#define Dir_1_L3G4200 0x69

#define L3G4200_WHO_I_AM 0x0F
#define L3G4200_CTRL_REG1 0x20
#define L3G4200_CTRL_REG2 0x21
#define L3G4200_CTRL_REG3 0x22
#define L3G4200_CTRL_REG4 0x23
#define L3G4200_CTRL_REG5 0x24
#define L3G4200_REFERENCE 0x25
#define L3G4200_OUT_TEMP 0x26
#define L3G4200_STATUS_REG 0x27
#define L3G4200_OUT_X_L 0x28

#define L3G4200_FIFO_CTRL_REG 0x2E

typedef struct{
	uint8_t Direccion;
	float32_t Sensibilidad_Giroscopo;
	enum{dps_250 = 0, dps_500, dps_2000}Ganancia;
	enum{ODR_100_Hz = 0, ODR_200_Hz, ODR_400_Hz, ODR_800_Hz}ODR;
	enum{Bypass = 0, FIFO, Stream, Bypass_to_Stream, Stream_to_FIFO}Modo;
	enum{LPF1_0 = 0, LPF1_1, LPF1_2, LPF1_3}BW_LPF;
	enum{HPF_No_Filtro = 0, HPF_Filtro = 1 }HPF_activar;
	enum{Filtrado_LPF = 0, Filtrado_HPF, Filtrado_LPF2}Modo_Filtro;
	enum{HPF_0 = 0, HPF_1, HPF_2, HPF_3, HPF_4, HPF_5, HPF_6, HPF_7}BW_HPF;
	enum{HPF_Normal_mode = 0, HPF_Reference, HPF_Normal, HPF_Autoreset}HPF_modo;
	enum{BDU_Continuo = 0, BDU_No_continuo}BDU;
	enum{BLE_Big_Endian = 0, BLE_Little_Endian}BLE;
}tpGiroscopo_L3G4200;


bool Iniciar_Giroscopo_L3G4200(I2C_Handle I2C, tpGiroscopo_L3G4200 Giroscopo_L3G4200);
bool Leer_Giroscopo_L3G4200(I2C_Handle I2C, tpGiroscopo_L3G4200 Giroscopo_L3G4200, tpLecturas_Giroscopo *Lecturas_Giroscopo);

//.............BRUJULA_HMC5883L............//
#define HMC5883L_DIR 0x1E
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_DATA_OUTPUT_X 0x03
#define HMC5883L_SATUS 0x09
/*
#ifdef SENSIBILIDAD_MAG
	const uint16_t Magnitud_HMC5883L[8] = {1370, 1090, 820, 660, 440, 390, 330, 230};
#endif
*/
typedef struct{
	float32_t Angulo_Rotacion;
	enum {MEDIA_1, MEDIA_2, MEDIA_4, MEDIA_8}Muestras_Media;
	enum {ODR_0_75_Hz, ODR_1_5_Hz, ODR_3_Hz, ODR_7_5_Hz, ODR_15_Hz, ODR_30_Hz, ODR_75_Hz}ODR;
	enum {Normal, Bias_Positivo, Bias_Negativo}Modo_Medida;
	enum {Gauss_0_88, Gauss_1_3, Gauss_1_9, Gauss_2_5, Gauss_4, Gauss_4_7, Gauss_5_6, Gauss_8_1}Ganancia;
	enum {I2C_400_Khz, I2C_3400_Khz}Velocidad_I2C;
	enum {Continuo, Simple, Idle}Modo_Operacion;
	enum {S_0 = 1370, S_1 = 1090, S_2 = 820, S_3 = 660, S_4 = 440, S_5 = 390, S_6 = 330, S_7 = 230}Sensibilidad;
}tpHMC5883L;

bool Iniciar_Brujula_HMC5883L(I2C_Handle I2C, tpHMC5883L HMC5883L);
bool Leer_Brujula_HMC5883L(I2C_Handle I2C, tpHMC5883L HMC5883L, tpLecturas_Brujula *Lecturas_Brujula);

//...BAROMETRO.............//

bool Iniciar_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro);
bool Iniciar_Medida_Temp_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro);
bool Leer_Temp_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro);
bool Iniciar_Medida_Presion_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro);
bool Leer_Presion_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro);


#endif /* QUADROTOR_V1_3_1_SENSORES_H_ */
