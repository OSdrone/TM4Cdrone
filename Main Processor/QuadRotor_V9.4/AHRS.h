/*
 * AHRS.h
 *
 *  Created on: 22/12/2015
 *      Author: Ruben
 */
#ifndef QUADROTOR_V9_2_AHRS_H_
#define QUADROTOR_V9_2_AHRS_H_

#include "arm_math.h"

typedef struct { //tpAHRS
	float32_t DCM_matriz[3][3];
	arm_matrix_instance_f32 DCM;

	float32_t Periodo_Muestreo; //s

	float32_t Vector_Velocidad_Angular[3]; //  grados / s
	float32_t Vector_Aceleracion_lineal[3];	//  m/s^2
	float32_t Vector_Magnetico[3]; //mT ??

	float32_t Roll;
	float32_t Pitch;
	float32_t Yaw;

	float32_t Kp_Roll_Pitch;
	float32_t Ki_Roll_Pitch;
	float32_t Kp_Yaw;
	float32_t Ki_Yaw;

	float32_t Orientacion_YAW;
	float32_t Correccion_Proporcional[3];
	float32_t Correccion_Integral[3];

}tpAHRS;

void Compensacion_Sensor_magnetico(tpAHRS *AHRS);
void Actualizar_Matriz_DCM(tpAHRS *AHRS);
void Actualizar_Matriz_DCM_V2(tpAHRS *AHRS);
void Normalizar_DCM(tpAHRS *AHRS);
void Correccion_deriva(tpAHRS *AHRS);
void Correccion_deriva_NO_YAW(tpAHRS *AHRS);
void Angulos_Euler(tpAHRS *AHRS);

void Algortimo_DCM_MAG(tpAHRS *AHRS);
void Algortimo_DCM_NO_YAW(tpAHRS *AHRS);

void ResetDCM();
#endif /* QUADROTOR_V9_2_AHRS_H_ */
