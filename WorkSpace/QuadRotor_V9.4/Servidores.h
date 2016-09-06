/*
 * Servidores.h
 *
 *  Created on: 17/5/2015
 *      Author: Ruben
 */

#ifndef QUADROTOR_V1_3_1_SERVIDORES_H_
#define QUADROTOR_V1_3_1_SERVIDORES_H_

#include "Sensores.h"
#include "arm_math.h"

void Iniciar_Servidores();

void Leer_servidor_Lecturas_IMU(tpLecturas_IMU *Lecturas_IMU);
void Escribir_servidor_Lecturas_IMU(tpLecturas_IMU *Lecturas_IMU);

void Leer_servidor_Lecturas_IMU_9DOF(tpLecturas_9DOF_IMU *Lecturas_9DOF_IMU);
void Escribir_servidor_Lecturas_IMU_9DOF(tpLecturas_9DOF_IMU *Lecturas_9DOF_IMU);

void Leer_servidor_Lecturas_Giroscopo(tpLecturas_Giroscopo *Lecturas_Giroscopo);
void Escribir_servidor_Lecturas_Giroscopo(tpLecturas_Giroscopo *Lecturas_Giroscopo);

void Leer_servidor_Lecturas_Brujula(tpLecturas_Brujula *Lecturas_Brujula);
void Escribir_servidor_Lecturas_Brujula(tpLecturas_Brujula *Lecturas_Brujula);

void Leer_servidor_DCM(float32_t* DCM);
void Escribir_servidor_DCM(float32_t* DCM);

void Leer_servidor_RPY(float32_t *Roll, float32_t *Pitch, float32_t *Yaw);
void Escribir_servidor_RPY(float32_t *Roll, float32_t *Pitch, float32_t *Yaw);

void Leer_servidor_quaternios(float32_t* q);
void Escribir_servidor_quaternios(float32_t* q);

void Leer_servidor_Variables_Estado_Medidas(float32_t* Variables_Estado_Medidas);
void Escribir_servidor_Variables_Estado_Medidas(float32_t* Variables_Estado_Medidas);

void Leer_servidor_Variables_Estado_Estimadas(float32_t* Variables_Estado_Estimadas);
void Escribir_servidor_Variables_Estado_Estimadas(float32_t* Variables_Estado_Estimadas);

void Leer_servidor_Perturbaciones_Estimadas(float32_t* Perturbaciones_Estimadas);
void Escribir_servidor_Perturbaciones_Estimadas(float32_t* Perturbaciones_Estimadas);
void Resetear_servidor_Perturbaciones_Estimadas();

void Leer_servidor_Referencia(float32_t* Referencia, int16_t* Referencia_Entero);
void Escribir_servidor_Referencia(float32_t* Referencia, int16_t* Referencia_Entero);
float32_t* Direccion_servidor_Referencia();

#endif /* QUADROTOR_V1_3_1_SERVIDORES_H_ */
