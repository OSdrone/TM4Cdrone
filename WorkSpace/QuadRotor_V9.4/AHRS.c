/*
 * AHRS.c
 *
 *  Created on: 22/12/2015
 *      Author: Ruben
 */
#include "AHRS.h"
#include "Servidores.h"

void Compensacion_Sensor_magnetico(tpAHRS *AHRS){
	//Rota el eje magnetico para alinearlo con el suelo, usando la ultima referencia ROLL PITCH
	float32_t mag_x;
	float32_t mag_y;
	float32_t cos_roll;
	float32_t sin_roll;
	float32_t cos_pitch;
	float32_t sin_pitch;

	cos_roll = arm_cos_f32(AHRS->Roll);
	sin_roll = arm_sin_f32(AHRS->Roll);
	cos_pitch = arm_cos_f32(AHRS->Pitch);
	sin_pitch = arm_sin_f32(AHRS->Pitch);

	// Rotamos
	mag_x = AHRS->Vector_Magnetico[0] * cos_pitch + AHRS->Vector_Magnetico[1] * sin_roll * sin_pitch + AHRS->Vector_Magnetico[2] * cos_roll * sin_pitch;
	mag_y = AHRS->Vector_Magnetico[1] * cos_roll - AHRS->Vector_Magnetico[2] * sin_roll;
	AHRS->Orientacion_YAW = atan2(-mag_y, mag_x);
}

void Actualizar_Matriz_DCM(tpAHRS *AHRS){
	float32_t Velocidad_Total[3] = {0, 0, 0};

	float32_t Rot_matriz[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Rotacion = {3, 3, (float32_t *)Rot_matriz};

	float32_t Aux_matriz[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Aux = {3, 3, Aux_matriz};

	Velocidad_Total[0] = AHRS->Vector_Velocidad_Angular[0] + AHRS->Correccion_Proporcional[0] + AHRS->Correccion_Integral[0];
	Velocidad_Total[1] = AHRS->Vector_Velocidad_Angular[1] + AHRS->Correccion_Proporcional[1] + AHRS->Correccion_Integral[1];
	Velocidad_Total[2] = AHRS->Vector_Velocidad_Angular[2] + AHRS->Correccion_Proporcional[2] + AHRS->Correccion_Integral[2];

	Rot_matriz[0][0] =  1;
	Rot_matriz[0][1] = -AHRS->Periodo_Muestreo*Velocidad_Total[2];//-z
	Rot_matriz[0][2] =  AHRS->Periodo_Muestreo*Velocidad_Total[1];//y
	Rot_matriz[1][0] =  AHRS->Periodo_Muestreo*Velocidad_Total[2];//z
	Rot_matriz[1][1] =  1;
	Rot_matriz[1][2] = -AHRS->Periodo_Muestreo*Velocidad_Total[0];//-x
	Rot_matriz[2][0] = -AHRS->Periodo_Muestreo*Velocidad_Total[1];//-y
	Rot_matriz[2][1] =  AHRS->Periodo_Muestreo*Velocidad_Total[0];//x
	Rot_matriz[2][2] =  1;

	arm_mat_mult_f32(&AHRS->DCM, &Rotacion, &Aux);
	arm_copy_f32(Aux.pData, AHRS->DCM.pData, 9);
}

void Actualizar_Matriz_DCM_V2(tpAHRS *AHRS){
	float32_t Velocidad_Total[3] = {0, 0, 0};
	float32_t Vector_Rotacion[3] = {0, 0, 0};

	float32_t Angulo_Rotacion = 0;
	float32_t Seno = 0;
	float32_t Coseno = 0;

	float32_t Rot_matriz[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Rotacion = {3, 3, (float32_t *)Rot_matriz};

	float32_t Aux_matriz[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 Aux = {3, 3, Aux_matriz};

	Velocidad_Total[0] = AHRS->Vector_Velocidad_Angular[0] + AHRS->Correccion_Proporcional[0] + AHRS->Correccion_Integral[0];
	Velocidad_Total[1] = AHRS->Vector_Velocidad_Angular[1] + AHRS->Correccion_Proporcional[1] + AHRS->Correccion_Integral[1];
	Velocidad_Total[2] = AHRS->Vector_Velocidad_Angular[2] + AHRS->Correccion_Proporcional[2] + AHRS->Correccion_Integral[2];

	//Velocidad absoluta
	arm_sqrt_f32((Velocidad_Total[0]*Velocidad_Total[0] + Velocidad_Total[1]*Velocidad_Total[1] +
				  Velocidad_Total[2]*Velocidad_Total[2]), &Angulo_Rotacion);

	//Normalizamos vector rotacion
	if (Angulo_Rotacion != 0.0){
		arm_scale_f32(Velocidad_Total, 1/Angulo_Rotacion, Vector_Rotacion, 3);
	}

	//Pasamos de velocidad a angulo
	Angulo_Rotacion = Angulo_Rotacion*AHRS->Periodo_Muestreo;

	Coseno = arm_cos_f32(Angulo_Rotacion);
	Seno = arm_sin_f32(Angulo_Rotacion);

	Rot_matriz[0][0] = Coseno + Vector_Rotacion[0]*Vector_Rotacion[0]*(1 - Coseno);
	Rot_matriz[0][1] = Vector_Rotacion[0]*Vector_Rotacion[1]*(1 - Coseno) - Vector_Rotacion[2]*Seno;
	Rot_matriz[0][2] = Vector_Rotacion[0]*Vector_Rotacion[2]*(1 - Coseno) + Vector_Rotacion[1]*Seno;
	Rot_matriz[1][0] = Vector_Rotacion[1]*Vector_Rotacion[0]*(1 - Coseno) + Vector_Rotacion[2]*Seno;
	Rot_matriz[1][1] = Coseno + Vector_Rotacion[1]*Vector_Rotacion[1]*(1 - Coseno);
	Rot_matriz[1][2] = Vector_Rotacion[1]*Vector_Rotacion[2]*(1 - Coseno) - Vector_Rotacion[0]*Seno;
	Rot_matriz[2][0] = Vector_Rotacion[2]*Vector_Rotacion[0]*(1 - Coseno) - Vector_Rotacion[1]*Seno;
	Rot_matriz[2][1] = Vector_Rotacion[2]*Vector_Rotacion[1]*(1 - Coseno) + Vector_Rotacion[0]*Seno;
	Rot_matriz[2][2] = Coseno + Vector_Rotacion[2]*Vector_Rotacion[2]*(1 - Coseno);

	arm_mat_mult_f32(&AHRS->DCM, &Rotacion, &Aux);
	arm_copy_f32(Aux.pData, AHRS->DCM.pData, 9);
}

void Normalizar_DCM(tpAHRS *AHRS){
	float error = 0;

	float32_t Vector_Aux[3] = {0, 0, 0};
	float32_t Matriz_Ortogonal[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

	arm_dot_prod_f32(&AHRS->DCM_matriz[0][0], &AHRS->DCM_matriz[1][0], 3, &error);
	error *= -0.5;

	arm_scale_f32(&AHRS->DCM_matriz[1][0], error, Vector_Aux, 3);
	arm_add_f32(&AHRS->DCM_matriz[0][0], Vector_Aux,  &Matriz_Ortogonal[0][0], 3); //Vector X ortogonal

	arm_scale_f32(&AHRS->DCM_matriz[0][0], error, Vector_Aux, 3);
	arm_add_f32(&AHRS->DCM_matriz[1][0], Vector_Aux,  &Matriz_Ortogonal[1][0], 3); //Vector Y ortogonal

	//Producto Cruz
	Matriz_Ortogonal[2][0] = Matriz_Ortogonal[0][1] * Matriz_Ortogonal[1][2] - Matriz_Ortogonal[0][2] * Matriz_Ortogonal[1][1];
	Matriz_Ortogonal[2][1] = Matriz_Ortogonal[0][2] * Matriz_Ortogonal[1][0] - Matriz_Ortogonal[0][0] * Matriz_Ortogonal[1][2];
	Matriz_Ortogonal[2][2] = Matriz_Ortogonal[0][0] * Matriz_Ortogonal[1][1] - Matriz_Ortogonal[0][1] * Matriz_Ortogonal[1][0  ];

	arm_dot_prod_f32(&Matriz_Ortogonal[0][0], &Matriz_Ortogonal[0][0], 3, &error);
	error = 0.5*(3.0 - error);
	arm_scale_f32(&Matriz_Ortogonal[0][0], error, &AHRS->DCM_matriz[0][0], 3);

	arm_dot_prod_f32(&Matriz_Ortogonal[1][0], &Matriz_Ortogonal[1][0], 3, &error);
	error = 0.5*(3.0 - error);
	arm_scale_f32(&Matriz_Ortogonal[1][0], error, &AHRS->DCM_matriz[1][0], 3);

	arm_dot_prod_f32(&Matriz_Ortogonal[2][0], &Matriz_Ortogonal[2][0], 3, &error);
	error = 0.5*(3.0 - error);
	arm_scale_f32(&Matriz_Ortogonal[2][0], error, &AHRS->DCM_matriz[2][0], 3);
}

void Correccion_deriva(tpAHRS *AHRS){
	float32_t error[3] = {0, 0, 0};
	float32_t Aux[3] = {0, 0, 0};

	//ROLL PITCH
	//faltaria filtrar???
	//Producto cruz
	error[0] = AHRS->Vector_Aceleracion_lineal[1] * AHRS->DCM_matriz[2][2] - AHRS->Vector_Aceleracion_lineal[2] * AHRS->DCM_matriz[2][1];
	error[1] = AHRS->Vector_Aceleracion_lineal[2] * AHRS->DCM_matriz[2][0] - AHRS->Vector_Aceleracion_lineal[0] * AHRS->DCM_matriz[2][2];
	error[2] = AHRS->Vector_Aceleracion_lineal[0] * AHRS->DCM_matriz[2][1] - AHRS->Vector_Aceleracion_lineal[1] * AHRS->DCM_matriz[2][0];

	arm_scale_f32((float32_t *)&error, AHRS->Kp_Roll_Pitch, AHRS->Correccion_Proporcional, 3);
	arm_scale_f32((float32_t *)&error, AHRS->Ki_Roll_Pitch, Aux, 3);
	arm_add_f32((float32_t *)&Aux, AHRS->Correccion_Integral, AHRS->Correccion_Integral, 3);

	//YAW
	arm_scale_f32(&AHRS->DCM_matriz[2][0], AHRS->DCM_matriz[0][0]*arm_sin_f32(AHRS->Orientacion_YAW) - AHRS->DCM_matriz[1][0]*arm_cos_f32(AHRS->Orientacion_YAW), error, 3);

	arm_scale_f32(error, AHRS->Kp_Yaw, Aux, 3);
	arm_add_f32((float32_t *)Aux, AHRS->Correccion_Proporcional, AHRS->Correccion_Proporcional, 3);

	arm_scale_f32(error, AHRS->Ki_Yaw, Aux, 3);
	arm_add_f32((float32_t *)&Aux, AHRS->Correccion_Integral, AHRS->Correccion_Integral, 3);

}

void Correccion_deriva_NO_YAW(tpAHRS *AHRS){
	float32_t error[3] = {0, 0, 0};
	float32_t Aux[3] = {0, 0, 0};

	//ROLL PITCH
	//faltaria filtrar???
	//Producto cruz
	error[0] = AHRS->Vector_Aceleracion_lineal[1] * AHRS->DCM_matriz[2][2] - AHRS->Vector_Aceleracion_lineal[2] * AHRS->DCM_matriz[2][1];
	error[1] = AHRS->Vector_Aceleracion_lineal[2] * AHRS->DCM_matriz[2][0] - AHRS->Vector_Aceleracion_lineal[0] * AHRS->DCM_matriz[2][2];
	error[2] = AHRS->Vector_Aceleracion_lineal[0] * AHRS->DCM_matriz[2][1] - AHRS->Vector_Aceleracion_lineal[1] * AHRS->DCM_matriz[2][0];

	arm_scale_f32((float32_t *)&error, AHRS->Kp_Roll_Pitch, AHRS->Correccion_Proporcional, 3);
	arm_scale_f32((float32_t *)&error, AHRS->Ki_Roll_Pitch, Aux, 3);
	arm_add_f32((float32_t *)&Aux, AHRS->Correccion_Integral, AHRS->Correccion_Integral, 3);
}

void Angulos_Euler(tpAHRS *AHRS){
	AHRS->Pitch = -asin(AHRS->DCM_matriz[2][0]);
	AHRS->Roll = atan2(AHRS->DCM_matriz[2][1],AHRS->DCM_matriz[2][2]);
	AHRS->Yaw = atan2(AHRS->DCM_matriz[1][0],AHRS->DCM_matriz[0][0]);
}

void ResetDCM(){
	tpLecturas_IMU Lecturas_IMU = {0, 0, 0, 0, 0, 0, 0 };
	tpLecturas_Brujula Lecturas_Brujula = {0, 0, 0};

	float32_t DCM_matriz[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	float32_t Roll = 0;
	float32_t Pitch = 0;
	float32_t Yaw = 0;

	float32_t sin_Roll = 0;
	float32_t cos_Roll = 0;
	float32_t sin_Pitch = 0;
	float32_t cos_Pitch = 0;
	float32_t sin_Yaw = 0;
	float32_t cos_Yaw = 0;

	Leer_servidor_Lecturas_IMU(&Lecturas_IMU);
	Leer_servidor_Lecturas_Brujula(&Lecturas_Brujula);

	Pitch = -atan2(Lecturas_IMU.Valor.x_acel, sqrt(Lecturas_IMU.Valor.y_acel * Lecturas_IMU.Valor.y_acel + Lecturas_IMU.Valor.z_acel * Lecturas_IMU.Valor.z_acel));
	Roll =   atan2(Lecturas_IMU.Valor.y_acel, sqrt(Lecturas_IMU.Valor.x_acel * Lecturas_IMU.Valor.x_acel + Lecturas_IMU.Valor.z_acel * Lecturas_IMU.Valor.z_acel));

	sin_Roll = arm_sin_f32(Roll);
	cos_Roll = arm_cos_f32(Roll);
	sin_Pitch = arm_sin_f32(Pitch);
	cos_Pitch = arm_cos_f32(Pitch);

	Yaw = -atan2( Lecturas_Brujula.Valor.Magnetismo_y * cos_Roll - Lecturas_Brujula.Valor.Magnetismo_z * sin_Roll, Lecturas_Brujula.Valor.Magnetismo_x * cos_Pitch + Lecturas_Brujula.Valor.Magnetismo_y * sin_Roll * sin_Pitch + Lecturas_Brujula.Valor.Magnetismo_z * cos_Roll * sin_Pitch);
	sin_Yaw = arm_sin_f32(Yaw);
	cos_Yaw = arm_cos_f32(Yaw);

	DCM_matriz[0][0] = cos_Pitch*cos_Yaw;
	DCM_matriz[0][1] = cos_Yaw*sin_Roll*sin_Pitch - cos_Roll*sin_Yaw;
	DCM_matriz[0][2] = sin_Roll*sin_Yaw + cos_Roll*cos_Yaw*sin_Pitch;

	DCM_matriz[1][0] = cos_Pitch*sin_Yaw;
	DCM_matriz[1][1] = cos_Roll*cos_Yaw + sin_Roll*sin_Pitch*sin_Yaw;
	DCM_matriz[1][2] = cos_Roll*sin_Pitch*sin_Yaw - cos_Yaw*sin_Roll;

	DCM_matriz[2][0] = -sin_Pitch;
	DCM_matriz[2][1] = cos_Pitch*sin_Roll;
	DCM_matriz[2][2] = cos_Roll*cos_Pitch;

	Escribir_servidor_DCM((float32_t*)DCM_matriz);
	Escribir_servidor_RPY(&Roll, &Pitch, &Yaw);
}

void Algortimo_DCM_MAG(tpAHRS *AHRS){

	Compensacion_Sensor_magnetico(AHRS);
	Actualizar_Matriz_DCM_V2(AHRS);
	Normalizar_DCM(AHRS);
	Correccion_deriva(AHRS);
	Angulos_Euler(AHRS);
}

void Algortimo_DCM_NO_YAW(tpAHRS *AHRS){

	Actualizar_Matriz_DCM_V2(AHRS);
	Normalizar_DCM(AHRS);
	Correccion_deriva_NO_YAW(AHRS);
	Angulos_Euler(AHRS);
}
