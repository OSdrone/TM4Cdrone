/*
 * Sensores.c
 *
 *  Created on: 11/5/2015
 *      Author: Ruben
 */
#include <ti/drivers/I2C.h>
#include "Sensores.h"

float32_t Normalizar_Grados(float32_t Grados){

	/*
	Grados += 180;
	Grados = fmodf(Grados, 360);
	Grados -= 180;
	 */
	if( Grados > 180.0 ){ Grados -= 360;}
	else if( Grados < -180.0 ){ Grados += 360;}

	return Grados;
}

bool Iniciar_IMU_MPU9250(I2C_Handle I2C, tpIMU9250 IMU92520){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[6] ={IMU_MPU9250_PWR_MGMT_1, 0, 0, 0, 0, 0}; //Ponemos a cero el registro Reg_Power_Managent_1 para arrancar;
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = IMU92520.Direccion_IMU;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 2;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	bufferEscritura[0] = IMU_MPU9250_SMPLRT_DIV;
	bufferEscritura[1] = IMU92520.SMPLRT_DIV;
	bufferEscritura[2] = IMU92520.DLPF_CFG_GYRO;
	bufferEscritura[3] = IMU92520.Ganancia_Gyro << 3;
	if (IMU92520.DLPF_CFG_GYRO == -1){
		bufferEscritura[3] |= 0b00000011;
	}
	bufferEscritura[4] = IMU92520.Ganancia_Acel << 3;
	bufferEscritura[5] = IMU92520.DLPF_CFG_ACEL;
	if (IMU92520.DLPF_CFG_ACEL == -1){
		bufferEscritura[3] |= 0b00000001;
	}

	I2C_Transmision.writeCount = 5;
	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);


	bufferEscritura[0] = IMU_MPU9250_INT_PIN_CFG;
	bufferEscritura[1] = 0x02;

	I2C_Transmision.writeCount = 2;
	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);

	//brujula

	bufferEscritura[0] = IMU_MPU9250_MAG_CTL1;
	bufferEscritura[1] = 0x16;

	I2C_Transmision.writeCount = 2;
	I2C_Transmision.slaveAddress = IMU92520.Direccion_MAG;
	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);

	return(TransmisionOK);
}

bool Leer_IMU_MPU9250(I2C_Handle I2C, tpIMU9250 IMU92520, tpLecturas_9DOF_IMU *Lecturas_9DOF_IMU){
	I2C_Transaction I2C_Transmision;

	uint8_t bufferEscritura[] = {IMU_MPU6050_ACCEL_XOUT_H};
	uint8_t bufferLectura[14];
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = IMU92520.Direccion_IMU;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
//	I2C_Transmision.readBuf = &(Lecturas_IMU->Reg.x_acel_h);
	I2C_Transmision.readCount = 14;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	Lecturas_9DOF_IMU->Reg.x_acel_h=bufferLectura[0];
	Lecturas_9DOF_IMU->Reg.x_acel_l=bufferLectura[1];
	Lecturas_9DOF_IMU->Reg.y_acel_h=bufferLectura[2];
	Lecturas_9DOF_IMU->Reg.y_acel_l=bufferLectura[3];
	Lecturas_9DOF_IMU->Reg.z_acel_h=bufferLectura[4];
	Lecturas_9DOF_IMU->Reg.z_acel_l=bufferLectura[5];

	Lecturas_9DOF_IMU->Reg.temp_h=bufferLectura[6];
	Lecturas_9DOF_IMU->Reg.temp_l=bufferLectura[7];

	Lecturas_9DOF_IMU->Reg.x_vel_h=bufferLectura[8];
	Lecturas_9DOF_IMU->Reg.x_vel_l=bufferLectura[9];
	Lecturas_9DOF_IMU->Reg.y_vel_h=bufferLectura[10];
	Lecturas_9DOF_IMU->Reg.y_vel_l=bufferLectura[11];
	Lecturas_9DOF_IMU->Reg.z_vel_h=bufferLectura[12];
	Lecturas_9DOF_IMU->Reg.z_vel_l=bufferLectura[13];

	I2C_Transmision.slaveAddress = IMU92520.Direccion_MAG;
	I2C_Transmision.readCount = 6;
	I2C_Transmision.readBuf = &Lecturas_9DOF_IMU->Reg.x_mag_l;

	bufferEscritura[0] = IMU_MPU9250_MAG_HXL;

	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);
	return(TransmisionOK);
}

bool Iniciar_IMU_MPU6050(I2C_Handle I2C, tpIMU6050 IMU6050){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[5] ={IMU_MPU6050_PWR_MGMT_1, 0, 0, 0, 0}; //Ponemos a cero el registro Reg_Power_Managent_1 para arrancar;
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = IMU6050.Direccion;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 2;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	bufferEscritura[0] = IMU_MPU6050_SMPLRT_DIV;
	bufferEscritura[1] = IMU6050.SMPLRT_DIV;
	bufferEscritura[2] = IMU6050.DLPF_CFG;
	bufferEscritura[3] = IMU6050.Ganancia_Gyro << 3;
	bufferEscritura[4] = IMU6050.Ganancia_Acel << 3;

	I2C_Transmision.writeCount = 5;

	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);
	return(TransmisionOK);
}

bool Leer_IMU_MPU6050(I2C_Handle I2C, tpIMU6050 IMU6050, tpLecturas_IMU *Lecturas_IMU){
	I2C_Transaction I2C_Transmision;

	uint8_t bufferEscritura[] = {IMU_MPU6050_ACCEL_XOUT_H};
	uint8_t bufferLectura[14];
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = IMU6050.Direccion;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
//	I2C_Transmision.readBuf = &(Lecturas_IMU->Reg.x_acel_h);
	I2C_Transmision.readCount = 14;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	Lecturas_IMU->Reg.x_acel_h=bufferLectura[0];
	Lecturas_IMU->Reg.x_acel_l=bufferLectura[1];
	Lecturas_IMU->Reg.y_acel_h=bufferLectura[2];
	Lecturas_IMU->Reg.y_acel_l=bufferLectura[3];
	Lecturas_IMU->Reg.z_acel_h=bufferLectura[4];
	Lecturas_IMU->Reg.z_acel_l=bufferLectura[5];

	Lecturas_IMU->Reg.temp_h=bufferLectura[6];
	Lecturas_IMU->Reg.temp_l=bufferLectura[7];

	Lecturas_IMU->Reg.x_vel_h=bufferLectura[8];
	Lecturas_IMU->Reg.x_vel_l=bufferLectura[9];
	Lecturas_IMU->Reg.y_vel_h=bufferLectura[10];
	Lecturas_IMU->Reg.y_vel_l=bufferLectura[11];
	Lecturas_IMU->Reg.z_vel_h=bufferLectura[12];
	Lecturas_IMU->Reg.z_vel_l=bufferLectura[13];

	//Comprobar a pasar el puntero de lectura la direccion de Lecturas_IMU

	return(TransmisionOK);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Iniciar_Brujula_HMC5883L(I2C_Handle I2C, tpHMC5883L HMC5883L){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[4] = {0, 0, 0, 0};


	I2C_Transmision.slaveAddress = HMC5883L_DIR;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 4;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	bufferEscritura[0] = HMC5883L_CONFIG_A;
	bufferEscritura[1] = (HMC5883L.Muestras_Media << 4) | (HMC5883L.ODR << 2) | HMC5883L.Modo_Medida;
	bufferEscritura[2] = HMC5883L.Ganancia << 5;
	bufferEscritura[3] = (HMC5883L.Velocidad_I2C << 7) | HMC5883L.Modo_Operacion;

	return(I2C_transfer(I2C, &I2C_Transmision));
}

bool Leer_Brujula_HMC5883L(I2C_Handle I2C, tpHMC5883L HMC5883L, tpLecturas_Brujula *Lecturas_Brujula){
	I2C_Transaction I2C_Transmision;
    uint8_t bufferEscritura[] = {HMC5883L_DATA_OUTPUT_X};
	uint8_t bufferLectura[6];
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = HMC5883L_DIR;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
	I2C_Transmision.readCount = 6;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	Lecturas_Brujula->Reg.Magnetismo_x_h = bufferLectura[0];
	Lecturas_Brujula->Reg.Magnetismo_x_l = bufferLectura[1];
	Lecturas_Brujula->Reg.Magnetismo_z_h = bufferLectura[2];
	Lecturas_Brujula->Reg.Magnetismo_z_l = bufferLectura[3];
	Lecturas_Brujula->Reg.Magnetismo_y_h = bufferLectura[4];
	Lecturas_Brujula->Reg.Magnetismo_y_l = bufferLectura[5];

	return(TransmisionOK);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Iniciar_Giroscopo_L3G4200(I2C_Handle I2C, tpGiroscopo_L3G4200 Giroscopo_L3G4200){
	I2C_Transaction I2C_Transmision;
	bool TransmisionOK;
	uint8_t bufferEscritura[6];

	bufferEscritura[0] = 0x80 | L3G4200_CTRL_REG1;
	bufferEscritura[1] = (Giroscopo_L3G4200.ODR<<6 | Giroscopo_L3G4200.BW_LPF<<4 | 0x0F);
    bufferEscritura[2] = (Giroscopo_L3G4200.HPF_modo << 4 | Giroscopo_L3G4200.BW_HPF);
    bufferEscritura[3] = 0;
    bufferEscritura[4] = (Giroscopo_L3G4200.BDU << 7 | Giroscopo_L3G4200.BLE << 6 | Giroscopo_L3G4200.Ganancia << 4);
    bufferEscritura[5] = ( 1 << 6 | Giroscopo_L3G4200.HPF_activar << 4 | Giroscopo_L3G4200.Modo_Filtro );


	I2C_Transmision.slaveAddress = Giroscopo_L3G4200.Direccion;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 6;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	bufferEscritura[0] = L3G4200_FIFO_CTRL_REG;
	bufferEscritura[1] = Giroscopo_L3G4200.Modo << 5;

	I2C_Transmision.slaveAddress = Giroscopo_L3G4200.Direccion;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 2;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	TransmisionOK |= I2C_transfer(I2C, &I2C_Transmision);

	return(TransmisionOK);
}

bool Leer_Giroscopo_L3G4200(I2C_Handle I2C, tpGiroscopo_L3G4200 Giroscopo_L3G4200, tpLecturas_Giroscopo *Lecturas_Giroscopo){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[] = {L3G4200_OUT_X_L};

	I2C_Transmision.slaveAddress = Giroscopo_L3G4200.Direccion;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = &(Lecturas_Giroscopo->Reg.x_vel_l);
	I2C_Transmision.readCount = 6;

	return( I2C_transfer(I2C, &I2C_Transmision));
}
/////////////////////////////////////////////////////////////////////////////////////////
bool Iniciar_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferLectura[22];
	uint8_t bufferEscritura[] = {Bar_Reg_Eprom_Barometro};
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = Dir_Barometro;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
	I2C_Transmision.readCount = 22;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	LecturasBarometro->AC1 = bufferLectura[0]<<8  | bufferLectura[1];
	LecturasBarometro->AC2 = bufferLectura[2]<<8  | bufferLectura[3];
	LecturasBarometro->AC3 = bufferLectura[4]<<8  | bufferLectura[5];
	LecturasBarometro->AC4 = bufferLectura[6]<<8  | bufferLectura[7];
	LecturasBarometro->AC5 = bufferLectura[8]<<8  | bufferLectura[9];
	LecturasBarometro->AC6 = bufferLectura[10]<<8 | bufferLectura[11];
	LecturasBarometro->B1  = bufferLectura[12]<<8 | bufferLectura[13];
	LecturasBarometro->B2  = bufferLectura[14]<<8 | bufferLectura[15];
	LecturasBarometro->MB  = bufferLectura[16]<<8 | bufferLectura[17];
	LecturasBarometro->MC  = bufferLectura[18]<<8 | bufferLectura[19];
	LecturasBarometro->MD  = bufferLectura[20]<<8 | bufferLectura[21];

	return(TransmisionOK);
}

bool Iniciar_Medida_Temp_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[] = {Bar_Reg_leer_temp, Bar_leer_Temp};

	I2C_Transmision.slaveAddress = Dir_Barometro;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 2;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	return(I2C_transfer(I2C, &I2C_Transmision));
}

bool Leer_Temp_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferLectura[2];
	uint8_t bufferEscritura[] = {Bar_Reg_MSB};
	bool TransmisionOK;
	int32_t X1;
	int32_t X2;
	int32_t B5;


	I2C_Transmision.slaveAddress = Dir_Barometro;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
	I2C_Transmision.readCount = 2;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	LecturasBarometro->UT = bufferLectura[0]<<8 | bufferLectura[1];

	X1 = ((LecturasBarometro->UT - LecturasBarometro->AC6) * LecturasBarometro->AC5) >> 15;
	X2 = LecturasBarometro->MC << 11 / ( X1 + LecturasBarometro->MD);
	B5 = X1 + X2;
	LecturasBarometro->Temperatura = (B5 + 8) / 160.0;

	return(TransmisionOK);
}

bool Iniciar_Medida_Presion_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[] = {Bar_Reg_leer_temp, Bar_leer_Presion};

	I2C_Transmision.slaveAddress = Dir_Barometro;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 2;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	return(I2C_transfer(I2C, &I2C_Transmision));
}

bool Leer_Presion_Barometro(I2C_Handle I2C, uint8_t Dir_Barometro, tpLecturasBarometro *LecturasBarometro){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferLectura[3];
	uint8_t bufferEscritura[] = {Bar_Reg_MSB};
	bool TransmisionOK;

	int32_t X1;
	int32_t X2;
	int32_t X3;
	int32_t B3;
	int32_t B4;
	int32_t B5;
	int32_t B6;
	int32_t B7;

	I2C_Transmision.slaveAddress = Dir_Barometro;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
	I2C_Transmision.readCount = 3;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	LecturasBarometro->UT = (bufferLectura[0]<<16 | bufferLectura[1] | bufferLectura[2])>>5;

	X1 = ((LecturasBarometro->UT - LecturasBarometro->AC6) * LecturasBarometro->AC5) >> 15;
	X2 = LecturasBarometro->MC << 11 / ( X1 + LecturasBarometro->MD);
	B5 = X1 + X2;

	B6 = B5 - 4000;
	X1 = (LecturasBarometro->B2 * (B6*B6 << 12 )) << 11;
	X2 = LecturasBarometro->AC2 * B6 << 11;
	X3 = X1 + X2;
	B3 = ((LecturasBarometro->AC1*4 + X3) << 5) / 4;
	X1 = LecturasBarometro->AC3 * B6 << 13;
	X2 = (LecturasBarometro->B1 * (B6*B6 << 12 )) << 16;
	X3 = (X1 + X2 + 2) / 4;

	B4 = LecturasBarometro->AC4 * (unsigned long)(X3 + 32768) << 15;
	B7 = ((unsigned long)LecturasBarometro->UP -B3 ) * (5000 >> 3);

	if (B7 < 0x80000000) {
		LecturasBarometro->Presion = B7 * 2 / B4;
	}else{
		LecturasBarometro->Presion = (B7 /B4) * 2;
	}

	X1 = (LecturasBarometro->Presion << 8) * (LecturasBarometro->Presion << 8);
	X1 = (X1 * 3038) << 16;
	X2 = (7357 * LecturasBarometro->Presion ) << 16;
	LecturasBarometro->Presion = LecturasBarometro->Presion + (X1 - X2 + 3791) << 4;

	return(TransmisionOK);
}
//////////////////////////////////////////////////////////////////////////////
bool Iniciar_Giroscopio_ITG3200(I2C_Handle I2C, uint8_t Dir_Giroscopo, uint16_t Frecuencia_muestreo, uint8_t Filtro){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[2] = {Reg_DLPF_FS, 0};

	bufferEscritura[1] = ((0x03<<3) | (Filtro & 0x7));

	I2C_Transmision.slaveAddress = Dir_Giroscopo;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 2;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;
/*
	I2C_transfer(I2C, &I2C_Transmision);

	bufferEscritura[0] = Reg_SMPRT_DIV;
	bufferEscritura[1] = 8000/Frecuencia_muestreo - 1;

	I2C_transfer(I2C, &I2C_Transmision);

	bufferEscritura[0] = Reg_CONFIG;
	bufferEscritura[1] = (Filtro & 0x07);
*/
	return(I2C_transfer(I2C, &I2C_Transmision));
}
//bool Sensibilidad_Giroscopio_ITG3200(I2C_Handle I2C , uint8_t Dir_Giroscopo, tpRangoGiroscopo_ITG3200 RangoGiroscopo);
bool Leer_Giroscopio_ITG3200(I2C_Handle I2C, uint8_t Dir_Giroscopo, tpLecturas_Giroscopo *Lecturas_Giroscopo){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[] = {Reg_TEMP_OUT_H};
	uint8_t bufferLectura[8];
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = Dir_Giroscopo;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
//	I2C_Transmision.readBuf = &(Lecturas_IMU->Reg.x_acel_h);
	I2C_Transmision.readCount = 8;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	Lecturas_Giroscopo->Reg.x_vel_h=bufferLectura[2];
	Lecturas_Giroscopo->Reg.x_vel_l=bufferLectura[3];
	Lecturas_Giroscopo->Reg.y_vel_h=bufferLectura[4];
	Lecturas_Giroscopo->Reg.y_vel_l=bufferLectura[5];
	Lecturas_Giroscopo->Reg.z_vel_h=bufferLectura[6];
	Lecturas_Giroscopo->Reg.z_vel_l=bufferLectura[7];

	return(TransmisionOK);
}

bool Iniciar_Barometro_BMP280(I2C_Handle I2C, tpBarometro_BMP280 *Barometro_BMP280, tpLecturasBarometro_BMP280 *LecturasBarometro_BMP280){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[] = {BMP_280_ctrl_meas, 0, 0};
	uint8_t bufferLectura[24];
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = Barometro_BMP280->Direccion;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 3;
	I2C_Transmision.readBuf = NULL;
	I2C_Transmision.readCount = 0;

	bufferEscritura[1] = (Barometro_BMP280->t_sampling << 5) | (Barometro_BMP280->Filtro_BMP280 << 2);
	bufferEscritura[2] = (Barometro_BMP280->Oversampling_Temperatura << 5) | (Barometro_BMP280->Oversampling_Presion<< 2) | (Barometro_BMP280->Modo);

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	bufferEscritura[0] = BMP_280_ctrl_meas;

	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
	I2C_Transmision.readCount = 24;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	LecturasBarometro_BMP280->dig_T1 = bufferLectura[1]<<8 |bufferLectura[0];
	LecturasBarometro_BMP280->dig_T2 = bufferLectura[3]<<8 |bufferLectura[2];
	LecturasBarometro_BMP280->dig_T3 = bufferLectura[5]<<8 |bufferLectura[4];

	LecturasBarometro_BMP280->dig_P1 = bufferLectura[7]<<8 |bufferLectura[6];
	LecturasBarometro_BMP280->dig_P2 = bufferLectura[9]<<8 |bufferLectura[8];
	LecturasBarometro_BMP280->dig_P3 = bufferLectura[11]<<8 |bufferLectura[10];
	LecturasBarometro_BMP280->dig_P4 = bufferLectura[13]<<8 |bufferLectura[12];
	LecturasBarometro_BMP280->dig_P5 = bufferLectura[15]<<8 |bufferLectura[14];
	LecturasBarometro_BMP280->dig_P6 = bufferLectura[17]<<8 |bufferLectura[16];
	LecturasBarometro_BMP280->dig_P7 = bufferLectura[19]<<8 |bufferLectura[18];
	LecturasBarometro_BMP280->dig_P8 = bufferLectura[21]<<8 |bufferLectura[20];
	LecturasBarometro_BMP280->dig_P9 = bufferLectura[23]<<8 |bufferLectura[22];


	return TransmisionOK;
}
bool Leer_Barometro_BMP280(I2C_Handle I2C, tpBarometro_BMP280 *Barometro_BMP280, tpLecturasBarometro_BMP280 *LecturasBarometro_BMP280){
	I2C_Transaction I2C_Transmision;
	uint8_t bufferEscritura[] = {BMP_280_ctrl_meas};
	uint8_t bufferLectura[6];
	bool TransmisionOK;

	I2C_Transmision.slaveAddress = Barometro_BMP280->Direccion;
	I2C_Transmision.writeBuf = bufferEscritura;
	I2C_Transmision.writeCount = 1;
	I2C_Transmision.readBuf = bufferLectura;
	I2C_Transmision.readCount = 6;

	TransmisionOK = I2C_transfer(I2C, &I2C_Transmision);

	LecturasBarometro_BMP280->Presion = bufferEscritura[0]<<12 | bufferEscritura[1]<<8 | bufferEscritura[2]>>3 ;
	LecturasBarometro_BMP280->Temperatura = bufferEscritura[3]<<12 | bufferEscritura[4]<<8 | bufferEscritura[5]>>3 ;

	return TransmisionOK;
}

float32_t Conversion_Temperatura(tpLecturasBarometro_BMP280 *LecturasBarometro_BMP280){
	float32_t var1, var2;

	var1 = (LecturasBarometro_BMP280->Temperatura/16384.0 - LecturasBarometro_BMP280->dig_T1/1024.0) * LecturasBarometro_BMP280->dig_T2;
	var2 = ( (LecturasBarometro_BMP280->Temperatura/131072.0 - LecturasBarometro_BMP280->dig_T1/8192.0) * (LecturasBarometro_BMP280->Temperatura/131072.0 - LecturasBarometro_BMP280->dig_T1/8192.0) ) * LecturasBarometro_BMP280->dig_T3 ;

	return ((var1 + var2) / 5120.0);
}

float32_t Conversion_Altura(float32_t Temperatura, tpLecturasBarometro_BMP280 *LecturasBarometro_BMP280){
	float32_t var1, var2, p;

	var1 = Temperatura * 2560.0 - 64000.0;
	var2 = var1 * var1 * LecturasBarometro_BMP280->dig_P6 / 32768.0;
	var2 = var2 + var1 * LecturasBarometro_BMP280->dig_P5 * 2.0;
	var2 = var2/4.0 + LecturasBarometro_BMP280->dig_P4 * 65536.0;
	var1 = (LecturasBarometro_BMP280->dig_P3 * var1 * var1 / 524288.0 + LecturasBarometro_BMP280->dig_P2 * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*LecturasBarometro_BMP280->dig_P1;

	if (var1 == 0.0)	return 0; // avoid exception caused by division by zero

	p = 1048576.0 - LecturasBarometro_BMP280->Presion;
	p = (p - var2/4096.0) * 6250.0 / var1;
	var1 = LecturasBarometro_BMP280->dig_P9 * p * p / 2147483648.0;
	var2 = p * LecturasBarometro_BMP280->dig_P8 / 32768.0;
	p = p + (var1 + var2 + LecturasBarometro_BMP280->dig_P7) / 16.0;

	return p;



}
