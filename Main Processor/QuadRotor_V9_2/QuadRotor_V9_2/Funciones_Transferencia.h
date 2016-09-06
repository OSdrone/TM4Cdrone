
#ifndef QUADROTOR_V1_3_1_FUNCIONES_TRANSFERENCIA_H_
#define QUADROTOR_V1_3_1_FUNCIONES_TRANSFERENCIA_H_

#define Jq1 0.8613
#define Jq2 0.8613
#define Jq3 0.8613

#define Jm  0.096369

#define Km 0.009637	   //Fuerza(N) / Accion (microseg)
#define K_sistema 1.8931
#define K_q K_sistema/Km;

#define Wn 10;
#define Wn_2 10;
#define chi 1;

#define Kpr 0.0
/*
Kp1 = Wn^2*Jq1/K;
Kp2 = Wn^2*Jq2/K;
Kp3 = Wn^2*Jq3/K2;

Kv1 = (2*chi*Wn - 1) / K;
Kv2 = (2*chi*Wn - 1) / K;
Kv3 = (2*chi*Wn - 1) / K2;
 */

/*
#define Kp1 0
#define Kv1 0

#define Kp2 0
#define Kv2 0
*/

#define Kp1 15.0
#define Kv1 2.5

#define Kp2 15.0
#define Kv2 2.5

#define Kp3 0.0
#define Kv3 0.0

#define Ki 0.02
#define Ki_EST 2


const float32_t F_matriz[] = {
		   0.994211639269529,                   0,                   0,                   0,                   0,                   0,   1.108053901852224,                   0,                   0,                   0,
		   0.004985515097154,   1.000000000000000,                   0,                   0,                   0,                   0,   0.002796838162984,                   0,                   0,                   0,
		                   0,                   0,   0.994211639269529,                   0,                   0,                   0,                   0,   1.108053901852224,                   0,                   0,
		                   0,                   0,   0.004985515097154,   1.000000000000000,                   0,                   0,                   0,   0.002796838162984,                   0,                   0,
		                   0,                   0,                   0,                   0,   0.994211639269529,                   0,                   0,                   0,   0.166208085277834,                   0,
		                   0,                   0,                   0,                   0,   0.004985515097154,   1.000000000000000,                   0,                   0,   0.000419525724448,                   0,
		                   0,                   0,                   0,                   0,                   0,                   0,   0.949439085978879,                   0,                   0,                   0,
		                   0,                   0,                   0,                   0,                   0,                   0,                   0,   0.949439085978879,                   0,                   0,
		                   0,                   0,                   0,                   0,                   0,                   0,                   0,                   0,   0.949439085978879,                   0,
		                   0,                   0,                   0,                   0,                   0,                   0,                   0,                   0,                   0,   0.949439085978879
};
const arm_matrix_instance_f32 F = {10, 10, (float32_t *)F_matriz};


const float32_t G_matriz[] = {
		   0.000279686718516044,                      0,  -0.000279686718516044,                      0,
		   0.000000468381516574,                      0,  -0.000000468381516574,                      0,
		                      0,   0.000279686718516044,                      0,  -0.000279686718516044,
		                      0,   0.000000468381516574,                      0,  -0.000000468381516574,
		   0.000041953007777407,  -0.000041953007777407,   0.000041953007777407,  -0.000041953007777407,
		   0.000000070257227486,  -0.000000070257227486,   0.000000070257227486,  -0.000000070257227486,
		   0.000487255528421544,                      0,  -0.000487255528421544,                      0,
		                      0,   0.000487255528421544,                      0,  -0.000487255528421544,
		   0.000487255528421544,  -0.000487255528421544,   0.000487255528421544,  -0.000487255528421544,
		   0.000487255528421544,   0.000487255528421544,   0.000487255528421544,   0.000487255528421544
};
const arm_matrix_instance_f32 G = {10, 4, (float32_t *)G_matriz};


const float32_t Gp_matriz[] = {

		  -0.029022176872060,                   0,                   0,                   0,
		  -0.000048602419485,                   0,                   0,                   0,
		                   0,  -0.029022176872060,                   0,                   0,
		                   0,  -0.000048602419485,                   0,                   0,
		                   0,                   0,  -0.004353326530809,                   0,
		                   0,                   0,  -0.000007290362923,                   0,
		  -0.050560914021121,                   0,                   0,                   0,
		                   0,  -0.050560914021121,                   0,                   0,
		                   0,                   0,  -0.050560914021121,                   0,
		                   0,                   0,                   0,  -0.050560914021121

};
const arm_matrix_instance_f32 Gp = {10, 4, (float32_t *)Gp_matriz};


const float32_t K_4_matriz[] = {  Kv1/2.0,   Kp1/2.0,        0,         0,    Kv3/4.0,   Kp3/4.0,   Kpr/2.0,         0,    Kpr/4.0,    Kpr/4.0,
								        0,         0,  Kv2/2.0,   Kp2/2.0,   -Kv3/4.0,  -Kp3/4.0,         0,   Kpr/2.0,   -Kpr/4.0,    Kpr/4.0,
							     -Kv1/2.0,  -Kp1/2.0,        0,         0,    Kv3/4.0,   Kp3/4.0,  -Kpr/2.0,         0,    Kpr/4.0,    Kpr/4.0,
							   	        0,         0, -Kv2/2.0,  -Kp2/2.0,   -Kv3/4.0,  -Kp3/4.0,         0,  -Kpr/2.0,   -Kpr/4.0,    Kpr/4.0   };
const arm_matrix_instance_f32 K_4 = {4, 10, (float32_t *)K_4_matriz};

const float32_t K_3_matriz[] = {  Kv1/2,   Kp1/2,       0,       0,     Kv3/4,   Kp3/4,         0,         0,   0,    0,
							   	      0,       0,   Kv2/2,   Kp2/2,   - Kv3/4,  -Kp3/4,         0,         0,   0,    0,
					    	     -Kv1/2,  -Kp1/2,       0,       0,     Kv3/4,   Kp3/4,         0,         0,   0,    0,
							   	      0,       0,  -Kv2/2,  -Kp2/2,   - Kv3/4,  -Kp3/4,         0,         0,   0,    0    };
const arm_matrix_instance_f32 K_3 = {4, 10, (float32_t *)K_3_matriz};

/*
const float32_t K_pre_4_matriz [] = {       Kp1/2.0,         0,   Kp3/4.0,    Kpr/4.0*(1.0+1.0/(Km*Kpr)),
											      0,   Kp2/2.0,  -Kp3/4.0,    Kpr/4.0*(1.0+1.0/(Km*Kpr)),
									       -Kp1/2.0,         0,   Kp3/4.0,    Kpr/4.0*(1.0+1.0/(Km*Kpr)),
									   	   	      0,  -Kp2/2.0,  -Kp3/4.0,    Kpr/4.0*(1.0+1.0/(Km*Kpr))   };
*/
const float32_t K_pre_4_matriz [] = {       Kp1/2.0,         0,   Kp3/4.0,    0,
											      0,   Kp2/2.0,  -Kp3/4.0,    0,
									       -Kp1/2.0,         0,   Kp3/4.0,    0,
									   	   	      0,  -Kp2/2.0,  -Kp3/4.0,    0   };

const arm_matrix_instance_f32 K_pre_4 = {4, 4, (float32_t *)K_pre_4_matriz};

const float32_t K_pre_3_matriz [] = {       Kp1/2,       0,   Kp3/4,    1,
											    0,   Kp2/2,  -Kp3/4,    1,
									       -Kp1/2,       0,   Kp3/4,    1,
									       	    0,  -Kp2/2,  -Kp3/4,    1   };
const arm_matrix_instance_f32 K_pre_3 = {4, 4, (float32_t *)K_pre_3_matriz};

const float32_t Lo_per_matriz[] = {

		   0.198434923933560,  -0.006363850507977,  -0.007659457807024,  -0.027653725397731,  -0.040303351039343,   0.032195672338781,                   0,                   0,                   0,  -0.119404737466306,
		   0.001086363950669,   0.188835322750985,  -0.016657957077870,  -0.024043417453002,  -0.020908295732614,  -0.011056332513453,                   0,                   0,                   0,   0.001170982384024,
		  -0.005045608483105,  -0.048871496093713,   0.157744315915222,  -0.041854699495902,   0.044350331784821,  -0.024550313312358,                   0,                   0,                   0,   0.016240261993865,
		  -0.005309682991509,  -0.023052431159453,  -0.007214113163155,   0.201894030185689,  -0.019095064558748,  -0.004329015112383,                   0,                   0,                   0,   0.004022347367876,
		  -0.002378769745064,  -0.002178446370954,   0.002055236651581,  -0.002107845235214,   0.111605875609966,   0.002983178665231,                   0,                   0,                   0,   0.004293308919750,
		   0.003865396849945,  -0.011754342406059,  -0.008725820535339,  -0.005326913222497,   0.035796312502292,   0.207633665528118,                   0,                   0,                   0,  -0.002449943423081,
		   0.037373088816668,   0.008171926131680,  -0.000749832805752,   0.012867191640016,   0.003870818022841,  -0.007501931764926,                   0,                   0,                   0,  -0.065266474827869,
		  -0.000255055739259,   0.049442260602111,   0.037401895624638,   0.033213986376183,  -0.001506645855643,   0.025470558191721,                   0,                   0,                   0,   0.001253319942105,
		   0.000379721483837,   0.022713862804404,  -0.000356562869897,   0.021199489086857,   0.258298575110510,  -0.036706281736491,                   0,                   0,                   0,   0.033732925270074,
		   0.021869479811627,   0.001408963971122,  -0.000641233000648,   0.010283330987783,  -0.030403824233784,  -0.009898280860548,                   0,                   0,                   0,   0.217541943060802

};
const arm_matrix_instance_f32 Lo_per = {10, 10, (float32_t *)Lo_per_matriz};

const float32_t Lp_matriz[] = {

		  -0.105325610557535,  -0.002714467035512,   0.0,   			  0.0,   			   0.0,                               0.0,                   0,                   0,                   0,   0.0,
		   0.0, 			   -0.0,  				-0.105325610557535,  -0.002714467035512,  -0.0,                              -0.0,                   0,                   0,                   0,  -0.0,
		   0.0,  			   -0.0,  				-0.0,  				 -0.0,                -0.105325610557535,   -0.002714467035512,                  0,                   0,                   0,  -0.0,
		  -0.0,  			   -0.0,  				 0.0,  				 -0.0,  			   0.0,                               0.0,                   0,                   0,                   0,  -0.278795314926213

/*
		  -0.105325610557535,   0.002714467035512,   0.008732682462314,   0.002559603908597,   0.014552758777108,   0.000528954644569,                   0,                   0,                   0,   0.102919655628366,
		   0.008312078498193,  -0.003819091813717,  -0.075060605398528,  -0.002074987127944,  -0.019057795543476,  -0.005361511672757,                   0,                   0,                   0,  -0.001723807565582,
		   0.004362113688891,  -0.013310664761345,  -0.004605587466494,  -0.014805749036137,  -0.378215610185594,   0.031471607720973,                   0,                   0,                   0,  -0.048012960151969,
		  -0.055502910072872,  -0.000414764856061,   0.005678012486231,  -0.012687401959021,   0.057454318249112,   0.016414143824185,                   0,                   0,                   0,  -0.278795314926213
*/
};
const arm_matrix_instance_f32 Lp = {4, 10, (float32_t *)Lp_matriz};

const float32_t La_matriz[] = {
/*
		 -51.883366192798583,   		        0, -25.941683096399295, -25.941683096399299,
         	 	 	       0, -51.883366192798583,  25.941683096399295, -25.941683096399299,
		  51.883366192798583,                   0, -25.941683096399302, -25.941683096399302,
		                   0,  51.883366192798583,  25.941683096399302, -25.941683096399302
*/

		 -51.883366192798583,   		        0, 0, 0,
        	 	 	       0, -51.883366192798583, 0, 0,
		  51.883366192798583,                   0, 0, 0,
		                   0,  51.883366192798583, 0, 0

};
const arm_matrix_instance_f32 La = {4, 4, (float32_t *)La_matriz};




//Parametros Filtros
#define Kc 1
#define numero_muestras_filtro_media 5


#define Kp_ROLLPITCH 0.00005
#define Ki_ROLLPITCH 0.00000001
//#define Kp_YAW 0.00005
//#define Ki_YAW 0.00000001

#define Kp_YAW 0.00005
#define Ki_YAW 0.00000001

//50Hz
/*
#define num_etapas_Filtro_Vel 2
const float32_t Coeficientes_Filtro_Vel_Valores[5*num_etapas_Filtro_Vel] =  { 0.361615673042922, 2*0.361615673042922, 0.361615673042922,	0,	-0.446462692171690, 0.259891532474145,	2*0.259891532474145, 0.259891532474145,	0,	-0.0395661298965801};
*/

#define num_etapas_Filtro_Per 2
const float32_t Coeficientes_Filtro_Pre_Valores[5*num_etapas_Filtro_Per] =  { 0.361615673042922, 2*0.361615673042922, 0.361615673042922,	0,	-0.446462692171690, 0.259891532474145,	2*0.259891532474145, 0.259891532474145,	0,	-0.0395661298965801};

//Filtro 20Hz
#define num_etapas_Filtro_Vel 2
const float32_t Coeficientes_Filtro_Vel_Valores[5*num_etapas_Filtro_Vel]  =  { 0.00376220298169900,	2*0.00376220298169900,	0.00376220298169900, 1.89341560102250,	-0.908464412949295, 0.00353349592337797, 2*0.00353349592337797,  0.00353349592337797, 1.77831348813944, -0.792447471832947};

#define num_etapas_Filtro_Acel 2
const float32_t Coeficientes_Filtro_Acel_Valores[5*num_etapas_Filtro_Acel] = { 0.00376220298169900,	2*0.00376220298169900,	0.00376220298169900, 1.89341560102250,	-0.908464412949295, 0.00353349592337797, 2*0.00353349592337797,  0.00353349592337797, 1.77831348813944, -0.792447471832947};

#define Coeficientes_Filtro_Mag_Valores {0, 0.0425, 0.0367, 1.565, -0.6442}
#define num_etapas_Filtro_Mag 1

/*
#define num_etapas_Filtro_Per 2
const float32_t Coeficientes_Filtro_Pre_Valores[5*num_etapas_Filtro_Per] =  { 0.0779563405164626, 2*0.0779563405164626, 0.0779563405164626, 1.32091343081943, -0.632738792885277, 0.0618851952997645, 2*0.0618851952997645, 0.0618851952997645, 1.04859957636261, -0.296140357561670 };
*/

//10Hz
/*
#define num_etapas_Filtro_Per 2
const float32_t Coeficientes_Filtro_Per_Valores[5*num_etapas_Filtro_Per] =  { 0.0218838519679430,   2*0.0218838519679430,	0.0218838519679430,	1.70096433194353, -0.788499739815298, 0.0190368315878239, 2*0.0190368315878239,	0.0190368315878239,	1.47967421693119, -0.555821543282489};
*/

#endif /* QUADROTOR_V1_3_1_FUNCIONES_TRANSFERENCIA_H_ */