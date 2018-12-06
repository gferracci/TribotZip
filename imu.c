#include "stm32l0xx_hal.h"

// Global variables needed at every step when calling th IMU
uint16_t acc_off[3]; 
uint16_t gyr_off[3];
uint16_t temp_off;

void IMU_who_am_i()
{
	uint8_t address=IMU_read(0x75);
	if(address==0x12)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		HAL_Delay(1000);
	}
	else 
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	}	
}

void IMU_init()
{
	IMU_write(0x6B, 0x00); //PWR_MGMT_1
	IMU_write(0x19, 0x05); //SMPLRT_DIV:: update rate of data register: smplr_rate=internal_smplr (1kHz)/(1+smplr_div)
	uint8_t config, fifo_mode, ext_sync_set, dlpf_cfg;
	fifo_mode=0; //bit 6
	ext_sync_set=0<<3; // bit 5-3
	dlpf_cfg=1;
	config=fifo_mode+ext_sync_set+dlpf_cfg;
	IMU_write(0x1A, config); 
	
	uint8_t gyro_config, xg_st, yg_st, zg_st, fs_sel, fchoice_b;
	xg_st=0<<7; //bit 7
	yg_st=0<<6; //bit 6
	zg_st=0<<5; //bit 5
	fs_sel=SENS_GYR<<3; // bit 4-3: :: +/- 250*2^Y dps
	fchoice_b=0;
	gyro_config=xg_st+yg_st+zg_st+fs_sel+fchoice_b;
	IMU_write(0x1B, gyro_config); 

	uint8_t accel_config, xa_st, ya_st, za_st, accel_fs_sel;
	xa_st=0<<7; //bit 7
	ya_st=0<<6; //bit 6
	za_st=0<<5; //bit 5
	accel_fs_sel=SENS_ACC<<3; //bit 4-3: +/- 2^(Y+1) g	
	accel_config=xa_st+ya_st+za_st+accel_fs_sel;
	IMU_write(0x1C, accel_config);
	
	uint8_t accel_config2, dec2_cfg, accel_fchoice_b, a_dlfp_cfg;
	dec2_cfg=1<<4; //bit 5-4: average 2^(2+X) samples for low power acc
	accel_fchoice_b=0<<3; // bit 3
	a_dlfp_cfg=2; // bit 2-0
	accel_config2=dec2_cfg+accel_fchoice_b+a_dlfp_cfg;
	IMU_write(0x1D, accel_config2);
}


int16_t IMU_get_data(short acc)
{
	uint8_t hi, lo;
	int16_t acc_data;
	switch(acc)
	{
		case ACC_X:
			lo=IMU_read(0x3C);
			hi=IMU_read(0x3B)<<8;
			break;
		case ACC_Y:
			lo=IMU_read(0x3E);
			hi=IMU_read(0x3D)<<8;
			break;
		case ACC_Z:
			lo=IMU_read(0x40);
			hi=IMU_read(0x3F)<<8;
			break;
		case GYR_X:
			lo=IMU_read(0x44);
			hi=IMU_read(0x43)<<8;
			break;
		case GYR_Y:
			lo=IMU_read(0x46);
			hi=IMU_read(0x45)<<8;
			break;
		case GYR_Z:
			lo=IMU_read(0x48);
			hi=IMU_read(0x47)<<8;
			break;
		case TEMP:
			lo=IMU_read(0x42);
			hi=IMU_read(0x41)<<8;
			break;
	}
	acc_data=hi+lo;
	return acc_data;
}

int16_t IMU_get_xa()
{
	int16_t xa;
	xa=IMU_get_data(ACC_X);
	return xa;
}

int16_t IMU_get_ya()
{
	int16_t ya;
	ya=IMU_get_data(ACC_Y);
	return ya;
}

int16_t IMU_get_za()
{
	int16_t za;
	za=IMU_get_data(ACC_Z);
	return za;
}

int16_t IMU_get_xg()
{
	int16_t xg;
	xg=IMU_get_data(GYR_X);
	return xg;
}

int16_t IMU_get_yg()
{
	int16_t yg;
	yg=IMU_get_data(GYR_Y);
	return yg;
}

int16_t IMU_get_zg()
{
	int16_t zg;
	zg=IMU_get_data(GYR_Z);
	return zg;
}

int16_t IMU_get_temp()
{
	int16_t temp;
	temp=IMU_get_data(TEMP);
	return temp;
}

uint16_t* IMU_raw_values()
{
	uint16_t IMU_val[7];
	IMU_val[ACC_X]=IMU_get_xa();
	IMU_val[ACC_Y]=IMU_get_ya();
	IMU_val[ACC_Z]=IMU_get_za();
	IMU_val[GYR_X]=IMU_get_xg();
	IMU_val[GYR_Y]=IMU_get_yg();
	IMU_val[GYR_Z]=IMU_get_zg();
	IMU_val[TEMP]=IMU_get_temp();
	return IMU_val;
}

bool IMU_check()
{
	float check[3], norm;
	
	check[X_COOR]=IMU_get_xa()*pow(2, SENS_ACC)/16384;
	check[Y_COOR]=IMU_get_ya()*pow(2, SENS_ACC)/16384;
	check[Z_COOR]=IMU_get_za()*pow(2, SENS_ACC)/16384;
	
	norm=sqrt(pow(check[X_COOR],2)+pow(check[Y_COOR],2)+pow(check[Z_COOR],2));
	if((norm<1+EPS) && (norm>1-EPS)) return TRUE;
	else return FALSE;
}
void IMU_offsets()
{
	uint16_t *val, acc[3], gyr[3], temp;
	short nb_samples=20, i;
	for(i=0; i<nb_samples; i++)
	{
		val=IMU_raw_values();
		
		acc[X_COOR]=acc[X_COOR]+val[ACC_X];
		acc[Y_COOR]=acc[Y_COOR]+val[ACC_Y];
		acc[Z_COOR]=acc[Z_COOR]+val[ACC_Z];
		
		gyr[X_COOR]=gyr[X_COOR]+val[GYR_X];
		gyr[Y_COOR]=gyr[Y_COOR]+val[GYR_Y];
		gyr[Z_COOR]=gyr[Z_COOR]+val[GYR_Z];
		
		temp=temp+val[TEMP];
	}
	
	for(i=0;i<3;i++)
	{
		acc[i]=acc[i]/nb_samples;
		gyr[i]=gyr[i]/nb_samples;
	}
	temp=temp/nb_samples;
	
	acc_off[X_COOR]=acc[X_COOR];
	acc_off[Y_COOR]=acc[Y_COOR]+G*16384/pow(2, SENS_ACC);
	acc_off[Z_COOR]=acc[Z_COOR];
	
	gyr_off[X_COOR]=gyr[X_COOR];
	gyr_off[Y_COOR]=gyr[Y_COOR];
	gyr_off[Z_COOR]=gyr[Z_COOR];
	
	temp_off=temp;
}

float* IMU_values()
{
	uint16_t *IMU_raw;
	float *IMU_conv;
	//Get the IMU values
	IMU_raw=IMU_raw_values();
	
	//Substract the offsets
	IMU_raw[ACC_X]=IMU_raw[ACC_X]-acc_off[X_COOR];
	IMU_raw[ACC_Y]=IMU_raw[ACC_Y]-acc_off[Y_COOR];
	IMU_raw[ACC_Z]=IMU_raw[ACC_Z]-acc_off[Z_COOR];
	
	IMU_raw[GYR_X]=IMU_raw[GYR_X]-gyr_off[X_COOR];
	IMU_raw[GYR_Y]=IMU_raw[GYR_Y]-gyr_off[Y_COOR];
	IMU_raw[GYR_Z]=IMU_raw[GYR_Z]-gyr_off[Z_COOR];
	
	IMU_raw[TEMP]=IMU_raw[TEMP]-temp_off;
	
	//convert to explicit units
	IMU_conv[ACC_X]=IMU_raw[ACC_X]*pow(2, SENS_ACC)/16384*G;// m/s^2
	IMU_conv[ACC_Y]=IMU_raw[ACC_Y]*pow(2, SENS_ACC)/16384*G;// m/s^2
	IMU_conv[ACC_Z]=IMU_raw[ACC_Z]*pow(2, SENS_ACC)/16384*G;// m/s^2
	
	IMU_conv[GYR_X]=IMU_raw[GYR_X]*W0*pow(2, SENS_GYR+1)/65536;// dps
	IMU_conv[GYR_Y]=IMU_raw[GYR_Y]*W0*pow(2, SENS_GYR+1)/65536;// dps
	IMU_conv[GYR_Z]=IMU_raw[GYR_Z]*W0*pow(2, SENS_GYR+1)/65536;// dps
	
	IMU_conv[TEMP]=IMU_raw[TEMP]*326.8+T_AMB;//°C
	
	return IMU_conv;
}

