
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
I2C_HandleTypeDef hi2c2;
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*enum IMU_acc_scale
{
	ACC_2G=0,
	ACC_4G,
	ACC_8G,
	ACC_16G
};

enum IMU_gyr_scale
{
	GYR_250=0, //in dps
	GYR_500,
	GYR_1000,
	GYR_2000
};*/

uint8_t IMU_read(uint8_t reg)
{
	uint8_t i2cData=0;
	HAL_I2C_Master_Transmit(&hi2c2, 0xD0, &reg, 1, 10);
	HAL_I2C_Master_Receive(&hi2c2, 0xD0, &i2cData, 1,10);
	return i2cData;
}

void IMU_write(uint8_t reg, uint8_t message)
{
	uint8_t i2cData[2];
	i2cData[0]=reg;
	i2cData[1]=message;
	HAL_I2C_Master_Transmit(&hi2c2, 0xD0, i2cData, 2, 10);
}

/*void IMU_who_am_i()
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
}*/

/*void IMU_init()
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
	fs_sel=GYR_250<<3; // bit 4-3: :: +/- 250*2^Y dps
	fchoice_b=0;
	gyro_config=xg_st+yg_st+zg_st+fs_sel+fchoice_b;
	IMU_write(0x1B, gyro_config); 

	uint8_t accel_config, xa_st, ya_st, za_st, accel_fs_sel;
	xa_st=0<<7; //bit 7
	ya_st=0<<6; //bit 6
	za_st=0<<5; //bit 5
	accel_fs_sel=ACC_2G<<3; //bit 4-3: +/- 2^(Y+1) g	
	accel_config=xa_st+ya_st+za_st+accel_fs_sel;
	IMU_write(0x1C, accel_config);
	
	uint8_t accel_config2, dec2_cfg, accel_fchoice_b, a_dlfp_cfg;
	dec2_cfg=1<<4; //bit 5-4: average 2^(2+X) samples for low power acc
	accel_fchoice_b=0<<3; // bit 3
	a_dlfp_cfg=2; // bit 2-0
	accel_config2=dec2_cfg+accel_fchoice_b+a_dlfp_cfg;
	IMU_write(0x1D, accel_config2);
}*/
/*int16_t IMU_get_data(short acc)
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
}*/


/*int16_t IMU_get_xa()
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
} */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	if (HAL_I2C_IsDeviceReady(&hi2c2, 0xD0, 2, 10) == HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	}	
	
	IMU_who_am_i();
	IMU_init();
	if(IMU_check()) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	if (HAL_I2C_IsDeviceReady(&hi2c2, 0xD0, 2, 10) == HAL_OK)
	{
		//xa=IMU_get_xa();
		//if(xa!=0) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		//printf("accelerometer x is: %d\n", xa);
	}
	HAL_Delay(500);

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000000;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
