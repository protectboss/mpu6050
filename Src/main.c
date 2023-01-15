/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MPU6050_ADDR 0xD1


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C

#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW =0;
int16_t Accel_Y_RAW =0;
int16_t Accel_Z_RAW =0;

int16_t Gyro_X_RAW =0;
int16_t Gyro_Y_RAW =0;
int16_t Gyro_Z_RAW =0;

int16_t Temp_RAW =0;

float Ax,Ay,Az,Gx,Gy,Gz,Temp;

void MPU6050_Init(void)
{
uint8_t check,Data;
	
	//检查设备ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,WHO_AM_I_REG,1,&check,1,1000);

	if(check==104)
	{
		
		//电源管理器0x6B我们因该写入所有0来唤醒传感器
		Data=0;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,PWR_MGMT_1_REG,1,&Data,1,1000);
		
		//通过写入SMPLRT_DIV寄存器设置1Khz的数据频率
		Data=0x07;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,SMPLRT_DIV_REG,1,&Data,1,1000);

		//在ACCEL_CONFIG配置寄存器中设置加速计配置
		Data=0x00;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,ACCEL_CONFIG_REG,1,&Data,1,1000);
		
		//在陀螺仪配置寄存器中设置陀螺配置
		Data=0x00;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,GYRO_CONFIG_REG,1,&Data,1,1000);
	}
}

//加速度读取函数
void MPU6050_Read_Accel(void)
{
   uint8_t Rec_Data[6];
	
	 HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,ACCEL_XOUT_H_REG,1,Rec_Data,6,1000);
	
	Accel_X_RAW=(int16_t)(Rec_Data[0]<<8|Rec_Data[1]);
	Accel_Y_RAW=(int16_t)(Rec_Data[2]<<8|Rec_Data[3]);
	Accel_Z_RAW=(int16_t)(Rec_Data[4]<<8|Rec_Data[5]);
	
	Ax=Accel_X_RAW/16384.0;
	Ay=Accel_Y_RAW/16384.0;
	Az=Accel_Z_RAW/16384.0;

}

//角速度读取函数
void MPU6050_Read_Gyro(void)
{
     uint8_t Rec_Data[6];

		 HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,GYRO_XOUT_H_REG,1,Rec_Data,6,1000);
	
	   Gyro_X_RAW=(int16_t)(Rec_Data[0]<<8|Rec_Data[1]);
	   Gyro_Y_RAW=(int16_t)(Rec_Data[2]<<8|Rec_Data[3]);
	   Gyro_Z_RAW=(int16_t)(Rec_Data[4]<<8|Rec_Data[5]);
	
	   Gx=Gyro_X_RAW/131.0;
	   Gy=Gyro_Y_RAW/131.0;
	   Gz=Gyro_Z_RAW/131.0;
}

//温度读取函数
void MPU6050_Read_Temp(void)
{
     uint8_t Rec_Data[6];

     HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,TEMP_OUT_H_REG,1,Rec_Data,2,1000);
	
	   Temp_RAW=(int16_t)(Rec_Data[0]<<8|Rec_Data[1]);
	   Temp=36.53+(Temp_RAW)/340;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
	
	
//	for(uint8_t i=0;i<255;i++)
//	{
//	if(HAL_I2C_IsDeviceReady(&hi2c1,i,1,1000)==HAL_OK)
//	{
//	 printf("%d\n",i);
//	 break;
//	}
//	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		MPU6050_Read_Temp();
		
//		printf("Ax=%.2f,Ay=%.2f,Az=%.2f \n",Ax,Ay,Az);//打印出X,Y,Z的加速度
//		printf("Gx=%.2f,Gy=%.2f,Gy=%.2f \n",Gx,Gy,Gz);//打印出X,Y,Z的角度加速度
//	  printf("Temperature=%.2f \n",Temp);//打印出温度
		printf("%.2f,%.2f,%.2f  \n",Gx,Gy,Gz);//在vofa上显示数据
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
