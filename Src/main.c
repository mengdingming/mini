/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pstwo.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
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

uint8_t aTxStartMessages[] = "\r\n******UART commucition using IT******\r\nPlease enter 10 characters:\r\n";
uint8_t aRxBuffer[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void int_char(int l, uint8_t *s)
{
    *s = l>>8;
    *(s+1) = l;  
}

void Motor_Speed_Control(int16_t motor1, int16_t motor2)	 
{
    int16_t motor1speed = 0, motor2speed = 0 ;	
    if(motor1>899)  motor1speed =899;
	    else if (motor1<-899)  motor1speed = -899;
			else  motor1speed = motor1;
	if(motor2>899)  motor2speed = 899;
	    else if (motor2<-899)  motor2speed = -899;
			else  motor2speed = motor2;
	if(motor2speed == 0) //刹车
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,motor1speed);
	}
  else if(motor2speed > 0)
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,motor1speed);
	}
	else
	{
		HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,motor1speed);
	}

	if(motor1speed == 0)
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,motor2speed);
	}
	else if(motor1speed > 0)
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,motor2speed);
	}
	else
	{
		HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,motor2speed);
	}
}
unsigned int  absolute(int a)
{
	if(a>=0){
		return a;
	}
	else 
	return -a;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int16_t speed,speed1,speed2; 
	int16_t swerve=0; 
	uint8_t txbuf[50];
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
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	PS2_SetInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//if(1)
		if( !PS2_RedLight()) //判断手柄是否为红灯模式，是，指示灯LED点亮
		{
			HAL_Delay(50);	 //延时很重要不可去
			//LED = 0;
			PS2_DataKey();	 //手柄按键捕获处理
			/*
			speed = (PS2_AnologData(PSS_LY)-128)*7;//meng
			swerve = (PS2_AnologData(PSS_RX)-127)*3;//meng
			speed1 = (speed + swerve);//meng
			speed2 = (speed - swerve);//meng
			Motor_Speed_Control(speed1, speed2);//meng*/
			/*
			Motor_Speed_Control(450, 450);//meng
			//HAL_Delay(5000);
			//Motor_Speed_Control(0, 0);//meng
			//HAL_Delay(500);
			//Motor_Speed_Control(-450, -450);//meng
			//HAL_Delay(5000);
			//Motor_Speed_Control(0, 0);//meng
			//HAL_Delay(500);*/
			
			speed = PS2_AnologData(PSS_LY)-127;	   
			swerve = (PS2_AnologData(PSS_RX)-128)*7*((float)absolute(speed)/128); //	speed取绝对值，	算数运算，得到转弯量。
			speed = -(PS2_AnologData(PSS_LY)-127)*7;	   //正：前进；  负：后退
			if(speed > 0) //向前
			{
				if(swerve < 0)//左转弯
				{
					speed1 = speed + swerve;
					speed2 = speed;
					
				}
				else          //右转弯
				{
					speed1 = speed; 
					speed2 = speed - swerve;;
					Motor_Speed_Control(speed1, speed2);
				}
			}
			else if(speed < 0)//向后
			{
				if(swerve < 0)//左转弯
				{
					speed1 = speed + swerve;
					speed2 = speed;
					Motor_Speed_Control(speed1, speed2);
				}
				else//右转弯
				{
					speed1 = speed; 
					speed2 = speed - swerve;;
					Motor_Speed_Control(speed1, speed2);
				}
			}
			else  Motor_Speed_Control(0, 0); //停止
			int_char(speed1,txbuf);
			//memcpy(txbuf,(char*)speed1,50);//
			//memcpy(txbuf,"这是一个串口中断接收回显实验\n",50);
			HAL_UART_Transmit(&huart1,txbuf,strlen((char *)txbuf),1000);
			memcpy(txbuf,"\n",50);
			HAL_UART_Transmit(&huart1,txbuf,strlen((char *)txbuf),1000);
		}
		else	//判断手柄不是红灯模式，指示灯LED熄灭
		{
			//LED = 1;
			Motor_Speed_Control(0, 0);
			HAL_Delay(50);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
