
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#include "sensors.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
double temperature1, temperature2, presuare1, presuare2;
uint32_t UpPres1, UpPres2;
char message_to_esp[100];
uint8_t Klapan[2];
uint32_t press_limit;
uint8_t stage_of_process;
uint8_t step_of_blow;
static const MODBUS_Std_command getTempPrsCommandensor1 = 
{
	.address = 0xF0,
	.func_code = 0x04,
	.start_index = 0x0000,
	.length = 0x0200,
	.CRC_low = 0x64,
	.CRC_high = 0xEA,
};

static const MODBUS_Std_command getTempPrsCommandSensor2 = 
{
	.address = 0xDE,
	.func_code = 0x04,
	.start_index = 0x0000,
	.length = 0x0200,
	.CRC_low = 0x62,
	.CRC_high = 0xA4,
};




sensorData firstSensorTempPrs;
sensorData secondSensorTempPrs;
command_message command_from_host, command_from_host_now;
uint8_t state;
uint32_t Time_pump, Time_speak;
#ifdef _1_SLAVE_PRESENT
static const uint8_t MODBUS_slaveAddrChange[11] = 
{
	0xF0, 0x10, 0x00, 0x14,
	0x00, 0x01, 0x02, 0x00,
	0xDE, 0x2C, 0x88
};
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void err_handler(void);
void zero_params(sensorData Params)
	{
	Params.address=0;
	Params.func_code=0;
	Params.byte_count=0;
	Params.raw_pressure_high=0;
	Params.raw_pressure_low=0;
	Params.raw_temperature_high=0;
	Params.raw_temperature_low=0;
	Params.CRC_low=0;
	Params.CRC_high=0;
	}
void Init_klapans(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, 	ALL_A_Klapans,GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = ALL_A_Klapans;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, 	ALL_B_Klapans,GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = ALL_B_Klapans;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOC, 	ALL_C_Klapans,GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = ALL_C_Klapans;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOD, 	ALL_D_Klapans,GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = ALL_D_Klapans;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	

}

void test_pins(void)
{
	HAL_GPIO_WritePin(GPIOA, 	ALL_A_Klapans,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, 	ALL_B_Klapans,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, 	ALL_C_Klapans,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, 	ALL_D_Klapans,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB, 	dev_13,GPIO_PIN_SET);


	
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t temp2, temp1;
uint16_t pres1, pres2;
uint8_t ch;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
//  /* USER CODE BEGIN 1 */

//  /* USER CODE END 1 */

//  /* MCU Configuration----------------------------------------------------------*/

//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

//  /* USER CODE BEGIN Init */

//  /* USER CODE END Init */

//  /* Configure the system clock */
  SystemClock_Config();
//  /* USER CODE BEGIN SysInit */
//	
//  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
//  /* USER CODE BEGIN 2 */
	Init_klapans();
	zero_down_system();
	HAL_TIM_Base_Start_IT(&htim1);
	
   
#ifdef _1_SLAVE_PRESENT
	
	HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_SET);
	if(HAL_UART_Transmit(&huart1, (uint8_t*)(&MODBUS_slaveAddrChange), sizeof(MODBUS_slaveAddrChange), 30) != HAL_OK) err_handler();
	HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	
#endif
HAL_UART_Receive_IT(&huart3, (uint8_t*)(&ch), 1);
//HAL_UART_Transmit(&huart3,"HELLO", 5,1); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch_addr, sizeof(ch_addr), 100);
	HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);*/
  while (1)
  {
	
		/* CRC check porc here! */
		
  /* USER CODE END WHILE */
	Timing_main();

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RS_485_TRANSMIT_Pin */
  GPIO_InitStruct.Pin = RS_485_TRANSMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS_485_TRANSMIT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void MATH_Go_to_ESP()
{
if(firstSensorTempPrs.address!=0)
		{
			
		}
if(secondSensorTempPrs.address!=0)
		{
			
		}
		temperature1 = (double)temp1*70/10000;
			UpPres1 =(uint32_t)pres1*2*10;
temperature2 = (double)temp2*70/10000;		
			UpPres2 =(uint32_t)pres2*3*10;
	
presuare1 = (double)pres1*2/10000;
presuare2 = (double)pres2*3/10000;
	
//sprintf(pres_ch,"%i%c%c",pres1_4, 'P','a');
//snprintf(pres_ch,5,"%d",pres1);

		sprintf(message_to_esp, "$p1 = %d Pa, p2 = %d Pa, t = %f C*", UpPres1, UpPres2, temperature2);
    HAL_UART_Transmit(&huart3,(uint8_t*)(&message_to_esp),strlen(message_to_esp)+1,1000);
	
		
}

void err_handler()
{
	
}
uint8_t hui=0;
void take_klapan() //write klapans state by number
{
	
if(Klapan[0])
	{
//	HAL_UART_Transmit(&huart3, Klapan, 2,1); 
switch(Klapan[0])
	{
	case 0x04:
		if(Klapan[1]==0x01)
		{
			
			HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET);
			HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET);
		}
		if(Klapan[1]==0x00)
		{
			HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_SET);
		}
		break;
	case 0x05:
		if(Klapan[1]==0x01)
		{
			HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
			HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
		}
		if(Klapan[1]==0x00)
		{
			HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_SET);
		}
		break;
	case 0x06:
		if(Klapan[1]==0x01)
		{
			HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
			HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
		}
		if(Klapan[1]==0x00)
		{
			
			HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_SET);
		}
		break;
	case 0x09:
		if(Klapan[1]==0x01)
		{
			HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_SET);
		}
		if(Klapan[1]==0x00)
		{
			HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_RESET);
		}
		break;
	case 0x01:
		if(Klapan[1]==0x01)
		{
			HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_SET);
			HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_RESET);
		}
		if(Klapan[1]==0x00)
		{
			
			HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_SET);
		}
		break;
	case 0x02:
		if(Klapan[1]==0x01)
		{
			HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_SET);
		}
		if(Klapan[1]==0x00)
		{
			HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_RESET);
		}
		break;
	case 0x03:
		if(Klapan[1]==0x01)
		{
			HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_SET);
		}
		if(Klapan[1]==0x00)
		{
			HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_RESET);
		}
		break;
		
	}
	
	}	
}
void zero_down_system() // go system to zerostate external
{
		HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET); //open/close klapans on start
		HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_RESET); // open 10,11,13 klapans
	
		HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET); //open/close klapans on start
		HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_SET); // open 10,11,13 klapans
	
		HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_RESET);
		HAL_UART_Transmit(&huart3,"SYSTEM STOP",11,1);
		step_of_blow=0;
		stage_of_process=0;
		//zero_params(firstSensorTempPrs);
		//zero_params(secondSensorTempPrs);
}

void zeroing() //zerostate when all done good
{
				HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_SET); // open 10,11,13 klapans
	
				HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_RESET);
	
				HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_RESET);
				
				step_of_blow=0;
				stage_of_process=0;
			//	zero_params(firstSensorTempPrs);
			//	zero_params(secondSensorTempPrs);
}

void all_zeroes()
{
				HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_RESET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_RESET); // open 10,11,13 klapans
	
				HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_SET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_SET);
	
				HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_RESET);
				
				step_of_blow=0;
				stage_of_process=0;
				zero_params(firstSensorTempPrs);
				zero_params(secondSensorTempPrs);

}

void Parser() // this is command parser 
{
	uint8_t start;
	
	 
			if(command_from_host_now.start==':')
			{
				if((command_from_host_now.powering==0x01)&&(stage_of_process==0)) // if process didn't start when we can write command to system
				{
					stage_of_process = 1;
					command_from_host=command_from_host_now;
					press_limit= command_from_host.pressure_4<<24|command_from_host.pressure_3<<16|command_from_host.pressure_2<<8|command_from_host.pressure_1;
				}
				if(command_from_host_now.powering==0x00) // if need to turn down
					{
					zero_down_system();
					stage_of_process=0;
					}
				if(command_from_host_now.powering==0x02)  // if process didn't start when we can write command to system
					{
						HAL_UART_Transmit(&huart3, (uint8_t*)(&stage_of_process),1,1); 
						stage_of_process++;
					}
					if(command_from_host_now.powering==0x03) //turn off all
					{
					all_zeroes();
					}
			}
			else
			{
				Klapan[0]=command_from_host_now.start;
				Klapan[1]=command_from_host_now.klapan_start;
				take_klapan();
			
			}
			//HAL_UART_Transmit(&huart3, (uint8_t*)(&command_from_host_now),sizeof(command_from_host_now),1); 
		}
	  
	 
	
void extrme_stop()
{
	if(UpPres2>(three_atm-200))
		{
			zero_down_system();
			HAL_UART_Transmit(&huart3,"EXTREME STOP, System Delay 2 min",24,1);
			HAL_Delay(2000);
			
		}

}

void blowing()
{
		if(((UpPres1<(two_atm+200))&&(UpPres1>(two_atm-200)))&&((UpPres2<(two_atm+200))&&(UpPres2>(two_atm-200))))
		{
		HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_RESET); // open 10,11,13 klapans
		HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_SET); // open 10,11,13 klapans
		HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_SET);
		step_of_blow++;
		if(step_of_blow==5)
			{
				HAL_GPIO_WritePin(GPIOB,dev_13,GPIO_PIN_RESET);
				stage_of_process++;
			}
		}
		if((UpPres1<(one_atm+200))&&(UpPres2<(one_atm+200))) //go one_atm
		{
		HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_SET); // open 10,11,13 klapans
		HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_RESET);
		}

}
void process_pump()
{
	extrme_stop();
	if(stage_of_process==1) //stage of  blowing
	{
		switch(command_from_host.klapan_start)
		{
			case 0x01:
				HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_RESET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
			
				HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_SET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
		
			case 0x02:
				HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
			
				HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
			
			case 0x03:
				HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_RESET);
			
			
				HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_SET);
			default:
				HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
			
				HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET); //open/close klapans on start
				HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
		}
	blowing();
		
	}
	if(stage_of_process==2) // go to 0.1 atm
	{
		if(((UpPres1<10333)&&(UpPres1>9933))&&((UpPres2<10433)&&(UpPres2>9833))) //down pressure to 0,1 atm 200pa
			{	
				HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_RESET);
				switch(command_from_host.klapan_start)
					{
					case 0x01:
						HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_RESET); //open/close klapans on start
						HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
					
						HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_SET); //open/close klapans on start
						HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
					case 0x02:
						HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET); //open/close klapans on start
						HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_SET);
					
						HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET); //open/close klapans on start
						HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_SET);
						HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_RESET);
					case 0x03:
						HAL_GPIO_WritePin(dev_4_gp,dev_4,GPIO_PIN_SET); //open/close klapans on start
						HAL_GPIO_WritePin(dev_5_gp,dev_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(dev_6_gp,dev_6,GPIO_PIN_RESET);
					
						HAL_GPIO_WritePin(dev_4_1_gp,dev_4_1,GPIO_PIN_RESET); //open/close klapans on start
						HAL_GPIO_WritePin(dev_5_1_gp,dev_5_1,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(dev_6_1_gp,dev_6_1,GPIO_PIN_SET);
					}
				stage_of_process++; //go next_stage
			}
		else
		{
				HAL_GPIO_WritePin(dev_9_gp,dev_9,GPIO_PIN_SET);//working pump
				HAL_GPIO_WritePin(dev_10_gp,dev_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dev_10_1_gp,dev_10_1,GPIO_PIN_SET);			
		}
	}
	if(stage_of_process==3)	// waiting pressure
	{
		if(((UpPres1-press_limit<200)||(press_limit-UpPres1>200))&&((UpPres2-press_limit<200)||(UpPres2-press_limit>200))) // waiting pressure near press_limit 200Pa
		{
		HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_SET); // if all done, close 11 klapan
		stage_of_process++; // go next_stage
		}
		else
		{
		HAL_GPIO_WritePin(dev_11_gp,dev_11,GPIO_PIN_RESET);//if pressure is not under limit than open 11 klapan
		HAL_GPIO_WritePin(dev_13_gp,dev_13,GPIO_PIN_RESET);
		}
	}
	if(stage_of_process==4) // go gas to camera
	{
		if(((UpPres1<101525)&&(UpPres1>101125))&&((UpPres2<101525)&&(UpPres2>101125)))
			{
				zeroing();
				HAL_UART_Transmit(&huart3,"Process end",11,1);
			}
			
	}
	

}


int h;
void Transmitting(uint8_t k)
{
		if(k==1){k++;}
		HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_SET);
		if (HAL_UART_Transmit(&huart1, (uint8_t*)(&getTempPrsCommandensor1), sizeof(getTempPrsCommandensor1), 30) != HAL_OK) err_handler();
		HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_RESET);
		if (HAL_UART_Receive(&huart1, (uint8_t*)(&firstSensorTempPrs), sizeof(firstSensorTempPrs), 100) != HAL_OK) err_handler();
		
		temp1 =SENSOR_1_TEMPERATURE_PTS ;
		pres1=SENSOR_1_PRESSURE_PTS;  
	if(k==2)
		{k=1;}
		HAL_Delay(20);	
		HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_SET);
		if (HAL_UART_Transmit(&huart1, (uint8_t*)(&getTempPrsCommandSensor2), sizeof(getTempPrsCommandSensor2), 30) != HAL_OK) err_handler();
		HAL_GPIO_WritePin(RS_485_TRANSMIT_GPIO_Port, RS_485_TRANSMIT_Pin, GPIO_PIN_RESET);
		if (HAL_UART_Receive(&huart1, (uint8_t*)(&secondSensorTempPrs), sizeof(secondSensorTempPrs), 100) != HAL_OK) err_handler();
		
		temp2 =SENSOR_2_TEMPERATURE_PTS  ;
		pres2=SENSOR_2_PRESSURE_PTS;  
		HAL_Delay(20);
		MATH_Go_to_ESP();
		Time_speak = HAL_GetTick();
			

}

void pumps(uint32_t Time)
{
	if(state==0&&(Time_pump==0)||(state==0 &&(Time-Time_pump>6000))) //if time_pump 0 or last6 seconds
		{
		HAL_GPIO_WritePin(GPIOC,Klapan_2 ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,Klapan_3 ,GPIO_PIN_RESET);
		Time_pump = HAL_GetTick();
		state++;
		}	
	if((state==1)&&((Time-Time_pump)>3000))
		{
		HAL_GPIO_WritePin(GPIOC,Klapan_2 ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,Klapan_3 ,GPIO_PIN_RESET);
		Time_pump = HAL_GetTick();
		state++;
		}
	if(state==2&&(Time-Time_pump)>1000)
		{
		HAL_GPIO_WritePin(GPIOC,Klapan_2 ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,Klapan_3 ,GPIO_PIN_SET);
		Time_pump = HAL_GetTick();
		state++;
		}
	if(state==3&&(Time-Time_pump)>3000)
		{
		HAL_GPIO_WritePin(GPIOC,Klapan_2 ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,Klapan_3 ,GPIO_PIN_RESET);
		Time_pump = HAL_GetTick();
		state=0;
		}


}
uint8_t chet=1;
void Timing_main()
{
uint32_t Time_now = HAL_GetTick();
	

	
	
  if((Time_now - Time_speak)>3) // if now time is longer than 400 ms than last speaking - ask with sensor
			{
	Transmitting(chet);			
			}
	Parser();
  process_pump();


}


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
