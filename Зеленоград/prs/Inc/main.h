/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "stdint.h"

/* USER CODE END Includes */



/* Private define ------------------------------------------------------------*/

#define RS_485_TRANSMIT_Pin GPIO_PIN_8
#define RS_485_TRANSMIT_GPIO_Port GPIOA

		/*IDC1*/
/*GPIOC*/
#define Klapan_1 GPIO_PIN_10
#define Klapan_2 GPIO_PIN_11
#define Klapan_3 GPIO_PIN_12
/*GPIOD*/
#define Klapan_4 GPIO_PIN_2
/*GPIOA*/
#define Klapan_5 GPIO_PIN_15	
#define Klapan_6 GPIO_PIN_13 

		/*IDC3*/
/*GPIOC*/
#define Klapan_7 GPIO_PIN_14
#define Klapan_10 GPIO_PIN_15
#define Klapan_12 GPIO_PIN_13
/*GPIOA*/
#define Klapan_8 GPIO_PIN_15
#define Klapan_11 GPIO_PIN_0
/*GPIOB*/
#define Klapan_9 GPIO_PIN_7

		/*IDC4*/
/*GPIOA*/
#define Klapan_13 GPIO_PIN_1
#define Klapan_15 GPIO_PIN_4
/*GPIOB*/
#define Klapan_14 GPIO_PIN_0
/*GPIOD*/
#define Klapan_16 GPIO_PIN_0
#define Klapan_17 GPIO_PIN_1
/*GPIOC*/
#define Klapan_18 GPIO_PIN_2

		/*IDC6*/
/*GPIOC*/
#define Klapan_19 GPIO_PIN_9
#define Klapan_23 GPIO_PIN_6
#define Klapan_24 GPIO_PIN_5
#define Klapan_22 GPIO_PIN_5
/*GPIOB*/
#define Klapan_20 GPIO_PIN_8
#define Klapan_21 GPIO_PIN_9

		/*IDC8*/
/*GPIOA*/
#define Klapan_25 GPIO_PIN_6
#define Klapan_26 GPIO_PIN_7
#define Klapan_28 GPIO_PIN_11
/*GPIOB*/
#define Klapan_27 GPIO_PIN_6
#define Klapan_29 GPIO_PIN_12
/*NC*/
#define Klapan_30 GPIO_PIN_19 

			/*IDC9*/
/*GPIOB*/
#define Klapan_31 GPIO_PIN_15
#define Klapan_34 GPIO_PIN_4
#define Klapan_35 GPIO_PIN_1
#define Klapan_36 GPIO_PIN_2
/*GPIOA*/
#define Klapan_32 GPIO_PIN_8
/*NC*/
#define Klapan_33 GPIO_PIN_


#define ALL_A_Klapans Klapan_5|Klapan_6|Klapan_8|Klapan_11|Klapan_13|Klapan_15|Klapan_25|Klapan_26|Klapan_28|Klapan_32
#define ALL_B_Klapans Klapan_9|Klapan_14|Klapan_20|Klapan_21|Klapan_27|Klapan_29|Klapan_31|Klapan_34|Klapan_35|Klapan_36
#define ALL_C_Klapans Klapan_1|Klapan_2|Klapan_3|Klapan_7|Klapan_10|Klapan_12|Klapan_18|Klapan_19|Klapan_23|Klapan_22|Klapan_24
#define ALL_D_Klapans Klapan_16|Klapan_17|Klapan_4


/*device by numbers*/
/*IDC6*/
#define dev_4_1 Klapan_31
#define dev_4_1_gp GPIOB

#define dev_5_1 Klapan_34
#define dev_5_1_gp GPIOB

#define dev_6_1 Klapan_35 
#define dev_6_1_gp GPIOB


#define dev_10_1 Klapan_36
#define dev_10_1_gp GPIOB

/*IDC8*/
#define dev_4 Klapan_25
#define dev_4_gp GPIOA

#define dev_5 Klapan_26
#define dev_5_gp GPIOA

#define dev_6 Klapan_28 
#define dev_6_gp GPIOA

#define dev_9 Klapan_27
#define dev_9_gp GPIOB

#define dev_10 Klapan_29
#define dev_10_gp GPIOB
/*IDC9*/
#define dev_11 Klapan_19
#define dev_11_gp GPIOC

#define dev_13 Klapan_23
#define dev_13_gp GPIOC

#define atm_start 10000
#define two_atm 	202649
#define one_atm 	101325
#define three_atm 303973
/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

void pumps(uint32_t Time);
void Transmitting(uint8_t k);
void Timing_main(void);
void zero_down_system(void);
void Parser(void);
extern double temperature1, temperature2, presuare1, presuare2;
typedef struct __attribute__((__packed__))
{
	uint8_t start;
	uint8_t klapan_start;
	uint8_t first_sign;
	uint8_t pressure_4;
	uint8_t pressure_3;
	uint8_t pressure_2;
	uint8_t pressure_1;
	uint8_t second_sign;
	uint8_t powering;
	uint8_t end;
} command_message;
typedef struct __attribute__((__packed__))
{
	uint8_t address;
	uint8_t func_code;
	uint16_t start_index;
	uint16_t length;
	uint8_t CRC_low;
	uint8_t CRC_high;
} MODBUS_Std_command;
extern command_message command_from_host;
extern command_message command_from_host_now;
extern uint8_t ch;
typedef struct __attribute__((__packed__))
{
	uint8_t address;
	uint8_t func_code;
	uint8_t byte_count;
	uint8_t raw_pressure_high;
	uint8_t raw_pressure_low;
	uint8_t raw_temperature_high;
	uint8_t raw_temperature_low;
	uint8_t CRC_low;
	uint8_t CRC_high;
} sensorData;

void MATH_Go_to_ESP(void);
void Init_klapans(void);

#define SENSOR_1_ADDR 0xF0
#define SENSOR_2_ADDR 0x0F
//#define _1_SLAVE_PRESENT

#define SENSOR_1_TEMPERATURE_PTS  ((uint16_t)((firstSensorTempPrs.raw_temperature_high << 8) | firstSensorTempPrs.raw_temperature_low))
#define SENSOR_1_PRESSURE_PTS     ((uint16_t)((firstSensorTempPrs.raw_pressure_high << 8) | firstSensorTempPrs.raw_pressure_low))
																										
#define SENSOR_2_TEMPERATURE_PTS  ((uint16_t)((secondSensorTempPrs.raw_temperature_high << 8) | secondSensorTempPrs.raw_temperature_low))
#define SENSOR_2_PRESSURE_PTS     ((uint16_t)((secondSensorTempPrs.raw_pressure_high << 8) |    secondSensorTempPrs.raw_pressure_low))

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
