/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct Phase {
  float a;
  float b;
  float c;
}Phase;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define DRIVE_EN_Pin GPIO_PIN_7
#define DRIVE_EN_GPIO_Port GPIOC
#define N_BRAKE_Pin GPIO_PIN_8
#define N_BRAKE_GPIO_Port GPIOC
#define IN_LX_Pin GPIO_PIN_9
#define IN_LX_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENC_CS_Pin GPIO_PIN_9
#define ENC_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define FAULT_ST_AddRead  0x00
#define FUNCT_ST_AddRead  0x03

#define FAULT_CLR_AddWrite  0x90
#define FAULT_CLR_HValue  0b00000000
#define FAULT_CLR_LValue  0b00000000

#define SUPPLY_CFG_AddWrite  0x91
#define SUPPLY_CFG_HValue  0b01101000
#define SUPPLY_CFG_LValue  0b00000010

#define PWM_CFG_AddWrite  0x93
#define PWM_CFG_HValue  0b00000000
#define PWM_CFG_LValue  0b00000001

#define SENSOR_CFG_AddWrite  0x94
#define SENSOR_CFG_HValue  0b00000000
#define SENSOR_CFG_LValue  0b00100001

#define IDRIVE_CFG_AddWrite  0x97
#define IDRIVE_CFG_HValue  0b10111011
#define IDRIVE_CFG_LValue  0b10111011

#define IDRIVE_PRE_CFG_AddWrite  0x98
#define IDRIVE_PRE_CFG_HValue  0b00000000
#define IDRIVE_PRE_CFG_LValue  0b10111011

#define TDRIVE_SRC_CFG_AddWrite  0x99
#define TDRIVE_SRC_CFG_HValue  0b00001001
#define TDRIVE_SRC_CFG_LValue  0b00000101

#define TDRIVE_SINK_CFG_AddWrite  0x9A
#define TDRIVE_SINK_CFG_HValue  0b00001101
#define TDRIVE_SINK_CFG_LValue  0b00001101

#define DT_CFG_AddWrite  0x9B
#define DT_CFG_HValue  0b00001100
#define DT_CFG_LValue  0b00001010

#define CSAMP_CFG_AddWrite  0x9D
#define CSAMP_CFG_HValue  0b11000000
#define CSAMP_CFG_LValue  0b01110011//4

#define CSAMP2_CFG_AddWrite  0x9E
#define CSAMP2_CFG_HValue  0b00101000
#define CSAMP2_CFG_LValue  0b00000000

#define CP_CFG_AddWrite 0x9C
#define CP_CFG_HValue  0b00000000
#define CP_CFG_LValue  0b00000001

#define CURRENT_SENSE_GAIN 16.0f
#define R_SENSE 0.005f

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern uint16_t i_ph1;
extern uint16_t i_ph2;
extern uint16_t i_ph3;
extern uint16_t adc_offset[3];
extern Phase Iabc;
extern double Va,Vb,Vc;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
