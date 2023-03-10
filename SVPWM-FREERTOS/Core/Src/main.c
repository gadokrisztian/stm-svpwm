/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "inverter.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void inverter_set_channel(InverterChannels ch, ChannelState state);
void inverter_set_fout(double f);
void inverter_set_Vm(double vm);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RON  HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, 1)
#define ROFF HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, 0)

#define BON  HAL_GPIO_WritePin(LDB_GPIO_Port, LDB_Pin, 1)
#define BOFF HAL_GPIO_WritePin(LDB_GPIO_Port, LDB_Pin, 0)

#define GON  HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, 1)
#define GOFF HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, 0)

#define BUF_SIZE 10000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

USART_HandleTypeDef husart3;

/* Definitions for Default */
osThreadId_t DefaultHandle;
const osThreadAttr_t Default_attributes = {
  .name = "Default",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DutyCycle */
osThreadId_t DutyCycleHandle;
const osThreadAttr_t DutyCycle_attributes = {
  .name = "DutyCycle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Main */
osThreadId_t MainHandle;
const osThreadAttr_t Main_attributes = {
  .name = "Main",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_Init(void);
static void MX_TIM2_Init(void);
void default_task(void *argument);
void calc_dutycycle(void *argument);
void main_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Inverter inv;

uint16_t adc_val_U[BUF_SIZE];
uint16_t adc_val_V[BUF_SIZE];
uint16_t adc_val_W[BUF_SIZE];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	inv_init(&inv);
	inv.f_sampling = 1500;
	inverter_set_fout(50);

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
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Default */
  DefaultHandle = osThreadNew(default_task, NULL, &Default_attributes);

  /* creation of DutyCycle */
  DutyCycleHandle = osThreadNew(calc_dutycycle, NULL, &DutyCycle_attributes);

  /* creation of Main */
  MainHandle = osThreadNew(main_task, NULL, &Main_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	vTaskSuspend(DefaultHandle);

	inverter_set_channel(U, ON);
	inverter_set_channel(UN, ON);
	inverter_set_channel(V, ON);
	inverter_set_channel(VN, ON);
	inverter_set_channel(W, ON);
	inverter_set_channel(WN, ON);

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start(&htim2);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 10000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4167-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_LOW;
  husart3.Init.CLKPhase = USART_PHASE_1EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LDG_Pin|LDR_Pin|LDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LDG_Pin LDR_Pin LDB_Pin */
  GPIO_InitStruct.Pin = LDG_Pin|LDR_Pin|LDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void inverter_set_channel(InverterChannels ch, ChannelState state) {
	switch (ch) {
	case U:
		if (state == ON) {
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		} else {
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		}
		break;

	case UN:
		if (state == ON) {
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		} else {
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		}
		break;

	case V:
		if (state == ON) {
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		} else {
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		}
		break;

	case VN:
		if (state == ON) {
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		} else {
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		}
		break;

	case W:
		if (state == ON) {
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		} else {
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		}
		break;

	case WN:
		if (state == ON) {
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		} else {
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		}
		break;
	}
}

void inverter_set_fout(double f) {
	f = (f < 0.01) ? 0.0 : f;
	f = (f > 100.0) ? 100.0 : f;
	inv.f_out = f;
	htim2.Instance->ARR = (uint32_t) 100e6 / (f * (htim2.Instance->PSC + 1));
}

void inverter_set_Vm(double vm) {
	vm = (vm < 0.01) ? 0.0 : vm;
	vm = (vm > MAXV) ? MAXV : vm;
	inv.Vm = vm;
}

void EXTI15_10_IRQHandler(void) {
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */

	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */
	xTaskResumeFromISR(MainHandle);

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_default_task */
/**
 * @brief  Function implementing the Default thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_default_task */
void default_task(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(LDB_GPIO_Port, LDB_Pin, 0);
		HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, 0);
		HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, 0);
		osDelay(500);
		HAL_GPIO_WritePin(LDB_GPIO_Port, LDB_Pin, 1);
		HAL_GPIO_WritePin(LDG_GPIO_Port, LDG_Pin, 1);
		HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, 1);
		osDelay(500);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_calc_dutycycle */
/**
 * @brief Function implementing the DutyCycle thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_calc_dutycycle */
void calc_dutycycle(void *argument)
{
  /* USER CODE BEGIN calc_dutycycle */
	/* Infinite loop */
	for (;;) {
		RON;
		inv.theta = (double) (htim2.Instance->CNT + 1)
				/ (htim2.Instance->ARR + 1) * 2 * PI;
		inv_calc_dc(&inv);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
				(10000-1) * inv.dc[U]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
				(10000-1) * inv.dc[V]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
				(10000-1) * inv.dc[W]);
		ROFF;
		vTaskSuspend(DutyCycleHandle);

	}
  /* USER CODE END calc_dutycycle */
}

/* USER CODE BEGIN Header_main_task */
/**
 * @brief Function implementing the Main thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_main_task */
void main_task(void *argument)
{
  /* USER CODE BEGIN main_task */
	const double nb_steps = 100;
	const double max_V = 20.0;
	const double max_f = 50.0;
	const double step_V = max_V / nb_steps;
	const double step_f = max_f / nb_steps;

	const uint32_t ramptime = 5;
	const uint32_t testcase_runtime = 200;
	const uint32_t testcase_recoverytime = 200;

	/* Infinite loop */
	for (;;) {
		 // RAMP UP
		 vTaskSuspend(MainHandle);
		 for (int i = 0; i < nb_steps; i++) {
		 inverter_set_fout((i + 1) * step_f);
		 inverter_set_Vm((i + 1) * step_V);
		 osDelay(ramptime);
		 }

		 vTaskSuspend(MainHandle);


		 inverter_set_channel(U, OFF);
		 vTaskSuspend(MainHandle);

		 inverter_set_channel(U, ON);
		 inverter_set_channel(UN, OFF);
                 vTaskSuspend(MainHandle);

		 inverter_set_channel(UN, ON);
		 inverter_set_channel(V, OFF);
                 vTaskSuspend(MainHandle);

		 inverter_set_channel(V, ON);
		 inverter_set_channel(VN, OFF);
                 vTaskSuspend(MainHandle);

		 inverter_set_channel(VN, ON);
		 inverter_set_channel(W, OFF);
                 vTaskSuspend(MainHandle);



		 inverter_set_channel(W, ON);		 
		 inverter_set_channel(WN, OFF);
                 vTaskSuspend(MainHandle);



		 inverter_set_channel(WN, ON);
		 vTaskSuspend(MainHandle);


		 // RAMPDOWN
		 for (int i = nb_steps; i > 0; i--) {
		 inverter_set_fout((i - 1) * step_f);
		 inverter_set_Vm((i - 1) * step_V);
		 osDelay(ramptime);
		 }

		 // DONE TEST
		 HAL_TIM_Base_Stop_IT(&htim4);
		 HAL_TIM_Base_Stop(&htim4);
		 inverter_set_channel(U, OFF);
		 inverter_set_channel(UN, OFF);
		 inverter_set_channel(V, OFF);
		 inverter_set_channel(VN, OFF);
		 inverter_set_channel(W, OFF);
		 inverter_set_channel(WN, OFF);
		 vTaskSuspend(DutyCycleHandle);
		 vTaskResume(DefaultHandle);
		 vTaskDelete(DutyCycleHandle);
		 vTaskDelete(MainHandle);
		 vTaskSuspend(MainHandle);
                 vTaskResume(DefaultHandle);

	}
  /* USER CODE END main_task */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM4) {
		xTaskResumeFromISR(DutyCycleHandle);
		portYIELD_FROM_ISR(pdTRUE);
	}

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
