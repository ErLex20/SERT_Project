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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
HAL_StatusTypeDef ret;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TS 50

#define TASK_SPI_PRIO 				osPriorityRealtime
#define TASK_HEARTBEAT_PRIO			osPriorityNormal

#define TASK_SPI_PERIOD				50
#define TASK_HEARTBEAT_PERIOD		1000

#define TASK_SPI_DEADLINE			TASK_SPI_PERIOD
#define TASK_PID_DEADLINE			TASK_PID_PERIOD
#define TASK_HEARTBEAT_DEADLINE		TASK_HEARTBEAT_PERIOD

#define PANIC_PERIOD				100

#define time_after(a,b) 			((long)((b)-(a))<0)
#define time_before(a,b) 			time_after(b,a)
#define time_after_eq(a,b) 			((long)((a)-(b))>=0)
#define time_before_eq(a,b) 		time_after_eq(b,a)
#define TASK_SPI_TIMER				40
#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId task_SPIHandle;
osThreadId task_PIDHandle;
osThreadId task_HeartbeatHandle;

SemaphoreHandle_t CountingSem;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data_tx = 1;
uint8_t data_rx = 0;
uint8_t new_encoder[16];
uint32_t newTick = 0, oldTick = 0;

/* pseudoDerivative variables ------------------------------------------------*/
int mediana;
int reset;
double counter;
double temp1;
double temp2;
uint8_t mybuffer[10];



/* PID variables -------------------------------------------------------------*/
double Kp = 1.1;
double Ki = 0.1;
double Kd = 1.5;
double S_u = 156;
double S_du = 1;
double S_i = 100;
double target = 200;
uint8_t u = 0;
double error = 0;
double d_error = 0;
double controlDeadzone = 41;
double PID_timeOn = 0;


int d = 10;
int c = 3;

uint8_t x_integral[1];
double u_internal;
double u_noint;
double d_integral;
double theta;
double xhat_integral;
double u_tot;

uint8_t enc_value = 0;


/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId task_SPIReceiveHandle;
osThreadId task_SPITransmitHandle;
osThreadId task_PIDHandle;
osThreadId task_HeartbeatHandle;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
void start_task_spi(void const * argument);
void start_task_heartbeat(void const * argument);
void panic(void);
double pseudoDerivative(double e, int d, int c);
double PID(unsigned int t, double e, double de, double Kp, double Ki, double Kd, double S_u, double S_du, double S_i, double PID_timeOn);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(task_spi, start_task_spi, TASK_SPI_PRIO , 0, 128);
  task_SPITransmitHandle = osThreadCreate(osThread(task_spi), NULL);

  osThreadDef(task_heartbeat, start_task_heartbeat, TASK_HEARTBEAT_PRIO, 0, 128);
  task_HeartbeatHandle = osThreadCreate(osThread(task_heartbeat), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void start_task_spi(void const * argument)
{
	TickType_t xLastWakeTime;
	TickType_t xTick;
	const TickType_t xPeriod = pdMS_TO_TICKS(TASK_SPI_PERIOD);


	for(;;)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

		/* SPI_Receive: data from slave */


		ret = HAL_SPI_Receive(&hspi1, (uint8_t *)&enc_value, (uint16_t)1, TASK_SPI_TIMER);

		 //---------------- START PID CODE -------------------//
		error = target - (double)enc_value;
		d_error = pseudoDerivative(error, d, c);
		enc_value = PID(newTick, error, d_error, Kp, Ki, Kd, S_u, S_du, S_i, PID_timeOn);

		//printf("%lu\n",enc_value);

		if (enc_value < controlDeadzone)
		{
			enc_value = 0;
		}

		//---------------- END PID CODE -------------------//



		if (ret == HAL_OK)
		{
			//printf("Task SPI: Receive OK\n");
			//printf("dato tx: %u\n", data_rx);
			//printf("dato rx: %u\n", data_rx);
		}
		else if (ret == HAL_ERROR)
		{
			printf("Error\n");
		}
		else if (ret == HAL_BUSY)
		{
			printf("Busy\n");
		}
		else if (ret == HAL_TIMEOUT)
		{
			printf("Timeout\n");

		 }



		//xSemaphoreTake(CountingSem, portMAX_DELAY);

		/* SPI_Transmit: control to slave */



		ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)&enc_value, (uint16_t)1, HAL_MAX_DELAY);

		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

		if (ret == HAL_OK)
		{
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			//printf("MASTER: Task SPI: Transmit OK dato : %u\n", enc_value);
			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			//HAL_Delay(100);
		}
		else if (ret == HAL_ERROR)
		{
			printf("Error\n");
		}
		else if (ret == HAL_BUSY)
		{
			printf("Busy\n");
		}
		else if (ret == HAL_TIMEOUT)
		{
			printf("Timeout\n");
		}


		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


		xTick = HAL_GetTick();
		if (time_after(xTick, TASK_SPI_DEADLINE + xLastWakeTime))
		{
			printf("Task SPI: Deadline not respected\n");
			panic();
		}

		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}
void panic(void)
{
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		HAL_Delay(PANIC_PERIOD);
	}
}

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void start_task_heartbeat(void const * argument)
{
	TickType_t xLastWakeTime;
	TickType_t xTick;
	const TickType_t xPeriod = pdMS_TO_TICKS(TASK_HEARTBEAT_PERIOD);

	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		//printf("HeartNeat Attivato\n");
		xTick = HAL_GetTick();
		if (time_after(xTick, TASK_HEARTBEAT_DEADLINE + xLastWakeTime))
		{
			//printf("Task Heartbeat: Deadline not respected\n");
			panic();
		}

		//printf("Task Heartbeat: OK\n");

		vTaskDelayUntil(&xLastWakeTime,xPeriod);
	}
}




double pseudoDerivative(double e, int d, int c)
{
  int i, j, k, num_empty = 0;

  mediana = 0;
  reset = 0;

  for (i = 0; i < 10; i++)
  {
	  if (mybuffer[i] == 0)
	  {
		  num_empty++;
	  }
  }
  if (num_empty == 10)
  {
	  reset = 1;
  }

  if (reset == 1)
  {
    for (i = 0; i < d; i++)
    {
      mybuffer[i] = 0;
    }
    counter = 0;
  }

  counter = counter + 1;

  for (k = 0; k < d - 1; k++)
  {
    mybuffer[k] = mybuffer[k + 1];
  }

  mybuffer[d] = e;

  if (counter >= d)
  {
    if (mediana == 1)
    {
      uint8_t mybuffer1[10];
      for (i = 0; i < d; i++)
      {
        mybuffer1[i] = mybuffer[i];
      }

      int min = 0;
      int temp = 0;
      for (int i = 0; i < d - 1; i++)
      {
        min = i;

        for (j = i + 1; j < d; j++)
        {
          if (mybuffer1[j] < mybuffer1[min])
          {
            min = j;
          }
          temp = mybuffer1[min];
          mybuffer1[min] = mybuffer1[i];
          mybuffer1[i] = temp;
        }
      }

      if (c % 2 != 0)
      {
        temp1 = mybuffer1[(c + 1) / 2];
        temp2 = mybuffer1[d - c + (c + 1) / 2];
      }
      else
      {
        temp1 = (mybuffer1[c / 2] + mybuffer1[(c + 1) / 2]) / 2;
        temp2 = (mybuffer1[d - c + c / 2] + mybuffer1[d - c + c / 2 + 1]) / 2;
      }
    }
    else
    {
      temp1 = 0;
      for (i = 0; i < c; i++)
      {
        temp1 = temp1 + mybuffer[i];
      }
      temp1 = temp1 / c;

      temp2 = 0;
      for (i = d - c + 1; i < d; i++)
      {
        temp2 = temp2 + mybuffer[i];
      }
      temp2 = temp2 / c;
    }
    d_error = (temp2 - temp1) / (TS * (d - c));
  }
  else
  {
    d_error = 0;
  }
  return d_error;
}

double PID(unsigned int t, double e, double de, double Kp, double Ki, double Kd, double S_u, double S_du, double S_i, double PID_timeOn)
{
  if (x_integral[0] == 0)
  {
    x_integral[0] = 0;
    u_internal = Kp * e;
  }

  if (t >= PID_timeOn)
  {
    u_noint =  Kp * e +  Kd * de;
    d_integral =  Ki * e * TS;

    xhat_integral = x_integral[0] + d_integral;
    theta =  S_u - abs(u_noint + xhat_integral);
    if (theta < 0)
    {
      if ( abs(u_noint + xhat_integral) < abs(u_noint + x_integral[0]))
      {
        x_integral[0] = xhat_integral;
      }
    }
    else
    {
      x_integral[0] = xhat_integral;
    }

    x_integral[0] = max(- S_i, min( S_i, x_integral[0]));

    u_tot = max(- S_u, min( S_u, u_noint + x_integral[0]));

    u_internal = u_internal + max(- S_du, min( S_du, u_tot - u_internal));
  }
  u = u_internal;

  return u;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
