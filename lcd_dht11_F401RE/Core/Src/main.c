/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ili9341.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint16_t average_temp;
	uint64_t time_idx;
	uint16_t average_by_the_hour[10];
}eeStorage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_STACK_SIZE 2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
eeStorage_t ee;

TaskHandle_t h_lcd_task = NULL;
TaskHandle_t h_DHT11_task = NULL;
TaskHandle_t h_stat_task = NULL;

uint8_t screen_power = 1;
uint16_t average_by_the_hour[10];


SemaphoreHandle_t DHT11_sema;
SemaphoreHandle_t stat_sema;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim10, 0);
	while((__HAL_TIM_GET_COUNTER(&htim10))<time);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == B1_Pin){
		HAL_GPIO_TogglePin(LCD_POWER_GPIO_Port, LCD_POWER_Pin);
		screen_power=!screen_power;
	}
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start (void){
	Set_Pin_Output(DHT11_GPIO_Port, DHT11_Pin);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 0);   // pull the pin low
	delay (18000);   // wait for 18ms
	Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);    // set as input
}

uint8_t Check_Response (void){
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))) Response = 1;
		else Response = -1;
	}
	uint16_t starting_count = __HAL_TIM_GET_COUNTER(&htim9)%5;
	while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))){
		if((__HAL_TIM_GET_COUNTER(&htim9)%5) == ((starting_count+3)%5)){
			return -1;   // wait for the pin to go low
		}
	}
	return Response;
}

uint8_t DHT11_Read (void){
	uint8_t i,j;
	for (j=0;j<8;j++){
		uint16_t starting_count = __HAL_TIM_GET_COUNTER(&htim9)%5;
		while (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))){
			if((__HAL_TIM_GET_COUNTER(&htim9)%5) == ((starting_count+3)%5)){
				return -1;   // wait for the pin to go low
			}
		}
		delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		starting_count = __HAL_TIM_GET_COUNTER(&htim9)%5;
		while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))){
			if((__HAL_TIM_GET_COUNTER(&htim9)%5) == ((starting_count+3)%5)){
				return -1;   // wait for the pin to go low
			};  // wait for the pin to go low
		}
	}
	return i;
}

void DHT11_task(void * unused){
	printf("DHT11 task launched\r\n");
	for(;;){
		vTaskDelay(3000);
		DHT11_Start();
		Check_Response();
		Rh_byte1 = DHT11_Read();
		Rh_byte2 = DHT11_Read();
		Temp_byte1 = DHT11_Read();
		Temp_byte2 = DHT11_Read();
		SUM = DHT11_Read();

		TEMP = Temp_byte1;
		RH = Rh_byte1;
		xSemaphoreGive(DHT11_sema);
	}
}

void lcd_task(void * unused){
	printf("lcd task launched\r\n");
	uint8_t toggle=0;
	for(;;){
		if(screen_power == 0){
			vTaskDelay(500);
		}else if(screen_power == 1){
			ILI9341_Unselect();
			ILI9341_Init();
			ILI9341_FillScreen(ILI9341_WHITE);
			screen_power = 2;
		}else{
			xSemaphoreTake(DHT11_sema, portMAX_DELAY);
			uint8_t display_buf[10];
			sprintf((char *)display_buf,"%d",TEMP);
			ILI9341_WriteString(((ILI9341_WIDTH>>1)-(Font_32x52.width + 12)), 0, (const char *)display_buf, Font_32x52, ILI9341_BLACK, ILI9341_WHITE);
			if(toggle%2){
				ILI9341_WriteString(((ILI9341_WIDTH>>1)-(Font_32x52.width + 12)), 0, (const char *)display_buf, Font_32x52, ILI9341_BLUE, ILI9341_WHITE);
			}
			toggle++;
			uint16_t colum_number = 10;
			uint16_t space_btwn_each_colum = 6;
			uint16_t first_space = (space_btwn_each_colum/2)-1;
			uint16_t colum_width = (ILI9341_WIDTH - colum_number*space_btwn_each_colum)/colum_number;
			uint16_t position_x_step = colum_width + (space_btwn_each_colum/2);
			for(int i = first_space; i<ILI9341_WIDTH; i+=position_x_step){
				uint16_t amplitude = ILI9341_HEIGHT-(average_by_the_hour[i/position_x_step]<<2);
				ILI9341_FillRectangle(i,amplitude,colum_width,ILI9341_HEIGHT,ILI9341_RED);
			}
		}
	}
}

void stat_task(void * unused){
	printf("stat task launched\r\n");
	uint16_t average_temp = 0;
	HAL_TIM_Base_Start_IT(&htim9);
	for(uint64_t idx = 0;;idx++){
		xSemaphoreTake(stat_sema, portMAX_DELAY);
		average_temp += TEMP;
		uint64_t is_hour = (idx + 1)%3600;
		//uint64_t is_hour = (idx + 1)%60;
		if(is_hour == 0){
			average_by_the_hour[(idx/3599)-1] = average_temp/3600;
			//average_by_the_hour[(idx/59)-1] = average_temp/60;
			average_temp = 0;
		}
	}
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
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_TIM10_Init();
	MX_TIM9_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim10);
	BaseType_t ret;
	DHT11_sema = xSemaphoreCreateBinary();
	stat_sema = xSemaphoreCreateBinary();
	ret = xTaskCreate(lcd_task, "lcd_task", DEFAULT_STACK_SIZE, NULL, 2, &h_lcd_task);
	if(ret != pdPASS)
	{
		printf("Could not create lcd task\r\n");
		Error_Handler();
	}
	ret = xTaskCreate(DHT11_task, "DHT11_task", DEFAULT_STACK_SIZE, NULL,1, &h_DHT11_task);
	if(ret != pdPASS)
	{
		printf("Could not create DHT11 task\r\n");
		Error_Handler();
	}
	ret = xTaskCreate(stat_task, "stat_task", DEFAULT_STACK_SIZE, NULL,3, &h_stat_task);
	if(ret != pdPASS)
	{
		printf("Could not create stat task\r\n");
		Error_Handler();
	}
	vTaskStartScheduler();
	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM11 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	BaseType_t xHigherPriorityTaskToken = pdFALSE;
	if(htim->Instance == TIM9){
		xSemaphoreGiveFromISR(stat_sema, &xHigherPriorityTaskToken);
	}
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM11) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	portYIELD_FROM_ISR(xHigherPriorityTaskToken);
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
