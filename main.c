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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx_hal.h"
#include "dwt_stm32_delay.h"
#include "i2c-lcd.h"
#include "stdlib.h"
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
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t time;
uint32_t time1;
uint32_t time2;
uint32_t time3;
uint16_t distance1, distance2, distance3, distance4;
float medium_speed, speed1, speed2, speed3;

UART_HandleTypeDef * esp_uart = &huart1;
volatile uint8_t esp_recv_char;
volatile uint8_t esp_char_counter = 0;
char esp_pattern[] = "+IPD,";

volatile uint8_t esp_recv_flag = 0;
volatile char esp_recv_mux;
volatile char esp_recv_buffer[1024];
volatile uint16_t esp_recv_len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_DCMI_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funkcja czujnika odległości
uint32_t Read_HCSR04(){
	uint32_t local_time = 0;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	DWT_Delay_us(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

	while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2));

	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)){
		local_time++;
		DWT_Delay_us(1);
	}

	return local_time;
}

// Funkcja wysyłająca podany ciąg znaków przez interfejs UART
void uart_write_line(UART_HandleTypeDef * handler, char * text) {
	HAL_UART_Transmit(handler, text, strlen(text), 1000);
	HAL_UART_Transmit(handler, "\r\n", 2, 100);
}



// Funkcja odbierająca linię tekstu przez interfejs UART
void uart_read_line(UART_HandleTypeDef * handler, char * buffer, uint16_t buffer_size) {
	HAL_StatusTypeDef status;
	char current_char;
	uint16_t char_counter = 0;
	while (char_counter < buffer_size - 1) {
	   status = HAL_UART_Receive(handler, &current_char, 1, 1);
	   if (status == HAL_OK) {
		   if (current_char == '\r' || current_char == '\n')
			   if (char_counter == 0) continue;
			   else break;
		   *(buffer + char_counter++) = current_char;
	   }
	}
	*(buffer + char_counter) = '\0';
}



// Funkcja odczytująca pojedynczy znak odebrany przez UART
char uart_read_char(UART_HandleTypeDef * handler) {
	char buffer = '\0';
	HAL_UART_Receive(handler, &buffer, 1, 1000);
	return buffer;
}

// Funkcja wysyłająca polecenie do modułu ESP8266 i oczekująca na jego potwierdzenie
uint8_t esp_send_cmd(UART_HandleTypeDef * uart, char * command) {
	char response[30];
	response[0] = '\0';
	uart_write_line(uart, command);
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	while (strcmp(response, "OK") != 0 && strcmp(response, "no change") != 0 && strcmp(response, "ERROR") != 0)
	   uart_read_line(uart, response, 30);
	if (strcmp(response, "ERROR") == 0) return 0;
	else return 1;
}

// Funkcja wysyłająca dane przez nawiązane połączenie TCP i zamykająca to połączenie
void esp_send_data_and_close(UART_HandleTypeDef * uart, char mux_id, char * content) {
	char cmd[17];
	sprintf(cmd, "AT+CIPSEND=%c,%d", mux_id, strlen(content));
	uart_write_line(uart, cmd);
	HAL_Delay(20);
	HAL_UART_Transmit(uart, content, strlen(content), 5000);
	HAL_Delay(100);
	sprintf(cmd, "AT+CIPCLOSE=%c", esp_recv_mux);
	uart_write_line(esp_uart, cmd);
}

// Funkcja uruchamiająca obsługę przerwań
void esp_start_int_recv(UART_HandleTypeDef * uart) {
	__HAL_UART_FLUSH_DRREGISTER(uart);
	HAL_UART_Receive_IT(uart, &esp_recv_char, 1);
}

// Funkcja obsługująca przerwanie, wywoływana w momencie odebrania przez interfejs UART pojedynczego bajtu danych
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * uart) {
	if (esp_recv_char == esp_pattern[esp_char_counter]) {
	   esp_char_counter++;
	   if (esp_char_counter == 5) {
		   // Jeśli odbierzemy ciąg znaków "+IPD,":

		   // Odczytujemy numer połączenia do zmiennej esp_recv_mux
		   esp_recv_mux = uart_read_char(uart);
		   uart_read_char(uart);

		   // Odczytujemy długość odebranych dancyh do esp_recv_len
		   char length_str[5];
		   char current_char = 0;
		   uint8_t char_counter = 0;
		   do {
			   current_char = uart_read_char(uart);
			   length_str[char_counter++] = current_char;
		   } while (current_char != ':');
		   length_str[char_counter] = '\0';
		   uint16_t esp_recv_len = atoi(&length_str);

		   // Odbieramy dane do bufora esp_recv_buffer
		   HAL_UART_Receive(uart, esp_recv_buffer, esp_recv_len, 1000);
		   esp_recv_flag = 1;
		   return;
	   }
	} else esp_char_counter = 0;

	// Ponowne uruchomienie przerwania
	HAL_UART_Receive_IT(uart, &esp_recv_char, 1);
}

// Funkcja przesyłająca do modułu ESP8266 polecenia konfigurujące
uint8_t esp_setup() {
	HAL_Delay(500); // Oczekujemy na uruchomienie modułu

	if (!esp_send_cmd(esp_uart, "AT+CWMODE=1")) return 0;
	if (!esp_send_cmd(esp_uart, "AT+CWJAP=\"NAZWA_SIECI\",\"KLUCZ_SIECIOWY\"")) return 0;
	if (!esp_send_cmd(esp_uart, "AT+CIPMUX=1")) return 0;
	if (!esp_send_cmd(esp_uart, "AT+CIPSERVER=1,80")) return 0;
	return 1;

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	// Forbot
	SystemCoreClock = 8000000; // taktowanie 8Mhz

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_DCMI_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  // Z Forbota
  __HAL_RCC_GPIOA_CLK_ENABLE();
  	 __HAL_RCC_GPIOB_CLK_ENABLE();
  	 __HAL_RCC_GPIOC_CLK_ENABLE();
  	 __HAL_RCC_I2C1_CLK_ENABLE();

  	 GPIO_InitTypeDef gpio;
  	 gpio.Mode = GPIO_MODE_AF_OD;
  	 gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9; // SCL, SDA
  	 gpio.Pull = GPIO_PULLUP;
  	 gpio.Speed = GPIO_SPEED_FREQ_LOW;
  	 HAL_GPIO_Init(GPIOB, &gpio);

  	 hi2c1.Instance             = I2C1;
  	hi2c1.Init.ClockSpeed      = 100000;
  	hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  	hi2c1.Init.OwnAddress1     = 0xff;
  	hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  	hi2c1.Init.OwnAddress2     = 0xff;
  	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  	hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  	 HAL_I2C_Init(&hi2c1);

  	 uint8_t test = 0x5a;
  	 HAL_I2C_Mem_Write(&hi2c1, 0xa0, 0x10, 1, (uint8_t*)&test, sizeof(test), HAL_MAX_DELAY);

  	 uint8_t result = 0;
  	 HAL_I2C_Mem_Read(&hi2c1, 0xa0, 0x10, 1, (uint8_t*)&result, sizeof(result), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Działa
//	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//	  time = Read_HCSR04();
//	  distance1 = time / 58;
//	  HAL_Delay(1000);
//	  time1+=1+time/1000000;
//	  time = Read_HCSR04();
//	  distance2 = time / 58;
//	  time1+=time/1000000;
//	  speed1=(distance1-distance2)/time1;
//	  	  HAL_Delay(1000);
//	  	  time2+=1+time/1000000;
//	  	  time = Read_HCSR04();
//	  	  distance3 = time / 58;
//	  	  time2+=time/1000000;
//	  	  speed2=(distance2-distance3)/time2;
//	  		  HAL_Delay(1000);
//	  		  time3+=1+time/1000000;
//	  		  time = Read_HCSR04();
//	  		  distance4 = time / 58;
//	  		  time3+=time/1000000;
//	  		  speed3=(distance3-distance4)/time3;
//	  		  medium_speed=(speed1+speed2+speed3)/3;
//	  if(medium_speed>10){
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
//	  }
//	  time1=0;
//	  time2=0;
//	  time3=0;
//	  HAL_Delay(200);
	  // Koniec działania

//	  	  if(distance > 6 && distance < 12){
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
//	  	  }
//	  	  else if(distance > 12 && distance < 18){
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
//	  	  }
//	  	  else if(distance > 18 && distance < 24){
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//	  	  }
//	  	  else if(distance > 24){
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
//	  	  }
//	  	  else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
//	  	  HAL_Delay(200);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
