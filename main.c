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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ff.h"
#include <stdbool.h>
#include "stdio.h"
#include "common.h"
#include "commonMsg.h"
#include "camera.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "dwt_stm32_delay.h"
#include "i2c-lcd.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FA_READ 0x01
#define FA_WRITE 0x02
#define FA_OPEN_EXISTING 0x00
#define FA_CREATE_NEW 0x04
#define FA_CREATE_ALWAYS 0x08
#define FA_OPEN_ALWAYS 0x10
#define FA_OPEN_APPEND 0x30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char buffer[256]; //bufor odczytu i zapisu
static FATFS FatFs; //uchwyt do urządzenia FatFs (dysku, karty SD...)
FRESULT fresult; //do przechowywania wyniku operacji na bibliotece
FIL file; //uchwyt do otwartego pliku
WORD bytes_written; //liczba zapisanych byte
WORD bytes_read; //liczba odczytanych byte

uint32_t time;
uint32_t time1;
uint32_t time2;
uint32_t time3;
uint16_t distance1, distance2, distance3, distance4;
float medium_speed, speed1, speed2, speed3;
float allowed_speed = 30;

volatile int i = 0;
uint8_t count=0; // do testów printf
uint8_t  cam_buf[320 * 120 * 2]; // hardcoded frame size było uint 16

UINT bw;

//int j;
//int k;
//int Duty_left, Duty_right, dir_left, dir_right;

uint8_t sendUART[4] = {'A', 'T','\r','\n'};
uint16_t sizeSendUART = 4;

uint16_t size_s_mux_0 = 8;
uint8_t s_mux_0[8] = {'A','T','+','R','S','T','\r','\n'};

uint16_t size_s_mux_1 = 13;
uint8_t s_mux_1[13] = {'A','T','+','C','W','M','O','D','E','=','2','\r','\n'};

uint16_t size_s_mux_2 = 14;
uint8_t s_mux_2[14] = {'A','T','+','C','I','P','M','O','D','E','=','0','\r','\n'};

uint16_t size_s_mux_3 = 13;
uint8_t s_mux_3[13] = {'A','T','+','C','I','P','M','U','X','=','1','\r','\n'};

uint16_t size_s_mux_4 = 21;
uint8_t s_mux_4[21] = {'A','T','+','C','I','P','S','E','R','V','E','R','=','1',',','5','2','0','0','\r','\n'};

uint16_t size_s_mux_5 = 37;
uint8_t s_mux_5[37] = {'A','T','+','C','W','S','A','P','=','"','S','T','M','3','2','"',',','"','S','T','M','3','2','D','Z','I','A','L','A','J','"',',','4',',','3','\r','\n'};

//uint16_t size_s_serv_1 = 21;
//uint8_t s_serv_1[22] = {'A','T','+','C','I','P','S','E,'R','V','E','R','=','1',',','5','2','0','0','\r','\n'};

//uint16_t send_wifi_test_size = 18;
//uint8_t send_wifi_test[18] = {'A','T','+','C','I','P','S','E','N','D','=','0',',','6','4','0','\r','\n'};
//uint16_t send_wifi_test_size = 4;
//uint8_t send_wifi_test[4] = {'A','T','\r','\n'};

uint16_t send_wifi_size_size_size = 16;
uint8_t send_wifi_size_size[16] = {'A','T','+','C','I','P','S','E','N','D','=','0',',','5','\r','\n'};

uint16_t size_size_size = 7;
uint8_t size_size[7] = {'7','6','8','0','0','\r','\n'};

uint16_t send_wifi_size = 18;
uint8_t send_wifi[18] = {'A','T','+','C','I','P','S','E','N','D','=','0',',','3','2','0','\r','\n'};

uint16_t send_id_size = 16;
uint8_t send_id[16] = {'A','T','+','C','I','P','S','E','N','D','=','0',',','3','\r','\n'};
//uint8_t send_wifi[18] = {'A','T','+','C','I','P','S','E','N','D','=','0',',','3','2','2','\r','\n'};

uint16_t send_wifit_size = 16;
uint8_t send_wifit[16] = {'A','T','+','C','I','P','S','E','N','D','=','0',',','4','\r','\n'};

uint16_t send_wifi_test_size = 4;
uint8_t send_wifi_test[4] = {'T','E','S','T'};

uint16_t send_data_size = 320;
uint8_t send_data[320] = {};

uint16_t send_ent_size = 2;
uint8_t send_ent[2] = {'\r','\n'};

uint16_t close_wifi_size = 15;
uint8_t close_wifi[15] = {'A','T','+','C','I','P','C','L','O','S','E','=','0','\r','\n'};


uint8_t receiveUART[12];
uint16_t sizeReceiveUART = 12;
uint8_t receiveUART_speed[2];
uint16_t sizeReceiveUART_speed = 2;

//char packet[100];
//uint16_t bufr_size=322;
//uint8_t bufr[322]={};
uint16_t bufr_size=320*120;
char bufr[320*120]={};

int read_bool=0;
int id=1;
char name[10]="image0.raw";
char idc[3]={};
char image_size[5]="76800";
int send_speed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_DCMI_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len){ // do fprint
	int i=0;
	for(i=0; i<len; i++){
		ITM_SendChar(*ptr++);
	}
	return len;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

char checkc[3]="cfg";

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART3){
		printf("%c", receiveUART[0]);
			if(strstr(receiveUART, checkc)!=NULL){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
				HAL_Delay(2000);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			}
		HAL_UART_Receive_IT(&huart3, receiveUART, sizeReceiveUART);
	}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_DCMI_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  printf("CONSOLE GOOD \n");


//  HAL_UART_Transmit_IT(&huart3, send_wifi_test, send_wifi_test_size);
//  HAL_Delay(1000);
//  HAL_UART_Transmit_IT(&huart3, send_data, send_data_size);
//  HAL_Delay(1000);



  //ITM_SendChar( 65 );
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
//
//
//  HAL_TIM_Base_Start_IT(&htim2);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


  //set wifi end leds
//  HAL_UART_Receive_IT(&huart3, receiveUART, sizeReceiveUART);
//
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
//   HAL_UART_Transmit_IT(&huart3, sendUART, sizeSendUART);
//   HAL_Delay(6000);
//   HAL_UART_Transmit_IT(&huart3, s_mux_1, size_s_mux_1);
//   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
//   HAL_Delay(6000);
//   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
//   HAL_UART_Transmit_IT(&huart3, s_serv_1, size_s_serv_1);
//   HAL_Delay(6000);
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);


//  zapis na kartę pliku tekstowego
//  fresult = f_mount(&FatFs, "", 0);
//  fresult = f_open(&file, "write.txt", FA_OPEN_ALWAYS | FA_WRITE);
//  int len = sprintf( buffer, "Hello PTM!\r\n");
//  fresult = f_write(&file, buffer, len, &bw);
//  fresult = f_close (&file);

//  HAL_UART_Receive_IT(&huart3, receiveUART, sizeReceiveUART);
     HAL_UART_Transmit_IT(&huart3, sendUART, sizeSendUART);
     HAL_Delay(50);
     HAL_UART_Transmit_IT(&huart3, s_mux_1, size_s_mux_1);
     HAL_Delay(50);
     HAL_UART_Transmit_IT(&huart3, s_mux_2, size_s_mux_2);
     HAL_Delay(50);
     HAL_UART_Transmit_IT(&huart3, s_mux_3, size_s_mux_3);
     HAL_Delay(50);
     HAL_UART_Transmit_IT(&huart3, s_mux_4, size_s_mux_4);
     HAL_Delay(50);
     HAL_UART_Transmit_IT(&huart3, s_mux_5, size_s_mux_5);
     HAL_Delay(50);



  if (camera_init() == RET_OK)
  {
	  if (camera_config(CAMERA_MODE_QVGA_RGB565) == RET_OK)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // init OK
	  }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  time = Read_HCSR04();
	  distance1 = time / 58;
	  while(distance1>200){
		  time=Read_HCSR04();
		  distance1=time/58;
	  }
	  HAL_Delay(1000);
	  time1+=1+time/1000000;
	  time = Read_HCSR04();
	  distance2 = time / 58;
	  while(distance2>200){
		  time=Read_HCSR04();
		  distance2=time/58;
	  }
	  time1+=time/1000000;
	  speed1=(distance1-distance2)/time1;
	  if(speed1<0) speed1=-speed1;
	  HAL_Delay(1000);
	  time2+=1+time/1000000;
	  time = Read_HCSR04();
	  distance3 = time / 58;
	  while(distance3>200){
	  		  time=Read_HCSR04();
	  		  distance3=time/58;
	  }
	  time2+=time/1000000;
	  speed2=(distance2-distance3)/time2;
	  if(speed2<0) speed2=-speed2;
	  HAL_Delay(1000);
	  time3+=1+time/1000000;
	  time = Read_HCSR04();
	  distance4 = time / 58;
	  while(distance4>200){
		  time=Read_HCSR04();
		  distance4=time/58;
	  }
	  time3+=time/1000000;
	  speed3=(distance3-distance4)/time3;
	  if(speed3<0) speed3=-speed3;
	  medium_speed=(speed1+speed2+speed3)/3;
	  if(medium_speed>allowed_speed){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

		  if (camera_startCap(CAMERA_CAP_SINGLE_FRAME, (uint32_t)cam_buf)  == RET_OK)
		  {
			  name[5]=name[5]+1;
			  send_speed=round(medium_speed);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			  camera_stopCap();
			  f_mount(&FatFs, "", 0);
			  f_open(&file, name, FA_OPEN_ALWAYS | FA_CREATE_ALWAYS | FA_WRITE);
			  HAL_Delay(5);
			  for (int i = 0; i < 320 * 120 * 2; i += 2)
			  {
				  f_write(&file, &cam_buf[i], 2, &bw);
			  }
			  int y=0;
			  for(int z=0;z<320*120*2;z++){
				  bufr[z]=cam_buf[z];
			  }
			  fresult = f_close (&file);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // captured
			  HAL_Delay(400);
			  HAL_UART_Transmit_IT(&huart3, send_id, send_id_size);
			  HAL_Delay(400);
			  send_speed=send_speed+id*100;
			  HAL_Delay(400);
			  sprintf(idc,"%d",send_speed);
			  HAL_Delay(400);
			  HAL_UART_Transmit_IT(&huart3, idc, 3);
			  HAL_Delay(4000);
			  HAL_UART_Transmit_IT(&huart3, send_wifi_size_size, send_wifi_size_size_size);
			  HAL_Delay(400);
			  HAL_UART_Transmit_IT(&huart3, image_size, 5);
			  HAL_Delay(400);
			  int n=0;
			  char bufor[320]={};
			  for(int j=0;j<120*2;j++){
				 for(int k=0;k<320;k++){
					 bufor[k]=bufr[k+n];
				 }
				 HAL_UART_Transmit_IT(&huart3,send_wifi, send_wifi_size);
				 HAL_Delay(2000);
				 HAL_UART_Transmit_IT(&huart3, bufor, 320);
				 HAL_Delay(2000);
				 n+=320;
			  }
			  HAL_Delay(2000);
		  } else {
			  printf("not good");
		  }
	  }
	  time1=0;
	  time2=0;
	  time3=0;
	  HAL_Delay(200);

//	  if (camera_startCap(CAMERA_CAP_SINGLE_FRAME, (uint32_t)cam_buf)  == RET_OK)
//	  {
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//		  camera_stopCap();
//		  f_mount(&FatFs, "", 0);
//		  f_open(&file, "image1.raw", FA_OPEN_ALWAYS | FA_CREATE_ALWAYS | FA_WRITE);
//		  HAL_Delay(5);
////			  HAL_UART_Transmit_IT(&huart3, send_data, send_data_size);
//		  HAL_Delay(5);
//
//		  for (int i = 0; i < 320 * 120 * 2; i += 2)
//		  {
//			  f_write(&file, &cam_buf[i], 2, &bw);
//			  HAL_UART_Transmit_IT(&huart3, &cam_buf[i], 2);
//		  }
//		  fresult = f_close (&file);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // captured
//		  HAL_Delay(2000);
//	  } else {
//		  printf("not good");
//	  }
//
//	  if (camera_startCap(CAMERA_CAP_SINGLE_FRAME, (uint32_t)cam_buf)  == RET_OK)
//	  {
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//		  camera_stopCap();
//		  f_mount(&FatFs, "", 0);
//		  f_open(&file, "image2.raw", FA_OPEN_ALWAYS | FA_CREATE_ALWAYS | FA_WRITE);
//		  HAL_Delay(5);
////			  HAL_UART_Transmit_IT(&huart3, send_data, send_data_size);
//		  HAL_Delay(5);
//
//		  for (int i = 0; i < 320 * 120 * 2; i += 2)
//		  {
//			  f_write(&file, &cam_buf[i], 2, &bw);
//			  HAL_UART_Transmit_IT(&huart3, &cam_buf[i], 2);
//		  }
//		  fresult = f_close (&file);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // captured
//		  HAL_Delay(2000);
//	  } else {
//		  printf("not good");
//	  }
//
//	  if (camera_startCap(CAMERA_CAP_SINGLE_FRAME, (uint32_t)cam_buf)  == RET_OK)
//	  {
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
//		  camera_stopCap();
//		  f_mount(&FatFs, "", 0);
//		  f_open(&file, "image3.raw", FA_OPEN_ALWAYS | FA_CREATE_ALWAYS | FA_WRITE);
//		  HAL_Delay(5);
////			  HAL_UART_Transmit_IT(&huart3, send_data, send_data_size);
//		  HAL_Delay(5);
//
//		  for (int i = 0; i < 320 * 120 * 2; i += 2)
//		  {
//			  f_write(&file, &cam_buf[i], 2, &bw);
//			  HAL_UART_Transmit_IT(&huart3, &cam_buf[i], 2);
//		  }
//		  fresult = f_close (&file);
//		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // captured
//		  HAL_Delay(2000);
//	  } else {
//		  printf("not good");
//	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pins : PA1 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
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
