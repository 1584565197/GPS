/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "lcd_i2c.h"
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

uint8_t rxbuffer[1000];
volatile uint8_t uartok=1;
volatile uint16_t rx_length = 0;
volatile int record=0;
volatile int raw_data_trans=0;
volatile int sendposition=0;
const char delimiter[]="\r\n";
typedef struct{
	char time[10];
	float lat;
	char latdir;
	float lon;
	char londir;
	float speed;
	char safe;
}nemarmc;
nemarmc gps;

typedef struct
{
	float N;
	float E;
}track;
track data[600];
int data_index=0;
float degree[2];
float minute[2];
char output[100];
char lcd_row0[20];
char lcd_row1[20];
char lcd_row2[20];
char lcd_row3[20];
uint8_t smiley_char[8] = {
    0b00000,
    0b01010,
    0b01010,
    0b00000,
    0b10001,
    0b01110,
    0b00000,
    0b00000
};

// ????? - ??
uint8_t heart_char[8] = {
    0b00000,
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  //HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_DMA(&huart2,rxbuffer,sizeof(rxbuffer));
	LCD_Init(&hi2c2);
	LCD_Backlight(1);
	LCD_CreateCustomChar(0, smiley_char);
  LCD_CreateCustomChar(1, heart_char);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(uartok==1)
		{
			uartok=0;
			//strcpy((char*)rxbuffer,"$BDRMC,021429.00,A,2942.95324,N,10647.29473,E,0.26,,190625,,,A,V*21\r\n$BDVTG,,,,,0.26,N,0.49,K,A*22\r\n$BDGGA,021429.00,2942.95324,N,10647.29473,E,1,08,2.1,297.09,M,-35.06,M,,*59\r\n$BDGSA,A,3,07,10,26,29,35,36,38,45,,,,,3.2,2.1,2.4,4*3F\r\n$BDGSV,3,1,11,07,78,061,32,08,,,28,10,77,351,30,13,,,38,1*77\r\n$BDGSV,3,2,11,26,10,314,32,29,56,341,36,35,13,288,34,36,27,204,36,1*7F\r\n$BDGSV,3,3,11,38,71,172,36,40,,,25,45,36,261,35,1*7D\r\n$BDGLL,2942.95324,N,10647.29473,E,021429.00,A,A*7F\r\n$BDZDA,021429.00,19,06,2025,00,00*70\r\n$GPTXT,01,01,01,ANTENNA OPEN*25");
			//rx_length=sizeof(rxbuffer);
			//HAL_Delay(1000);
			if(rx_length>0)
			{
				if(raw_data_trans!=0)CDC_Transmit_FS(rxbuffer,rx_length);
				char *token1=strtok((char*)rxbuffer,delimiter);
				while(token1!=NULL)
				{
					if(strstr(token1,"RMC")!=NULL)
					{
						char *token2=strtok(token1,",");
						int field=0;
						while(token2!=NULL)
						{
							switch(field)
							{
								case 1:
									strncpy(gps.time, token2, sizeof(gps.time)-1);break;
								case 2:
									gps.safe=*token2;break;
								case 3:
									gps.lat=atof(token2);break;
								case 4:
									gps.latdir=*token2;break;
								case 5:
									gps.lon=atof(token2);break;
								case 6:
									gps.londir=*token2;break;
								case 7:
									gps.speed=atof(token2);break;
							}
							token2=strtok(NULL,",");
							field++;

						}
						snprintf(output, sizeof(output), 
							"Time: %s, N: %2.4f, E:%2.4f\r\n",
							gps.time,degree[0]+minute[0],degree[1]+minute[1]);
						char time[2]={gps.time[0],gps.time[1]};
						int hour=atoi(time);
						hour+=8;
						snprintf(lcd_row0,sizeof(lcd_row0),
							"TIME:%d:%c%c:%c%c,%d%d%d",hour%24,gps.time[2],gps.time[3],gps.time[4],gps.time[5],record,raw_data_trans,sendposition);
						//CDC_Transmit_FS((uint8_t*)output,strlen(output));
						LCD_Clear();
						LCD_SetCursor(0,0);
						LCD_WriteString(lcd_row0);
						if(gps.safe=='A')
							{
								gps.lat=gps.latdir=='N'?gps.lat:-gps.lat;
								gps.lon=gps.londir=='E'?gps.lon:-gps.lon;
								degree[0]=(int)gps.lat/100;
								minute[0]=(gps.lat-degree[0]*100.0)/60.0;
								degree[1]=(int)gps.lon/100;
								minute[1]=(gps.lon-degree[1]*100.0)/60.0;
							

								snprintf(lcd_row1,sizeof(lcd_row1),
								"N:%2.4f",degree[0]+minute[0]);
								snprintf(lcd_row2,sizeof(lcd_row2),
									"E:%2.4f",degree[1]+minute[1]);
								snprintf(lcd_row3,sizeof(lcd_row3),
								"V:%.2fkm/h",gps.speed*0.511*3.6);
								
								if(record!=0)
									{
										LCD_SetCursor(3,19);
										LCD_WriteChar(1);
										data[data_index].N=degree[0]+minute[0];
										data[data_index].E=degree[1]+minute[1];
										data_index++;
									}
									else
									{}
							}
						else
							{
								snprintf(lcd_row1,sizeof(lcd_row1),"N:UNKOWN");
								snprintf(lcd_row2,sizeof(lcd_row2),"E:UNKOWN");
								snprintf(lcd_row3,sizeof(lcd_row3),"V:UNKOWN");
							}
						LCD_SetCursor(1,0);
						LCD_WriteString(lcd_row1);
						LCD_SetCursor(2,0);
						LCD_WriteString(lcd_row2);
						LCD_SetCursor(3,0);
						LCD_WriteString(lcd_row3);
						break;
					}
					else
					{
						LCD_Clear();
						LCD_SetCursor(0,0);
						LCD_WriteString("SR2631 ERROR!");
						LCD_SetCursor(1,0);
						LCD_WriteString("NO RMC");
						HAL_Delay(1000);
					}
					token1=strtok(NULL,delimiter);
				}
				rx_length=0;
				if(sendposition!=0)
				{
					char position[30];
					LCD_SetCursor(0,19);
					LCD_WriteChar(0);
					for(int i=0;i<data_index;i++)
					{
						snprintf(position,sizeof(position),"\x02%f,%f\x03",data[i].N,data[i].E);
						CDC_Transmit_FS((uint8_t*)position,sizeof(position));
					}
					sendposition=0;
				}
				else
				{}
				HAL_UART_Receive_DMA(&huart2,rxbuffer,sizeof(rxbuffer));
				
		}
		else
		{
			LCD_Clear();
			LCD_SetCursor(0,0);
			LCD_WriteString("SR2631 ERROR!");
			LCD_SetCursor(1,0);
			LCD_WriteString("NO DATA");
			HAL_Delay(1000);
		}
	}
		else
		{}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void USART2_IRQHandler(void)
{
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    uint32_t tmp = 0;
    tmp = huart2.hdmarx->Instance->CNDTR;
    rx_length = sizeof(rxbuffer) - tmp;
		HAL_UART_DMAStop(&huart2);
		__disable_irq();    
		uartok=1;
		__enable_irq();
  }
  
  HAL_UART_IRQHandler(&huart2);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{  
  if(GPIO_pin==GPIO_PIN_3)
  {
		record=!record;
  }
  
  if(GPIO_pin==GPIO_PIN_4)
  {
		sendposition=!sendposition;
  }
}
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
