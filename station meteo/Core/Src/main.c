/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lcd16x2_i2c.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofweek;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} TimeStruct;
typedef struct {
    char heure[10];
    char date[14];
} HorlogeMessage;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
TimeStruct time;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
osThreadId TmesureHandle;
osThreadId ThorlogeHandle;
osThreadId TaffichageHandle;
/* USER CODE BEGIN PV */
static const uint8_t LM75_ADDR = 0x48 << 1;
static const uint8_t REG_TEMP = 0x00;

static const uint16_t DS3231_ADDR= 0x68 << 1;
static const uint8_t REG_HOR = 0x00;
QueueHandle_t Qtemperature, QHorloge;
SemaphoreHandle_t  Mutex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void vTmesure(void const * argument);
void vThorloge(void const * argument);
void vTaffichage(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t decToBcd(int);
int bcdToDec(uint8_t);
void Get_Time (void);
void Set_Time (uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Set_Time(15, 8, 4, 7, 5, 25); //minutes; hour; dayofweek; dayofmonth; month; year;
  lcd16x2_i2c_init(&hi2c1);
  lcd16x2_i2c_clear();
  lcd16x2_i2c_display(true);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of Mutex */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  Mutex= xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  Qtemperature=xQueueCreate(1,sizeof(char[16]));
  QHorloge=xQueueCreate(1,sizeof(HorlogeMessage));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Tmesure */
  osThreadDef(Tmesure, vTmesure, osPriorityNormal, 0, 128);
  TmesureHandle = osThreadCreate(osThread(Tmesure), NULL);

  /* definition and creation of Thorloge */
  osThreadDef(Thorloge, vThorloge, osPriorityNormal, 0, 128);
  ThorlogeHandle = osThreadCreate(osThread(Thorloge), NULL);

  /* definition and creation of Taffichage */
  osThreadDef(Taffichage, vTaffichage, osPriorityNormal, 0, 128);
  TaffichageHandle = osThreadCreate(osThread(Taffichage), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t decToBcd(int val) {
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

// Convert BCD to normal decimal
int bcdToDec(uint8_t val) {
    return (int)((val / 16 * 10) + (val % 16));
}

void Get_Time(void) {
    uint8_t get_time[6];
    if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, REG_HOR, 1, get_time, 6, 1000) == HAL_OK) {
        time.minutes     = bcdToDec(get_time[1]);
        time.hour        = bcdToDec(get_time[2] & 0x3F);
        time.dayofweek   = bcdToDec(get_time[3]);
        time.dayofmonth  = bcdToDec(get_time[4]);
        time.month       = bcdToDec(get_time[5] & 0x1F);
        time.year        = bcdToDec(get_time[6]);
    }
}

void Set_Time(uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year) {
    uint8_t set_time[7];
    set_time[0] = 0;
    set_time[1] = decToBcd(min);
    set_time[2] = decToBcd(hour);
    set_time[3] = decToBcd(dow);
    set_time[4] = decToBcd(dom);
    set_time[5] = decToBcd(month);
    set_time[6] = decToBcd(year);

    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, REG_HOR, 1, set_time, 7, 1000);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_vTmesure */
/**
  * @brief  Function implementing the Tmesure thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vTmesure */
void vTmesure(void const * argument)
{
  /* USER CODE BEGIN 5 */
	  HAL_StatusTypeDef ret;
	  uint8_t buf[2];
	  int16_t raw;
	  float temp_c;
	  char Tmessage[16];
	  buf[0] = REG_TEMP;
  /* Infinite loop */
  for(;;)
  {
	      ret = HAL_I2C_Master_Transmit(&hi2c1, LM75_ADDR, buf, 1, HAL_MAX_DELAY);
	      if ( ret != HAL_OK ) {
	    	  printf("Error Tx");
	      } else {

	        // Read 2 bytes from the temperature register
	        ret = HAL_I2C_Master_Receive(&hi2c1, LM75_ADDR, buf, 2, HAL_MAX_DELAY);
	        if ( ret != HAL_OK ) {
	        	printf("Error Rx");
	        } else {

	        	raw = ((int16_t)buf[0] << 8) | buf[1];
	     	    raw >>= 5;
	        	if (raw & (1 << 10)) {
	        	    raw |= 0xF800; // étendre sur 16 bits
	        	}
	        	temp_c = raw * 0.125f;
	        	snprintf(Tmessage, sizeof(Tmessage), "%d.%02d C\r\n",
	        	         (int)temp_c, (int)((temp_c - (int)temp_c) * 100));
	        }
	      }

	      xQueueSendToBack(Qtemperature, Tmessage, portMAX_DELAY);
	vTaskDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vThorloge */
/**
* @brief Function implementing the Thorloge thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vThorloge */
void vThorloge(void const * argument) {
    HorlogeMessage Horloge;

    for (;;) {

            Get_Time();

            sprintf(Horloge.heure, "%02d:%02d", time.hour, time.minutes);
            sprintf(Horloge.date, "%02d-%02d-20%02d", time.dayofmonth, time.month, time.year);

            xQueueSendToBack(QHorloge, &Horloge, portMAX_DELAY);

        vTaskDelay(1000);
    }
}


/* USER CODE BEGIN Header_vTaffichage */
/**
* @brief Function implementing the Taffichage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaffichage */
void vTaffichage(void const * argument)
{
  /* USER CODE BEGIN vTaffichage */
	HorlogeMessage Horloge;
	char Tmessage[16];
  /* Infinite loop */
    for (;;)
    {

            // Receive and display time
            if (xQueueReceive(QHorloge, &Horloge, pdMS_TO_TICKS(100)) == pdPASS)
            {
                lcd16x2_i2c_setCursor(0, 0);
                lcd16x2_i2c_printf("%s", Horloge.heure);

                lcd16x2_i2c_setCursor(1, 0);
                lcd16x2_i2c_printf("%s", Horloge.date);
            }
            else
            {
                printf("Error receiving Horloge\n");
            }

            // Receive and display temperature
            if (xQueueReceive(Qtemperature, Tmessage, pdMS_TO_TICKS(100)) == pdPASS)
            {
                lcd16x2_i2c_setCursor(0, 10);
                lcd16x2_i2c_printf("Temp= %s", Tmessage);
            }
            else
            {
                printf("Error receiving Tmessage\n");
            }
            lcd16x2_i2c_display(true);
            vTaskDelay(1000);
    }
  /* USER CODE END vTaffichage */
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
  if (htim->Instance == TIM1)
  {
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
