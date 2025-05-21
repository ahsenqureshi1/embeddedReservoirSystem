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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B);
int findMin(int zone1, int zone2, int zone3);
void ADC_Select_CH(int CH);
float GetPwmPercentage();
void DisplayWaterDepth();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int clockSecs = 0;
int mins = 0;
int hours = 0;

int terminated = 0;

uint8_t commandToStartDistanceSensor = 0x55;
volatile uint16_t distance = 0;

volatile uint8_t distanceValueRxFlag = 0;
uint8_t distanceValueReturnedBuff[2] = {0};
uint8_t rcv_intpt_flag = 00;
int twoBytesRecieved = 0;
uint8_t bytes[2] = {0};

uint8_t puttyPrintMessage[256] = {0};

volatile uint8_t inletMotorSpeed = 0;
volatile uint8_t zone1MotorSpeed = 0;
volatile uint8_t zone2MotorSpeed = 0;
volatile uint8_t zone3MotorSpeed = 0;

volatile uint8_t wallClockTime = 0;

volatile uint8_t inletStartTime = 0;

volatile uint8_t inletStopTme = 0;
volatile uint8_t zone1StartTime = 0;
volatile uint8_t zone1StopTme = 0;

volatile uint8_t zone2StartTime = 0;
volatile uint8_t zone2StopTme = 0;
volatile uint8_t zone3StartTime = 0;
volatile uint8_t zone3StopTme = 0;

volatile int inletRunning = 0;
volatile int zone1Running = 0;
volatile int zone2Running = 0;
volatile int zone3Running = 0;
volatile int printCondition = 0;

volatile int firstRun = 1;
volatile int currentlyActiveZone = -1;
volatile int currentMotorPwm = 0;
volatile int currentMotorRpm = 0;

volatile int deltaTime = 0;
volatile int speed = 0;

volatile int inletRunTime = 0;
volatile int zone1RunTime = 0;
volatile int zone2RunTime = 0;
volatile int zone3RunTime = 0;

int waterReservoirDepth = 0;

volatile int rpmTickValue = 0;

volatile int setupModeFlag = 1;

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    setupModeFlag = 1;

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim4);

    TIM2->PSC = 16 - 1;
    TIM2->ARR = 20000 - 1;
    TIM2->CCR1 = 2500;

    // DC MOTOR
    HAL_TIM_Base_Init(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    TIM3->PSC = 16 - 1;
    //   TIM3->ARR = 2000-1;
    TIM3->CCR1 = 0;
    TIM3->CCR3 = 0;

    sprintf((char *)puttyPrintMessage, "\r\n SETUP MODE ");
    HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
        if (setupModeFlag == 1)
        {
            rcv_intpt_flag = 00;
            sprintf((char *)puttyPrintMessage, "\r\n INLET MOTOR SPEED PWM (option 0-3): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, &bytes[0], 1);
            while (rcv_intpt_flag == (00))
            {
            }
            inletMotorSpeed = bytes[0] - 48;
            rcv_intpt_flag = (00);
            bytes[0] = 0;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 1 MOTOR SPEED PWM (option 0-3): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, &bytes[0], 1);
            while (rcv_intpt_flag == (00))
            {
            }
            zone1MotorSpeed = bytes[0] - 48;
            rcv_intpt_flag = (00);
            bytes[0] = 0;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 2 MOTOR SPEED PWM (option 0-3): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, &bytes[0], 1);

            while (rcv_intpt_flag == (00))
            {
            }
            zone2MotorSpeed = bytes[0] - 48;
            rcv_intpt_flag = (00);
            bytes[0] = 0;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 3 MOTOR SPEED PWM (option 0-3): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, &bytes[0], 1);

            while (rcv_intpt_flag == (00))
            {
            }
            zone3MotorSpeed = bytes[0] - 48;
            rcv_intpt_flag = (00);
            bytes[0] = 0;
            twoBytesRecieved = 1;
            sprintf((char *)puttyPrintMessage, "\r\n CURRENT WALL CLOCK TIME (0-23): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            int value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            wallClockTime = value;
            hours = wallClockTime;

            sprintf((char *)puttyPrintMessage, "\r\n INLET WALL CLOCK START TIME (0-23): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            inletStartTime = value;

            sprintf((char *)puttyPrintMessage, "\r\n INLET WALL CLOCK STOP TIME (0-23) : ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            inletStopTme = value;

            inletRunTime = (inletStopTme - inletStartTime) * 6;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 1 WALL CLOCK START TIME (0-23): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            zone1StartTime = value;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 1 WALL CLOCK STOP TIME (0-23) : ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            zone1StopTme = value;

            zone1RunTime = (zone1StopTme - zone1StartTime) * 6;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 2 WALL CLOCK START TIME (0-23): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            zone2StartTime = value;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 2 WALL CLOCK STOP TIME (0-23) : ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            zone2StopTme = value;

            zone2RunTime = (zone2StopTme - zone2StartTime) * 6;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 3 WALL CLOCK START TIME (0-23): ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            zone3StartTime = value;

            sprintf((char *)puttyPrintMessage, "\r\n ZONE 3 WALL CLOCK STOP TIME (0-23) : ");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
            HAL_UART_Receive_IT(&huart6, bytes, 2);

            while (rcv_intpt_flag == (00))
            {
            }
            rcv_intpt_flag = (00);
            value = (bytes[0] - 48) * 10 + (bytes[1] - 48);
            bytes[0] = 0;
            bytes[1] = 0;
            zone3StopTme = value;

            zone3RunTime = (zone3StopTme - zone3StartTime) * 6;

            setupModeFlag = 0;
            // Wait for blue button to be pressed
            while (HAL_GPIO_ReadPin(GPIOC, B1_Pin) == 1)
            {
                HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
                HAL_Delay(10);
                HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
                HAL_Delay(490);
            }

            HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);

            clockSecs = 0;

            sprintf((char *)puttyPrintMessage, "\r\n WALL CLOCK | ZONE/INLET | MOTOR PWM | MOTOR RPM | WATER DEPTH");
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
        }

        // END OF SETUP MODE

        // START OF RUN MODE
        if (clockSecs >= 6)
        {
            hours++;
            clockSecs = 0;
            printCondition = 1;
            DisplayWaterDepth();
        }
        DisplayWaterDepth();
        // STARTING OF INLET
        HAL_UART_Receive_IT(&huart1, distanceValueReturnedBuff, 2);
        HAL_UART_Transmit(&huart1, &commandToStartDistanceSensor, 1, 500);

        while (terminated == 1)
        {
            TIM3->CCR3 = 0;
            TIM3->CCR1 = 0;
            HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, RGB_RED_Pin, GPIO_PIN_SET);
            HAL_Delay(500);

            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, RGB_GREEN_Pin, GPIO_PIN_SET);
            HAL_Delay(500);

            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin, GPIO_PIN_SET);
            HAL_Delay(500);
        }

        if ((hours == inletStartTime) && (inletRunning == 0))
        {
            inletRunning = 1;
            currentlyActiveZone = 0;
            if (inletMotorSpeed == 0)
            {
                float adc = GetPwmPercentage();
                float adc_speed = (adc * 2000);
                TIM3->CCR1 = adc_speed;
                currentMotorPwm = (int)(adc * 100);
            }

            TIM2->CCR1 = 500;
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin, GPIO_PIN_SET);

            // Rotating DC Motor for inlet
            TIM3->CCR3 = 0;
            if (inletMotorSpeed == 1)
            {
                // 60% pwm -> 0.6x2000
                TIM3->CCR1 = 1200;
                currentMotorPwm = 60;
            }
            if (inletMotorSpeed == 2)
            {
                // 80% pwm -> 0.8x2000
                TIM3->CCR1 = 1600;
                currentMotorPwm = 80;
            }
            if (inletMotorSpeed == 3)
            {
                // 99% pwm -> 0.99x2000
                TIM3->CCR1 = 1980;
                currentMotorPwm = 99;
            }
        }

        // END INLET
        if ((hours == inletStopTme) && (inletRunning == 1))
        {
            if ((hours == zone3StartTime) || (hours == zone2StartTime) || (hours == zone1StartTime))
            {
            }
            else
            {
                TIM3->CCR1 = 0;
                TIM3->CCR3 = 0;
                HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            }
        }

        // ZONE 1 START
        if ((hours == zone1StartTime) && (zone1Running == 0))
        {
            zone1Running = 1;
            currentlyActiveZone = 1;
            if (zone1MotorSpeed == 0)
            {
                float adc = GetPwmPercentage();
                float adc_speed = (adc * 2000);
                TIM3->CCR3 = adc_speed;
                currentMotorPwm = (int)(adc * 100);
            }

            TIM2->CCR1 = 2000;
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_RED_Pin, GPIO_PIN_SET);

            TIM3->CCR1 = 0;
            if (zone1MotorSpeed == 1)
            {
                // 60% pwm -> 0.6x2000
                TIM3->CCR3 = 1200;
                currentMotorPwm = 60;
            }
            if (zone1MotorSpeed == 2)
            {
                // 80% pwm -> 0.8x2000
                TIM3->CCR3 = 1600;
                currentMotorPwm = 80;
            }
            if (zone1MotorSpeed == 3)
            {
                // 99% pwm -> 0.99x2000
                TIM3->CCR3 = 1980;
                currentMotorPwm = 99;
            }
        }

        // ZONE 1 END
        if ((hours == zone1StopTme) && (zone1Running == 1))
        {
            if ((hours == zone3StartTime) || (hours == zone2StartTime) || (hours == zone1StartTime))
            {
            }
            else
            {
                TIM3->CCR1 = 0;
                TIM3->CCR3 = 0;
                HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            }
        }

        // ZONE 2 START
        if ((hours == zone2StartTime) && (zone2Running == 0))
        {
            zone2Running = 1;
            currentlyActiveZone = 2;
            if (zone2MotorSpeed == 0)
            {
                float adc = GetPwmPercentage();
                float adc_speed = (adc * 2000);
                TIM3->CCR3 = adc_speed;
                currentMotorPwm = (int)(adc * 100);
            }

            TIM2->CCR1 = 1500;
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_GREEN_Pin, GPIO_PIN_SET);

            TIM3->CCR1 = 0;
            if (zone2MotorSpeed == 1)
            {
                TIM3->CCR3 = 1200;
                currentMotorPwm = 60;
            }
            if (zone2MotorSpeed == 2)
            {
                TIM3->CCR3 = 1600;
                currentMotorPwm = 80;
            }
            if (zone2MotorSpeed == 3)
            {
                TIM3->CCR3 = 1980;
                currentMotorPwm = 99;
            }
        }

        // ZONE 2 END
        if ((hours == zone2StopTme) && (zone2Running == 1))
        {
            if ((hours == zone3StartTime) || (hours == zone2StartTime) || (hours == zone1StartTime))
            {
            }
            else
            {
                TIM3->CCR1 = 0;
                TIM3->CCR3 = 0;
                HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            }
        }

        // ZONE 3 START
        if ((hours == zone3StartTime) && (zone3Running == 0))
        {
            zone3Running = 1;
            currentlyActiveZone = 3;
            if (zone3MotorSpeed == 0)
            {
                float adc = GetPwmPercentage();
                float adc_speed = (adc * 2000);
                TIM3->CCR3 = adc_speed;
                currentMotorPwm = (int)(adc * 100);
            }

            TIM2->CCR1 = 1000;
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin, GPIO_PIN_SET); // turn on blue

            TIM3->CCR1 = 0;
            if (zone3MotorSpeed == 1)
            {
                // 60% pwm -> 0.6x2000
                TIM3->CCR3 = 1200;
                currentMotorPwm = 60;
            }
            if (zone3MotorSpeed == 2)
            {
                // 80% pwm -> 0.8x2000
                TIM3->CCR3 = 1600;
                currentMotorPwm = 80;
            }
            if (zone3MotorSpeed == 3)
            {
                // 99% pwm -> 0.99x2000
                TIM3->CCR3 = 1980;
                currentMotorPwm = 99;
            }
        }

        // ZONE 3 END
        if ((hours == zone3StopTme) && (zone3Running == 1))
        {
            currentMotorPwm = 0;
            if ((hours == zone3StartTime) || (hours == zone2StartTime) || (hours == zone1StartTime))
            {
            }
            else
            {
                TIM3->CCR1 = 0;
                TIM3->CCR3 = 0;
                HAL_GPIO_WritePin(GPIOA, LD2_Pin | RGB_BLUE_Pin | RGB_RED_Pin | RGB_GREEN_Pin, GPIO_PIN_RESET);
            }
        }
        if (printCondition == 1)
        {

            printCondition = 0;
            sprintf((char *)puttyPrintMessage, "\r\n %02d | %02d | %02d | %02d | %02d ", hours, currentlyActiveZone, currentMotorPwm, speed, waterReservoirDepth);
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
        }
        if (firstRun == 1 && clockSecs == 1)
        {
            DisplayWaterDepth();
            firstRun = 0;
            sprintf((char *)puttyPrintMessage, "\r\n %02d | %02d | %02d | %02d | %02d ", hours, currentlyActiveZone, currentMotorPwm, speed, waterReservoirDepth);
            HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
        }

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 16000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RGB_BLUE_Pin|RGB_GREEN_Pin|RGB_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin|DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_A2_Pin|DIGIT_A3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RGB_BLUE_Pin RGB_GREEN_Pin RGB_RED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RGB_BLUE_Pin|RGB_GREEN_Pin|RGB_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_TICK_Pin */
  GPIO_InitStruct.Pin = RPM_TICK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_B3_Pin DIGIT_B0_Pin DIGIT_B1_Pin DIGIT_B2_Pin */
  GPIO_InitStruct.Pin = DIGIT_B3_Pin|DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_A0_Pin DIGIT_A1_Pin DIGIT_A2_Pin DIGIT_A3_Pin */
  GPIO_InitStruct.Pin = DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_A2_Pin|DIGIT_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int findMin(int zone1, int zone2, int zone3)
{
    int min = zone1;
    int zoneNumber = 1;

    if (zone2 < min)
    {
        min = zone2;
        zoneNumber = 2;
    }

    if (zone3 < min)
    {
        min = zone3;
        zoneNumber = 3;
    }
    return zoneNumber;
}

void ADC_Select_CH(int CH)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    switch (CH)
    {
    case 0:
        sConfig.Channel = ADC_CHANNEL_0;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 1:
        sConfig.Channel = ADC_CHANNEL_1;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 2:
        sConfig.Channel = ADC_CHANNEL_2;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 3:
        sConfig.Channel = ADC_CHANNEL_3;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 4:
        sConfig.Channel = ADC_CHANNEL_4;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 5:
        sConfig.Channel = ADC_CHANNEL_5;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 6:
        sConfig.Channel = ADC_CHANNEL_6;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 7:
        sConfig.Channel = ADC_CHANNEL_7;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 8:
        sConfig.Channel = ADC_CHANNEL_8;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 9:
        sConfig.Channel = ADC_CHANNEL_9;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 10:
        sConfig.Channel = ADC_CHANNEL_10;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 11:
        sConfig.Channel = ADC_CHANNEL_11;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 12:
        sConfig.Channel = ADC_CHANNEL_12;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 13:
        sConfig.Channel = ADC_CHANNEL_13;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 14:
        sConfig.Channel = ADC_CHANNEL_14;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    case 15:
        sConfig.Channel = ADC_CHANNEL_15;
        sConfig.Rank = 1;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        break;
    }
}

float GetPwmPercentage()
{
    ADC_Select_CH(9);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return (ADC_CH9 / 255.00);
}

void DisplayWaterDepth()
{
    distance = (distanceValueReturnedBuff[0] << 8) | distanceValueReturnedBuff[1];
    if (distance > 200) // 15 cm
    {
        DIGITS_Display(10, 0);
        terminated = 1;
        waterReservoirDepth = 0;
        sprintf((char *)puttyPrintMessage, "\r\n RESERVOIR IS EMPTY");
        HAL_UART_Transmit(&huart6, puttyPrintMessage, strlen((char *)puttyPrintMessage), 1000);
    }
    else if (distance < 70) // 7 cm
    {
        waterReservoirDepth = 99;
        DIGITS_Display(9, 9);
    }
    else
    {
        uint8_t dig_1 = (100 - floor((distance / 500.0) * 100)) / 10;
        uint8_t dig_2 = 100 - (floor((distance / 500.0) * 100));
        waterReservoirDepth = (dig_1 * 10) + (dig_2 % 10);
        DIGITS_Display(dig_1, dig_2 % 10);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        if (twoBytesRecieved == 0)
        {
            HAL_UART_Transmit(&huart6, &bytes[0], 1, 100);
        }
        else
        {
            HAL_UART_Transmit(&huart6, bytes, 2, 200);
        }

        rcv_intpt_flag = 1;
    }

    if (huart->Instance == USART1)
    {
        distanceValueRxFlag = 01;
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        clockSecs += 1;
        if (clockSecs == 6)
        {
            speed = (rpmTickValue / 20.0);
            rpmTickValue = 0;
        }
    }
}

void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B)
{
    uint8_t DIGITA_VAL = 0x0F & DIGIT_A;
    int Abit0 = (DIGITA_VAL) & 1;
    int Abit1 = (DIGITA_VAL >> 1) & 1;
    int Abit2 = (DIGITA_VAL >> 2) & 1;
    int Abit3 = (DIGITA_VAL >> 3) & 1;

    uint8_t DIGITB_VAL = 0x0F & DIGIT_B;
    int Bbit0 = (DIGITB_VAL) & 1;
    int Bbit1 = (DIGITB_VAL >> 1) & 1;
    int Bbit2 = (DIGITB_VAL >> 2) & 1;
    int Bbit3 = (DIGITB_VAL >> 3) & 1;

    if (Abit0 == (0))
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
    }
    if (Abit1 == (0))
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);
    }
    if (Abit2 == (0))
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);
    }
    if (Abit3 == (0))
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);
    }

    if (Bbit0 == (0))
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET);
    }
    if (Bbit1 == (0))
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET);
    }
    if (Bbit2 == (0))
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_SET);
    }
    if (Bbit3 == (0))
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == RPM_TICK_Pin)
    {
        rpmTickValue += 1;
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
