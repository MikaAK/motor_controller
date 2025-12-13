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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart_print(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

static inline void set_pin_high(GPIO_TypeDef *port, uint16_t pin) { HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); }
static inline void set_pin_low(GPIO_TypeDef *port, uint16_t pin) { HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); }
static inline void delay_ms(uint32_t ms) { HAL_Delay(ms); }

static inline void set_step_high() { set_pin_high(TMC_STEP_GPIO_Port, TMC_STEP_Pin); }
static inline void set_step_low() { set_pin_low(TMC_STEP_GPIO_Port, TMC_STEP_Pin); }

static inline void enable_motor() {
  uart_print("Enabling motor...\r\n");

  set_pin_low(TMC_EN_GPIO_Port, TMC_EN_Pin);
}
static inline void disable_motor() {
  uart_print("Disable motor...\r\n");

  set_pin_high(TMC_EN_GPIO_Port, TMC_EN_Pin);
}

static inline void set_dir_high() {
  uart_print("Set direction high...\r\n");

  set_pin_high(TMC_DIR_GPIO_Port, TMC_DIR_Pin);
}

static inline void set_dir_low() {
  uart_print("Set direction low...\r\n");

  set_pin_low(TMC_DIR_GPIO_Port, TMC_DIR_Pin);
}

static inline void cycle_step(int delay) {
  set_step_high();
  delay_ms(delay);
  set_step_low();
  delay_ms(delay);
}

// Motion
typedef struct {
  int current_delay;
  int target_delay;
  int accel_step;
  int steps_since_last_accel;
} stepper_motion_t;

stepper_motion_t motion;

static inline void start_move(int start_delay, int target_delay) {
  motion.current_delay = start_delay;
  motion.target_delay  = target_delay;
  motion.accel_step    = 1;

  motion.steps_since_last_accel    = 0;
}

static inline void update_motion(void) {
  cycle_step(motion.current_delay);

  if (motion.steps_since_last_accel > 10) {
    uart_print("Accelerating motor...\r\n");
    if (motion.current_delay > motion.target_delay)
      motion.current_delay -= motion.accel_step;

    if (motion.current_delay < motion.target_delay)
      motion.current_delay -= motion.accel_step;

    motion.steps_since_last_accel = 0;
  } else {
    motion.steps_since_last_accel++;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uart_print("=== Booting ===\r\n");
  set_dir_high();
  enable_motor();
  /* USER CODE END 2 */


  start_move(50, 1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */


    update_motion();

    // How is this faster??:
    // cycle_step(0);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
