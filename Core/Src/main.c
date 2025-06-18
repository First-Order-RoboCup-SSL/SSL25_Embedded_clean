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
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_pid.h"
#include "bsp_motor_feedback.h"
#include "bsp_inverse_kinematics.h"
#include "bsp_fdcan.h"
#include "bsp_sbus_controller.h"
#include <math.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEADER 0xAA
#define FRAME_FOOTER 0x55
#define FRAME_SIZE 12  // 1 byte header + 10 bytes data + 1 byte footer

/* Motor control parameters */
#define MOTOR_COUNT 3
#define MOTOR_ID_1 0x201
#define MOTOR_ID_2 0x202
#define MOTOR_ID_3 0x203
#define RPM_TO_RADS 0.10472f  // 2*pi/60 to convert RPM to rad/s

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t remote_IN[FRAME_SIZE];
uint16_t decoded[6];
uint8_t sync_state = 0;  // 0: looking for header, 1: receiving frame

// Debug flags
volatile uint8_t can_init_status = 0;    // 0=not initialized, 1=success, 2=failed
volatile uint32_t can_tx_count = 0;      // Count of successful transmissions
volatile uint32_t can_error_count = 0;   // Count of transmission errors

float motor1_error_sum = 0.0f;
float motor1_last_error = 0.0f;
float motor2_error_sum = 0.0f;
float motor2_last_error = 0.0f;
float motor3_error_sum = 0.0f;
float motor3_last_error = 0.0f;

// 全局变量，便于调试器监控
float output1 = 0, output2 = 0, output3 = 0;
int16_t output1_int = 0, output2_int = 0, output3_int = 0;
// 新增：滤波后的输出
float output1_filtered = 0, output2_filtered = 0, output3_filtered = 0;

// 新增：滤波系数，便于调试
volatile float alpha = 0.2f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirect printf to UART
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void decode_uart_buffer(uint8_t* uart_buffer, uint16_t* decoded) {
    // Decode ADC values (4 channels)
    decoded[0] = (uart_buffer[0] << 8) | uart_buffer[1];  // ADC1
    decoded[1] = (uart_buffer[2] << 8) | uart_buffer[3];  // ADC2
    decoded[2] = (uart_buffer[4] << 8) | uart_buffer[5];  // ADC3
    decoded[3] = (uart_buffer[6] << 8) | uart_buffer[7];  // ADC4
    // Decode button states (2 channels)
    decoded[4] = uart_buffer[8];  // AUX1 button
    decoded[5] = uart_buffer[9];  // AUX2 button
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART10_UART_Init();
  MX_UART7_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_FDCAN3_Init();
  MX_TIM8_Init();
  MX_TIM23_Init();
  /* USER CODE BEGIN 2 */
  /*power init begin*/
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
  
  // Initialize FDCAN with BSP function
  BSP_FDCAN_Init();
  BSP_InverseKinematics_Init();
  can_init_status = 1;  // Success
  
  // Initialize SBUS receiver
  BSP_SBUS_Init();
  BSP_SBUS_UART_StartReceive();
  
  // Start UART reception with single byte
  HAL_UART_Receive_IT(&huart1, remote_IN, FRAME_SIZE);
  
  HAL_TIM_Base_Start_IT(&htim4);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Check SBUS signal status
    if (!BSP_SBUS_IsSignalValid()) {
        // If signal is lost, stop all motors
        uint8_t can_cmd[8] = {0};  // All zeros = stop
        BSP_FDCAN_Send_Message(&hfdcan3, 0x200, can_cmd, 8);
        HAL_Delay(g_control_period_ms);
        continue;  // Skip the rest of the loop
    }
    
    // 1. Map remote control values to velocities
    float vx, vy, omega;
    int16_t* sbus_mapped = BSP_SBUS_GetMappedChannels();
    
    // Map SBUS channels to velocities according to the specified mapping:
    // sbus_mapped[1] -> decoded[2]  (第2通道 -> 第3通道)
    // sbus_mapped[0] -> decoded[3]  (第1通道 -> 第4通道)
    // sbus_mapped[3] -> decoded[0]  (第4通道 -> 第1通道)
    // sbus_mapped[2] -> decoded[1]  (第3通道 -> 第2通道)
    BSP_MapRemoteToVelocities(sbus_mapped[1],    // forward/back (原decoded[2])
                             sbus_mapped[0],    // left/right (原decoded[3])
                             sbus_mapped[3],    // rotation (原decoded[0])
                            &vx, &vy, &omega);
    
    // 2. Calculate target wheel velocities using inverse kinematics
    float wheel_velocities[3];
    BSP_InverseKinematics_Calculate(vx, vy, omega, wheel_velocities);
    
    // 3. Get actual motor velocities from FDCAN feedback
    motor_feedback_t* feedback1 = BSP_MotorFeedback_GetMotorData(0);
    motor_feedback_t* feedback2 = BSP_MotorFeedback_GetMotorData(1);
    motor_feedback_t* feedback3 = BSP_MotorFeedback_GetMotorData(2);
    
    // Convert rotor speed to wheel speed by dividing by gear ratio
    float actual_velocity1 = feedback1 ? (feedback1->rotor_speed * RPM_TO_RADS) / GEAR_RATIO : 0.0f;
    float actual_velocity2 = feedback2 ? (feedback2->rotor_speed * RPM_TO_RADS) / GEAR_RATIO : 0.0f;
    float actual_velocity3 = feedback3 ? (feedback3->rotor_speed * RPM_TO_RADS) / GEAR_RATIO : 0.0f;
    
    // 4. Calculate motor outputs using PID
    float output1 = BSP_PID_Calculate_Indiv(&pid_params[0], wheel_velocities[0], actual_velocity1);
    float output2 = BSP_PID_Calculate_Indiv(&pid_params[1], wheel_velocities[1], actual_velocity2);
    float output3 = BSP_PID_Calculate_Indiv(&pid_params[2], wheel_velocities[2], actual_velocity3);
    
    // 5. Apply low-pass filter
    output1_filtered = alpha * output1 + (1.0f - alpha) * output1_filtered;
    output2_filtered = alpha * output2 + (1.0f - alpha) * output2_filtered;
    output3_filtered = alpha * output3 + (1.0f - alpha) * output3_filtered;
    
    // 6. Convert to CAN format (-32768 to 32767)
    output1_int = (int16_t)(output1_filtered * 32767.0f / PID_OUTPUT_MAX);
    output2_int = (int16_t)(output2_filtered * 32767.0f / PID_OUTPUT_MAX);
    output3_int = (int16_t)(output3_filtered * 32767.0f / PID_OUTPUT_MAX);
    
    // 7. Pack data into CAN message
    uint8_t can_cmd[8] = {0};
    can_cmd[0] = (output1_int >> 8) & 0xFF;
    can_cmd[1] = output1_int & 0xFF;
    can_cmd[2] = (output2_int >> 8) & 0xFF;
    can_cmd[3] = output2_int & 0xFF;
    can_cmd[4] = (output3_int >> 8) & 0xFF;
    can_cmd[5] = output3_int & 0xFF;
    
    // 8. Send CAN message
    BSP_FDCAN_Send_Message(&hfdcan3, 0x200, can_cmd, 8);
    
    // 9. Control update rate (default 200Hz, but now adjustable)
    HAL_Delay(g_control_period_ms);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (sync_state == 0) {
      // Looking for frame header
      if (remote_IN[0] == FRAME_HEADER) {
        sync_state = 1;
        // Start receiving the rest of the frame
        HAL_UART_Receive_IT(&huart1, &remote_IN[1], FRAME_SIZE-1);
      } else {
        // Keep looking for header
        HAL_UART_Receive_IT(&huart1, remote_IN, 1);
      }
    } else {
      // We already have the header, check the footer
      if (remote_IN[FRAME_SIZE-1] == FRAME_FOOTER) {
        // Valid frame received, decode the data (skip header and footer)
        decode_uart_buffer(&remote_IN[1], decoded);
      }
      // Reset sync state and start looking for next header
      sync_state = 0;
      HAL_UART_Receive_IT(&huart1, remote_IN, 1);
    }
  }
  else if (huart->Instance == UART5)
  {
    BSP_SBUS_UART_RxCpltCallback(huart);
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
