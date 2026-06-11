/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "sonar_task.h"   /* Sonar_IC_Callback, Sonar_PeriodElapsed_Callback */
#include "lsm6ds3tr_c_reg.h"
#include "lis3mdl_reg.h"

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
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void UART4_Print(uint8_t* Message);
static void I2C_SelfTest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }
  HAL_UART_Transmit(&huart4, (uint8_t*)&ch, 1, 10);
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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

//  Servo test_wheel = AxelFlow_servo_init(0x03, &huart3, false);
//
//  DX64_GetFullState(&test_wheel);

  // Sygnalizacja startu (trzy diody)
  UART4_Print((uint8_t*)"Peripherals OK. Starting RTOS...\n");

  /* Test I2C — wykonywany przed FreeRTOS, wyniki przez UART4 (printf) */
  I2C_SelfTest();

  /* -----------------------------------------------------------------------
   * TRYB OSCYLOSKOPOWY: jesli #define I2C_SCOPE_MODE jest odkomentowane,
   * program wchodzi w nieskonczona petle wysylajaca pakiety I2C co 10ms.
   * Pozwala to zlapac sygnal na oscyloskopie.
   *
   * Obserwuj piny PC6 (SCL) i PC7 (SDA) na ukladzie STM32G474.
   * Jesli oscyloskop dalej nic nie widzi na tych pinach — to bledne GPIO/AF.
   *
   * ODKOMENTUJ ponizej, wgraj, sprawdz piny, potem zakomentuj z powrotem.
   * ----------------------------------------------------------------------- */
// #define I2C_SCOPE_MODE
#ifdef I2C_SCOPE_MODE
  printf("TRYB OSCYLOSKOPOWY\r\n");

  /* FAZA 1: GPIO blink na PC6 i PC7 przez 3 sekundy.
   * Jesli oscyloskop widzi sygnaly ~500Hz — jestes na wlasciwych pinach.
   * Jesli nie widzi nic — zly pin na PCB lub zle mapowanie.
   * Po 3 sekundach automatycznie przechodzimy do FAZY 2 (I2C). */
  printf("FAZA 1 (3s): Blink GPIO na PC6 i PC7 @ 500Hz. Sprawdz oscyloskop.\r\n");
  {
      /* Tymczasowo przelacz PC6/PC7 na Output Open-Drain */
      GPIO_InitTypeDef gpio_test = {0};
      gpio_test.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
      gpio_test.Mode  = GPIO_MODE_OUTPUT_OD;
      gpio_test.Pull  = GPIO_NOPULL;
      gpio_test.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOC, &gpio_test);

      for (int i = 0; i < 1500; i++) {   /* 1500 × 2ms = 3 sekundy */
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET); /* LOW  */
          HAL_Delay(1);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);   /* HIGH */
          HAL_Delay(1);
      }

      /* Przywroc PC6/PC7 jako AF8 (I2C4) */
      gpio_test.Mode      = GPIO_MODE_AF_OD;
      gpio_test.Alternate = GPIO_AF8_I2C4;
      HAL_GPIO_Init(GPIOC, &gpio_test);
      /* Reinit I2C4 po zmianie GPIO */
      HAL_I2C_DeInit(&hi2c3);
      HAL_I2C_Init(&hi2c3);
  }

  /* FAZA 2: Petla I2C — pakiety co 10ms.
   * Na oscyloskopie powinienes zobaczyc serie ~9 impulsow co 10ms na SCL. */
  printf("FAZA 2: Petla I2C IsDeviceReady co 10ms (SCL=PC6, SDA=PC7).\r\n");
  uint32_t scope_cnt = 0;
  for (;;) {
      HAL_StatusTypeDef r1 = HAL_I2C_IsDeviceReady(&hi2c3, LSM6DS3TR_C_I2C_ADD_L, 1, 5);
      HAL_StatusTypeDef r2 = HAL_I2C_IsDeviceReady(&hi2c3, LIS3MDL_I2C_ADD_L, 1, 5);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
      if ((scope_cnt % 100) == 0) {
          printf("[%lu] LSM6=%d LIS3=%d  (0=OK 1=NACK 3=TIMEOUT)\r\n",
                 scope_cnt, (int)r1, (int)r2);
      }
      scope_cnt++;
      HAL_Delay(10);
  }
#endif /* I2C_SCOPE_MODE */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ---------------------------------------------------------------------------
 * I2C_SelfTest — wywolywany raz przed FreeRTOS, wyniki przez printf (UART4).
 *
 * Co sprawdza:
 *  1. HAL_I2C_IsDeviceReady — czy peripheral w ogole generuje START na SCL/SDA.
 *     Jesli timeout (HAL_TIMEOUT=3): linia stuck lub bledne GPIO AF/piny.
 *     Jesli NACK (HAL_ERROR=1): peripheral dziala ale sensor nie odpowiada
 *     (zly adres, brak zasilania, zly pin SDO/SA1).
 *  2. WHO_AM_I — weryfikacja czy odczyt rejestrow dziala poprawnie.
 *  3. Bus recovery — jesli I2C jest w stanie bledu po poprzedniej transakcji,
 *     ResetBus resetuje peripheral i generuje 9 impulsow SCL zeby odblokowa SDA.
 *
 * Oczekiwane wyniki:
 *   LSM6DS3TR-C (I2C addr 0x6A, SDO=GND): IsDeviceReady=OK, WHO_AM_I=0x6A
 *   LIS3MDL     (I2C addr 0x1C, SA1=GND):  IsDeviceReady=OK, WHO_AM_I=0x3D
 *   Alternatywne adresy jesli piny SDO/SA1 podciagniete do VCC:
 *     LSM6: 0x6B (uzyj LSM6DS3TR_C_I2C_ADD_H = 0xD7)
 *     LIS3: 0x1E (uzyj LIS3MDL_I2C_ADD_H = 0x3D)
 * --------------------------------------------------------------------------- */

static void I2C_BusRecovery(I2C_HandleTypeDef *hi2c)
{
    /* Wymus reset peripheral I2C i wygeneruj 9 taktow SCL aby odblokowa SDA.
     * Wymagane gdy poprzednia transakcja skoncyla sie bledem i SDA jest stuck LOW. */
    HAL_I2C_DeInit(hi2c);
    HAL_Delay(5);

    /* Reconfigure pins as GPIO output for manual clocking */
    GPIO_InitTypeDef gpio = {0};
    uint16_t scl_pin, sda_pin;
    GPIO_TypeDef *port = GPIOC;

    if (hi2c->Instance == I2C4) {
        scl_pin = GPIO_PIN_6;
        sda_pin = GPIO_PIN_7;
    } else {
        scl_pin = GPIO_PIN_8;
        sda_pin = GPIO_PIN_9;
    }

    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = scl_pin | sda_pin;
    HAL_GPIO_Init(port, &gpio);

    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_SET);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }
    /* STOP condition */
    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(port, scl_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(port, sda_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    HAL_I2C_Init(hi2c);
}

static void I2C_SelfTest(void)
{
    HAL_StatusTypeDef st;
    uint8_t who = 0;

    printf("\r\n=== I2C Self-Test (przed FreeRTOS) ===\r\n");

    /* --- I2C3: LSM6DS3TR-C (0x6A) — IMU na I2C3 (PC8=SCL, PC9=SDA) --- */
    printf("[I2C3] LSM6DS3TR-C probe (addr=0x%02X, SDO=GND)...\r\n",
           LSM6DS3TR_C_I2C_ADD_L >> 1);
    st = HAL_I2C_IsDeviceReady(&hi2c3, LSM6DS3TR_C_I2C_ADD_L, 3, 200);
    if (st == HAL_OK) {
        printf("  IsDeviceReady: OK\r\n");
        who = 0;
        HAL_I2C_Mem_Read(&hi2c3, LSM6DS3TR_C_I2C_ADD_L,
                         LSM6DS3TR_C_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
        printf("  WHO_AM_I: 0x%02X  (oczekiwane 0x%02X) %s\r\n",
               who, LSM6DS3TR_C_ID,
               (who == LSM6DS3TR_C_ID) ? "[OK]" : "[BLAD - zly sensor lub rejestr]");
    } else if (st == HAL_TIMEOUT) {
        printf("  IsDeviceReady: TIMEOUT (3) - bus stuck lub bledne GPIO/AF!\r\n");
        I2C_BusRecovery(&hi2c3);
        printf("  Bus recovery wykonany.\r\n");
    } else {
        printf("  IsDeviceReady: NACK (1) - sensor nie odpowiada.\r\n");
        printf("  Probuje takze adres HIGH (SDO=VCC, 0x%02X)...\r\n",
               LSM6DS3TR_C_I2C_ADD_H >> 1);
        st = HAL_I2C_IsDeviceReady(&hi2c3, LSM6DS3TR_C_I2C_ADD_H, 3, 200);
        printf("  Adres HIGH: %s\r\n",
               (st == HAL_OK) ? "OK! SDO jest podciagniete do VCC, zmien adres."
                               : (st == HAL_TIMEOUT ? "TIMEOUT" : "NACK - nie ma sensora"));
    }

    /* --- I2C3: LIS3MDL (0x1C) --- */
    printf("[I2C3] LIS3MDL probe (addr=0x%02X, SA1=GND)...\r\n",
           LIS3MDL_I2C_ADD_L >> 1);
    st = HAL_I2C_IsDeviceReady(&hi2c3, LIS3MDL_I2C_ADD_L, 3, 200);
    if (st == HAL_OK) {
        printf("  IsDeviceReady: OK\r\n");
        who = 0;
        HAL_I2C_Mem_Read(&hi2c3, LIS3MDL_I2C_ADD_L,
                         LIS3MDL_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
        printf("  WHO_AM_I: 0x%02X  (oczekiwane 0x%02X) %s\r\n",
               who, LIS3MDL_ID,
               (who == LIS3MDL_ID) ? "[OK]" : "[BLAD - zly sensor lub rejestr]");
    } else if (st == HAL_TIMEOUT) {
        printf("  IsDeviceReady: TIMEOUT (3) - bus stuck lub bledne GPIO/AF!\r\n");
        I2C_BusRecovery(&hi2c3);
        printf("  Bus recovery wykonany.\r\n");
    } else {
        printf("  IsDeviceReady: NACK (1) - LIS3MDL nie odpowiada.\r\n");
        printf("  Probuje takze adres HIGH (SA1=VCC, 0x%02X)...\r\n",
               LIS3MDL_I2C_ADD_H >> 1);
        st = HAL_I2C_IsDeviceReady(&hi2c3, LIS3MDL_I2C_ADD_H, 3, 200);
        printf("  Adres HIGH: %s\r\n",
               (st == HAL_OK) ? "OK! SA1 jest podciagniete do VCC."
                               : (st == HAL_TIMEOUT ? "TIMEOUT" : "NACK - nie ma sensora"));
    }

    /* --- Pelny skan I2C3 i I2C4 --- */
    static const struct { I2C_HandleTypeDef *hi2c; const char *name; } buses[] = {
        { &hi2c3, "I2C3 (PC8=SCL, PC9=SDA)" },
        { &hi2c4, "I2C4 (PC6=SCL, PC7=SDA)" },
    };
    for (int b = 0; b < 2; b++) {
        printf("[%s] Skan 0x08..0x77:\r\n", buses[b].name);
        uint8_t found = 0;
        for (uint8_t addr7 = 0x08; addr7 <= 0x77; addr7++) {
            if (HAL_I2C_IsDeviceReady(buses[b].hi2c, (uint16_t)(addr7 << 1), 1, 20) == HAL_OK) {
                /* Podpowiedz co to moze byc */
                const char *hint = "";
                if      (addr7 == 0x6A || addr7 == 0x6B) hint = " <- LSM6DS3TR-C";
                else if (addr7 == 0x1C || addr7 == 0x1E) hint = " <- LIS3MDL";
                else if (addr7 >= 0x40 && addr7 <= 0x4F) hint = " <- INA219 / PCF8523 / podobne";
                else if (addr7 >= 0x50 && addr7 <= 0x57) hint = " <- EEPROM / Flash";
                printf("  0x%02X%s\r\n", addr7, hint);
                found++;
            }
        }
        if (!found) printf("  Brak urzadzen.\r\n");
    }

    printf("=== Koniec testu I2C ===\r\n\r\n");
}

// --- ZADANIE PRZETWARZANIA ADC ---


void UART4_Print(uint8_t* Message)
{
	HAL_UART_Transmit(&huart4, Message, strlen((char*)Message), 100);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Sonar_IC_Callback(htim, htim->Channel);
}
/* USER CODE END 4 */

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
  Sonar_PeriodElapsed_Callback(htim);
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
