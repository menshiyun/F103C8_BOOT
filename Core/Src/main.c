
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bsp_norflash.h"
#include "rtd266x_main.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void FPGA_Download(void);
static void RTD266X_Download(void);
static void JumpToApplication(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  NORFLASH_OBJ *norflash = BSP_NORFLASH_OBJ();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_FATFS_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);
  __HAL_SPI_ENABLE(&hspi1);
  norflash->Init();
  retUSER = f_mount(&USERFatFS, USERPath, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    FPGA_Download();
    RTD266X_Download();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    JumpToApplication();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
#define DONE_GPIO (GPIOA)
#define DONE_Pin  (GPIO_PIN_2)
#define DONE_High (DONE_GPIO->BSRR = DONE_Pin)
#define DONE_Low  (DONE_GPIO->BSRR = DONE_Pin << 16)
#define DONE_Read (DONE_GPIO->IDR  & DONE_Pin)

#define STA_GPIO  (GPIOA)
#define STA_Pin   (GPIO_PIN_3)
#define STA_High  (STA_GPIO->BSRR = STA_Pin)
#define STA_Low   (STA_GPIO->BSRR = STA_Pin << 16)
#define STA_Read  (STA_GPIO->IDR  & STA_Pin)

#define CFG_GPIO  (GPIOB)
#define CFG_Pin   (GPIO_PIN_1)
#define CFG_High  (CFG_GPIO->BSRR = CFG_Pin)
#define CFG_Low   (CFG_GPIO->BSRR = CFG_Pin << 16)

#define DCLK_GPIO (GPIOB)
#define DCLK_Pin  (GPIO_PIN_8)
#define DCLK_High (DCLK_GPIO->BSRR = DCLK_Pin)
#define DCLK_Low  (DCLK_GPIO->BSRR = DCLK_Pin << 16)

#define DATA_GPIO (GPIOB)
#define DATA_Pin  (GPIO_PIN_9)
#define DATA_High (DATA_GPIO->BSRR = DATA_Pin)
#define DATA_Low  (DATA_GPIO->BSRR = DATA_Pin << 16)

#define FPGA_FILE "fpga.rbf"

#define LENS 0x400

static void FPGA_Download(void)
{
    char buf[LENS] = {0};
    int  filesize  = 0;
    int  fileptr   = 0;
    int  length    = 0;
    int  rcnt      = 0;

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin   = DONE_Pin;
    HAL_GPIO_Init(DONE_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = STA_Pin;
    HAL_GPIO_Init(STA_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;

    CFG_High;
    GPIO_InitStruct.Pin   = CFG_Pin;
    HAL_GPIO_Init(CFG_GPIO, &GPIO_InitStruct);

    DCLK_Low;
    GPIO_InitStruct.Pin   = DCLK_Pin;
    HAL_GPIO_Init(DCLK_GPIO, &GPIO_InitStruct);

    DATA_Low;
    GPIO_InitStruct.Pin   = DATA_Pin;
    HAL_GPIO_Init(DATA_GPIO, &GPIO_InitStruct);

    retUSER = f_open(&USERFile, FPGA_FILE, FA_READ);
    if (retUSER != FR_OK)
        return;

    while (!STA_Read);

    CFG_Low;
    usDelay(1);
    CFG_High;

    if (STA_Read)
        goto RETURN;

    usDelay(230);

    if (!STA_Read)
        goto RETURN;

    filesize = f_size(&USERFile);
    fileptr  = 0;

    while (fileptr < filesize)
    {
        if (filesize - fileptr > sizeof(buf))
            length = sizeof(buf);
        else
            length = filesize - fileptr;

        retUSER = f_read(&USERFile, buf, length, (UINT *)&rcnt);
        if ((retUSER != FR_OK)||(rcnt != length))
            goto RETURN;

        for (int i = 0; i < length; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                if (buf[i] & 0x01)
                    DATA_High;
                else
                    DATA_Low;
                DCLK_High;DCLK_Low;
                buf[i] >>= 1;
            }
        }

        fileptr += length;
    }

    for (int i = 0; (i < 250)&&(!DONE_Read); i++)
    {
        DCLK_High;DCLK_Low;
    }

    DCLK_High;DCLK_Low;
    DCLK_High;DCLK_Low;

    RETURN:
    retUSER = f_close(&USERFile);
}

#define RTD266X_FILE "rtd266x.bin"

static void RTD266X_Download(void)
{
    const FlashDesc *chip = NULL;
    uint32_t jedec_id = 0;

    rtd266x_init();

    retUSER = f_open(&USERFile, RTD266X_FILE, FA_READ);
    if (retUSER != FR_OK)
        return;

    for(int i = 0; i < 4; i++)
    {
        WriteReg(0x6f, 0x80);
        if(!(ReadReg(0x6f) & 0x80))
            continue;
        jedec_id = SPICommonCommand(E_CC_READ, 0x9f, 3, 0, 0);
        chip = FindChip(jedec_id);
        break;
    }

    if (chip == NULL)
        goto RETURN;

    SetupChipCommands(chip->jedec_id);

    if (!ProgramFlash(&USERFile, chip->size_kb * 1024))
        goto RETURN;

    retUSER = f_close(&USERFile);

    retUSER = f_unlink(RTD266X_FILE);

    return;

    RETURN:
    retUSER = f_close(&USERFile);
}

#define APP_OFST (0xB000)
#define APP_ADDR (FLASH_BASE + APP_OFST)

typedef void (*pJumpFunc)(void);

static void JumpToApplication(void)
{
    uint32_t StackTop = 0;

    StackTop = *(__IO uint32_t*)APP_ADDR;

    if ((StackTop >= 0x2000000)&&(StackTop <= 0x20004FFF))
    {
        pJumpFunc JumpApp = (pJumpFunc)(*(__IO uint32_t*)(APP_ADDR + 4));
        HAL_SPI_DeInit(&hspi1);
        HAL_TIM_Base_DeInit(&htim4);
        __disable_irq();
        __set_MSP(*(__IO uint32_t*)APP_ADDR);
        JumpApp();
    }
    else
        HAL_NVIC_SystemReset();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
