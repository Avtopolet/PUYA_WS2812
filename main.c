/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
// Sorry for my eanglish! :)
#include "main.h"
#include "py32f002b_hal.h"

#include "ws2812_spi.h"
#include <py32f002bx5.h>


/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
SPI_HandleTypeDef Spi1Handle;


// Structures for ADC settings
ADC_HandleTypeDef             AdcHandle;
ADC_HandleTypeDef             AdcHandle_t;
ADC_ChannelConfTypeDef        sConfig;
ADC_ChannelConfTypeDef        sConfig_t;
// Variable for storing ADC
volatile uint16_t             aADCxConvertedData;
// Arrays for ADC
uint32_t adc_value[6]; // Battary voltage
uint32_t adc_temp[6];  // temoperature
float T_VCC; // This for conversions formlas from DS, in this case im no use it
float Temperature;


static void APP_AdcConfig(void);   // Config for battary measurements
static void APP_AdcConfig_t(void); // Config for temperature measurements
static void policeLights(void);    // Red-blue simple effect
static void showPower(void);       // To display battary voltage by leds
static void showTemp(void);        // To display temperature by leds 
static void APP_GpioConfig(void);  // GPIO for power up leds

/**
  * @brief  Application Entry Function.
  * @retval int
  */
int main(void)
{
  

  float factor1, factor2; // For rainbow effect
  uint16_t ind;           // For rainbow effect, how many rainbows on line
  int i;                  // This switch rainbow step
  HAL_Init();
    __HAL_RCC_GPIOA_CLK_ENABLE();
  
 uint8_t delay=0;
  APP_GpioConfig();
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0); // turn on leds - 0, off - 1
    // Set leds off, you must set up all array of data for leds
    ws2812_pixel(0, 0, 0, 0);
    ws2812_pixel(1, 0, 0, 0);
    ws2812_pixel(2, 0, 0, 0);
    ws2812_pixel(3, 0, 0, 0);
    ws2812_pixel(4, 0, 0, 0);
    ws2812_pixel(5, 0, 0, 0);
    ws2812_pixel(6, 0, 0, 0);
    ws2812_pixel(7, 0, 0, 0);
 
  while (1)
  {
    // Use adc channel for battary measurements
    APP_AdcConfig();
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle,1000000);
    adc_value[0]=HAL_ADC_GetValue(&AdcHandle);
    T_VCC=adc_value[0];
    // Use adc channel for temperature measurements
    APP_AdcConfig_t();
    HAL_ADC_Start(&AdcHandle_t);
    HAL_ADC_PollForConversion(&AdcHandle_t,1000000);
    adc_temp[0]=HAL_ADC_GetValue(&AdcHandle_t);
    Temperature=adc_temp[0];
    // If battary > 2.9 V and temperature < 27 deg C
    if ((T_VCC<1600) & (Temperature<950))
    {
      for (size_t k = 0; k < 1800; k++) // Raibow cycle
        {
        // This speed up rainbow
        if (k<100) {delay=14;}
        if ((k<400)&(k>100)) {delay=10;}
        if ((k<1200)&(k>400)) {delay=9;}
        if ((k<2000)&(k>1200)) {delay=5;}
        // Raindow effect begin. Thanks https://adrianotiger.github.io/Neopixel-Effect-Generator/
        i++;
      for(uint16_t j=0;j<WS2812_NUM_LEDS;j++) {
        ind = i+j * 1;
        switch((int)((ind % WS2812_NUM_LEDS) / 3)) {
          case 0: factor1 = 1.0 - ((float)(ind % WS2812_NUM_LEDS - 0 * 3) / 3);
                  factor2 = (float)((int)(ind - 0) % WS2812_NUM_LEDS) / 3;
                  ws2812_pixel(j, 128 * factor1 + 0 * factor2, 0 * factor1 + 128 * factor2, 0 * factor1 + 0 * factor2);
                  ws2812_send_spi();
                  HAL_Delay(delay);
                  break;
          case 1: factor1 = 1.0 - ((float)(ind % WS2812_NUM_LEDS - 1 * 3) / 3);
                  factor2 = (float)((int)(ind - 3) % WS2812_NUM_LEDS) / 3;
                  ws2812_pixel(j, 0 * factor1 + 0 * factor2, 128 * factor1 + 0 * factor2, 0 * factor1 + 128 * factor2);
                  ws2812_send_spi();
                  HAL_Delay(delay);
                  break;
          case 2: factor1 = 1.0 - ((float)(ind % WS2812_NUM_LEDS - 2 * 3) / 3);
                  factor2 = (float)((int)(ind - 9) % WS2812_NUM_LEDS) / 3;
                  ws2812_pixel(j, 0 * factor1 + 128 * factor2, 0 * factor1 + 0 * factor2, 128 * factor1 + 0 * factor2);
                  ws2812_send_spi();
                  HAL_Delay(delay);
                  break;
        }
      }
      // RAinbow effect end. Consume 47 mA approx
    }     
    policeLights();
    ws2812_pixel(0, 0, 0, 0);
    ws2812_pixel(1, 0, 0, 0);
    ws2812_pixel(2, 0, 0, 0);
    ws2812_pixel(3, 0, 0, 0);
    ws2812_pixel(4, 0, 0, 0);
    ws2812_pixel(5, 0, 0, 0);
    ws2812_pixel(6, 0, 0, 0);
    ws2812_pixel(7, 0, 0, 0);
    ws2812_pixel(8, 0, 0, 0);
    ws2812_send_spi();
    // 30 sec wait
    HAL_Delay(30000);
   
    }
    
    
    APP_AdcConfig();
    showPower();

    APP_AdcConfig_t(); 
    showTemp();

  }
}


/**
  * @brief  GPIO configuration.
  * @param  None
  * @retval None
  */
static void APP_GpioConfig(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;

  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
   
    /* PA7 -> AF0 -> MOSI */
    GPIO_InitStruct.Pin       = GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Вывод для управления транзистором Q3*/
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode =  GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  Spi1Handle.Instance               = SPI1;
  Spi1Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  Spi1Handle.Init.Direction         = SPI_DIRECTION_1LINE;
  Spi1Handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  Spi1Handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  Spi1Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  Spi1Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  Spi1Handle.Init.NSS               = SPI_NSS_SOFT;
  Spi1Handle.Init.Mode              = SPI_MODE_MASTER;

  HAL_SPI_Init(&Spi1Handle);

  if (HAL_SPI_DeInit(&Spi1Handle) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  if (HAL_SPI_Init(&Spi1Handle) != HAL_OK)
  {
    APP_ErrorHandler();
  }               
}

// >>>> ADC begin
static void APP_AdcConfig(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();                                                  /* Reset ADC */
  __HAL_RCC_ADC_CLK_ENABLE();                                                     /* Enable ADC clock */

  AdcHandle.Instance = ADC1;
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)                         
  {
    APP_ErrorHandler();
  }
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;                /* ADC clock no division */
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;                      /* 12bit */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;                     /* Right alignment */
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;             /* Backward */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;                     /* End flag */
  AdcHandle.Init.LowPowerAutoWait      = ENABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* External trigger: TIM1_TRGO */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;  /* Triggered by both edges */
  
  //AdcHandle.Init.DMAContinuousRequests = DISABLE;                                 /* No DMA */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  AdcHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  sConfig.Channel      = ADC_CHANNEL_VREFINT;
  




  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)                      
  {
    APP_ErrorHandler();
  }
  /* Start ADC with Interrupt 
  if (HAL_ADC_Start_IT(&AdcHandle) != HAL_OK)                                     
  {
    APP_ErrorHandler();
  }*/
}

static void APP_AdcConfig_t(void)
{
  ADC_ChannelConfTypeDef sConfig_t = {0};
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();                                                  /* Reset ADC */
  __HAL_RCC_ADC_CLK_ENABLE();                                                     /* Enable ADC clock */

  AdcHandle_t.Instance = ADC1;
  if (HAL_ADCEx_Calibration_Start(&AdcHandle_t) != HAL_OK)                         
  {
    APP_ErrorHandler();
  }
  AdcHandle_t.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;                /* ADC clock no division */
  AdcHandle_t.Init.Resolution            = ADC_RESOLUTION_12B;                      /* 12bit */
  AdcHandle_t.Init.DataAlign             = ADC_DATAALIGN_RIGHT;                     /* Right alignment */
  AdcHandle_t.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;             /* Backward */
  AdcHandle_t.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;                     /* End flag */
  AdcHandle_t.Init.LowPowerAutoWait      = ENABLE;
  AdcHandle_t.Init.ContinuousConvMode    = DISABLE;
  AdcHandle_t.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle_t.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* External trigger: TIM1_TRGO */
  AdcHandle_t.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;  /* Triggered by both edges */
  
  //AdcHandle_t.Init.DMAContinuousRequests = DISABLE;                                 /* No DMA */
  AdcHandle_t.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  AdcHandle_t.Init.SamplingTimeCommon    = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_Init(&AdcHandle_t) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  sConfig_t.Rank         = ADC_RANK_CHANNEL_NUMBER;
 // sConfig.Channel      = ADC_CHANNEL_VREFINT;
  sConfig_t.Channel      = ADC_CHANNEL_TEMPSENSOR;




  if (HAL_ADC_ConfigChannel(&AdcHandle_t, &sConfig_t) != HAL_OK)                      
  {
    APP_ErrorHandler();
  }
  /* Start ADC with Interrupt 
  if (HAL_ADC_Start_IT(&AdcHandle) != HAL_OK)                                     
  {
    APP_ErrorHandler();
  }*/
}

// <<<< ADC  end
static void showPower(void) {
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle,1000000);
    adc_value[0]=HAL_ADC_GetValue(&AdcHandle);
    //T_VCC=(4095*1.2)/adc_valie[0];
    T_VCC=adc_value[0];
        //T_VCC=11;

 
      if (T_VCC < 1600) {
      ws2812_pixel(7, 128, 0, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

      if (T_VCC < 1590) {
      ws2812_pixel(6, 128, 0, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

      if (T_VCC < 1550) {
      ws2812_pixel(5, 90, 30, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

    if (T_VCC < 1500) {
      ws2812_pixel(4, 10, 60, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }  

    if (T_VCC < 1400) {
      ws2812_pixel(3, 10, 90, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

    if (T_VCC < 1300) {
      ws2812_pixel(2, 5, 120, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    } 

    if (T_VCC < 1200) {
      ws2812_pixel(1, 0, 128, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

    if (T_VCC < 1100) {
      ws2812_pixel(0, 0, 0, 90);
      ws2812_send_spi();
      HAL_Delay(100);
    } 
    
    HAL_Delay(5000);
              ws2812_pixel(0, 0, 0, 0);
              ws2812_pixel(1, 0, 0, 0);
              ws2812_pixel(2, 0, 0, 0);
              ws2812_pixel(3, 0, 0, 0);
              ws2812_pixel(4, 0, 0, 0);
              ws2812_pixel(5, 0, 0, 0);
              ws2812_pixel(6, 0, 0, 0);
              ws2812_pixel(7, 0, 0, 0);
              ws2812_send_spi();
    HAL_Delay(1000);          
              


    if (T_VCC < 1650) {
              ws2812_pixel(0, 0, 0, 0);
              ws2812_pixel(1, 0, 0, 0);
              ws2812_pixel(2, 0, 0, 0);
              ws2812_pixel(3, 0, 0, 0);
              ws2812_pixel(4, 0, 0, 0);
              ws2812_pixel(5, 0, 0, 0);
              ws2812_pixel(6, 0, 0, 0);
              ws2812_pixel(7, 0, 0, 0);
              ws2812_send_spi();
              HAL_Delay(200);
              ws2812_pixel(8, 255, 0, 0);
              ws2812_send_spi();
              HAL_Delay(200);
              ws2812_pixel(8, 0, 0, 0);
              ws2812_send_spi();
              T_VCC=0;
    }




}

static void showTemp(void) {
    HAL_ADC_Start(&AdcHandle_t);
    HAL_ADC_PollForConversion(&AdcHandle_t,1000000);
    adc_temp[0]=HAL_ADC_GetValue(&AdcHandle_t);
    //T_VCC=(4095*1.2)/adc_valie[0];
    Temperature=adc_temp[0];
        //T_VCC=11;
      ws2812_pixel(8, 0, 0, 128);
      ws2812_send_spi();
      HAL_Delay(100);



       if (Temperature < 965) {
      ws2812_pixel(7, 128, 0, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

      if (Temperature < 960) {
      ws2812_pixel(6, 128, 10, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

      if (Temperature < 955) {
      ws2812_pixel(5, 120, 10, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

    if (Temperature < 952) {  // aprox 25 deg
      ws2812_pixel(4, 120, 20, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }  

    if (Temperature < 949) { // aprox 26 deg
      ws2812_pixel(3, 120, 30, 0);
      ws2812_send_spi();
      HAL_Delay(100);
    }

    if (Temperature < 944) { // aprox 27 deg
      ws2812_pixel(2, 0, 30, 50);
      ws2812_send_spi();
      HAL_Delay(100);
    } 

    if (Temperature <  942) {
      ws2812_pixel(1, 0, 0, 50);
      ws2812_send_spi();
      HAL_Delay(100);
    }

    if (Temperature< 940) {
      ws2812_pixel(0, 0, 0, 90);
      ws2812_send_spi();
      HAL_Delay(100);
    } 


    HAL_Delay(5000);
              ws2812_pixel(0, 0, 0, 0);
              ws2812_pixel(1, 0, 0, 0);
              ws2812_pixel(2, 0, 0, 0);
              ws2812_pixel(3, 0, 0, 0);
              ws2812_pixel(4, 0, 0, 0);
              ws2812_pixel(5, 0, 0, 0);
              ws2812_pixel(6, 0, 0, 0);
              ws2812_pixel(7, 0, 0, 0);
              ws2812_send_spi();
    HAL_Delay(1000);          
              Temperature=0;

}

static void policeLights(void) {
    for (size_t i = 0; i < 10; i++)
  {
    
  
  
   ws2812_pixel(0, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(1, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(2, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(3, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(4, 0, 0, 0); ////
   ws2812_send_spi();
   ws2812_pixel(5, 0, 0, 255);
   ws2812_send_spi();
   ws2812_pixel(6, 0, 0, 255);
   ws2812_send_spi();
   ws2812_pixel(7, 0, 0, 255);
   ws2812_send_spi();
   ws2812_pixel(8, 0, 0, 255);
   ws2812_send_spi();

   HAL_Delay(500);

   ws2812_pixel(8, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(7, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(6, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(5, 255, 0, 0);
   ws2812_send_spi();
   ws2812_pixel(4, 0, 0, 0); ////
   ws2812_send_spi();
   ws2812_pixel(3, 0, 0, 255);
   ws2812_send_spi();
   ws2812_pixel(2, 0, 0, 255);
   ws2812_send_spi();
   ws2812_pixel(1, 0, 0, 255);
   ws2812_send_spi();
   ws2812_pixel(0, 0, 0, 255);
   ws2812_send_spi();

   HAL_Delay(500);
  }
}

/**
  * @brief  Error executing function.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  while (1)
  {
  }
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
  /* Users can add their own printing information as needed,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
