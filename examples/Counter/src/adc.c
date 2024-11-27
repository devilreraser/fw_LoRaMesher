
#include "BuildOptions.h"
#include "adc.h"


/* Private variables ---------------------------------------------------------*/

HAL_StatusTypeDef rcc_osc_status = 0;
HAL_StatusTypeDef rcc_clk_status = 0;

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Variables for ADC conversion data */
__IO   uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data (array of data) */

/* Variables for ADC conversion data computation to physical values */
uint16_t   aADCxConvertedData_Voltage_mVolt[ADC_CONVERTED_DATA_BUFFER_SIZE];  /* Value of voltage calculated from ADC conversion data (unit: mV) (array of data) */

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not yet been started yet (initial state)              */
__IO   uint8_t ubDmaTransferStatus = 2; /* Variable set into DMA interruption callback */

/* Variable to manage push button on board: interface between ExtLine interruption and main program */
__IO   uint8_t ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

#define ERROR_REASON_COUNT  32
int error_reason_counter[ERROR_REASON_COUNT] = {0};
int error_reason_unknown_counter = 0;
int error_reason_unknown_last = 0;

HAL_StatusTypeDef dma_hal_status = 0;

void SystemClock_Config(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void Generate_waveform_SW_update_Config(void);
static void Generate_waveform_SW_update(void);


/* USER CODE END PV */

int32_t RED_LED_Init()
{
  GPIO_InitTypeDef  gpio_init_structure = {0};

  /* Enable the GPIO_LED Clock */
  //LEDx_GPIO_CLK_ENABLE(Led); porta clock already enabled

  /* Configure the GPIO_LED pin */
  gpio_init_structure.Pin = GPIO_PIN_9;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOA, &gpio_init_structure);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  return BSP_ERROR_NONE;
}

void adc_dma_initialization(void)
{
  uint32_t tmp_index_adc_converted_data = 0;

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
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  for (tmp_index_adc_converted_data = 0; tmp_index_adc_converted_data < ADC_CONVERTED_DATA_BUFFER_SIZE; tmp_index_adc_converted_data++)
  {
    aADCxConvertedData[tmp_index_adc_converted_data] = VAR_CONVERTED_DATA_INIT_VALUE;
  }
  
  /* Initialize LED on board */
  RED_LED_Init();
  //BSP_LED_Init(LED2);
  
  /* Configure User push-button (B1) */
  //BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  
  /* Run the ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    /* Calibration Error */
    error_handler(1);
  }

  /* Configure the DAC peripheral and generate a constant voltage of Vdda/2.  */
  Generate_waveform_SW_update_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*## Enable Timer ########################################################*/
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    /* Counter enable error */
    error_handler(2);
  }
  
  /*## Start ADC conversions ###############################################*/
  /* Start ADC group regular conversion with DMA */
  dma_hal_status = HAL_ADC_Start_DMA(&hadc, (uint32_t *)aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
  if (dma_hal_status != HAL_OK)
  {
    /* ADC conversion start error */
    error_handler(3);
  }  

  adc_print_init_errors();
  
}

void adc_dma_task(void* arg)
{
    int print_counter = 0;
    while (1)
    {
        /* Modifies modifies the voltage level, to generate a waveform circular,  */
        /* shape of ramp: Voltage is increasing at each press on push button,     */
        /* from 0 to maximum range (Vdda) in 4 steps, then starting back from 0V. */
        /* Voltage is updated incrementally at each call of this function.        */
        Generate_waveform_SW_update();

        /* Wait for event on push button to perform following actions */
        // while ((ubUserButtonClickEvent) == RESET)
        // {
        // }
        /* Reset variable for next loop iteration (with debounce) */
        vTaskDelay(10);
        //ubUserButtonClickEvent = RESET;

        /* Note: Variable "ubUserButtonClickEvent" is set into push button        */
        /*       IRQ handler, refer to function "HAL_GPIO_EXTI_Callback()".       */

        /* USER CODE END WHILE */


        /* Note: LED state depending on DMA transfer status is set into DMA       */
        /*       IRQ handler, refer to functions "HAL_ADC_ConvCpltCallback()"     */
        /*       and "HAL_ADC_ConvHalfCpltCallback()".                            */

        /* Note: ADC conversions data are stored into array                       */
        /*       "aADCxConvertedData"                                             */
        /*       (for debug: see variable content into watch window).             */

        /* Note: ADC conversion data are computed to physical values              */
        /*       into array "aADCxConvertedData_Voltage_mVolt"                    */
        /*       using helper macro "__ADC_CALC_DATA_VOLTAGE()".                  */
        /*       (for debug: see variable content into watch window).             */
        print_counter++;
        if (print_counter >= 500)
        {
            print_counter = 0;
            adc_print_init_errors();
        }
    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  rcc_osc_status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if (rcc_osc_status != HAL_OK)
  {
    error_handler(4);
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
//                               |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
//                               |RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
//   rcc_clk_status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
//   if (rcc_clk_status != HAL_OK)
//   {
//     error_handler(5);
//   }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    error_handler(6);
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    error_handler(7);
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    error_handler(8);
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    error_handler(9);
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    error_handler(10);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    error_handler(11);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    error_handler(12);
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  For this example, generate a waveform voltage on a spare DAC
  *         channel, so user has just to connect a wire between DAC channel 
  *         (pin PA10) and ADC channel (pin PA10) to run this example.
  *         (this prevents the user from resorting to an external signal
  *         generator).
  *         This function configures the DAC and generates a constant voltage of Vdda/2.
  * @note   Voltage level can be modifying afterwards using function
  *         "Generate_waveform_SW_update()".
  * @param  None
  * @retval None
  */
static void Generate_waveform_SW_update_Config(void)
{
  /* Set DAC Channel data register: channel corresponding to ADC channel ADC_CHANNEL_6 */
  /* Set DAC output to 1/2 of full range (4095 <=> Vdda=3.3V): 2048 <=> 1.65V */
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DIGITAL_SCALE_12BITS/2) != HAL_OK)
  {
    /* Setting value Error */
    error_handler(13);
  }
  
  /* Enable DAC Channel: channel corresponding to ADC channel ADC_CHANNEL_6 */
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    error_handler(14);
  }

}

/**
  * @brief  For this example, generate a waveform voltage on a spare DAC
  *         channel, so user has just to connect a wire between DAC channel 
  *         (pin PA10) and ADC channel (pin PA10) to run this example.
  *         (this prevents the user from resorting to an external signal
  *         generator).
  *         This function modifies the voltage level, to generate a
  *         waveform circular, shape of ramp: Voltage is increasing at each 
  *         press on push button, from 0 to maximum range (Vdda) in 4 steps,
  *         then starting back from 0V.
  *         Voltage is updated incrementally at each call of this function.
  * @note   Preliminarily, DAC must be configured once using
  *         function "Generate_waveform_SW_update_Config()".
  * @param  None
  * @retval None
  */
static void Generate_waveform_SW_update(void)
{
    static const uint8_t ub_dac_steps_total = 32;
    static uint8_t ub_dac_steps_count = 0;      /* Count number of clicks: Incremented after User Button interrupt */
    
    /* Set DAC voltage on channel corresponding to ADC_CHANNEL_6              */
    /* in function of user button clicks count.                                   */
    /* Set DAC output on 5 voltage levels, successively to:                       */
    /*  - minimum of full range (0 <=> ground 0V)                                 */
    /*  - 1/4 of full range (4095 <=> Vdda=3.3V): 1023 <=> 0.825V                 */
    /*  - 1/2 of full range (4095 <=> Vdda=3.3V): 2048 <=> 1.65V                  */
    /*  - 3/4 of full range (4095 <=> Vdda=3.3V): 3071 <=> 2.475V                 */
    /*  - maximum of full range (4095 <=> Vdda=3.3V)                              */
    if (HAL_DAC_SetValue(&hdac,
                        DAC_CHANNEL_1,
                        DAC_ALIGN_12B_R,
                        ((DIGITAL_SCALE_12BITS * ub_dac_steps_count) / ub_dac_steps_total)
                        ) != HAL_OK)
    {
        /* Start Error */
        error_handler(15);
    }
    
    /* Wait for voltage settling time */
    //HAL_Delay(1);
    
    /* Manage ub_dac_steps_count to increment it in ub_dac_steps_total steps and circularly.   */
    if (ub_dac_steps_count < ub_dac_steps_total)
    {
        ub_dac_steps_count++;
    }
    else
    {
        ub_dac_steps_count = 0;
    }

}



// /**
//   * @brief EXTI line detection callbacks
//   * @param GPIO_Pin: Specifies the pins connected EXTI line
//   * @retval None
//   */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
// //  if (GPIO_Pin == BUTTON_SW1_PIN)
// //  {
// //    /* Set variable to report push button event to main program */
// //    ubUserButtonClickEvent = SET;
// //  }
// }

/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc: ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t tmp_index = 0;
  
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
  /* Management of the 2nd half of the buffer */
  for (tmp_index = (ADC_CONVERTED_DATA_BUFFER_SIZE/2); tmp_index < ADC_CONVERTED_DATA_BUFFER_SIZE; tmp_index++)
  {
    aADCxConvertedData_Voltage_mVolt[tmp_index] = __ADC_CALC_DATA_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index]);
  }
  
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 1;
  
  /* Set LED depending on DMA transfer status */
  /* - Turn-on if DMA transfer is completed */
  /* - Turn-off if DMA transfer is not completed */
  //BSP_LED_On(LED2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t tmp_index = 0;
  
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
  /* Management of the 1st half of the buffer */
  for (tmp_index = 0; tmp_index < (ADC_CONVERTED_DATA_BUFFER_SIZE/2); tmp_index++)
  {
    aADCxConvertedData_Voltage_mVolt[tmp_index] = __ADC_CALC_DATA_VOLTAGE(VDDA_APPLI, aADCxConvertedData[tmp_index]);
  }
  
  /* Update status variable of DMA transfer */
  ubDmaTransferStatus = 0;
  
  /* Set LED depending on DMA transfer status */
  /* - Turn-on if DMA transfer is completed */
  /* - Turn-off if DMA transfer is not completed */
  //BSP_LED_Off(LED2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  error_handler(16);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void error_handler(int reason)
{
    if (reason < ERROR_REASON_COUNT)
    {
        error_reason_counter[reason]++;
    }
    else
    {
        error_reason_unknown_counter++;
        error_reason_unknown_last = reason;
    }
    
  /* USER CODE BEGIN error_handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//   while(1) 
//   {
//     /* Toggle LED2 */
//     // BSP_LED_Off(LED2);
//     // HAL_Delay(800);
//     // BSP_LED_On(LED2);
//     // HAL_Delay(10);
//     // BSP_LED_Off(LED2);
//     // HAL_Delay(180);
//     // BSP_LED_On(LED2);
//     HAL_Delay(10);
//   }
  /* USER CODE END error_handler_Debug */
}


void adc_print_init_errors(void)
{
    uint16_t id_spare = 0;
    uint16_t messages_spare = 0;
    char print_line_dash[256] = {0};
    char print_line_idnt[256] = {0};
    char print_line_mess[256] = {0};


    for (int index = 0; index < ERROR_REASON_COUNT; index++)
    {
        sprintf(&print_line_dash[strlen(print_line_dash)], "---");
        sprintf(&print_line_idnt[strlen(print_line_idnt)], "|%02d", index);
        sprintf(&print_line_mess[strlen(print_line_mess)], "|%02d", error_reason_counter[index]);
    }
    sprintf(&print_line_dash[strlen(print_line_dash)], "---");
    sprintf(&print_line_idnt[strlen(print_line_idnt)], "|%02d", error_reason_unknown_last);
    sprintf(&print_line_mess[strlen(print_line_mess)], "|%02d", error_reason_unknown_counter);
    
    sprintf(&print_line_dash[strlen(print_line_dash)], "-\r\n");
    sprintf(&print_line_idnt[strlen(print_line_idnt)], "|\r\n");
    sprintf(&print_line_mess[strlen(print_line_mess)], "|\r\n");

    printf("%s", print_line_dash);
    printf("%s", print_line_idnt);
    printf("%s", print_line_mess);
    printf("%s", print_line_dash);

    printf("dma_hal_status %d\r\n", dma_hal_status);



}
