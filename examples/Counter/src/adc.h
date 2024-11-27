#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/



//#include "stm32wlxx_nucleo.h"


/* Common Error codes */
#define BSP_ERROR_NONE                         0
#define BSP_ERROR_NO_INIT                     -1
#define BSP_ERROR_WRONG_PARAM                 -2
#define BSP_ERROR_BUSY                        -3
#define BSP_ERROR_PERIPH_FAILURE              -4
#define BSP_ERROR_COMPONENT_FAILURE           -5
#define BSP_ERROR_UNKNOWN_FAILURE             -6
#define BSP_ERROR_UNKNOWN_COMPONENT           -7
#define BSP_ERROR_BUS_FAILURE                 -8
#define BSP_ERROR_CLOCK_FAILURE               -9
#define BSP_ERROR_MSP_FAILURE                 -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED       -11




#include "stm32wlxx_nucleo_conf.h"





/* ## Definition of ADC related resources ################################### */
/* Definition of ADCx clock resources */
#define ADCx                            ADC
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition of ADCx channels */
#define ADCx_CHANNELa                   ADC_CHANNEL_6

/* Definition of ADCx NVIC resources */
#define ADCx_IRQn                       ADC_IRQn
#define ADCx_IRQHandler                 ADC_IRQHandler

/* Definition of ADCx channels pins */
#define ADCx_CHANNELa_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNELa_GPIO_PORT         GPIOA
#define ADCx_CHANNELa_PIN               GPIO_PIN_10

/* Definition of ADCx DMA resources */
#define ADCx_DMA_CLK_ENABLE()           do {                                \
                                            __HAL_RCC_DMA1_CLK_ENABLE();    \
                                            __HAL_RCC_DMAMUX1_CLK_ENABLE(); \
                                        }while(0)
#define ADCx_DMA                        DMA1_Channel1

#define ADCx_DMA_IRQn                   DMA1_Channel1_IRQn
#define ADCx_DMA_IRQHandler             DMA1_Channel1_IRQHandler



/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       ((uint32_t)3300)
  
/* Definitions of data related to this example */
  /* Full-scale digital value with a resolution of 12 bits (voltage range     */
  /* determined by analog voltage references Vref+ and Vref-,                 */
  /* refer to reference manual).                                              */
  #define DIGITAL_SCALE_12BITS             ((uint32_t) 0xFFF)

  /* Init variable out of ADC expected conversion data range */
  #define VAR_CONVERTED_DATA_INIT_VALUE    (DIGITAL_SCALE_12BITS + 1)

  /* Definition of ADCx conversions data table size */
  #define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  64)

/* Private macro -------------------------------------------------------------*/

/**
  * @brief  Macro to calculate the voltage (unit: mVolt)
  *         corresponding to a ADC conversion data (unit: digital value).
  * @note   ADC measurement data must correspond to a resolution of 12bits
  *         (full scale digital value 4095). If not the case, the data must be
  *         preliminarily rescaled to an equivalent resolution of 12 bits.
  * @note   Analog reference voltage (Vref+) must be known from
  *         user board environment.
  * @param  __VREFANALOG_VOLTAGE__ Analog reference voltage (unit: mV)
  * @param  __ADC_DATA__ ADC conversion data (resolution 12 bits)
  *                       (unit: digital value).
  * @retval ADC conversion data equivalent voltage value (unit: mVolt)
  */
#define __ADC_CALC_DATA_VOLTAGE(__VREFANALOG_VOLTAGE__, __ADC_DATA__)       \
  ((__ADC_DATA__) * (__VREFANALOG_VOLTAGE__) / DIGITAL_SCALE_12BITS)

/* USER CODE END Private defines */
void adc_dma_initialization(void);
void adc_dma_task(void* arg);
void adc_print_init_errors(void);
void error_handler(int reason);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */