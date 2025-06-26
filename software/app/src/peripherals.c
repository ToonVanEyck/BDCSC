#include "peripherals.h"
#include "bdcsc.h"

static void peripheral_gpio_init(void);
// static void APP_Tim1Init(void);
// static void APP_DmaInit(void);
static void peripheral_tim3_init(void); /**< TIM3 initialization. (quadrature encoder) */
// static void APP_ComparatorInit(void);

//======================================================================================================================
//                                                         PUBLIC FUNCTIONS
//======================================================================================================================

void peripherals_init(void)
{
    BSP_RCC_HSI_24MConfig();

    peripheral_gpio_init();
    peripheral_tim3_init();
}

//----------------------------------------------------------------------------------------------------------------------

void peripherals_deinit(void)
{
    // Call all your peripheral deinit functions here
    // (if needed)
}

//======================================================================================================================
//                                                         PRIVATE FUNCTIONS
//======================================================================================================================

static void peripheral_gpio_init(void)
{
    /* Enable GPIO clocks. */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
}

// static void APP_GpioConfig(void)
// {
//     GPIO_InitTypeDef GPIO_InitStruct;

//     __HAL_RCC_GPIOB_CLK_ENABLE();
//     __HAL_RCC_GPIOA_CLK_ENABLE();
//     __HAL_RCC_GPIOF_CLK_ENABLE();

//     // /* Configure column-end detection pin. */
//     // GPIO_InitStruct.Pin   = COLEND_GPIO_PIN;
//     // GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
//     // GPIO_InitStruct.Pull  = GPIO_PULLUP;
//     // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     // HAL_GPIO_Init(COLEND_GPIO_PORT, &GPIO_InitStruct);

//     // /* Configure IR LED */
//     // GPIO_InitStruct.Pin   = ENCODER_LED_GPIO_PIN;
//     // GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//     // GPIO_InitStruct.Pull  = GPIO_PULLUP;
//     // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     // HAL_GPIO_Init(ENCODER_LED_GPIO_PORT, &GPIO_InitStruct);

//     /* Configure debug pins, these pins can be used for pin wiggling during development. */
//     GPIO_InitStruct.Pin   = DEBUG_GPIO_1_PIN | DEBUG_GPIO_2_PIN;
//     GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull  = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(DEBUG_GPIO_1_2_PORT, &GPIO_InitStruct);

//     GPIO_InitStruct.Pin   = DEBUG_GPIO_3_PIN | DEBUG_GPIO_4_PIN;
//     GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull  = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(DEBUG_GPIO_3_4_PORT, &GPIO_InitStruct);

//     // Configure PA2 (COMP2 non-inverting input) as analog
//     GPIO_InitStruct.Pin  = GPIO_PIN_2;
//     GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
// }

// //----------------------------------------------------------------------------------------------------------------------

// static void APP_Tim1Init(void)
// {
//     /* 24MHz / ( 255 x 466 x 2 ) = +/- 200Hz */
//     Tim1Handle.Instance           = TIM1;
//     Tim1Handle.Init.Period        = 255 - 1;
//     Tim1Handle.Init.Prescaler     = 470 - 1;
//     Tim1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
//     Tim1Handle.Init.CounterMode   = TIM_COUNTERMODE_UP;
//     // Tim1Handle.Init.RepetitionCounter = 0;
//     // Tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//     if (HAL_TIM_PWM_Init(&Tim1Handle) != HAL_OK) {
//         APP_ErrorHandler();
//     }

//     TIM_MasterConfigTypeDef sMasterConfig;
//     sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//     sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
//     HAL_TIMEx_MasterConfigSynchronization(&Tim1Handle, &sMasterConfig);

//     TIM_OC_InitTypeDef sConfigOC;
//     sConfigOC.OCMode     = TIM_OCMODE_PWM1;
//     sConfigOC.Pulse      = 0;
//     sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//     sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

//     // Configure channel 1
//     HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfigOC, TIM_CHANNEL_3);
//     HAL_TIM_PWM_Start(&Tim1Handle, TIM_CHANNEL_3);

//     // Configure channel 2
//     HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &sConfigOC, TIM_CHANNEL_4);
//     HAL_TIM_PWM_Start(&Tim1Handle, TIM_CHANNEL_4);

//     /* Configure TIM1 Channel 3 for Input Capture */
//     TIM_IC_InitTypeDef sConfigIC = {0};
//     sConfigIC.ICPolarity         = TIM_INPUTCHANNELPOLARITY_RISING;
//     sConfigIC.ICSelection        = TIM_ICSELECTION_DIRECTTI;
//     sConfigIC.ICPrescaler        = TIM_ICPSC_DIV1;
//     sConfigIC.ICFilter           = 0;
//     HAL_TIM_IC_ConfigChannel(&Tim1Handle, &sConfigIC, TIM_CHANNEL_1);

//     /* Configure DMA for TIM1_CH1 */
//     __HAL_RCC_DMA_CLK_ENABLE();
//     hdma_tim1_ch1.Instance                 = DMA1_Channel1;
//     hdma_tim1_ch1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
//     hdma_tim1_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
//     hdma_tim1_ch1.Init.MemInc              = DMA_MINC_ENABLE;
//     hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//     hdma_tim1_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
//     hdma_tim1_ch1.Init.Mode                = DMA_CIRCULAR;
//     hdma_tim1_ch1.Init.Priority            = DMA_PRIORITY_LOW;
//     HAL_DMA_Init(&hdma_tim1_ch1);

//     __HAL_LINKDMA(&Tim1Handle, hdma[TIM_DMA_ID_CC1], hdma_tim1_ch1);

//     // /* Enable DMA interrupt if needed */
//     // HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
//     // HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

//     /* Start Input Capture in DMA mode */
//     HAL_TIM_IC_Start_DMA(&Tim1Handle, TIM_CHANNEL_1, (uint32_t *)ic_dma_buf, IC_DMA_BUF_LEN);

//     /* Enable TIM1 update interrupt */
//     __HAL_TIM_ENABLE_IT(&Tim1Handle, TIM_IT_UPDATE);
//     HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2, 0);
//     HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
// }

// //----------------------------------------------------------------------------------------------------------------------

// static void APP_DmaInit(void)
// {
//     HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
//     HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
// }

// //----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes TIM3 for quadrature encoder input.
 */
static void peripheral_tim3_init(void)
{

    // Enable TIM3 Clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    // Configure PA5 (TIM3_CH2) and PA6 (TIM3_CH1) as alternate function.
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode                = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed               = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull                = LL_GPIO_PULL_UP;

    GPIO_InitStruct.Pin       = LL_GPIO_PIN_5;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_13; // TIM3_CH2
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = LL_GPIO_PIN_6;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1; // TIM3_CH1
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Reset TIM3
    LL_TIM_DeInit(TIM3);

    // Set encoder mode
    LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);

    // Configure input capture for channel 1
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV4_N8);

    // Configure input capture for channel 2
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV4_N8);

    // Set prescaler, auto-reload, and counter mode
    LL_TIM_SetPrescaler(TIM3, 0);
    LL_TIM_SetAutoReload(TIM3, MOTOR_GEAR_RATIO * MOTOR_ENCODER_PPR - 1);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1);

    // Enable TIM3
    LL_TIM_EnableCounter(TIM3);
}

// encoderHandle.Instance               = TIM3;
// encoderHandle.Init.Prescaler         = 0;
// encoderHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
// encoderHandle.Init.Period            = 298 * 7 * 4 - 1; /* 298*7*4 = 8344, so period is 8343 */
// encoderHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
// encoderHandle.Init.RepetitionCounter = 0;
// encoderHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

// TIM_Encoder_InitTypeDef sConfig = {0};
// sConfig.EncoderMode             = TIM_ENCODERMODE_TI12;
// sConfig.IC1Polarity             = TIM_ICPOLARITY_RISING;
// sConfig.IC1Selection            = TIM_ICSELECTION_DIRECTTI;
// sConfig.IC1Prescaler            = TIM_ICPSC_DIV1;
// sConfig.IC1Filter               = 10;
// sConfig.IC2Polarity             = TIM_ICPOLARITY_RISING;
// sConfig.IC2Selection            = TIM_ICSELECTION_DIRECTTI;
// sConfig.IC2Prescaler            = TIM_ICPSC_DIV1;
// sConfig.IC2Filter               = 10;

// if (HAL_TIM_Encoder_Init(&encoderHandle, &sConfig) != HAL_OK) {
//     APP_ErrorHandler();
// }

// HAL_TIM_Encoder_Start(&encoderHandle, TIM_CHANNEL_ALL);

// //----------------------------------------------------------------------------------------------------------------------

// static void APP_ComparatorInit(void)
// {
//     /* Configure the comparator */
//     Comp2Handle.Instance         = COMP2;
//     Comp2Handle.Init.InputPlus   = COMP_INPUT_PLUS_IO3;           // Use IO1 as non-inverting input
//     Comp2Handle.Init.InputMinus  = COMP_INPUT_MINUS_1_2VREFINT;   // Use DAC1 as inverting input
//     Comp2Handle.Init.OutputPol   = COMP_OUTPUTPOL_NONINVERTED;    // Non-inverted output
//     Comp2Handle.Init.Mode        = COMP_POWERMODE_MEDIUMSPEED;    // Medium-speed mode
//     Comp2Handle.Init.WindowMode  = COMP_WINDOWMODE_DISABLE;       // Disable window mode
//     Comp2Handle.Init.TriggerMode = COMP_TRIGGERMODE_EVENT_RISING; // Trigger on rising edge

//     if (HAL_COMP_Init(&Comp2Handle) != HAL_OK) {
//         APP_ErrorHandler();
//     }

//     /* Start the comparator */
//     HAL_COMP_Start(&Comp2Handle);
// }

// //----------------------------------------------------------------------------------------------------------------------
