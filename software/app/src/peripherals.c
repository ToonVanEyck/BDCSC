#include "peripherals.h"
#include "bdcsc.h"
#include "rbuff.h"

//======================================================================================================================
//                                                  DEFINES AND CONSTS
//======================================================================================================================

/** Number of debug GPIO pins. */
#define DEBUG_GPIO_PINS_COUNT (4)
/** Debug GPIO pins. */
const uint32_t DEBUG_GPIO_PINS[DEBUG_GPIO_PINS_COUNT] = {LL_GPIO_PIN_1, LL_GPIO_PIN_0, LL_GPIO_PIN_6, LL_GPIO_PIN_7};
/** Debug GPIO ports. */
GPIO_TypeDef *const DEBUG_GPIO_PORTS[DEBUG_GPIO_PINS_COUNT] = {GPIOF, GPIOF, GPIOB, GPIOB};

//======================================================================================================================
//                                                   GLOBAL VARIABLES
//======================================================================================================================

uint32_t tim1_tick_count = 0; /**< Counter for TIM1 update events. */

/* COMP2 Input Capture DMA Buffer */
#define TIM1_IC_DMA_BUF_LEN 32
volatile uint32_t tim1_ic_dma_buf[TIM1_IC_DMA_BUF_LEN] = {0};
rbuff_t tim1_ic_dma_rb                                 = {0};
//======================================================================================================================
//                                                  FUNCTION PROTOTYPES
//======================================================================================================================

static void peripheral_gpio_init(void);           /**< GPIO initialization. */
static void peripheral_tim1_init(void);           /**< TIM1 initialization. (Motor PWM & Sensorless Input Capture) */
static void peripheral_tim3_init(void);           /**< TIM3 initialization. (quadrature encoder) */
static void peripheral_tim1_dma_init(void);       /**< DMA initialization for TIM1 input capture */
static void *peripheral_tim1_dma_w_ptr_get(void); /**< Get the write pointer for TIM1 input capture DMA buffer. */

//======================================================================================================================
//                                                   PUBLIC FUNCTIONS
//======================================================================================================================

void peripherals_init(void)
{
    BSP_RCC_HSI_24MConfig();

    peripheral_gpio_init();
    peripheral_tim1_init();
    peripheral_tim3_init();
    peripheral_tim1_dma_init();
}

//----------------------------------------------------------------------------------------------------------------------

void peripherals_deinit(void)
{
}

//------------------------------------------------------------------------------------------------------------------------

void debug_pin_set(uint8_t pin, bool value)
{
    if (pin >= DEBUG_GPIO_PINS_COUNT) {
        return; // Invalid pin number
    }
    if (value) {
        LL_GPIO_SetOutputPin(DEBUG_GPIO_PORTS[pin], DEBUG_GPIO_PINS[pin]);
    } else {
        LL_GPIO_ResetOutputPin(DEBUG_GPIO_PORTS[pin], DEBUG_GPIO_PINS[pin]);
    }
}

//----------------------------------------------------------------------------------------------------------------------

void debug_pin_toggle(uint8_t pin)
{
    if (pin >= DEBUG_GPIO_PINS_COUNT) {
        return; // Invalid pin number
    }
    LL_GPIO_TogglePin(DEBUG_GPIO_PORTS[pin], DEBUG_GPIO_PINS[pin]);
}

//----------------------------------------------------------------------------------------------------------------------

bool debug_pin_get(uint8_t pin)
{
    if (pin >= DEBUG_GPIO_PINS_COUNT) {
        return false; // Invalid pin number
    }
    return LL_GPIO_IsOutputPinSet(DEBUG_GPIO_PORTS[pin], DEBUG_GPIO_PINS[pin]);
}

//----------------------------------------------------------------------------------------------------------------------

uint32_t tim1_tick_count_get(void)
{
    return tim1_tick_count;
}

//----------------------------------------------------------------------------------------------------------------------

uint16_t encoder_position_get(void)
{
    return LL_TIM_GetCounter(TIM3) * MOTOR_SENSORLESS_PPR / MOTOR_ENCODER_PPR;
}

//----------------------------------------------------------------------------------------------------------------------

void motor_control(const motor_t *motor)
{
    if (motor->enable) {
        if (motor->direction_cw) {
            LL_TIM_OC_SetCompareCH3(TIM1, motor->speed);
            LL_TIM_OC_SetCompareCH4(TIM1, 0);
        } else {
            LL_TIM_OC_SetCompareCH3(TIM1, 0);
            LL_TIM_OC_SetCompareCH4(TIM1, motor->speed);
        }
    } else {
        LL_TIM_OC_SetCompareCH3(TIM1, 0);
        LL_TIM_OC_SetCompareCH4(TIM1, 0);
    }
}

//======================================================================================================================
//                                                         PRIVATE FUNCTIONS
//======================================================================================================================

/**
 * @brief Initializes GPIOs used for debugging.
 *
 * This function configures 4 GPIO pins as outputs for debugging purposes.
 * The pins are set to high speed and pull-up mode.
 */
static void peripheral_gpio_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks. */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

    /* Initialize the 4 debug pins. */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    for (int i = 0; i < DEBUG_GPIO_PINS_COUNT; i++) {
        GPIO_InitStruct.Pin = DEBUG_GPIO_PINS[i];
        LL_GPIO_Init(DEBUG_GPIO_PORTS[i], &GPIO_InitStruct);
    }
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes TIM1 for motor control and sensorless input capture.
 *
 * This function configures TIM1 for PWM output on channels 3 and 4, and sets up channel 1 for input capture.
 * It also enables the update interrupt and input capture interrupt.
 */
static void peripheral_tim1_init(void)
{
    // Enable TIM1 clock
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode                = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed               = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull                = LL_GPIO_PULL_UP;

    // Configure PA0 (TIM1_CH3) and PA1 (TIM1_CH4) as alternate function for PWM.
    // Configure PA3 (TIM1_CH1) as alternate function for Input Capture.
    GPIO_InitStruct.Pin       = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_13; // TIM1_CH3 & TIM1_CH4
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure TIM1 base: 24MHz / (1000 * 120) ≈ 200Hz
    LL_TIM_SetPrescaler(TIM1, 240 - 1);
    LL_TIM_SetAutoReload(TIM1, 1000 - 1);
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);

    // Configure PWM mode for Channel 3 and 4
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetCompareCH3(TIM1, 0);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetCompareCH4(TIM1, 0);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);

    // Configure Input Capture for Channel 1
    LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

    // Enable update interrupt
    LL_TIM_EnableIT_UPDATE(TIM1);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

    LL_TIM_EnableIT_CC1(TIM1);
    NVIC_SetPriority(TIM1_CC_IRQn, 2);
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    // Enable TIM1 main output (for advanced timers)
    LL_TIM_EnableAllOutputs(TIM1);

    // Enable TIM1 counter
    LL_TIM_EnableCounter(TIM1);
}

//----------------------------------------------------------------------------------------------------------------------

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

    // Enable update interrupt for TIM3 to process DMA buffer
    LL_TIM_EnableIT_UPDATE(TIM3);

    // Enable TIM3
    LL_TIM_EnableCounter(TIM3);
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes DMA for TIM1 input capture.
 *
 * Sets up DMA to capture TIM1 counter values on COMP2 events.
 */
static void peripheral_tim1_dma_init(void)
{
    // Enable DMA1 clock
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    rbuff_init_dma_ro(&tim1_ic_dma_rb, (void *)tim1_ic_dma_buf, TIM1_IC_DMA_BUF_LEN, sizeof(tim1_ic_dma_buf[0]),
                      peripheral_tim1_dma_w_ptr_get);

    // Configure DMA1 Channel1 for TIM1_CH1 (COMP2 input capture)
    LL_DMA_InitTypeDef DMA_InitStruct     = {0};
    DMA_InitStruct.PeriphOrM2MSrcAddress  = (uint32_t)&TIM1->CCR1;
    DMA_InitStruct.MemoryOrM2MDstAddress  = (uint32_t)tim1_ic_dma_buf;
    DMA_InitStruct.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    DMA_InitStruct.Mode                   = LL_DMA_MODE_CIRCULAR;
    DMA_InitStruct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStruct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
    DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
    DMA_InitStruct.NbData                 = TIM1_IC_DMA_BUF_LEN;
    DMA_InitStruct.Priority               = LL_DMA_PRIORITY_HIGH;

    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);

    // Remap TIM1_CH1 to DMA1 Channel 1
    LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_TIM1_CH1);

    // Enable TIM1 CH1 DMA request
    LL_TIM_EnableDMAReq_CC1(TIM1);

    // Enable DMA channel
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    // Configure and enable NVIC for DMA interrupt
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Gets the write pointer for the TIM1 input capture DMA buffer.
 *
 * This function calculates the number of bytes written to the DMA buffer
 * and returns the write pointer, which is the current position in the buffer.
 *
 * @return Pointer to the current write position in the TIM1 input capture DMA buffer.
 */
void *peripheral_tim1_dma_w_ptr_get(void)
{
    uint32_t remaining = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
    uint32_t written   = TIM1_IC_DMA_BUF_LEN - remaining;
    return (void *)(tim1_ic_dma_buf + written);
}