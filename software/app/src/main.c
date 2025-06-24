/* Includes ------------------------------------------------------------------*/
#include "bdcsc.h"

/* Private define ------------------------------------------------------------*/

#ifndef VERSION
#define VERSION "not found"
#endif

#define MOTOR_GEAR_RATIO                       (298)
#define MOTOR_ENCODER_PULSES_PER_REVOLUTION    (7 * 4)
#define MOTOR_SENSORLESS_PULSES_PER_REVOLUTION (6)

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef sConfig;
uint32_t aADCxConvertedData[ENCODER_CHANNEL_CNT] = {0};

TIM_HandleTypeDef Tim1Handle;     // ADC/IR timer
TIM_HandleTypeDef encoderHandle;  // Encoder timer
TIM_HandleTypeDef motorPwmHandle; // PWM

typedef struct {
    bool enable;       /**< Enable motor. */
    uint8_t speed;     /**< Motor speed (0-255). */
    bool direction_cw; /**< Motor direction: true for clockwise, false for counter-clockwise. */
} motor_t;

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void APP_ErrorHandler(void);
static void APP_GpioConfig(void);
static void APP_TimerInit(void);
static void APP_PwmInit(void);
static void APP_AdcConfig(void);
static void APP_DmaInit(void);
static void APP_EncoderInit(void);

void motor_update(const motor_t *motor);

uint16_t encoder_position_get(void);

/* Debug terminal functions. */
void debug_term_motor_toggle(const char *input, void *arg);
void debug_term_motor_set_speed(const char *input, void *arg);
void debug_term_motor_position_set(const char *input, void *arg);
void debug_term_motor_set_direction(const char *input, void *arg);
void debug_term_encoder_toggle(const char *input, void *arg);
void debug_term_pid_set(const char *input, void *arg);
void debug_term_pid_print(const char *input, void *arg);
/* Motor stuff */
static motor_t debug_motor           = {0};
static uint16_t debug_motor_position = 0; /**< Current motor position. */

/* Debug GPIO */
static bool debug_gpio[4] = {false, false, false, false};

/* Quad encoder */
static uint16_t prev_encoder_value = 0;
static bool enable_encoder         = false;

/* PID */
pid_ctx_t pid_ctx = {0};

/* Debug IO scope struct */
typedef struct {
    uint32_t setpoint;
    uint32_t actual;
    int32_t p;
    int32_t i;
    int32_t d;
    int32_t error;
} debug_io_scope_t;
static debug_io_scope_t debug_io_scope = {0};

int main(void)
{
    HAL_Init();
    BSP_HSI_24MHzClockConfig();

    debug_io_init(LOG_LVL_DEBUG);
    debug_io_scope_init("u4u4i4i4i4i4");

    APP_GpioConfig();
    // APP_DmaInit();
    // APP_AdcConfig();
    APP_PwmInit();
    APP_EncoderInit();
    motor_update(&debug_motor);
    // APP_TimerInit();

    debug_io_term_register_keyword("m", debug_term_motor_toggle, &debug_motor);
    debug_io_term_register_keyword("ms", debug_term_motor_set_speed, &debug_motor);
    debug_io_term_register_keyword("md", debug_term_motor_set_direction, &debug_motor);
    debug_io_term_register_keyword("mp", debug_term_motor_position_set, &debug_motor);
    debug_io_term_register_keyword("enc", debug_term_encoder_toggle, NULL);
    debug_io_term_register_keyword("pid", debug_term_pid_set, &pid_ctx);
    debug_io_term_register_keyword("pe", debug_term_pid_print, &pid_ctx); /* Register the new function */

    debug_io_log_info("BDCSC dev board\n");
    debug_io_log_info("Version: %s\n", GIT_VERSION);
    debug_io_log_info("Compilation Date: %s %s\n", __DATE__, __TIME__);

    pid_init(&pid_ctx, 3000, 300, -800); // Initialize PID with kp=1, ki=0, kd=0
    while (1) {

        debug_io_term_process();
        uint16_t encoder_count = encoder_position_get();
        if (encoder_count != prev_encoder_value) {
            prev_encoder_value = encoder_count;
            if (enable_encoder) {
                debug_io_log_info("ENC: %u\n", encoder_count);
            }
        }

        static uint32_t last_tick = 0;
        uint32_t current_tick     = HAL_GetTick();
        if (current_tick - last_tick >= 50) { // every 10 ms
            last_tick = current_tick;
            if (debug_motor.enable) {
                pid_ctx.setpoint   = debug_motor_position / 6;
                int16_t pid_output = pid_compute(&pid_ctx, encoder_count / 6);
                if (pid_output > 0) {
                    debug_motor.direction_cw = true;
                    debug_motor.speed        = (uint8_t)(pid_output > 255 ? 255 : pid_output);
                } else {
                    debug_motor.direction_cw = false;
                    debug_motor.speed        = (uint8_t)(-pid_output > 255 ? 255 : -pid_output);
                }
                if (debug_motor.speed < 10) {
                    debug_motor.speed = 0; // If speed is too low, stop the motor
                }
                if (encoder_count != prev_encoder_value) {
                    prev_encoder_value = encoder_count;
                    // debug_io_log_info("PID: %d, Speed: %u, Position: %u\n", pid_output, debug_motor.speed,
                    //                   encoder_count);
                }
                motor_update(&debug_motor);
            }
            // debug_io_log_debug("Hello World!\n");
            debug_io_scope.setpoint = pid_ctx.setpoint;
            debug_io_scope.actual   = encoder_count / 6;
            debug_io_scope.p        = pid_ctx.p_error;
            debug_io_scope.i        = pid_ctx.i_error;
            debug_io_scope.d        = pid_ctx.d_error;
            debug_io_scope.error    = pid_ctx.output;

            debug_io_scope_push(&debug_io_scope, sizeof(debug_io_scope));
        }
    }
}

static void APP_GpioConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // /* Configure column-end detection pin. */
    // GPIO_InitStruct.Pin   = COLEND_GPIO_PIN;
    // GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    // GPIO_InitStruct.Pull  = GPIO_PULLUP;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // HAL_GPIO_Init(COLEND_GPIO_PORT, &GPIO_InitStruct);

    // /* Configure IR LED */
    // GPIO_InitStruct.Pin   = ENCODER_LED_GPIO_PIN;
    // GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    // GPIO_InitStruct.Pull  = GPIO_PULLUP;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // HAL_GPIO_Init(ENCODER_LED_GPIO_PORT, &GPIO_InitStruct);

    /* Configure debug pins, these pins can be used for pin wiggling during development. */
    GPIO_InitStruct.Pin   = DEBUG_GPIO_1_PIN | DEBUG_GPIO_2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DEBUG_GPIO_1_2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = DEBUG_GPIO_3_PIN | DEBUG_GPIO_4_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DEBUG_GPIO_3_4_PORT, &GPIO_InitStruct);
}

static void APP_AdcConfig(void)
{
    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET(); /* Reset ADC */
    __HAL_RCC_ADC_CLK_ENABLE();    /* Enable ADC clock */

    AdcHandle.Instance = ADC1;
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK) {
        APP_ErrorHandler();
    }
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;   /* ADC clock no division */
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_10B;         /* 12bit */
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;        /* Right alignment */
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD; /* Forward */
    AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;           /* End flag */
    AdcHandle.Init.LowPowerAutoWait      = ENABLE;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConv      = ADC1_2_EXTERNALTRIG_T1_TRGO; /* External trigger: TIM1_TRGO */
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.DMAContinuousRequests = ENABLE; /* DMA */
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    AdcHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_41CYCLES_5;
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
        APP_ErrorHandler();
    }

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    /* Configure the ADC channels. */
    for (uint8_t i = 0; i < ENCODER_CHANNEL_CNT; i++) {
        sConfig.Channel = ENCODER_ADC_CHANNEL_LIST[i];
        if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
            APP_ErrorHandler();
        }
    }
}

static void APP_TimerInit(void)
{
    // /* (240 * 10) / 24Mhz = 100us */
    // Tim1Handle.Instance               = TIM1;
    // Tim1Handle.Init.Period            = 10 - 1;
    // Tim1Handle.Init.Prescaler         = 240 - 1;
    // Tim1Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    // Tim1Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    // Tim1Handle.Init.RepetitionCounter = 0;
    // Tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // if (HAL_TIM_Base_Init(&Tim1Handle) != HAL_OK) {
    //     APP_ErrorHandler();
    // }

    // TIM_MasterConfigTypeDef sMasterConfig;
    // sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    // sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    // HAL_TIMEx_MasterConfigSynchronization(&Tim1Handle, &sMasterConfig);
    // if (HAL_TIM_Base_Start_IT(&Tim1Handle) != HAL_OK) {
    //     APP_ErrorHandler();
    // }
}

static void APP_PwmInit(void)
{
    /* 24MHz / ( 255 x 466 x 2 ) = +/- 200Hz */
    motorPwmHandle.Instance           = TIM1;
    motorPwmHandle.Init.Period        = 255 - 1;
    motorPwmHandle.Init.Prescaler     = 470 - 1;
    motorPwmHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    motorPwmHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
    // motorPwmHandle.Init.RepetitionCounter = 0;
    // motorPwmHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&motorPwmHandle) != HAL_OK) {
        APP_ErrorHandler();
    }

    TIM_MasterConfigTypeDef sMasterConfig;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&motorPwmHandle, &sMasterConfig);

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // Configure channel 1
    HAL_TIM_PWM_ConfigChannel(&motorPwmHandle, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&motorPwmHandle, TIM_CHANNEL_3);

    // Configure channel 2
    HAL_TIM_PWM_ConfigChannel(&motorPwmHandle, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&motorPwmHandle, TIM_CHANNEL_4);
}

void APP_EncoderInit(void)
{
    encoderHandle.Instance               = TIM3;
    encoderHandle.Init.Prescaler         = 0;
    encoderHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    encoderHandle.Init.Period            = 298 * 7 * 4 - 1; /* 298*7*4 = 8344, so period is 8343 */
    encoderHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    encoderHandle.Init.RepetitionCounter = 0;
    encoderHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef sConfig = {0};
    sConfig.EncoderMode             = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity             = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection            = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler            = TIM_ICPSC_DIV1;
    sConfig.IC1Filter               = 10;
    sConfig.IC2Polarity             = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection            = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler            = TIM_ICPSC_DIV1;
    sConfig.IC2Filter               = 10;

    if (HAL_TIM_Encoder_Init(&encoderHandle, &sConfig) != HAL_OK) {
        APP_ErrorHandler();
    }

    HAL_TIM_Encoder_Start(&encoderHandle, TIM_CHANNEL_ALL);
}

static void APP_DmaInit(void)
{
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* Disable IR led. */
    // HAL_GPIO_WritePin(ENCODER_LED_GPIO_PORT, ENCODER_LED_GPIO_PIN, GPIO_PIN_RESET);

    if (false) {
        debug_io_log_debug("ABZ: %04ld %04ld %04ld\n", aADCxConvertedData[ENCODER_CHANNEL_A],
                           aADCxConvertedData[ENCODER_CHANNEL_B], aADCxConvertedData[ENCODER_CHANNEL_Z]);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
    }
}

void APP_ErrorHandler(void)
{
    while (1)
        ;
}

void motor_update(const motor_t *motor)
{
    if (motor->enable) {
        if (motor->direction_cw) {
            __HAL_TIM_SET_COMPARE(&motorPwmHandle, TIM_CHANNEL_3, 0xff - motor->speed);
            __HAL_TIM_SET_COMPARE(&motorPwmHandle, TIM_CHANNEL_4, 0xff);
        } else {
            __HAL_TIM_SET_COMPARE(&motorPwmHandle, TIM_CHANNEL_3, 0xff);
            __HAL_TIM_SET_COMPARE(&motorPwmHandle, TIM_CHANNEL_4, 0xff - motor->speed);
        }
    } else {
        __HAL_TIM_SET_COMPARE(&motorPwmHandle, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&motorPwmHandle, TIM_CHANNEL_4, 0);
    }
}

/**
 * \brief Get the current encoder position.
 *
 * The encoder position is scaled to match the encoder position as it would be measured with the sensorless method.
 */
uint16_t encoder_position_get(void)
{
    return __HAL_TIM_GET_COUNTER(&encoderHandle) * MOTOR_SENSORLESS_PULSES_PER_REVOLUTION /
           MOTOR_ENCODER_PULSES_PER_REVOLUTION;
}

void debug_term_motor_toggle(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    motor->enable  = !motor->enable;
    debug_io_log_info("Motor: %s\n", motor->enable ? "enabled" : "disabled");
    motor_update(motor);
}

void debug_term_motor_set_speed(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    int speed      = atoi(input);
    if (speed < 0 || speed > 255) {
        debug_io_log_info("Invalid speed value. Must be between 0 and 255.\n");
        return;
    }
    motor->speed = (uint8_t)speed;
    debug_io_log_info("Motor speed set to: %u\n", motor->speed);
    motor_update(motor);
}

void debug_term_motor_set_direction(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    if (strcmp(input, "cw") == 0) {
        motor->direction_cw = true;
        debug_io_log_info("Motor direction set to: CW\n");
    } else if (strcmp(input, "ccw") == 0) {
        motor->direction_cw = false;
        debug_io_log_info("Motor direction set to: CCW\n");
    } else {
        debug_io_log_info("Invalid direction. Use 'cw' or 'ccw'.\n");
    }
    motor_update(motor);
}

void debug_term_motor_position_set(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    int position   = atoi(input);
    if (position < 0 || position > 1787) {
        debug_io_log_info("Invalid position value. Must be between 0 and 1787.\n");
        return;
    }
    debug_motor_position = (uint16_t)position;
    debug_io_log_info("Motor position set to: %u\n", debug_motor_position);
}

void debug_term_encoder_toggle(const char *input, void *arg)
{
    enable_encoder = !enable_encoder;
    debug_io_log_info("Encoder: %s\n", enable_encoder ? "enabled" : "disabled");
}

void debug_term_pid_set(const char *input, void *arg)
{
    pid_ctx_t *pid = (pid_ctx_t *)arg;
    int kp, ki, kd;
    if (sscanf(input, "%d %d %d", &kp, &ki, &kd) == 3) {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
        debug_io_log_info("PID gains set: kp=%d, ki=%d, kd=%d\n", kp, ki, kd);
    } else {
        debug_io_log_info("Usage: pid <kp> <ki> <kd>\n");
    }
}

void debug_term_pid_print(const char *input, void *arg)
{
    pid_ctx_t *pid = (pid_ctx_t *)arg;

    // Print all internal PID values for debugging
    debug_io_log_info("PID Internal State:\n");
    debug_io_log_info("  Gains: kp=%d, ki=%d, kd=%d\n", pid->kp, pid->ki, pid->kd);
    debug_io_log_info("  Setpoint: %d, Current Input: %d\n", pid->setpoint, encoder_position_get() / 6);
    debug_io_log_info("  Error: %d\n", pid->setpoint - (encoder_position_get() / 6));
    debug_io_log_info("  P-term: %d\n", pid->p_error);
    debug_io_log_info("  I-term: %d\n", pid->i_error);
    debug_io_log_info("  D-term: %d\n", pid->d_error);
    debug_io_log_info("  Output: %d\n", pid->output);
}
