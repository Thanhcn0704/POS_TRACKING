/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Conveyor Encoder + Dual Relay + UART -> RPi5
  *          STM32F407VET6 | CMSIS Bare-metal
  *
  *  Ban hop nhat:
  *    - Encoder (TIM2, PA0/PA1) + giao thuc UART nhi phan (0xAA/0xBB TX,
  *      0xCC RX) GIU NGUYEN theo cau hinh da xac nhan chay dung thuc te.
  *    - Relay1 (PB8, xy-lanh day phoi) tu dong kich moi 3s, KHONG nhan
  *      lenh tu Pi.
  *    - Relay2 (PB9, hut chan khong) nhan lenh BAT/TAT tu Pi qua byte
  *      r1 trong goi UART RX (khop voi uart_receiver.send_relay()).
  *    - Motor bang tai (PE11/PE12) chay LIEN TUC 1 chieu ngay sau khoi
  *      dong, KHONG con nhan lenh Forward/Stop/Reset tu Pi qua UART.
  *
  *  >>> HEARTBEAT (Feature: UART communication status indicator) <<<
  *    - Them khung PING 0xDD (Pi -> STM32) va ACK 0xCD/0xCE (STM32 -> Pi).
  *    - LED (config HEARTBEAT_LED_*) DOI TRANG THAI moi khi nhan duoc 1
  *      khung hop le (0xCC relay HOAC 0xDD ping) tu Pi.
  *    - Khi nhan 0xDD: toggle LED + tra loi ACK (gui o main context).
  *    - TAT CA cac thay doi heartbeat duoc danh dau bang "(HB)".
  *      Hanh vi 0xAA/0xCC/relay/motor/encoder GIU NGUYEN.
  *
  *  De thay doi GPIO hoac timing:
  *    -> Chi chinh file config.h, KHONG sua file nay.
  *
  *  UART protocol:
  *    TX (STM32 -> Pi), moi 100ms, 11 byte:
  *      [0xAA][0xBB][float rpm(4B)][int32 total_ticks(4B)][checksum]
  *      checksum = XOR cua 8 byte payload
  *    RX (Pi -> STM32), 4 byte:
  *      [0xCC][r1][r2][checksum]   checksum = r1 XOR r2
  *      r1 = 0x01 -> bat Relay2 (hut chan khong)
  *      r1 = 0x00 -> tat Relay2
  *      r2 hien khong dung
  *    PING (Pi -> STM32), 4 byte:   (HB)
  *      [0xDD][seq][0x00][seq^0x00]
  *    ACK  (STM32 -> Pi), 4 byte:   (HB)
  *      [0xCD][0xCE][seq][0xCE^seq]
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "config.h"
#include <string.h>

/* ============================================================
 * BIEN TOAN CUC
 * ============================================================ */
/* USER CODE BEGIN PV */
volatile int32_t total_ticks = 0;
volatile float   rpm_now     = 0.0f;

volatile uint8_t relay1_state = 0;
volatile uint8_t relay2_state = 0;

volatile uint8_t rx_r1_cmd = 0;
volatile uint8_t rx_r2_cmd = 0;

volatile uint8_t hb_led_state     = 0;   /* HB: trang thai LED (chi ghi trong IRQ) */
volatile uint8_t ping_ack_pending = 0;   /* HB: co ping can tra loi ACK */
volatile uint8_t ping_seq         = 0;   /* HB: seq cua ping de echo lai trong ACK */

#define TX_BUF_SIZE   32U
volatile uint8_t tx_buf[TX_BUF_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;

volatile uint32_t sys_tick_ms = 0;
/* USER CODE END PV */

/* ============================================================
 * FUNCTION PROTOTYPES
 * ============================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void Encoder_TIM2_Init(void);
static void SysTick_Init(void);
static void USART2_Init(void);
static void Uart_Send_Data(const uint8_t *data, uint16_t len);
static void Relay_Set(GPIO_TypeDef *port, uint8_t pin, uint8_t on);
static void Motor_Forward(uint16_t duty);
static void Heartbeat_LED_Toggle(void);          /* HB */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ------------------------------------------------------------------
 * Heartbeat_LED_Toggle — doi trang thai LED bao link UART  (HB)
 *   Dung BSRR + bien trang thai phan mem (khong doc-sua-ghi ODR)
 *   nen an toan khi goi tu IRQ.
 * ------------------------------------------------------------------ */
static void Heartbeat_LED_Toggle(void)
{
    hb_led_state ^= 1U;
    HEARTBEAT_LED_GPIO->BSRR = hb_led_state
        ? (1U <<  HEARTBEAT_LED_PIN)
        : (1U << (HEARTBEAT_LED_PIN + 16U));
}

/* ------------------------------------------------------------------
 * Relay_Set
 * ------------------------------------------------------------------ */
static void Relay_Set(GPIO_TypeDef *port, uint8_t pin, uint8_t on)
{
    if (on)
        port->BSRR = (1U << (RELAY_ON_LEVEL  ? pin : (pin + 16U)));
    else
        port->BSRR = (1U << (RELAY_OFF_LEVEL ? pin : (pin + 16U)));

    if (port == RELAY1_GPIO && pin == RELAY1_PIN) relay1_state = on ? 1U : 0U;
    if (port == RELAY2_GPIO && pin == RELAY2_PIN) relay2_state = on ? 1U : 0U;
}

/* ------------------------------------------------------------------
 * Motor_Forward — bang tai chay lien tuc 1 chieu (PE11/PE12)
 * ------------------------------------------------------------------ */
static void Motor_Forward(uint16_t duty)
{
    (void)duty;
    MOTOR_GPIO->BSRR = (1U << MOTOR_IN1_PIN) | (1U << (MOTOR_IN2_PIN + 16U));
}

/* ------------------------------------------------------------------
 * Uart_Send_Data — day vao ring buffer TX, kich hoat ngat TXE
 * ------------------------------------------------------------------ */
static void Uart_Send_Data(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t next_head = (uint8_t)((tx_head + 1U) % TX_BUF_SIZE);
        while (next_head == tx_tail) { }
        tx_buf[tx_head] = data[i];
        tx_head = next_head;
    }
    UART_PERIPH->CR1 |= USART_CR1_TXEIE;
}

/* ------------------------------------------------------------------
 * Encoder_TIM2_Init — PA0 (CH1), PA1 (CH2)
 * ------------------------------------------------------------------ */
static void Encoder_TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    __DSB();

    GPIOA->MODER   &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOA->MODER   |=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD0   | GPIO_PUPDR_PUPD1);
    GPIOA->PUPDR   |=  (GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0);

    GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1);
    GPIOA->AFR[0]  |=  ((1U << GPIO_AFRL_AFSEL0_Pos) | (1U << GPIO_AFRL_AFSEL1_Pos));

    TIM2->CR1    = 0;
    TIM2->ARR    = 0xFFFFFFFF;
    TIM2->PSC    = 0;
    TIM2->CNT    = 0;

    TIM2->CCMR1  = 0;
    TIM2->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                   (15U << TIM_CCMR1_IC1F_Pos) |
                   (15U << TIM_CCMR1_IC2F_Pos));

    TIM2->CCER   = 0;
    TIM2->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1P;

    TIM2->SMCR  &= ~TIM_SMCR_SMS;
    TIM2->SMCR  |=  (3U << TIM_SMCR_SMS_Pos);

    TIM2->CR1   |=  TIM_CR1_CEN;
}

/* ------------------------------------------------------------------
 * SysTick_Init — ngat moi 1ms
 * ------------------------------------------------------------------ */
static void SysTick_Init(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = SYSTICK_LOAD;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

/* ------------------------------------------------------------------
 * USART2_Init — PA2 TX, PA3 RX, 115200 baud @ HSI 16MHz
 * ------------------------------------------------------------------ */
static void USART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    __DSB();

    GPIOA->MODER   &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER   |=  (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
    GPIOA->AFR[0]  |=  ((UART_AF << GPIO_AFRL_AFSEL2_Pos) | (UART_AF << GPIO_AFRL_AFSEL3_Pos));

    UART_PERIPH->CR1 = 0;
    UART_PERIPH->BRR = UART_BAUD_BRR;
    UART_PERIPH->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE END 0 */

/* ============================================================
 * INTERRUPT HANDLER — USART2
 * ============================================================ */
void USART2_IRQHandler(void)
{
    static uint8_t rx_buf[4];
    static uint8_t rx_idx = 0;
    static uint32_t last_rx_tick = 0;

    if (UART_PERIPH->SR & USART_SR_RXNE)
    {
        uint8_t data = (uint8_t)(UART_PERIPH->DR & 0xFF);
        uint32_t current_tick = sys_tick_ms;

        if ((current_tick - last_rx_tick) > 5U)
        {
            rx_idx = 0;
        }
        last_rx_tick = current_tick;

        if (rx_idx == 0)
        {
            if (data == 0xCC || data == UART_PING_CMD)   /* HB: chap nhan ca ping 0xDD */
            {
                rx_buf[rx_idx++] = data;
            }
        }
        else
        {
            rx_buf[rx_idx++] = data;

            if (rx_idx == 4)
            {
                if ((uint8_t)(rx_buf[1] ^ rx_buf[2]) == rx_buf[3])
                {
                    if (rx_buf[0] == 0xCC)
                    {
                        rx_r1_cmd = rx_buf[1];
                        rx_r2_cmd = rx_buf[2];

                        Relay_Set(RELAY2_GPIO, RELAY2_PIN, (rx_r1_cmd == 0x01U) ? 1U : 0U);

                        Heartbeat_LED_Toggle();          /* HB: khung hop le tu Pi */
                    }
                    else /* rx_buf[0] == UART_PING_CMD */
                    {
                        Heartbeat_LED_Toggle();          /* HB: ping hop le tu Pi */
                        ping_seq         = rx_buf[1];    /* HB: echo seq trong ACK */
                        ping_ack_pending = 1U;           /* HB: gui ACK o main loop */
                    }
                }
                rx_idx = 0;
            }
        }
    }

    if ((UART_PERIPH->SR & USART_SR_TXE) && (UART_PERIPH->CR1 & USART_CR1_TXEIE))
    {
        if (tx_tail != tx_head)
        {
            UART_PERIPH->DR = tx_buf[tx_tail];
            tx_tail = (uint8_t)((tx_tail + 1U) % TX_BUF_SIZE);
        }
        else
        {
            UART_PERIPH->CR1 &= ~USART_CR1_TXEIE;
        }
    }

    if (UART_PERIPH->SR & USART_SR_ORE)
    {
        (void)UART_PERIPH->DR;
    }
}

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* USER CODE BEGIN 2 */
    Encoder_TIM2_Init();
    SysTick_Init();
    USART2_Init();

    Relay_Set(RELAY1_GPIO, RELAY1_PIN, 0);
    Relay_Set(RELAY2_GPIO, RELAY2_PIN, 0);

    Motor_Forward(MOTOR_DUTY);

    uint32_t last_cnt        = 0;
    uint32_t last_sample_tick = sys_tick_ms;
    uint32_t relay1_on_ms     = 0;
    uint32_t last_r1_cycle    = sys_tick_ms;
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        uint32_t now = sys_tick_ms;

        /* --------------------------------------------------------
         * TASK 1: Doc encoder + gui UART moi ENC_SAMPLE_MS
         * -------------------------------------------------------- */
        if ((now - last_sample_tick) >= ENC_SAMPLE_MS)
        {
            last_sample_tick = now;

            uint32_t current_ticks = TIM2->CNT;
            total_ticks = (int32_t)current_ticks;

            int32_t diff = (int32_t)(current_ticks - last_cnt);
            last_cnt = current_ticks;

            int32_t abs_diff = (diff < 0) ? -diff : diff;
            rpm_now = (float)abs_diff * SAMPLES_PER_MIN / ENCODER_PPR;

            uint8_t tx_pack[11];
            tx_pack[0] = 0xAA;
            tx_pack[1] = 0xBB;
            memcpy((void*)&tx_pack[2], (const void*)&rpm_now, 4);
            memcpy((void*)&tx_pack[6], (const void*)&total_ticks, 4);

            uint8_t ck = 0;
            for (int i = 2; i < 10; i++)
            {
                ck ^= tx_pack[i];
            }
            tx_pack[10] = ck;

            Uart_Send_Data(tx_pack, 11);
        }

        /* --------------------------------------------------------
         * TASK 2: Relay1 — tu dong kich moi RELAY1_PERIOD_MS,
         *         giu ON trong RELAY1_ON_MS. KHONG nhan lenh tu Pi.
         * -------------------------------------------------------- */
        if ((now - last_r1_cycle) >= RELAY1_PERIOD_MS)
        {
            last_r1_cycle = now;
            Relay_Set(RELAY1_GPIO, RELAY1_PIN, 1);
            relay1_on_ms = now;
        }

        if (relay1_state && (now - relay1_on_ms) >= RELAY1_ON_MS)
        {
            Relay_Set(RELAY1_GPIO, RELAY1_PIN, 0);
        }

        /* --------------------------------------------------------   (HB)
         * TASK 3: HEARTBEAT — tra loi ACK cho ping tu Pi.
         *   Gui o main context (KHONG trong IRQ) de tranh tranh chap
         *   tx_head voi TASK 1. ACK = [0xCD, 0xCE, seq, 0xCE^seq].
         * -------------------------------------------------------- */
        if (ping_ack_pending)
        {
            ping_ack_pending = 0U;

            uint8_t ack[4];
            ack[0] = UART_ACK_HDR1;
            ack[1] = UART_ACK_HDR2;
            ack[2] = ping_seq;
            ack[3] = (uint8_t)(UART_ACK_HDR2 ^ ping_seq);

            Uart_Send_Data(ack, 4);
        }

        /* USER CODE END 3 */
    }
}

/* ============================================================
 * SYSTEM CLOCK — HSI 16 MHz, khong dung PLL
 *   (giu nguyen theo cau hinh da xac nhan chay dung thuc te)
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================
 * GPIO INIT — Motor + Relay1 + Relay2 + Heartbeat LED
 * ============================================================ */
static void MX_GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
    RCC->AHB1ENR |= HEARTBEAT_LED_AHB1EN;                         /* HB: clock port LED */
    __DSB();

    /* Motor IN1/IN2 (PE11/PE12) */
    GPIOE->MODER   &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOE->MODER   |=  (GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0);
    GPIOE->OTYPER  &= ~((1U << 11U) | (1U << 12U));
    GPIOE->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12);
    GPIOE->PUPDR   &= ~(GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);
    GPIOE->BSRR     =  GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12;

    /* Relay1 (PB8) */
    RELAY1_GPIO->MODER   &= ~(0x3U << (RELAY1_PIN * 2U));
    RELAY1_GPIO->MODER   |=  (0x1U << (RELAY1_PIN * 2U));
    RELAY1_GPIO->OTYPER  &= ~(1U   <<  RELAY1_PIN);
    RELAY1_GPIO->OSPEEDR |=  (0x2U << (RELAY1_PIN * 2U));
    RELAY1_GPIO->PUPDR   &= ~(0x3U << (RELAY1_PIN * 2U));

    /* Relay2 (PB9) */
    RELAY2_GPIO->MODER   &= ~(0x3U << (RELAY2_PIN * 2U));
    RELAY2_GPIO->MODER   |=  (0x1U << (RELAY2_PIN * 2U));
    RELAY2_GPIO->OTYPER  &= ~(1U   <<  RELAY2_PIN);
    RELAY2_GPIO->OSPEEDR |=  (0x2U << (RELAY2_PIN * 2U));
    RELAY2_GPIO->PUPDR   &= ~(0x3U << (RELAY2_PIN * 2U));

    /* Heartbeat LED — output push-pull, mac dinh TAT  (HB) */
    HEARTBEAT_LED_GPIO->MODER   &= ~(0x3U << (HEARTBEAT_LED_PIN * 2U));
    HEARTBEAT_LED_GPIO->MODER   |=  (0x1U << (HEARTBEAT_LED_PIN * 2U));
    HEARTBEAT_LED_GPIO->OTYPER  &= ~(1U   <<  HEARTBEAT_LED_PIN);
    HEARTBEAT_LED_GPIO->OSPEEDR |=  (0x1U << (HEARTBEAT_LED_PIN * 2U));
    HEARTBEAT_LED_GPIO->PUPDR   &= ~(0x3U << (HEARTBEAT_LED_PIN * 2U));
    HEARTBEAT_LED_GPIO->BSRR     =  (1U   << (HEARTBEAT_LED_PIN + 16U));

    /* Tat ca 2 relay ban dau */
    if (RELAY_OFF_LEVEL)
    {
        RELAY1_GPIO->BSRR = (1U << RELAY1_PIN);
        RELAY2_GPIO->BSRR = (1U << RELAY2_PIN);
    }
    else
    {
        RELAY1_GPIO->BSRR = (1U << (RELAY1_PIN + 16U));
        RELAY2_GPIO->BSRR = (1U << (RELAY2_PIN + 16U));
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
