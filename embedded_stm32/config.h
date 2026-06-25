#ifndef CONFIG_H
#define CONFIG_H

/**
  ******************************************************************************
  * @file    config.h
  * @brief   FILE CAU HINH TRUNG TAM - Chi chinh file nay, khong cham main.c
  *
  *  Moi thay doi o day tu dong ap dung cho toan bo chuong trinh.
  *  Sau khi chinh, Build lai la xong.
  ******************************************************************************
  */

/* ============================================================
 * 1. ENCODER
 * ============================================================
 *
 *  Timer dung     : TIM2 (PA0/PA1)
 *  Mode           : Encoder Mode 3 (x4, dem tren ca 2 canh)
 *
 *  ENCODER_PPR    : Tong so xung dem duoc tren 1 vong truc encoder
 *                   (da tinh ca ti so hop so va x4 mode neu can)
 *                   Do thuc te: 32308 xung/vong
 */
#define ENCODER_PPR         32308U
#define SAMPLES_PER_MIN     600.0f

/* ============================================================
 * 2. RELAY 1 — Xilanh day phoi
 * ============================================================ */

#define RELAY1_GPIO         GPIOB
#define RELAY1_PIN          8U

#define RELAY1_PERIOD_MS    3000U
#define RELAY1_ON_MS        100U

/* ============================================================
 * 3. RELAY 2 — Hut chan khong
 * ============================================================ */

#define RELAY2_GPIO         GPIOB
#define RELAY2_PIN          9U

/* ============================================================
 * 4. MOTOR L298N — PE11/PE12, jumper 100%, khong dung PWM
 * ============================================================ */

#define MOTOR_GPIO          GPIOE
#define MOTOR_IN1_PIN       11U
#define MOTOR_IN2_PIN       12U
#define MOTOR_DUTY          0U

/* ============================================================
 * 5. UART — Giao tiep voi RPi5
 * ============================================================ */

#define UART_PERIPH         USART2
#define UART_GPIO           GPIOA
#define UART_TX_PIN         2U
#define UART_RX_PIN         3U
#define UART_AF             0x7U

#define UART_BAUD_BRR       0x008BU
#define UART_TX_BUF_SIZE    64U
#define UART_RX_BUF_SIZE    32U

/* ============================================================
 * 6. LOGIC MUC RELAY — Active HIGH
 * ============================================================ */

#define RELAY_ACTIVE_HIGH   1

#if RELAY_ACTIVE_HIGH
  #define RELAY_ON_LEVEL    1U
  #define RELAY_OFF_LEVEL   0U
#else
  #define RELAY_ON_LEVEL    0U
  #define RELAY_OFF_LEVEL   1U
#endif

/* ============================================================
 * 7. SYSTEM CLOCK — HSI 16MHz, khong dung PLL
 * ============================================================
 *
 *  SysTick IRQ moi 1ms:
 *    SYSTICK_LOAD = 16000000 / 1000 - 1 = 15999
 */
#define HCLK_HZ             16000000UL
#define SYSTICK_LOAD        (HCLK_HZ / 1000U - 1U)

/* ============================================================
 * 8. ENCODER SAMPLING
 * ============================================================ */

#define ENC_SAMPLE_MS       100U

/* ============================================================
 * 9. HEARTBEAT / HANDSHAKE  (Feature: UART communication status)
 * ============================================================
 *
 *  LED bao trang thai link UART. No DOI TRANG THAI moi khi nhan
 *  duoc 1 khung hop le tu Pi:
 *    - khung relay  0xCC (BAT/TAT hut chan khong), HOAC
 *    - khung ping   0xDD (heartbeat tu uart_receiver.send_ping()).
 *  Khi Pi ngung gui -> LED "dong bang" => mat ket noi (nhin la biet).
 *
 *  >>> CHON CHAN LED THEO BOARD CUA BAN <<<
 *  Mac dinh PA6 (chan trong; clock GPIOA da bat san cho encoder/uart).
 *  Tranh cac chan da dung:
 *    PA0,PA1 (encoder) | PA2,PA3 (uart) | PB8,PB9 (relay) | PE11,PE12 (motor)
 *  Vi du LED on-board pho bien khac: PC13, PE0... -> doi 3 #define duoi.
 */
#define HEARTBEAT_LED_GPIO    GPIOA
#define HEARTBEAT_LED_PIN     6U
#define HEARTBEAT_LED_AHB1EN  RCC_AHB1ENR_GPIOAEN

/*  Opcode UART — PHAI khop voi uart_receiver.py  */
#define UART_PING_CMD         0xDDU    /* Pi -> STM32 : [0xDD, seq, 0x00, seq^0x00] */
#define UART_ACK_HDR1         0xCDU    /* STM32 -> Pi : [0xCD, 0xCE, seq, 0xCE^seq] */
#define UART_ACK_HDR2         0xCEU

#endif /* CONFIG_H */
