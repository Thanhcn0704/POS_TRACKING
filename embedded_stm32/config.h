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
 * 6. LOGIC MUC RELAY — Active LOW (open-drain, board keo len 5V)
 * ============================================================
 *
 *  Module relay opto-cach-ly pho bien la ACTIVE LOW: chan IN da duoc
 *  keo len VCC=5V san tren board. MCU keo IN xuong 0V (SINK dong qua
 *  LED opto) de KICH relay; tha noi (hi-Z) de NHA relay.
 *
 *    RELAY_ACTIVE_HIGH = 0  ->  ON_LEVEL=0 (keo xuong), OFF_LEVEL=1 (tha)
 *
 *  >>> CHE DO GPIO: OPEN-DRAIN <<<  (xem RELAY_OPEN_DRAIN ben duoi)
 *  Push-pull se day chan len 3.3V doi dau voi dien tro keo 5V cua board
 *  -> tranh chap dong / opto co the khong tat sach. Open-drain chi co 2
 *  trang thai: SINK xuong 0V (ON) hoac tha hi-Z (OFF, board keo len 5V).
 *
 *  >>> AN TOAN 5V <<<  PB8 va PB9 cua STM32F407 la I/O loai "FT"
 *  (5 V tolerant — DS8626/DM00037051 Table 10; cung la chan I2C1/CAN1
 *  von chiu 5V). Chan FT KHONG co diode bao len VDD nen o trang thai
 *  hi-Z co the noi len 5V ma KHONG bom dong vao rail 3.3V -> khong hong MCU.
 *
 *  >>> DIEU KIEN BAT BUOC <<<  Dien tro keo cua board PHAI ve <= 5V.
 *  Neu chan IN idle o 12V/24V thi open-drain se VUOT gioi han FT (5V)
 *  -> HONG CHAN. Truong hop do PHAI dung transistor/MOSFET hoac
 *  level-shifter, KHONG noi truc tiep PB8/PB9.
 */

#define RELAY_ACTIVE_HIGH   0

#if RELAY_ACTIVE_HIGH
  #define RELAY_ON_LEVEL    1U
  #define RELAY_OFF_LEVEL   0U
#else
  #define RELAY_ON_LEVEL    0U
  #define RELAY_OFF_LEVEL   1U
#endif

/*  Kieu ngo ra chan relay (PB8/PB9):
 *    1 = OPEN-DRAIN  (bat buoc cho board active-low keo 5V; PB8/PB9 = FT)
 *    0 = PUSH-PULL   (chi dung neu board thiet ke cho muc logic 3.3V)
 *  Ngo ra cung dat OSPEEDR = LOW (trong main.c) de giam EMI/ringing tren
 *  day dieu khien relay.  */
#define RELAY_OPEN_DRAIN    1

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
