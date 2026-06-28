# HARDWARE PINOUT & INTERCONNECT SPECIFICATION
## POS_TRACKING / PICKTEST conveyor pick-and-place

> Machine-readable hardware spec for PCB/schematic generation.
> **`[V]` = VERIFIED** from firmware (`embedded_stm32/config.h`, `embedded_stm32/main.c`)
> or Pi software (`vision_pi5/config.py`). **`[TBD]` = integrator must specify/verify** —
> the firmware does not define this electrical detail; do NOT assume a value.

---

## 0. NODES

| Node | Part | Role | Logic level |
|------|------|------|-------------|
| U1 | Raspberry Pi 5 | Vision + math + master scheduler | 3.3 V `[V]` (NOT 5 V tolerant) |
| U2 | STM32F407VET6 | Real-time I/O: encoder capture, relays, motor, heartbeat | 3.3 V `[V]` |
| U3 | SCARA controller (Toshiba/Shibaura TSL3000 / THL400) | Motion execution (SCOL) | Ethernet (TCP) `[V]` |
| U4 | USB camera | 1280x720 @ 60 FPS MJPG `[V]` | USB |
| ENC1 | Quadrature encoder (conveyor) | Belt position/speed | `[TBD]` (5 V? 12 V? 24 V? line-driver/OC/push-pull) |
| K1 | Relay — feeder cylinder | Part pusher, autonomous | coil V `[TBD]` |
| K2 | Relay — vacuum valve | Suction gripper (Pi-commanded) | coil V `[TBD]` |
| M1 | Conveyor DC motor (via L298N `[V]`) | Belt drive, continuous 1-direction | motor V `[TBD]` |
| LED1 | Heartbeat indicator LED | UART link-alive | 3.3 V `[V]` |

---

## 1. SYSTEM TOPOLOGY

```
            USB                      Ethernet (TCP 192.168.0.124:1001) [V]
 [U4 Camera]───►[U1 Raspberry Pi 5]◄══════════════════════════►[U3 SCARA / TSL3000]
                       │ 3.3V UART (USART2, 115200 8N1) [V]                │
                       │   Pi.TXD ──────────► U2.PA3 (RX)                  │ pneumatic
                       │   Pi.RXD ◄────────── U2.PA2 (TX)                  ▼
                       │   GND ────────────── U2.GND   (common, mandatory) [vacuum cup
                       ▼                                                    on arm]
              [U2 STM32F407VET6]
                 PA0/PA1 ◄──[level shift?]── ENC1 (A/B quadrature)  [TBD level]
                 PB8 ──►[driver+flyback]──► K1 feeder cylinder valve
                 PB9 ──►[driver+flyback]──► K2 vacuum valve
                 PE11/PE12 ──► [L298N] ──► M1 conveyor motor
                 PA6 ──►[R]──► LED1
```

Note: the **vacuum cup is on the SCARA arm**, but its valve (K2) is switched by **U2 (STM32)**
on command from **U1 (Pi)** — vacuum control is Pi-side, never SCOL `DOUT`. The SCARA only
prints `REL` to tell the Pi when to drop. `[V]`

---

## 2. LOGIC LEVELS & POWER RAILS

| Rail | Value | Source | Status |
|------|-------|--------|--------|
| Pi 5 I/O | 3.3 V | on-board | `[V]` — **not 5 V tolerant** |
| STM32 I/O | 3.3 V | on-board LDO | `[V]` |
| STM32 core clock | HSI 16 MHz, no PLL | `config.h: HCLK_HZ=16000000` | `[V]` |
| Pi 5 supply | 5 V / 5 A (USB-C PD) | — | `[TBD]` integrator |
| STM32 board supply | — | — | `[TBD]` (typ. 5 V → 3.3 V) |
| Relay coil rail | — | — | `[TBD]` (e.g. 5 V / 12 V / 24 V) |
| Motor rail (L298N VS) | — | — | `[TBD]` (L298N up to ~46 V) |
| Encoder rail | — | — | `[TBD]` |

**Grounding:** all nodes MUST share a common ground (star topology recommended). The Pi↔STM32
UART will not work without GND↔GND. `[V — engineering requirement]`

---

## 3. INTERCONNECT — Pi 5 ↔ STM32 (UART, USART2) `[V]`

Protocol: **115200 baud, 8 data, no parity, 1 stop (8N1)**, AF7. `config.h: UART_BAUD_BRR=0x008B`.
**Both sides are 3.3 V → NO level shifter required.** Cross TX↔RX.

| Pi 5 (U1) | dir | STM32 (U2) | Signal | Notes |
|-----------|-----|------------|--------|-------|
| UART TXD (`/dev/ttyAMA0` TX) | ──► | **PA3** (USART2_RX) | Pi→STM32 | relay `0xCC` + ping `0xDD` |
| UART RXD (`/dev/ttyAMA0` RX) | ◄── | **PA2** (USART2_TX) | STM32→Pi | telemetry `0xAA/0xBB` + ACK `0xCD/0xCE` |
| GND | ─── | GND | common 0 V | mandatory |

- **Pi 5 physical pins `[TBD — verify device tree]`:** conventionally GPIO14 = header pin 8 (TXD),
  GPIO15 = header pin 10 (RXD), GND = pin 6/9/14/20/25/30/34/39. **Pi 5 changed UART routing** —
  confirm `/dev/ttyAMA0` maps to GPIO14/15 (enable UART, disable the serial-login console).
- Recommended: **100–330 Ω** series resistor in each signal line; optional 3.3 V TVS for ESD.
- Wire format (all `[V]`, see `embedded_stm32/main.c` header):
  - Telemetry STM32→Pi, 100 ms, 11 B: `[0xAA][0xBB][f32 rpm][i32 total_ticks][XOR]`
  - Relay Pi→STM32, 4 B: `[0xCC][r1][r2][r1^r2]` (r1=0x01 → K2 vacuum ON)
  - Ping Pi→STM32, 4 B: `[0xDD][seq][0x00][seq^0x00]`; ACK STM32→Pi: `[0xCD][0xCE][seq][0xCE^seq]`

---

## 4. INTERCONNECT — Pi 5 ↔ SCARA (U3) `[V]`

| Link | Spec | Notes |
|------|------|-------|
| Ethernet / TCP | Pi client → robot **192.168.0.124 : 1001** | `vision_pi5/config.py`. NOT a GPIO link. |

Numeric-only ASCII handshake over the socket (`REQ`/`ID,CMD,X,Y,Z,C,SHP`/`ACK`/gate/`DONE`).
No level shifting (standard Ethernet PHY/magnetics).

## 5. INTERCONNECT — Pi 5 ↔ Camera (U4) `[V]`

USB (UVC). `cv2.VideoCapture` at CAM 1280x720 @ 60 FPS, MJPG. Use a powered hub if the camera
draws near the port limit. `[TBD]` camera model/power.

---

## 6. STM32F407VET6 PIN MAP `[V]` (from `config.h` + `main.c`)

| MCU pin | Peripheral / AF | Direction | Net | Function |
|---------|-----------------|-----------|-----|----------|
| **PA0** | TIM2_CH1 (AF1, enc mode) | IN | ENC_A | Encoder channel A `[V]` |
| **PA1** | TIM2_CH2 (AF1, enc mode) | IN | ENC_B | Encoder channel B `[V]` |
| **PA2** | USART2_TX (AF7) | OUT | UART_TX→Pi.RXD | Telemetry + ACK `[V]` |
| **PA3** | USART2_RX (AF7) | IN | UART_RX←Pi.TXD | Relay cmd + ping `[V]` |
| **PA6** | GPIO | OUT | HB_LED | Heartbeat LED (toggles on valid 0xCC/0xDD) `[V]` |
| **PB8** | GPIO | OUT | RELAY1 | Feeder cylinder, **active HIGH**, autonomous 3 s / 100 ms ON `[V]` |
| **PB9** | GPIO | OUT | RELAY2 | Vacuum suction, **active HIGH**, Pi-commanded via 0xCC r1 `[V]` |
| **PE11**| GPIO | OUT | MOTOR_IN1 | L298N IN1 `[V]` |
| **PE12**| GPIO | OUT | MOTOR_IN2 | L298N IN2 `[V]` |

Encoder: TIM2 Encoder Mode 3 (x4), `ENCODER_PPR = 32308` pulses/rev `[V]`. Relay polarity:
`RELAY_ACTIVE_HIGH = 1` `[V]`. Motor: jumper 100% (no PWM), runs continuous one direction `[V]`.

---

## 7. SENSOR INPUTS

### ENC1 — quadrature encoder → PA0 (A), PA1 (B) `[V pins]`
- **Electrical type/voltage `[TBD]`** — the firmware does not define it. CRITICAL for level shifting:
  - Encoder ≤ 3.3 V push-pull → direct to PA0/PA1.
  - 5 V **open-collector** → pull-up to **3.3 V** (reads valid 3.3 V high). OK without a shifter.
  - 5 V **push-pull** → requires PA0/PA1 to be 5 V-tolerant — **verify FT in the F407 datasheet**;
    if not FT, add a level shifter / divider.
  - 12 V or 24 V industrial / line-driver → **MANDATORY** opto-isolator or level shifter to 3.3 V.
- Add RC/Schmitt input filtering for noise; the firmware already enables max input filter
  (`IC1F/IC2F = 15`) in TIM2, but board-level filtering is still recommended `[TBD]`.
- Index/Z channel: not used by firmware (TIM2 CH1/CH2 only).

---

## 8. ACTUATOR OUTPUTS

All three are inductive / higher-power loads — the 3.3 V STM32 GPIO CANNOT drive them
directly. Each needs a driver stage with flyback protection `[TBD device choice]`.

| Output | MCU pin | Drive stage `[TBD]` | Load | Behaviour `[V]` |
|--------|---------|---------------------|------|-----------------|
| K1 feeder valve | PB8 (active HIGH) | opto-isolated relay module **or** logic-level N-MOSFET/transistor + **flyback diode** across coil | feeder cylinder solenoid | autonomous: ON every 3 s for 100 ms |
| K2 vacuum valve | PB9 (active HIGH) | same | vacuum generator/valve | Pi-commanded; idempotent 0xCC sent ×3 |
| M1 conveyor motor | PE11, PE12 | **L298N** H-bridge `[V named in firmware]` (ENA jumpered 100 %) | DC gear motor | continuous, one direction |
| LED1 heartbeat | PA6 | series resistor (~330 Ω–1 kΩ) | indicator LED | toggles on each valid Pi frame |

Driver notes `[TBD/best-practice]`:
- Relay modules: if 3.3 V GPIO drives a **5 V-logic** opto module, verify the trigger threshold;
  use a 3.3 V-rated module or a logic-level MOSFET if it won't trigger at 3.3 V.
- **Flyback diode** (e.g. 1N4007) across every relay/solenoid coil; snubber on contacts switching
  inductive AC.
- L298N: IN1/IN2 are TTL (≈2.3 V high threshold) so 3.3 V drive is generally fine `[verify]`;
  L298N logic VSS = 5 V, VS = motor rail; include the H-bridge flyback diodes (or use internal/
  module-provided). Keep motor rail star-grounded separately back to the common point.

---

## 9. LEVEL-SHIFTING SUMMARY

| Interface | Shifter? | Reason |
|-----------|----------|--------|
| Pi 5 ↔ STM32 UART | **NO** `[V]` | both 3.3 V — direct, cross TX/RX, common GND |
| STM32 ↔ relays K1/K2 | driver stage (not "shifter") | 3.3 V GPIO → opto/MOSFET → coil rail; flyback required |
| STM32 ↔ L298N | **NO** (verify TTL threshold) | 3.3 V → L298N TTL inputs |
| STM32 ↔ encoder | **CONDITIONAL** `[TBD]` | depends on encoder voltage (see §7) |
| Pi ↔ SCARA | NO | Ethernet |
| Pi ↔ camera | NO | USB |

---

## 10. CRITICAL DESIGN NOTES

1. **Common ground** between Pi 5, STM32, and all driver grounds — star point. UART fails otherwise.
2. **3.3 V everywhere on logic** — Pi 5 is NOT 5 V tolerant; never feed 5 V into a Pi GPIO.
3. **Flyback diodes** on every inductive load (relay coils, solenoids); decoupling 100 nF at each
   IC supply pin + bulk caps on the relay/motor rails (switching transients corrupt UART — this is
   why the firmware/Pi already harden against dropped frames).
4. **Firmware is FROZEN** — the STM32 pin assignments above are fixed in `config.h`; a PCB must
   match these exact pins (PA0/PA1, PA2/PA3, PA6, PB8/PB9, PE11/PE12). Changing a pin = editing
   `config.h` and re-flashing.
5. **Pi 5 UART** — confirm `/dev/ttyAMA0` ↔ GPIO14/15 in the Pi 5 device tree before layout.
6. Items marked `[TBD]` (rail voltages, encoder electrical, relay/motor part numbers, connectors)
   must be filled in by the integrator from the actual BOM — they are NOT defined by the firmware
   and must not be guessed.
