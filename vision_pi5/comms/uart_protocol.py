"""UART wire protocol with the STM32 (stateless definitions + encoders).

Frame formats (must match embedded_stm32/):
    Telemetry  STM32 -> Pi  (every 100 ms, 11 bytes):
        [0xAA][0xBB][float rpm(4)][int32 ticks(4)][XOR of the 8 payload bytes]
    Relay cmd  Pi -> STM32  (4 bytes):
        [0xCC][r1][r2][r1^r2]      r1=0x01 vacuum ON, 0x00 OFF
    Ping       Pi -> STM32  (4 bytes):
        [0xDD][seq][0x00][seq^0x00]
    ACK        STM32 -> Pi  (4 bytes):
        [0xCD][0xCE][seq][0xCE^seq]
"""

import struct

HEADER1  = 0xAA
HEADER2  = 0xBB
HDR_CMD  = 0xCC
HDR_PING = 0xDD
HDR_ACK1 = 0xCD
HDR_ACK2 = 0xCE

SYNC_BYTES = bytes([HEADER1, HEADER2])
ACK_BYTES  = bytes([HDR_ACK1, HDR_ACK2])


def encode_relay(suction: bool, cylinder_override: bool = False) -> bytes:
    r1 = 0x01 if suction else 0x00
    r2 = 0x01 if cylinder_override else 0x00
    return struct.pack("BBBB", HDR_CMD, r1, r2, r1 ^ r2)


def encode_ping(seq: int) -> bytes:
    seq &= 0xFF
    return struct.pack("BBBB", HDR_PING, seq, 0x00, seq ^ 0x00)


def telemetry_checksum(payload) -> int:
    ck = 0
    for b in payload:
        ck ^= b
    return ck & 0xFF


def ack_is_valid(seq: int, checksum: int) -> bool:
    return ((HDR_ACK2 ^ seq) & 0xFF) == checksum
