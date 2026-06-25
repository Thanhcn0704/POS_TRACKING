# robot_scara — SCARA side of the verified TCP coordinate handshake

`PICKTEST.scol` runs on the **Shibaura/Toshiba THL400 / TSL3000** controller. It is the
robot half of the **3-way ACK-verified handshake** whose Pi half is
`vision_pi5/comms/robot_link.py`. The robot no longer *assumes* a one-way coordinate write
succeeded — it **echoes a checksum** of what it parsed, and only moves after the Pi sends an
explicit `GO`.

```
  Pi (RobotLink)                         Robot (PICKTEST.scol)
  --------------                         ---------------------
                         <---- REQ ----  PRINT IP1,"REQ"            (asks for a target)
   STX,id,cmd,x,y,z,c,shp,ETX  ------->  INPUT  (read + parse frame)
                                         ACC/CKSUM  (recompute checksum)
                         <- ACK id ck -  PRINT IP1,"ACK",id,cksum   (within 50 ms)
   verify ck == expected
     match  --> GO  ----------------->   INPUT GATE$ == "GO"  -> move
     else   --> ABORT ------------->     INPUT GATE$ != "GO"  -> GOTO START (no motion)
                       <- DONE/ARRIVED -  PRINT after the move completes
```

If the Pi does not see a **valid** `ACK` within **50 ms** it logs `Robot Comms Timeout`,
sends `ABORT`, and either **retries** (up to `ACK_RETRIES`, default 2) or **safe-faults**
(skips that object — the robot is never commanded to move on unverified data).

---

## Wire protocol (exact bytes)

All tokens are **ASCII**, each terminated by a single **CR (`\r`, 0x0D)**. The SCARA emits
CR via the `CRE` constant; the Pi splits incoming lines on CR **or** LF.

### 1. Robot → Pi : request
```
REQ\r
```

### 2. Pi → Robot : coordinate frame (9 CR-delimited lines)
```
<STX>\r          byte 0x02   — frame start marker
{id}\r           integer     — command id, echoed back in the ACK
{cmd}\r          integer     — 1 = move-to-boundary (WAIT), 2 = pick
{x}\r            real, 3 dp   (mm)
{y}\r            real, 3 dp   (mm)
{z}\r            real, 3 dp   (mm)  — pick depth for cmd 2; lift height for cmd 1
{c}\r            real, 3 dp   (deg) — tool roll
{shp}\r          integer     — 1 circle, 2 square, 3 triangle, 0 default
<ETX>\r          byte 0x03   — frame end marker
```
`STX`/`ETX` are read into throwaway string vars (`STXC$`, `ETXC$`) purely to keep the
token stream aligned — **the checksum is the real integrity gate**, so they are consumed,
not strictly compared. (See *Dialect assumptions* if your controller cannot read control
bytes.)

### 3. Robot → Pi : acknowledgement (must arrive < 50 ms after the frame)
```
ACK {id} {cksum}\r
```
Space- or comma-separated; extra/leading whitespace is fine (the Pi `split()`s). The Pi
parses the id and checksum with `int(float(...))`, so `ACK 1 3872` and `ACK 1.000 3872.000`
are both accepted.

### 4. Pi → Robot : gate
```
GO\r        the Pi's expected checksum matched the ACK  -> robot executes the move
ABORT\r     timeout or checksum mismatch                -> robot returns to START, no move
```

### 5. Robot → Pi : completion (after the motion)
```
ARRIVED\r   cmd 1 (boundary pre-position) finished
DONE\r      cmd 2 (pick + place) finished
```
`NG` or `OUT` anywhere is treated by the Pi as a robot-reported error.

---

## Checksum

Both sides compute the **same** 16-bit value. Each coordinate is biased by `+1000`
(`CHK_OFFSET`) before truncation so **every term is non-negative** over the work envelope —
that makes truncation and the 16-bit fold identical in Python and SCOL (no signed/negative
dialect ambiguity).

```
ACC   = id + cmd + shp
      + INT(x + 1000) + INT(y + 1000) + INT(z + 1000) + INT(c + 1000)
cksum = ACC - INT(ACC / 65536) * 65536        # == ACC AND 0xFFFF for ACC >= 0
```

`INT()` truncates toward zero; because every term is positive, that equals `floor`, which
matches Python's `int()`. The Python side is the byte-for-byte reference,
`frame_checksum()` in `vision_pi5/comms/robot_link.py`.

### Worked examples (verified against the Pi unit tests)

| field | pick frame | boundary frame |
|-------|-----------:|---------------:|
| id    | 1          | 5              |
| cmd   | 2          | 1              |
| x     | 0.000      | -207.000       |
| y     | -250.000   | -250.000       |
| z     | 28.000     | 146.439        |
| c     | 89.167     | 0.000          |
| shp   | 2          | 0              |

Pick: `1 + 2 + 2 + INT(1000.000) + INT(750.000) + INT(1028.000) + INT(1089.167)`
`= 1+2+2 + 1000+750+1028+1089 = 3872` → **`ACK 1 3872`**

Boundary: `5 + 1 + 0 + 793 + 750 + 1146 + 1000 = 3695` → **`ACK 5 3695`**

---

## Motion semantics (unchanged from the original PICKTEST)

* **cmd 1 — WAIT_BOUNDARY**: park at `POINT(X+10, Y, Z, C)` (the Pi sends `Z = 146.439`,
  the lift height), then `ARRIVED`. Used to pre-position the arm at `ROBOT_X_MIN` while a
  far object travels into the pick window.
* **cmd 2 — DO_PICK**: approach `POINT(X+10, Y, ZLIFT, C)` → descend to the **received**
  pick depth `POINT(X, Y, Z, C)` → lift → place by shape
  (1→T2/T1/T2, 2→T4/T3/T4, 3/0→T6/T5/T6) → `DONE`.
  `ZLIFT = 146.439` stays a program constant (travel height); only the **pick** Z comes
  from the verified frame.

The **vacuum** is **not** driven here. SCOL has no relay command in this cell; the Pi
energizes Relay 1 by dead-reckoning timer (`vision_pi5/pipeline/sender_worker.py`), so the
SCARA program is motion-only.

---

## Dialect assumptions (verify on your controller)

The original PICKTEST only ever read **numerics** and printed **string literals**. The
handshake adds three things that depend on your firmware's BASIC dialect. Each row lists the
assumption, the symptom if it's wrong, and the fix.

| # | Assumption | Symptom if wrong | Fix |
|---|------------|------------------|-----|
| 1 | String vars use a `$` suffix; `INPUT IP1, GATE$` reads a string; `IF GATE$ == "GO"` compares strings | Compile error on `GATE$`/`STXC$`, or the GO test never matches | Use **Variant B (numeric gate)** below — removes all string vars |
| 2 | `INPUT IP1, STXC$` can read a line containing control byte 0x02 / 0x03 | INPUT errors or stalls on the STX/ETX line | Switch to **printable markers**: in `vision_pi5/config.py` set `STX_BYTE = ord('<')` and `ETX_BYTE = ord('>')` — the markers become `<` / `>`, read fine as strings |
| 3 | `PRINT IP1, "ACK ", ID, " ", CKSUM, CRE` concatenates items (any separator is OK) | ACK arrives as `ACK13872` with no spaces | The Pi already collapses whitespace and strips commas; if items are glued with *no* separator, build the string explicitly (see note) |
| 4 | `INT()` truncates; `ACC` stays an integer through the sum | Checksum mismatch | The Pi parses with `int(float(...))`, so `3872.000` is fine; if values still differ, log the raw `ACC` and compare to the table above |

> Separators (row 3): the interleaved `" "` literals in the `PRINT` make the ACK robust
> whether your controller inserts its own separator between print items or none at all.
> Only if items are concatenated with *no* gap do you need explicit string building.

### Variant B — numeric-only gate (no string variables)

If your controller lacks string support (row 1), drop all string handling:

1. **Pi** — in `vision_pi5/comms/robot_link.py`, send a numeric gate and numeric frame
   markers instead of `"GO"/"ABORT"` and the STX/ETX bytes: e.g. `GO -> "1\r"`,
   `ABORT -> "0\r"`, and replace the `<STX>`/`<ETX>` lines with sentinel integers such as
   `911\r` … `913\r`. (Keep the `ACK_TIMEOUT_S` / retry logic untouched.)
2. **SCOL** — replace the three string reads/compares:
   ```
   INPUT IP1, SMARK            ' was: INPUT IP1, STXC$   (SMARK is numeric)
   INPUT IP1, ID, CMD, X, Y, Z, C, SHP
   INPUT IP1, EMARK            ' was: INPUT IP1, ETXC$
   ...
   INPUT IP1, GO               ' was: INPUT IP1, GATE$
   IF GO == 1 THEN GOTO EXECUTE
   GOTO START
   ```
This keeps the exact same handshake timing and checksum — only the marker/gate encoding
changes from text to integers.

---

## Timing — the 50 ms ACK budget

* The Pi sets `TCP_NODELAY` (no Nagle batching) so the frame is flushed immediately.
* The robot must `INPUT` the frame, compute the checksum, and `PRINT` the ACK **before**
  any motion. Checksum math is microseconds; the budget is for the controller's TCP/IP
  send latency. On a wired LAN this is comfortably < 50 ms, but **verify on your bench** and
  widen `ACK_TIMEOUT_S` in `vision_pi5/config.py` if your controller's stack is slower.
* Do **not** move before the ACK — motion takes seconds and would blow the window.

---

## Integration checklist

1. Load `PICKTEST.scol` onto the controller; confirm `IP1` is the TCP channel to the Pi
   (`ROBOT_IP = 192.168.0.124`, `ROBOT_PORT = 1001` in `vision_pi5/config.py`).
2. Confirm teach points `T1`…`T6` and `CONFIG = LEFTY` exist (unchanged from your original
   program).
3. Start `PICKTEST`, then run the Pi: `python -m vision_pi5.main`.
4. First contact must be the robot's `REQ`; the Pi blocks in `wait_for_signal("REQ")` until
   it sees one. (If the Pi prints no `[SENDER]` pick activity, the robot isn't emitting
   `REQ` / the program isn't running.)
5. Watch the Pi log for `[RX] 'ACK <id> <cksum>'` followed by the `GO` and `DONE`. A
   `Robot Comms Timeout` or `ACK checksum sai` line means the handshake tripped — compare
   the printed checksum against the worked examples above to localize the dialect issue.

The Pi side is covered by `tests/test_robot_handshake.py` (happy path, ACK-timeout→retry,
bad-checksum→safe-fault-never-moves, boundary `ARRIVED`), which drives the real `RobotLink`
against an in-process mock of this program over a socket pair.
