# robot_scara — SCARA side of the verified TCP coordinate handshake

`PICKTEST.scol` runs on the **Shibaura/Toshiba THL400 / TSL3000** controller. It is the
robot half of the **3-way ACK-verified handshake** whose Pi half is
`vision_pi5/comms/robot_link.py`. The robot no longer *assumes* a one-way coordinate write
succeeded — it **echoes a checksum** of what it parsed, and only moves after the Pi sends an
explicit **GO** gate.

> ### ⚠️ Hardware rule (root cause of error `2-046`)
> The controller's `INPUT IPn, ...` in **Non-protocol mode** accepts **numeric data only,
> comma-separated, terminated by a single CR (`\r`, 0x0D)**. Any non-numeric character or
> ASCII **control byte (`STX` 0x02 / `ETX` 0x03)** sent into `INPUT` crashes SCOL with
> **`2-046 Invalid Channel`**. The Pi→robot payload is therefore bare numbers only — no
> framing bytes. (An earlier `STX…ETX` framed design caused exactly this fault; the fix was
> to change the Pi transmission format.)

```
  Pi (RobotLink)                         Robot (PICKTEST.scol)
  --------------                         ---------------------
                         <---- REQ ----  PRINT IP1,"REQ"            (asks for a target)
   id,cmd,x,y,z,c,shp\r  -------->        INPUT IP1, ID,CMD,X,Y,Z,C,SHP
                                          ACC/CKSUM  (recompute checksum)
                         <- ACK id ck -   PRINT IP1,"ACK",id,cksum  (within 50 ms)
   verify ck == expected
     match  --> 1 (GO) -------------->    INPUT GATE ; IF GATE == 1 -> move
     else   --> 0 (ABORT) ----------->    INPUT GATE ; GATE <> 1    -> GOTO START (no motion)
                       <- DONE/ARRIVED -   PRINT after the move completes
```

If the Pi does not see a **valid** `ACK` within **50 ms** it logs `Robot Comms Timeout`,
sends the abort gate (`0`), flushes its buffers (`buffreset`), and either **retries** (up to
`ACK_RETRIES`, default 2) or **safe-faults** (skips that object — the robot is never
commanded to move on unverified data).

---

## Wire protocol (exact tokens)

All tokens are **ASCII**, terminated by **CR (`\r`, 0x0D)** (the SCARA emits CR via the `CR`
constant; the Pi splits incoming lines on CR **or** LF). Everything the robot **reads** is
numeric; everything it **prints** back (`REQ`, `ACK …`, `ARRIVED`/`DONE`) is text and the Pi
parses it leniently.

### 1. Robot → Pi : request
```
REQ\r
```

### 2. Pi → Robot : coordinate record (one comma-separated numeric line)
```
ID,CMD,X,Y,Z,C,SHP\r
```
| field | type | meaning |
|-------|------|---------|
| ID    | int  | command id, echoed back in the ACK |
| CMD   | int  | 1 = move-to-boundary (WAIT), 2 = pick |
| X     | real (3 dp) | mm |
| Y     | real (3 dp) | mm |
| Z     | real (3 dp) | mm — pick depth for cmd 2; lift height for cmd 1 |
| C     | real (3 dp) | deg — tool roll |
| SHP   | int  | 1 circle, 2 square, 3 hexagon, 0 default |

Example: `1,2,0.000,-250.000,28.000,89.167,2\r`. Read in one statement:
`INPUT IP1, ID, CMD, X, Y, Z, C, SHP`. **No `STX`/`ETX`, no per-field CR.**

### 3. Robot → Pi : acknowledgement (must arrive < 50 ms after the record)
```
ACK {id} {cksum}\r
```
Space- or comma-separated; extra/leading whitespace is fine (the Pi `split()`s). The Pi
parses id and checksum with `int(float(...))`, so `ACK 1 3872` and `ACK 1.000 3872.000` are
both accepted.

### 4. Pi → Robot : numeric gate
```
1\r   GATE_GO    — checksum matched -> robot executes the move
0\r   GATE_ABORT — timeout/mismatch -> robot returns to START, no motion
```
`INPUT IP1, GATE` then `IF GATE == 1 THEN GOTO EXECUTE`; any other value re-loops to `START`.

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
making truncation and the 16-bit fold identical in Python and SCOL (no signed/negative
dialect ambiguity).

```
ACC   = ID + CMD + SHP
      + INT(X + 1000) + INT(Y + 1000) + INT(Z + 1000) + INT(C + 1000)
CKSUM = ACC - INT(ACC / 65536) * 65536        # == ACC AND 0xFFFF for ACC >= 0
```

`INT()` truncates toward zero; because every term is positive, that equals `floor`, matching
Python's `int()`. The byte-for-byte reference is `frame_checksum()` in
`vision_pi5/comms/robot_link.py`; the Pi validates the robot's returned `CKSUM` against its
own computed value and only sends `GO` on a match.

### Worked examples (verified against the Pi unit tests)

| field | pick frame | boundary frame |
|-------|-----------:|---------------:|
| ID    | 1          | 5              |
| CMD   | 2          | 1              |
| X     | 0.000      | -207.000       |
| Y     | -250.000   | -250.000       |
| Z     | 28.000     | 146.439        |
| C     | 89.167     | 0.000          |
| SHP   | 2          | 0              |

Pick: `1+2+2 + INT(1000.000)+INT(750.000)+INT(1028.000)+INT(1089.167)`
`= 5 + 1000+750+1028+1089 = 3872` → **`ACK 1 3872`**

Boundary: `5+1+0 + 793+750+1146+1000 = 3695` → **`ACK 5 3695`**

---

## Fault recovery — REQ handling & BUFFRESET (Pi-internal)

* **REQ**: the Pi blocks in `wait_for_signal("REQ")` until the robot prints `REQ`, then sends
  the coordinate record. `RobotLink.initialize()` runs a one-time flush before the first REQ
  so boot-time garbage can't desync the first exchange.
* **BUFFRESET**: on any fault (ACK timeout, checksum mismatch) the Pi sends the abort gate
  and calls `RobotLink.buffreset()` — it clears the carried-over RX buffer and drains stale
  socket bytes so the next REQ/ACK cycle starts clean. The robot's own fault path is
  `GOTO START` (re-emit `REQ`).
* **Never** transmit a literal `"BUFFRESET"` (or any text) toward the robot's `INPUT` — a
  non-numeric token would itself trip `2-046`. BUFFRESET is a Pi-side routine only.

---

## Motion semantics

* **cmd 1 — WAIT_BOUNDARY**: park at `POINT(X+10, Y, Z, C, 0.0, LEFTY)` (the Pi sends
  `Z = 146.439`, the lift height), then `ARRIVED`. Pre-positions the arm at `ROBOT_X_MIN`
  while a far object travels into the pick window.
* **cmd 2 — DO_PICK**: approach `POINT(X+10, Y, ZSAFE, C, 0.0, LEFTY)` → descend to the
  **received** pick depth `POINT(X, Y, Z, C, 0.0, LEFTY)` → lift → place by shape
  (1→T2/T1/T2, 2→T4/T3/T4, 3/0→T6/T5/T6) → `DONE`.
  `ZSAFE = 146.439` is a program constant (travel height); only the **pick** Z comes from the
  verified record. `LEFTY` (config constant 1) is pinned as the 6th `POINT()` argument so the
  controller never flips arm solution between approach and pick. `MOVE` reads the config from
  the point, so no separate `CONFIG = LEFTY` statement is needed.

The **vacuum** is not driven here (SCOL has no relay command in this cell); the Pi energizes
Relay 1 by dead-reckoning timer (`vision_pi5/pipeline/sender_worker.py`), so the SCARA
program is motion-only.

---

## Timing — the 50 ms ACK budget

* The Pi sets `TCP_NODELAY` (no Nagle batching) so the record is flushed immediately.
* The robot must `INPUT` the record, compute the checksum, and `PRINT` the ACK **before** any
  motion. Checksum math is microseconds; the budget is for the controller's TCP send latency.
  On a wired LAN this is comfortably < 50 ms — **verify on your bench** and widen
  `ACK_TIMEOUT_S` in `vision_pi5/config.py` if your stack is slower.
* Do **not** move before the ACK — motion takes seconds and would blow the window.

---

## Integration checklist

1. Load `PICKTEST.scol`; confirm `IP1` is the TCP **server** channel the Pi connects to
   (`ROBOT_IP = 192.168.0.124`, `ROBOT_PORT = 1001` in `vision_pi5/config.py`).
2. Confirm teach points `T1`…`T6` and the `LEFTY` configuration exist.
3. **Connection order matters.** The Pi is the TCP *client*; `IP1` is only valid while the Pi
   is connected. If the Pi isn't up, `PRINT IP1` raises **`2-046 Invalid Channel`**. So start
   the Pi first, confirm `[NET] OK` and that it's waiting, then run `PICKTEST`. Launch the Pi
   as a module from the repo root:
   ```
   python3 -m vision_pi5.main            # add --no-display for a headless Pi
   ```
   (Answer `n` to the calibration prompts to load the saved `models/homography_test.npz` and
   avoid the GUI.)
4. First contact must be the robot's `REQ`. No `[SENDER]` activity on the Pi ⇒ the robot
   isn't emitting `REQ` / the program isn't running / the link dropped.
5. Watch the Pi log for `[RX] 'ACK <id> <cksum>'`, then the gate, then `DONE`. A
   `Robot Comms Timeout` or `ACK checksum sai` line means the handshake tripped — compare the
   printed checksum against the worked examples to localize it.

The Pi side is covered by `tests/test_robot_handshake.py` (happy path, ACK-timeout→retry,
bad-checksum→safe-fault-never-moves, boundary `ARRIVED`) and the real-TCP
`tools/sim_task1.py`, both exercising this exact comma-separated numeric protocol.
