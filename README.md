# INF2004 P2-8 — Autonomous Robot Car

Team P2-8's autonomous robot car runs on the Raspberry Pi Pico W with a
Cytron Robo Pico, in C with the Pico SDK and µT-Kernel 3.0. It follows a
line, decodes Code 39 barcodes and turns at the next junction, measures the
highest hump with an accelerometer, scans and bypasses obstacles with a
servo-mounted ultrasonic sensor, rejoins the original line, and reports
telemetry over WiFi/MQTT to a laptop dashboard. All decisions are made on
the car; WiFi is only for monitoring and START/STOP.

All five subsystems are integrated into one mission image, built for **one
core** (`SMP=0`, S2 decision 33). The image used in the assessed demo on
2026-11-24 is tag **`demo-2026-11-24-flashed`** (`a41c19c`); a clean build
of that tag reproduces the demo UF2 byte for byte (SHA-256 `8cf2c419…`).
Until 2026-11-25 later commits changed only tests, tools, comments and
documentation. From 2026-11-25 `main` also carries **post-demo firmware
changes** (S2 decision 35), each listed with its tests in section 9 of the
[software review](docs/software_quality_review.md). They were verified on
the computer and by target builds; none had run on the car when they were
committed, so the results below describe the demonstrated firmware unless
a row says otherwise. The demo evidence is unchanged.

## Results at a glance

| Mission requirement | Result | Evidence |
|---|---|---|
| Line following | Pass: 3 laps (SYS-01). The demo loop sampled every 20 ms, not the required 10 ms (F-01); post-demo: 10 ms cyclic period, car test pending | [system test log](testing/system/TEST_LOG.md) |
| Barcode decoding | Partial: below 4 of 5 passes; no wrong letter seen on the car. Post-demo: the noise-blip wrong letter found on the host (F-18) is fixed; one double fault remains (F-25); car test pending | [demo report](docs/DEMO_REPORT_2026-11-24.md) D-01 |
| Barcode commands at junctions | Partial: left, right and U-turn shown before or at the demo; no turn in the assessed runs | D-02 |
| Hump detection and highest peak | Detected in each assessed attempt; 6, 6, 7 mm for a 5 mm hump (within our ±2 mm engineering criterion). Post-demo: the detection threshold is back at the intended 6 mm (S4 d17); calibration for small humps needs the car | D-03 |
| Obstacle detection | Pass (SYS-05) | system test log |
| Obstacle profiling | Nearest point and side pass; width accuracy not shown (SYS-06 not met) | [software review](docs/software_quality_review.md) F-05 |
| Bypass and line reacquisition | Pass 3/3 (SYS-07); attempt 3 of the demo | D-02 |
| Telemetry over WiFi | Pass: 98.6% of frames received on demo day | [traceability](docs/requirements_traceability.md) FR10 |

The full course was not completed in one run. Every result, including
failures, is kept in the test logs.

## Where to find things

| Topic | Document |
|---|---|
| Assessed demo, its logs and the backup video | [docs/DEMO_REPORT_2026-11-24.md](docs/DEMO_REPORT_2026-11-24.md), [videos/README.md](videos/README.md) |
| Requirements → design → code → tests → results | [docs/requirements_traceability.md](docs/requirements_traceability.md) |
| Architecture, RTOS tasks, state machines, sensors, timers, efficiency, edge computing | [docs/final_architecture.md](docs/final_architecture.md) |
| Code and logic review, defects and proposals | [docs/software_quality_review.md](docs/software_quality_review.md) |
| Proposed fixes after the demo (not in the demo firmware) | [docs/post_demo_proposals.md](docs/post_demo_proposals.md) |
| Tribunal 1 and 2 closure, fault handling | [docs/tribunal_closure.md](docs/tribunal_closure.md) |
| BARR-C:2018 compliance and static analysis | [docs/barr_c_compliance.md](docs/barr_c_compliance.md) |
| Data bit widths and memory | [docs/data_widths.md](docs/data_widths.md) |
| Every agreed design decision | [docs/decisions.md](docs/decisions.md) |
| Owners, pins, task priorities, interfaces | [TEAM.md](TEAM.md), [AGENTS.md](AGENTS.md) |
| Test definitions and every run | `testing/*/TEST_LOG.md`, [testing/README.md](testing/README.md) |
| Original design and tribunals | [docs/AAI08_Design_Review.pdf](docs/AAI08_Design_Review.pdf), [docs/AAI08_PostTri1.pdf](docs/AAI08_PostTri1.pdf), [docs/AAI08_ImmediateTri2.pdf](docs/AAI08_ImmediateTri2.pdf) |
| Brief | [docs/Project Description.pdf](docs/Project%20Description.pdf) |

## Module topics in this project

| Lecture | Where it is applied |
|---|---|
| W1 Embedded systems fundamentals | A Cortex-M0+ at 133 MHz with 264 KB SRAM and no FPU, so all maths is integer or fixed point; hard timing (10 ms motion loop, STOP to motors off in under 1 ms, S2-H11); safety by design (latched STOP, stop before scan); measured flash, RAM and a static stack estimate ([architecture](docs/final_architecture.md) §8) |
| W2.1 GPIO, W2.2 Interrupts | Encoder and echo ISRs with sequence counters and a ring buffer; one shared GPIO dispatcher ([architecture](docs/final_architecture.md) §7, [BARR-C](docs/barr_c_compliance.md) §3) |
| W3.1 Digital communication | I2C to the LSM303DLHC; MQTT binary telemetry ([data widths](docs/data_widths.md)) |
| W3.2 Timers | 10 ms cyclic Motion Control, 1 µs timestamps, 20 kHz motor and 50 Hz servo PWM; a 10 ms cyclic S3 loop since the post-demo fix of the 20 ms barcode sampling (review F-01) ([architecture](docs/final_architecture.md) §7) |
| W3.3 ADC | 12-bit line and barcode sensing on ADC0–2 with adaptive white baselines and hysteresis (`s3_ir.c`, `s3_barcode.c`; [architecture](docs/final_architecture.md) §6, §7) |
| W4.1 Embedded OS | µT-Kernel tasks and priorities, mutexes with priority inheritance, event flags for urgent stops, message buffers for commands and reports ([architecture](docs/final_architecture.md) §2, §3) |
| W4.2 State machines | Vehicle Controller and mode statecharts ([architecture](docs/final_architecture.md) §4), with the transitions tested on the host (S2-U41 to U52, U78) |
| Lecture 5 Requirements and design | Requirement review (shall, tolerance, verifiable, no design in the requirement) and requirement-to-box/arrow IDs ([traceability](docs/requirements_traceability.md) §10); sequence diagrams and statecharts ([architecture](docs/final_architecture.md) §4, §5) |
| Lecture 6 Traceability and testing | [traceability](docs/requirements_traceability.md); unit, integration, system, boundary, negative and fault-injection tests; coverage |
| W9 Sensor fusion | Encoder speed gates IMU pitch; encoder travel times the junction step; line sensors confirm the end of a bypass (no formal filter) |
| W10 Optimisations | Minimum bit widths and a packed 32-byte frame (version 5; 31 bytes in the demo); no floating point; unused code removed at link time; measured flash and RAM and a static stack estimate ([data widths](docs/data_widths.md), [architecture](docs/final_architecture.md) §8) |
| W11 Edge computing | All sensing, decisions and safety on the Pico; only a 32-byte summary leaves the car; it keeps driving with the link lost (SYS-01) ([architecture](docs/final_architecture.md) §9) |

## Tests

```sh
make -C testing/host                       # five host suites and the controller integration suite (INT-08 on)
python3 -m unittest discover -s testing/subsystem1/dashboard/tests
sh testing/host/coverage.sh                # line and branch coverage
python3 testing/host/target_static_check.py  # strict warnings and -fanalyzer
python3 tools/barr_c_report.py             # BARR-C rules a script can check
```

Host tests check logic with synthetic input; they are not evidence that
the robot works on the track (see [testing/README.md](testing/README.md)).

---

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and WiFi driver source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver
```

The `submodule` line is required: it downloads the USB and WiFi drivers.
(SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone your project**

```sh
cd ~/pico
git clone https://github.com/timothyckl/INF2004-P2-08-RobotCar.git
cd INF2004-P2-08-RobotCar
```

Place the Pico SDK in this directory as `pico-sdk/` and point `PICO_SDK_PATH`
to it. The directory is ignored by Git.

**2.2 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp0_usb_cdc_wifi.elf mtk3pico_smp0_usb_cdc_wifi.uf2`.
Your firmware is **`build_make/mtk3pico_smp0_usb_cdc_wifi.uf2`**.

This image runs the robot's mission (section 3, *The mission image*).
For telemetry over WiFi, build the robot's firmware with
`make NET=lwip MQTT=1 -j8` instead, which gives
`build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

To build a single-core, UART-only or test version, see section 3.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.4 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
You should see the task reports.

For the Subsystem 3 hardware profiles, the repository logger displays every
line and saves it under `testing/subsystem3/logs/` (close any other serial
monitor first):

```powershell
powershell -ExecutionPolicy Bypass -File testing/subsystem3/capture_serial.ps1 -Port COM4
```

Start the logger before resetting the Pico to capture the complete boot output.

**2.5 Write your own code**

Your code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It first calls
  every subsystem's `subsystemN_init()` (create kernel objects), then every
  `subsystemN_start()` (start tasks). Read its comments first.
- **One folder per student:** `app_program/subsystem1/` … `subsystem5/`.
  Each holds that subsystem's implementation. The course's starter
  examples were removed on 2026-11-20 (S2 decision 29).
- `subsystemN.h` is that subsystem's public interface: the object IDs and
  data formats the other subsystems may use.
- `app.h` holds the definitions shared by the whole team.
- **Drivers go in your subsystem folder too**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `subsystem2/drv_ultrasonic.c`. They are compiled
  automatically. Do not add drivers under `device/`: that folder holds the
  kernel's own serial, I2C and ADC drivers, and adding to it means editing
  kernel and makefile files shared by the whole team.

**Working as a team:** record owners, pins, priorities and the interfaces
between subsystems in **`TEAM.md`**, and record every test in **`testing/`**
(see `testing/README.md`). `make TEST=1` builds a test image that runs all
the subsystem tests at power-up. Short videos of tests and demos are shared
as unlisted YouTube links, listed in **`videos/`**.

Pins: the robot's wiring is in `AGENTS.md`, section *Agreed pin mapping*,
which is the authoritative GPIO reference. GP0/GP1 carry the right wheel
encoder, so build with `CRASH_REPORT=0` and `CONSOLE=usb_cdc` (section 3).
Never use GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico
W's on-board LED is also on the radio, so use an external LED instead.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=0 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=1 -j8                    # dual-core build (not used: S2 decision 33)
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (see note below)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc` and
`WIFI=cyw43`). Options are not remembered: give them on every `make`, as
the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | **`0`**, `1` | `0` uses core 0 only (default, S2 decision 33); `1` runs the kernel on both RP2040 cores | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `S3_HW_TEST` | **`0`**, `1` | `1` runs only subsystem 3's continuous raw two-front-IR monitor; requires `TEST=1` | `_s3hw` / *(nothing)* | `S3_HARDWARE_TEST` (1 or 0) |
| `S3_LINE_TEST` | **`0`**, `1` | `1` runs the mission in a `TEST=1` image instead of the self-tests (same mission as `TEST=0`); requires `TEST=1` | `_s3line` / *(nothing)* | `APP_MISSION` (1 with `TEST=0` or `S3_LINE_TEST=1`) |
| `S2_CAL_TEST` | **`0`**..`7` | starts only subsystem 2; each GP20 press runs one manoeuvre: forward 1000 mm (`1`), left 360 degrees (`2`), forward 450 mm (`3`), forward 500 mm (`4`), left 90 (`5`), right 90 (`6`) or left 180 degrees (`7`), then prints odometry, STOP latency and a timer check (S2-H06..H08, S2-H10, S2-H11); requires `TEST=1` | `_s2cal1`, `_s2cal2` / *(nothing)* | `S2_CAL_TEST` (0..2) |
| `S2_HW_TEST` | **`0`**..`4` | subsystem 2 bench sequence instead of the motion and controller tasks: `1` encoder counts, motors off (S2-H03); `2` motors both ways (S2-H01, H02, H04); `3` full speed (S2-H04); `4` watchdog (S2-H05). `2`..`4` SPIN THE MOTORS; requires `TEST=1` | `_s2hw1`..`_s2hw4` / *(nothing)* | `S2_HW_TEST` (0..4) |
| `S2_MOTION_LOG` | **`0`**, `1` | `1` prints Motion Control's speed loop (target, measured speed, duty, counts per wheel) every 50 ms while moving, for PID tuning | `_mclog` / *(nothing)* | `S2_MOTION_LOG` (1 or 0) |
| `S5_RANGE_DIAG` | **`0`**, `1` | `1` prints every forward HC-SR04 attempt (echo duration, validity, distance) for S5-U48 | `_s5range` / *(nothing)* | `S5_RANGE_DIAGNOSTIC` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. On this robot GP0/GP1 carry the right wheel encoder
(`TEAM.md` section 2), so the makefile refuses `CRASH_REPORT=1` and
`CONSOLE=uart` (`docs/decisions.md`, subsystem 2, decision 6). `CONSOLE=uart WIFI=none` also
no longer compiles without the Pico SDK: subsystem 2's GPIO interrupt driver
includes an SDK header.

`S3_HW_TEST=1` (only with `TEST=1`) runs the dedicated subsystem 3 raw
front-IR monitor without starting the other application subsystems. It adds
the `_s3hw` suffix; see `app_program/subsystem3/README.md`.
`S3_LINE_TEST=1` builds the mission (below) into a `TEST=1` image and adds
`_s3line`, so the commands in earlier test-log rows still build. Do not
combine it with `S3_HW_TEST=1`.
`S4_HW_TEST=1` similarly adds subsystem 4's live IMU start-up check and the
`_s4hw` image suffix; see [subsystem 4 documentation](app_program/subsystem4/README.md).

**The mission image.** A `TEST=0` image (and `TEST=1 S3_LINE_TEST=1`)
runs the robot's mission: the Vehicle Controller and Motion Control
(subsystem 2), the line and barcode loop (subsystem 3), the IMU terrain
monitor (subsystem 4) and Obstacle Scan (subsystem 5); subsystem 1 runs in
the WiFi task when built with `NET=lwip MQTT=1`. Press GP20 (or send
`START`) in Ready to start a run and GP20 again to stop it. There is no
run-time limit, and motor PWM is capped at 20% (`motion_config.h`). The
RP2040 hardware watchdog is not armed (`S2_WATCHDOG`, subsystem 2
decision 28). `S3_NAV_F` maps the test barcode letter F to a command.

The options are part of the image name, e.g.
`mtk3pico_smp0_usb_cdc_wifi_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;   /* core affinity only exists on the dual-core build */
#endif

#if APP_TEST
    /* code that only exists in the test build */
#endif
```

### Where the options live in the makefile

All five are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI` only) | an `ifeq ($(CONSOLE),usb_cdc)` or `ifeq ($(WIFI),cyw43)` block that adds the USB/radio files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).

The makefile also has `NET=lwip` and `MQTT=1` (TCP/IP and MQTT over WiFi).
They need the SDK's `lib/lwip` submodule and a `config/wifi_secrets.h`
(copy `config/wifi_secrets.example.h`). This guide does not cover them.

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
