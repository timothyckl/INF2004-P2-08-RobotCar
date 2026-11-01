# Subsystem 2 — test log

**Owner:** Dalton (2504003)  ·  **Planned subsystem:** Vehicle Controller and motion control  ·  **Code:** `app_program/subsystem2/`

The cases below test the current producer starter example, not the planned subsystem.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | PID proportional term is kp * error | PID speed control | auto | `subsystem2_test.c`, synthetic gains | `[TEST] S2-U03 PASS` |
| S2-U04 | PID output stays within its limits | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | PID integral adds ki * error per sample | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | PID integral does not wind up while saturated | PID speed control | auto | `subsystem2_test.c`, 50 saturated samples | `[TEST] S2-U06 PASS` |
| S2-U07 | PID derivative ignores setpoint steps | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | PID rejects invalid tuning and inputs | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | PID reset clears history | PID speed control | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | counts convert to millimetres | Odometry | auto | `subsystem2_test.c`, synthetic geometry | `[TEST] S2-U10 PASS` |
| S2-U11 | count delta survives counter wraparound | Odometry | auto | `subsystem2_test.c` | `[TEST] S2-U11 PASS` |
| S2-U12 | speed from counts over an interval | Odometry | auto | `subsystem2_test.c` | `[TEST] S2-U12 PASS` |
| S2-U13 | turn angle sign and magnitude | Odometry, turns | auto | `subsystem2_test.c` | `[TEST] S2-U13 PASS` |
| S2-U14 | odometry rejects invalid geometry and overflow | Odometry | auto | `subsystem2_test.c` | `[TEST] S2-U14 PASS` |
| S2-U15 | motor duty maps to A/B compare values: +d gives A=d, B=0; -d gives A=0, B=d; 0 gives brake (0, 0) | Robo Pico truth table | auto | `subsystem2_test.c`, `drv_motor_map()` | `[TEST] S2-U15 PASS` |
| S2-U16 | motor duty beyond +/-DRV_MOTOR_DUTY_MAX, and a NULL output, are rejected with the output unchanged | drv_motor | auto | `subsystem2_test.c` | `[TEST] S2-U16 PASS` |
| S2-U17 | GPIO IRQ registration rejects pin >= 30, radio pins 23-25 and 29, no edge, level events and a NULL callback | GPIO dispatcher | auto | `subsystem2_test.c`; invalid calls only, no pin is enabled | `[TEST] S2-U17 PASS` |
| S2-U18 | quadrature step: A rise with B low = +1, A rise with B high = -1, A fall with B high = +1, A fall with B low = -1; both edges or no edge = 0 | drv_encoder (x2 decoding) | auto | `subsystem2_test.c`, `drv_encoder_step()` | `[TEST] S2-U18 PASS` |
| S2-U19 | encoder snapshot rejects NULL and returns E_OK with counts that do not change while the encoders are idle | drv_encoder | auto | `subsystem2_test.c`; encoders not initialised | `[TEST] S2-U19 PASS` |
| S2-H01 | each motor drives the expected way | Robo Pico truth table, design review section 2 | manual | Wheels off the ground. Build with `S2_HW_TEST` set to 2 in `subsystem2_test.c`. Per motor: 30% forward 2 s, brake 3 s, 30% backward 2 s, coast 3 s. Watch the wheel and the board's MxA/MxB LEDs | Forward: wheel turns forward, MxA LED on. Backward: reverse, MxB LED on |
| S2-H02 | brake and coast differ as the truth table says | Robo Pico truth table | manual | During S2-H01, turn each wheel by hand in the brake and coast steps | Brake: noticeably stiff. Coast: turns freely |
| S2-H03 | encoder count sign and counts per revolution | drv_encoder, calibration | manual | GP28 battery-sense jumper open. `S2_HW_TEST` set to 1 (motors stay off) prints both counts every 100 ms for 60 s. Mark each wheel, turn it exactly one revolution forward by hand, then one back. Repeat 3 times per wheel | Count changes by the same amount each way, opposite signs, error count unchanged. Record counts per rev and which encoders count down going forward |
| S2-H04 | encoder edges are not missed at full speed | drv_encoder, GPIO dispatcher, PostTri1 interrupt load | manual | Wheels off the ground, `S2_HW_TEST` set to 2: after S2-H01, both motors 100% forward for 10 s | Error counts stay 0. Record counts/s per wheel: that is the interrupt rate per wheel (one interrupt per A edge) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
