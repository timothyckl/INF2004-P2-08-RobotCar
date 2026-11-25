# 24 November 2026 — host regression review

Three runs of the existing host suites (`make -C testing/host`, C99,
`-Wall -Wextra -Werror -Wshadow -pedantic`; S1 and S3 add `-Wconversion`,
S1 also `-Wsign-conversion`). All three are software-only checks after the
demo.
They are not physical tests and do not change any recorded result.

| Run | Source | Compiler | Result | Raw output |
|---|---|---|---|---|
| 1 | Source ZIP of `0843102` (archive SHA-256 `a54c379ea6ccf96c5869e2a9f9d54e0c3e9118012fcf29a838e2e2ac072b86cc`), run by a teammate outside this repository | host `cc` | exit 0, `[HOST] all suites passed` | [2026-11-24_host-review.txt](logs/2026-11-24_host-review.txt) (kept exactly as supplied) |
| 2 | `a41c19c`, the demonstrated source (tag `demo-2026-11-24-flashed`), 23:47 | Apple clang 17.0.0 | exit 0, `[HOST] all suites passed` | [2026-11-24_host-review-a41c19c.txt](logs/2026-11-24_host-review-a41c19c.txt) (absolute paths replaced by `<repo>` and `<tmp>`) |
| 3 | Run on 2026-11-25 00:34: the demonstrated firmware plus the post-demo tests (S2-U79, S3-U02, S4-U41) | Apple clang 17.0.0 | exit 0; S2 51/51, S3 782/782, S4 36/36 (S4-H01 blocked), S5 47/47; dashboard 23/23; clean mission build reproduces the demo UF2 | [2026-11-25_final-verification.txt](logs/2026-11-25_final-verification.txt) |

Later runs on 2026-11-25, after more post-demo tests and the BARR-C clean-up, are in [2026-11-25_final-verification-3.txt](logs/2026-11-25_final-verification-3.txt): S2 52/52, S3 765 checks with one known defect (F-18) reported separately, S4 37/37 (S4-H01 blocked), S5 48/48, dashboard 23/23, mission UF2 unchanged.

`0843102` is the committed code before telemetry version 4, so run 1 did
not test the firmware that ran in the demo; run 2 did.

| Subsystem | Run 1 (`0843102`) | Run 2 (`a41c19c`) | Scope |
|---|---|---|---|
| S1 | S1-U04 to U07 PASS | S1-U04 to U07 PASS (U05 now checks the version 4 frame) | Commands, telemetry, timers, bridge |
| S2 | 50 passed, 0 failed | 50 passed, 0 failed | Motion, controller, line, scan, bypass and navigation logic |
| S3 | 773 checks, 0 failures | 773 checks, 0 failures | IR and barcode algorithms, navigation mapping; checks, not physical trials |
| S4 | 34 passed, 0 failed; S4-H01 BLOCKED | 35 passed, 0 failed; S4-H01 BLOCKED | Terrain logic; S4-U40 (hump count and last height) is new. S4-H01 needs the target and a connected IMU |
| S5 | 47 passed, 0 failed, 0 blocked; telemetry view passed | same | Ranging, scanning, planning, safety, telemetry view |

The dashboard's Python unit tests (`testing/subsystem1/dashboard/tests`,
run with `python3 -m unittest`) also passed: 20 tests.

These results show that the host-tested logic passes its own checks. They
do not show BARR-C compliance, RTOS timing, sensor reliability or mission
acceptance, and they cannot confirm any reading from the demo. For the demo
see the [demo report](../../docs/DEMO_REPORT_2026-11-24.md) and the
[system test log](../system/TEST_LOG.md).
