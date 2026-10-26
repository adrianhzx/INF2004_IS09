IS09 - Buddy 4 (Subsystem 4: IMU, hump and collision) - ImuTask on GY-511 (LSM303DLHC)
=====================================================================================
Tested 26 Oct 2026 on the Pico: S4-U01..U03, U04a, U05, U06 PASS; ImuTask ~100 cycles/s,
longest cycle 438 us, I2C scan finds 0x19 + 0x1e. Host tests: test_imu 56/56, test_shared 418/418.

WIRING (Grove 3): GY-511 SDA -> GP4, SCL -> GP5, VIN -> 3V3 or 5V, GND -> GND.
Level = chip side such that the loop shows z ~ +1000 mg (else set APP_CAL_IMU_UP_SIGN -1
in app_cal.h). Lift the nose: pitch must go +, else APP_CAL_IMU_FWD_SIGN -1 (HV-12).

WHAT IS IN THIS ZIP (paths are from the project root; extract straight into the project)
  app_program/subsystem4/*        Buddy 4 code (drivers, imu_core, task_imu, tests)
  testing/host/test_imu.c         PC unit tests (56)
  testing/subsystem4/TEST_LOG.md  test log
  docs/evidence/phase4_*          build/test evidence from Phase 4
  app_program/shared/app_cal.h    shared settings  } needed only if your branch starts from
  app_program/shared/app_hw.c     shared hardware  } GitHub feat/revb-autonomous-integration
  app_program/shared/app_hw.h                      } (c3c7dcf). Identical to the ones already
                                                     on feat/obstacle-detect-avoid.

STEPS (Ubuntu / WSL terminal)
  git clone https://github.com/adrianhzx/INF2004_IS09.git && cd INF2004_IS09
  git switch feat/obstacle-detect-avoid          # (recommended base) or feat/revb-autonomous-integration
  git switch -c feat/imu-hump-collision
  unzip -o /path/to/IS09_buddy4_for_push.zip -d .
  find . -name '*Zone.Identifier' -delete
  rm -f README_BUDDY4.txt
  cd build_make && make clean && make SMP=0 NET=lwip MQTT=1 TEST=1 -j8
  (always "make clean" after copying files in, so every file is rebuilt)

PC TEST
  cd testing/host
  cc -std=c99 -DAPP_HOST_TEST=1 -D_DEFAULT_SOURCE -I. -I../../app_program/shared \
     -I../../app_program/subsystem4 -o test_imu test_imu.c host_port.c \
     ../../app_program/shared/app_blackboard.c ../../app_program/subsystem4/imu_core.c -lm \
     && ./test_imu && rm test_imu

LIVE LOOP (S4-U07)
  In app_program/subsystem4/subsystem4_test.c set  #define S4_T_LIVE_LOOP 1  to print live
  x/y/z, |a|, pitch, motion class, hump state forever after S4-U06 (plus an I2C wiring check).
  It is 0 in this zip so the normal test run finishes - keep it 0 when committing.

KNOWN
  - S1-U06 (Buddy 1 test) FAILS once ImuTask runs: ImuTask overwrites the IMU blackboard the
    test writes a pattern into. Test issue, not a bug - tell Xavier.
  - q=INVALID / STALE_MOTION fault until Buddy 2 (motion) is merged: hump height needs wheel
    distance.

COMMIT
  git add -A
  git commit -m "Subsystem 4 (Buddy 4): IMU, hump detector and collision (GY-511)"
  git push -u origin feat/imu-hump-collision
