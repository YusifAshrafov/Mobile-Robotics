# Lab 03 — Ultrasonic Distance Control with P, PD and PID

This laboratory explores distance control using a LEGO EV3 robot equipped with an ultrasonic sensor. The robot attempts to maintain a 200 mm separation from a target box. Proportional gain trials are followed by a comparison of P, PD and PID controllers during stationary-target approach, bidirectional target tracking, and sudden stopping of a moving target.

## Folder contents

```text
Lab-03/
├── README.md
├── codes/
│   ├── KPTests/
│   │   └── p_kp_testA.py
│   ├── PTests/
│   │   ├── p_testA.py
│   │   ├── p_testB.py
│   │   └── p_testC.py
│   ├── PDTests/
│   │   ├── pd_testA.py
│   │   ├── pd_testB.py
│   │   └── pd_testС.py
│   └── PIDTests/
│       ├── pid_testA.py
│       ├── pid_testB.py
│       └── pid_testC.py
└── data/
    ├── kp/       # Five proportional-gain CSV logs
    └── tests/    # Nine P, PD and PID comparison CSV logs
```

## Programs

| File | Description |
| --- | --- |
| [p_kp_testA.py](codes/KPTests/p_kp_testA.py) | Stationary-target approach used to compare proportional gains. The saved version uses `KP = 6.0`; change the gain and log name for each new gain trial. |
| [p_testA.py](codes/PTests/p_testA.py) | P controller, stationary-target approach, 6-second run. |
| [p_testB.py](codes/PTests/p_testB.py) | P controller, bidirectional moving-target experiment, 10-second run. |
| [p_testC.py](codes/PTests/p_testC.py) | P controller, moving-target sudden-stop experiment, 10-second run. |
| [pd_testA.py](codes/PDTests/pd_testA.py) | PD controller, stationary-target approach, 6-second run. |
| [pd_testB.py](codes/PDTests/pd_testB.py) | PD controller, bidirectional moving-target experiment, 10-second run. |
| [pd_testС.py](codes/PDTests/pd_test%D0%A1.py) | PD controller, moving-target sudden-stop experiment, 10-second run. |
| [pid_testA.py](codes/PIDTests/pid_testA.py) | PID controller, stationary-target approach, 6-second run. |
| [pid_testB.py](codes/PIDTests/pid_testB.py) | PID controller for the 10-second bidirectional experiment. The saved source contains the startup issue described below. |
| [pid_testC.py](codes/PIDTests/pid_testC.py) | PID controller, moving-target sudden-stop experiment, 10-second run. |

The final letter in the saved filename `pd_testС.py` is a Cyrillic `С`, rather than a Latin `C`. The link above preserves the actual filename.

All controllers calculate `e = measured_distance - 200`. The P controller uses `u = KP × e`. The PD controller adds a filtered derivative term, while PID also includes accumulated error:

```text
raw_derivative = (error - previous_error) / H
filtered_derivative = 0.3 × raw_derivative + 0.7 × previous_filtered_derivative

P command   = KP × error
PD command  = KP × error + KD × filtered_derivative
PID command = KP × error + KI × integral + KD × filtered_derivative
```

The robot's linear speed command is limited to ±300 mm/s, with zero commanded turning rate. In the PID programs, integration is updated only when the unsaturated command magnitude is below 300 mm/s. This updated integral is used in the following command calculation. Distance readings above 2500 mm cause the robot to stop and skip that iteration's controller calculations and logging.

## Robot configuration

| Parameter | Value |
| --- | --- |
| Controller | LEGO EV3 |
| Runtime | Pybricks EV3 MicroPython |
| Left motor | Port B |
| Right motor | Port C |
| Ultrasonic sensor | Port S4 |
| Wheel diameter used by DriveBase | 55.5 mm |
| Distance between wheels used by DriveBase | 104 mm |
| Desired separation | 200 mm |
| Saved proportional gain | `KP = 6.0` |
| Derivative gain in PD and PID | `KD = 0.5` |
| Integral gain in PID | `KI = 3.0` |
| Derivative filter weight | `ALPHA = 0.3` |
| Linear speed-command limits | -300 to +300 mm/s |
| Delay after each iteration | 50 ms, in addition to processing and output time |
| Nominal calculation interval in PD and PID | `H = 0.05` seconds |
| Gain tests and Test A duration | 6 seconds |
| Test B and Test C duration | 10 seconds |

Unlike the motor rotation speeds used in Labs 1 and 2, `robot.drive(u, 0)` receives linear speed in mm/s. The 50 ms wait does not guarantee an exact 50 ms sampling period; use the recorded timestamps when analyzing the logs. The derivative and integral calculations in the saved code use the fixed nominal interval `H`.

## Running the programs

1. Prepare the EV3 with the Pybricks EV3 MicroPython environment used in the course.
2. Transfer the selected program to the EV3 using your existing EV3 workflow.
3. Check the motor ports, sensor port, wheel diameter, and wheel spacing against the actual robot.
4. Position the box for the selected experiment, then launch the program. The scripts start the timed controller automatically after initialization and their introductory terminal output.
5. Let the timed run finish. The program stops the robot and prints a completion message. Retrieve the generated DataLog CSV from the brick and retain its original filename and the settings used.

For proportional-gain testing, change both `KP` and the DataLog base name in `p_kp_testA.py` before each trial. The saved source uses the base name `P_TestA_KP6`.

| Experiment | Procedure |
| --- | --- |
| Test A | Start approximately 600 mm from a stationary box and allow the robot to approach the 200 mm setpoint. |
| Test B | Move the box manually away from the robot and then back toward it, requiring corrections in both directions. |
| Test C | Move the box and stop it abruptly while the robot follows. Some recorded trials also included unintended movement of the box toward the robot while stopping. |

The box movement is performed manually; the programs do not control its trajectory or record the exact movement and stopping events.

### Existing PID Test B startup issue

The saved `pid_testB.py` contains an extra calculation block before the sensor reading, controller variables, timer, and logger have been initialized. Its first line, `e = d - SETPOINT`, references undefined `d` and prevents the script from reaching its timed test loop. This block requires correction before running the saved program. The PID Test B CSV remains available as an experimental record, but the current source cannot reproduce it as-is. Preparing this README does not change the program.

## CSV format

| Column | Meaning | Unit |
| --- | --- | --- |
| `t_ms` | Elapsed timer reading when the row was logged | ms |
| `d_mm` | Ultrasonic distance measurement | mm |
| `e_mm` | Measured distance minus the 200 mm setpoint | mm |
| `u_mms` | Limited linear speed command passed to DriveBase | mm/s |

The speed column records a command, not an independent measurement of actual robot speed.

## Recorded proportional-gain results

The following values are calculated from the five CSV files in `data/kp/`. Mean absolute error is calculated over all samples in each log, including the initial approach.

| Gain and source log | Minimum distance (mm) | Final distance (mm) | Mean absolute error (mm) |
| --- | ---: | ---: | ---: |
| [KP = 1, inferred](data/kp/P_TestA_2020_04_11_16_16_51_279161.csv) | 195 | 198 | 76.23 |
| [KP = 2](data/kp/P_TestA_KP2_2020_04_12_00_32_00_845566.csv) | 198 | 198 | 59.27 |
| [KP = 4](data/kp/P_TestA_KP4_2020_04_12_00_34_18_959727.csv) | 197 | 198 | 57.93 |
| [KP = 6](data/kp/P_TestA_KP6_2020_04_12_00_40_52_088480.csv) | 190 | 197 | 55.47 |
| [KP = 8](data/kp/P_TestA_KP8_2020_04_12_00_38_26_323370.csv) | 179 | 204 | 57.56 |

The first filename does not specify its gain. A gain of 1 is inferred from its unsaturated logged commands, where `u / e = 1` for nonzero error. The other gain labels follow their filenames and agree with the corresponding unsaturated command-to-error ratios.

In these runs, KP = 6 produced the smallest whole-log mean absolute error. KP = 8 reached a minimum separation of 179 mm, showing a larger excursion below the 200 mm target. These observations describe the individual recorded trials rather than repeated-trial averages.

## Recorded controller-comparison results

The table summarizes the nine CSV files in `data/tests/`. Each linked row corresponds to one recorded trial.

| Test | Controller and source log | Distance range (mm) | Final distance (mm) | Mean absolute error (mm) |
| --- | --- | --- | ---: | ---: |
| A | [P](data/tests/P_TestA_2020_04_12_18_41_06_850503.csv) | 190–662 | 203 | 62.02 |
| A | [PD](data/tests/PD_TestA_2020_04_12_18_41_45_181406.csv) | 195–649 | 198 | 63.55 |
| A | [PID](data/tests/PID_TestA_2020_04_12_18_42_32_286718.csv) | 190–651 | 198 | 65.14 |
| B | [P](data/tests/P_TestB_KP6_2020_04_12_18_51_16_354228.csv) | 174–243 | 197 | 15.16 |
| B | [PD](data/tests/PD_TestB_2020_04_12_18_51_56_419856.csv) | 166–249 | 198 | 16.49 |
| B | [PID](data/tests/PID_TestB_2020_04_12_18_52_25_090147.csv) | 174–251 | 203 | 12.79 |
| C | [P](data/tests/P_TestC_KP6_2020_04_12_18_53_35_900653.csv) | 143–251 | 198 | 12.54 |
| C | [PD](data/tests/PD_TestC_2020_04_12_18_54_23_693556.csv) | 162–289 | 198 | 14.78 |
| C | [PID](data/tests/PID_TestC_2020_04_12_18_54_53_999324.csv) | 135–265 | 199 | 17.06 |

```text
mean_absolute_error_mm = sum(abs(distance_mm - 200)) / number_of_samples
```

These are sample-averaged, whole-log errors, not time-weighted errors or errors restricted to a particular stopping event. In the available runs, P has the lowest mean absolute error in Tests A and C, while PID has the lowest in Test B. No controller has the lowest error in every test.

Manual movement differed between trials, and exact start, reversal, and stop times are not recorded in the CSV files. Consequently, a lower whole-log error in Test C does not by itself establish the best sudden-stop response. Ultrasonic fluctuations, different initial distances, and speed saturation also affect the comparison. Graphs and plotting scripts are not currently included in this folder.
