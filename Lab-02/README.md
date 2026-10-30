# Lab 02 — Color Sensor Calibration and Line Following

This laboratory explores line following using a LEGO EV3 robot equipped with a single color sensor. Reflection measurements are used to distinguish black tape from the surrounding floor. The controller combines proportional steering near the tape edge with locked turning states when the sensor moves outside the normal reflection band.

## Folder contents

```text
Lab-02/
├── README.md
├── codes/
│   ├── main.py
│   └── edge_scan.py
├── data/
│   ├── 10mmTest_1.png
│   ├── 10mmTest_2.png
│   ├── calibration1.png
│   ├── calibration2.png
│   └── calibration3.png
└── graphs/
    ├── Graph1.png
    └── Graph2.png
```

## Programs

| File | Description |
| --- | --- |
| [main.py](codes/main.py) | Follows the tape edge using a right-mounted color sensor, proportional steering, and locked left/right turning states. Includes a manual stop and elapsed-time output. |
| [edge_scan.py](codes/edge_scan.py) | Guides manual sensor positioning, takes ten reflection readings at each position, and prints the readings, averages, and a final comma-separated summary. |

In normal following, the controller calculates `error = reflection - TARGET` and `correction = error × 7`. It subtracts the correction from the left motor speed and adds it to the right motor speed. Normal-state speeds are limited to 80–320 degrees/s.

When reflection exceeds 30, the controller enters a locked left turn. When reflection falls below 22, it enters a locked right turn. A left turn remains locked until reflection reaches 26 or below; a right turn remains locked until reflection reaches 26 or above. Normal following resumes on the next loop. Final speed limits prevent the controller from commanding reverse motion.

## Robot configuration

| Parameter | Value |
| --- | --- |
| Controller | LEGO EV3 |
| Runtime | Pybricks EV3 MicroPython |
| Left motor | Port B |
| Right motor | Port C |
| Color sensor | Port S3, mounted on the right side |
| Sensor measurement | Reflected light intensity (%) |
| Black reference stored in code | `BLACK = 10` |
| Floor reference stored in code | `FLOOR = 55` |
| Tuned reflection target | `TARGET = 26` |
| Normal reflection band | 22–30, inclusive |
| Proportional steering gain | 7 |
| Normal following base speed | 300 degrees/s |
| Outer wheel speed during locked turns | 248 degrees/s |
| Inner wheel speed during locked turns | 49 degrees/s |
| Maximum motor speed | 320 degrees/s |
| Controller loop delay | 5 ms, in addition to processing time |
| Debug output | Disabled by default; when enabled, printed every 50 loops |
| Current scan positions | 0, 5, 10, 15, and 20 mm from the tape edge toward the floor |
| Samples per scan position | 10, with a 100 ms wait after each reading |

Motor speed values are wheel/motor rotation rates, not the robot's linear speed. The target of 26 is set manually; it is not automatically calculated from `BLACK` and `FLOOR`, whose midpoint would be 32.5.

## Running the programs

1. Prepare the EV3 with the Pybricks EV3 MicroPython environment used in the course.
2. Transfer the selected script from `codes/` to the EV3 using your existing EV3 workflow.
3. Check that the motors and color sensor are connected to the ports listed above.
4. Keep the sensor position and height consistent between calibration, scanning, and line-following trials.
5. Save the terminal output and record the settings used for each experiment.

For `main.py`, place the sensor on the tape edge and press **CENTER**. Release the button to begin the run. Press **DOWN** to stop the robot. The program brakes both motors, beeps, and prints the elapsed time in seconds. Stopping is manual; the program does not automatically detect completion of a lap. Its printed time includes the stopping and final-beep overhead, so record the timing method when comparing runs.

For `edge_scan.py`, move the sensor manually to each position shown on the EV3 screen. Press **CENTER**, then release the button to take ten readings after a 500 ms pause. Press **DOWN** while waiting at a position to end the scan and print the results collected so far. The program does not move the sensor automatically or save a CSV file; copy the final `position_mm,reflection` rows from the terminal if a data file is needed.

## Recorded calibration and scan results

The calibration screenshots contain the following reported averages and midpoint targets:

| Record | Floor/white average (%) | Black average (%) | Midpoint target (%) |
| --- | ---: | ---: | ---: |
| [Calibration 1](data/calibration1.png) | 47.0 | 7.0 | 27.0 |
| [Calibration 2](data/calibration2.png) | 40.0 | 4.1 | 22.05 |
| [Calibration 3](data/calibration3.png) | 42.7 | 7.5 | 25.1 |

These values describe the recorded calibration sessions. They differ from the fixed reference values in the current controller and should not be treated as settings that the program loads automatically. The separate program that produced these calibration screenshots is not included in `codes/`.

[10mmTest_1.png](data/10mmTest_1.png) shows individual readings and averages for a scan from -10 to +10 mm in 2 mm steps. [10mmTest_2.png](data/10mmTest_2.png) shows the matching final summary. For example, the recorded mean reflection increases from 14.3% at 0 mm to 42.0% at 2 mm. These two images document the same set of scan values rather than two independent repetitions.

The saved `edge_scan.py` currently uses positions `[0, 5, 10, 15, 20]`. Reproducing the screenshot scan requires the recorded -10 to +10 mm position list with 2 mm steps. Keep the position list with each scan record so the datasets remain distinguishable.

## Recorded line-following results

[Graph2.png](graphs/Graph2.png) presents the following completion times. The motor configurations are listed as straight / outer / inner wheel speed.

| Configuration (degrees/s) | Trial label | Completion time (s) |
| --- | --- | ---: |
| 190 / 145 / 20 | Initial | 85.27 |
| 250 / 210 / 40 | Intermediate configuration | 57.88 |
| 300 / 230 / 45 | Intermediate configuration | 53.15 |
| 300 / 245 / 49 | Intermediate configuration | 52.23 |
| 300 / 248 / 49 | Best | 49.59 |
| 300 / 248 / 49 | Repeat | 50.17 |

The best recorded time is 35.68 seconds shorter than the initial time, a reduction of approximately 41.8%:

```text
time_reduction_percent = (85.27 - 49.59) / 85.27 × 100
```

The current `main.py` uses the 300 / 248 / 49 speed configuration. The graph provides a summary of earlier trials; the original timing records and all earlier controller versions are not included in this folder. These results show the recorded time difference but do not isolate the effect of motor speed from other controller or setup changes.

## Graphs

| File | Description |
| --- | --- |
| [Graph1.png](graphs/Graph1.png) | Reflection across the tape boundary, with coarse and fine scan curves, a target line, and a normal-control band. |
| [Graph2.png](graphs/Graph2.png) | Completion times for the tested motor configurations. |

Graph1 shows a target of 32.5 and a normal-control band of 28.5–36.5. These are different from the current controller's target of 26 and band of 22–30. It also shows different scan positions from the -10 to +10 mm screenshots. Retain these distinctions when referring to the graph or regenerating it. The plotting scripts and complete numeric inputs for Graph1 are not currently included in this folder.
