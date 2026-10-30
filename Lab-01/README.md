# Lab 01 — Square Navigation and Odometry

This laboratory explores differential-drive motion using a LEGO EV3 robot. The robot follows a square with 50 cm sides. A baseline program is compared with a version that estimates position from motor encoders and applies proportional heading correction during straight movement.

## Folder contents

```text
Lab-01/
├── README.md
├── codes/
│   ├── square_before.py
│   └── square_new.py
└── data/
    ├── test1_90+.png
    ├── test1_90-.png
    ├── test2_90+.png
    └── test2_90-.png
```

## Programs

| File | Description |
| --- | --- |
| [square_before.py](codes/square_before.py) | Baseline program. Converts the requested distance and turn angle into motor rotations, then executes four straight segments and four turns. |
| [square_new.py](codes/square_new.py) | Adds encoder-based odometry, proportional heading correction, terminal output, and measurement pauses at each corner. |

The feedback program updates the estimated position `(x, y)` and heading from changes in the left and right motor encoders. During each straight segment, it compares the current estimated heading with the heading at the start of that segment. The resulting correction is added to one motor's speed and subtracted from the other's speed.

## Robot configuration

| Parameter | Value |
| --- | --- |
| Controller | LEGO EV3 |
| Runtime | Pybricks EV3 MicroPython |
| Left motor | Port B |
| Right motor | Port C |
| Wheel radius | 2.74 cm |
| Distance between wheels | 12.0 cm |
| Square side length | 50 cm |
| Straight-motion motor speed | 300 degrees/s |
| Baseline turning motor speed | 300 degrees/s |
| Feedback turning motor speed | 200 degrees/s |
| Proportional gain | `KP = 4.0` |
| Heading-correction limit | ±80 degrees/s |
| Feedback loop delay | 10 ms, in addition to processing time |
| Corner measurement pause | 10 seconds in the feedback program |

Motor speed values are wheel/motor rotation rates, not the robot's linear speed.

## Running the programs

1. Prepare the EV3 with the Pybricks EV3 MicroPython environment used in the course.
2. Transfer the selected script from `codes/` to the EV3 using your existing EV3 workflow.
3. Check that the motor ports and robot dimensions match the values listed above.
4. Place the robot at a marked starting position with enough space for the square path.
5. Run one program per trial. For the feedback version, use the corner pauses to mark or measure the robot's position and save the terminal output.

The committed versions of both programs use `turn(-90)` at all four corners. To reproduce a positive-angle square, use `turn(90)` at all four corners in the selected program and record that setting with the trial. Keep a copy of the exact version used for each experiment.

In `square_new.py`, `Corner 1` through `Corner 4` are printed after each straight segment and before its following turn. The final position is printed after the fourth turn. The program prints results to the terminal; it does not automatically save a CSV file.

## Recorded terminal results

The `data/` folder contains two screenshots for each angle sign. The table transcribes their final encoder-based position estimates. Angle labels follow the screenshot filenames.

| Trial | Final x (cm) | Final y (cm) | Final heading (degrees) | Estimated closing error (mm) |
| --- | ---: | ---: | ---: | ---: |
| [Test 1, +90°](data/test1_90%2B.png) | 1.2 | -0.6 | 361.9 | 13.4 |
| [Test 1, -90°](data/test1_90-.png) | 0.9 | 0.6 | -361.9 | 10.8 |
| [Test 2, +90°](data/test2_90%2B.png) | 0.9 | -0.6 | 362.1 | 10.8 |
| [Test 2, -90°](data/test2_90-.png) | 1.0 | 0.6 | -361.9 | 11.7 |

The estimated closing error is the distance from the final estimated position to the initial `(0, 0)` position:

```text
closing_error_mm = 10 × sqrt(x_final_cm² + y_final_cm²)
```

These values are calculated from the rounded positions visible in the screenshots. They describe the odometry estimate; independent floor measurements are needed to establish the robot's actual closing error. The screenshots alone do not establish an improvement over the baseline program.

The screenshots show an execution filename of `new_square.py`, while the feedback program in this repository is named `square_new.py`. The exact historical source versions for the screenshot trials have not been separately archived. In particular, the current programs are set to negative-angle turns, while the screenshots include both angle signs.
