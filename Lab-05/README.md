# Lab 05 — Maze Exploration, Mapping and Route Planning

This laboratory explores maze mapping using a LEGO EV3 robot equipped with a color sensor, wheel encoders, and a gyroscope. The robot follows both sides of a tape maze over two exploration laps and records its estimated motion. A separate computer-side program reconstructs the sensor paths, detects junctions and dead ends, builds a weighted graph, and can calculate a shortest route using Dijkstra's algorithm.

## Folder contents

```text
Lab-05/
├── README.md
├── codes/
│   ├── main.py
│   └── chart.py
└── graphs/
    ├── explore_signals.png
    ├── map.png
    ├── path.png
    ├── raw_map.png
    └── t.txt
```

The figures are included as saved experiment outputs. The original exploration CSV, generated node and edge tables, and route instructions are not currently included. `t.txt` is a blank text placeholder rather than an experimental record.

## Programs

| File | Description |
| --- | --- |
| [main.py](codes/main.py) | Runs on the EV3. Follows the tape, estimates motion from wheel rotation and gyro heading, handles the transition between tape sides, and saves the exploration measurements to `explore.csv`. |
| [chart.py](codes/chart.py) | Runs on a computer. Reads an exploration CSV, prepares and aligns the lap trajectories, detects or loads nodes, measures edges, produces figures and tables, and optionally writes a shortest-route plan. |

The exploration controller uses proportional steering based on reflection error. It smooths the error used to select forward speed, slows near difficult sections, and allows limited reverse base motion when far from the tape edge. A progress check can trigger a short forward push when the robot has barely moved or turned.

Before exploration, the program measures gyro drift while the robot is stationary. During position estimation, it subtracts this estimated drift from the gyro heading and combines the corrected heading with average wheel movement. Measurements are stored in compact arrays and written to CSV after exploration finishes.

## Robot configuration

| Parameter | Value |
| --- | --- |
| Controller | LEGO EV3 |
| Robot runtime | Pybricks EV3 MicroPython |
| Left motor | Port B |
| Right motor | Port C |
| Color sensor | Port S3 |
| Gyroscope | Port S1, counterclockwise rotation positive |
| Wheel radius used in exploration | 2.8 cm |
| Target reflection | 27% |
| Normal base wheel speed | 120 degrees/s |
| Proportional steering gain | `KP = 3` |
| Smoothed-error update weight | `AVG = 0.1` |
| Smoothed error giving zero forward base speed | 14 reflection units |
| Reverse base-speed limit | 40% of normal base speed |
| Progress-check interval | 2000 ms |
| Recovery push | 400 ms at 40% base speed, with steering correction halved |
| Crossing turn angle | 50° toward the tape, then the opposite rotation |
| Required sideways crossing distance | 3.5 cm |
| Crossing wheel speed | 150 degrees/s |
| Controller delay | 10 ms after each step |
| Stationary gyro drift measurement | 10 seconds, after a 1-second pause |
| Live terminal measurements | Every tenth logged row by default |

The speed values in `main.py` are motor rotation rates in degrees/s. The reverse limit applies to the base speed before the steering correction is added or subtracted. The actual control interval also includes sensor reading, processing, logging, and any terminal output.

## Mapping configuration

| Parameter in `chart.py` | Value and purpose |
| --- | --- |
| `SENSOR_OFFSET` | 7.0 cm from wheel axle center to color sensor; check against the actual mounting. |
| `WHEEL_R` | 2.8 cm, used when heading is reconstructed from wheel encoders. |
| `TRACK` | 11.8 cm wheel spacing, used for wheel-derived heading. |
| `STEP` | Resample sensor paths every 1.0 cm. |
| `SMOOTH` | Nine-sample smoothing window for edge-length measurement. |
| `PASS_R` | 7.0 cm spatial radius for nearby path passes. |
| `ARC_GAP` | Ignore same-lap path neighbors within 20.0 cm along the trace. |
| `NODE_R` | Group same-kind visits within 10.0 cm into one node. |
| `MERGE_D` | Merge a junction into a nearby dead end within 8.0 cm. |
| `HIT_R` | 6.0 cm visit radius for manually labelled nodes. |
| `DRIFT_MIN` | Optional heading drift correction starts at a 45° mismatch. |
| `START_DIR` | 0°, so the robot's initial direction is drawn toward the right. |

The mapping program excludes crossing/turning-mode samples when preparing each lap. It distributes the estimated endpoint position mismatch across the lap, accounts for the sensor offset, resamples the path, and aligns the second lap onto the first by rotation and translation. These corrections assume that each exploration lap returns to its starting mark. Optional heading drift correction additionally assumes the lap ends facing its starting direction, up to a whole number of rotations.

## Running the exploration program

1. Prepare the EV3 with the Pybricks EV3 MicroPython environment used in the course and transfer `codes/main.py` to the brick.
2. Check the motor, color sensor, and gyro connections. Place the robot at the marked start with the tape on its left side.
3. Press **CENTER**, then release it. Keep the robot still during the settling pause and 10-second gyro drift check.
4. Let the robot follow the first tape edge. When it returns to the starting mark in the required orientation, press and release an EV3 button to mark completion of lap 1.
5. The program crosses the tape and starts lap 2 with the tape on the robot's right side.
6. At the end of lap 2, press and release a button again. The robot stops and writes `explore.csv` in its current working directory.
7. Wait for the saved-file message, then copy the CSV to the computer. Preserve each trial under a distinct name before another run overwrites `explore.csv`.

Lap completion is marked by the user; it is not detected automatically. During exploration, the program checks for any pressed EV3 button after a following step. It also attempts to save the measurements collected if an ordinary exception interrupts exploration.

If measured gyro drift exceeds 1 degree/s, the program displays a gyro warning but does not automatically stop or repeat calibration. `PRINT_EVERY = 10` prints only every tenth row live; the saved CSV contains all logged rows.

## Running the mapping program

Use computer-side Python with NumPy and Matplotlib installed:

```bash
python -m pip install numpy matplotlib
```

Place a retrieved exploration log named `explore.csv` in the `Lab-05` directory, alongside `codes/` and `graphs/`, then run the following command from that directory:

```bash
python codes/chart.py explore.csv
```

The input CSV must be supplied first; it is not currently stored in the repository. If its name contains spaces or parentheses, quote the filename when passing it to the program.

The default heading source is the logged gyro heading. To use wheel-derived heading or request optional heading drift correction:

```bash
python codes/chart.py explore.csv --heading wheels
python codes/chart.py explore.csv --heading gyro --fix-drift
```

For optional manual node positions, provide a text file with `name,x,y` columns in the coordinate frame of the displayed map:

```bash
python codes/chart.py explore.csv --nodes labelled_nodes.csv
```

After inspecting the generated node table and checking the graph against the maze layout, a route can be planned between connected nodes. Replace `START_NODE` and `GOAL_NODE` with actual identifiers:

```bash
python codes/chart.py explore.csv --start START_NODE --goal GOAL_NODE
```

The program uses mean measured edge lengths as graph weights. Route instructions select a tape side and use a matching-side encoder distance when available, with geometric edge length as a fallback. These instructions are a plan; the supplied EV3 exploration program does not execute `route.txt` as a shortest-route driving test.

## Exploration CSV format

| Column | Meaning | Unit or values |
| --- | --- | --- |
| `time_ms` | Elapsed time after exploration timer reset | ms |
| `arc_cm` | Accumulated signed center movement from wheel encoders | cm |
| `x_cm`, `y_cm` | Estimated robot-center position | cm |
| `heading_deg` | Rounded gyro heading after stationary drift compensation | degrees |
| `reflection` | Color sensor reflected light intensity | % |
| `lap` | Exploration lap | 1 or 2 |
| `mode` | Motion classification | 0 = following; 1 = crossing/turning |
| `left_deg`, `right_deg` | Raw motor encoder angles | degrees |

The compact logging arrays store distance and position values as integer millimeters, which are converted back to centimeters when writing the CSV. The signed encoder travel is distinct from the geometric length of the reconstructed sensor path.

## Generated mapping files

| Output | Description |
| --- | --- |
| `map.png` | Corrected and aligned sensor paths with detected nodes, measured edge labels, and an optional highlighted route. |
| `explore_signals.png` | Reflection, prepared heading, and nearby-pass counts for each lap, with detected visit markers. |
| `nodes.csv` | Node identifiers, kinds, map coordinates, degrees, visit counts, and neighboring exits. |
| `edges.csv` | Undirected node connections, mean geometric edge lengths, and individual pass lengths. |
| `route.txt` | Optional tape-side and distance instructions when start and goal are supplied. |

Outputs are written into the directory where the command is run, not automatically into `graphs/`. Preserve the input log and command options with each set of archived outputs. The supplied script generates `map.png` and `explore_signals.png`; the separate script or command that produced the archived `raw_map.png` is not included.

## Recorded mapping results

The legend in the saved [map.png](graphs/map.png) reports the following endpoint mismatches:

| Lap | Tape side | Reported loop-closure drift (cm) |
| --- | --- | ---: |
| 1 | Left | 3.8 |
| 2 | Right | 8.4 |

In the mapping code, this value is the estimated endpoint offset measured before applying the per-lap position-closure adjustment. Any selected heading processing can already have affected the coordinates. It is not an independently measured physical closing error or a remaining error after all map corrections.

The displayed reconstruction labels one junction, J1, and six endpoints, E1–E6. These are algorithm-generated nodes rather than verified identities from the reference maze. Their number and placement should be checked against [path.png](graphs/path.png) before using the graph for navigation. The supplied figures do not establish a successful physical shortest-path run.

The source exploration CSV and exact command options used to generate the saved figures are not currently archived in this folder, so the numerical results cannot yet be independently regenerated from the repository alone.

## Graphs and reference layout

| File | Description |
| --- | --- |
| [map.png](graphs/map.png) | Reconstructed sensor paths with detected nodes and edge lengths in centimeters. |
| [explore_signals.png](graphs/explore_signals.png) | Recorded reflection, processed heading, and nearby-pass counts across both laps. |
| [raw_map.png](graphs/raw_map.png) | Raw exploration trajectories with start/end markers and crossing/turning samples. |
| [path.png](graphs/path.png) | Reference maze layout with labelled locations and the marked starting position. |

The raw and reconstructed figures illustrate differences in path alignment and mapping output. Without their corresponding logs and processing settings, they should not be treated as a controlled numerical comparison of two robot configurations.
