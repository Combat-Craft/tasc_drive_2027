# TASC 2027 Judge Console

Judge interface for the CIRC 2027 **Automated Demolition and Clearing** task. Sources: the task page (https://circ.cstag.ca/2027/tasks/) and the Autonomy Guidelines in the 2027 Rules and Guidelines page (initial publication 2026-09-30). One labelled button per operation, so the judge gives a single input and the rover does the rest.

Status: **stage 1, no ROS.** Phases 1-2 (drive) are live and print to the terminal and the on-screen log. Phases 3-7 are shown greyed out for the design review.

![Drive tab](judge_console_drive.png)
![End effector tab](judge_console_eff.png)

## Run it

Needs Python 3 and PySide6 (already on the VM from the Jazzy setup).

```bash
python3 judge_console.py              # opens the GUI (run from a terminal inside the VM desktop)
python3 judge_console.py --selftest   # no screen needed; exit code 0 = pass
```

A click prints, for example: `[19:39:37] drive_backwards {'distance': 2.0}: going backwards 2 m`

## Rules this follows

| Rule (2027 task page) | How the console handles it |
| --- | --- |
| Labelled button per operation, format `[shortname] long name` | Short names are the official bracketed names from the rules, e.g. `[Drive Forward] Drive forward 3 m`. The self-test checks every label. |
| Text boxes for operations that take input, in the rule's units | Units shown next to every box: m, °, cm, WGS 84 decimal degrees |
| Returned values must show their units | Right panel: bearing (°), GNSS (°, lat, lon), marker IDs, EFF position (m, x/y/z) |
| Single judge input, no further input from judge or team | No toggles on the judge screen. Obstacle avoidance is a team setting in code (`OBSTACLE_AVOIDANCE = True`), shown read-only in the top bar |
| Any order within a phase, repeatable | Buttons grouped by phase, no locking. Each row shows its attempt number |
| Resets free; skips count as modification interventions | "Team log" buttons record these in the log only. Nothing is sent to the rover |

| Rule (2027 Rules, Autonomy Guidelines) | How the console handles it |
| --- | --- |
| Teams must declare when entering autonomous mode | **Declare autonomous mode** button and banner. A judged operation sent without it logs a warning |
| During autonomy no team member may touch the operations computer, any input at all (even switching cameras). Autonomy only counts again after stop and restart | Banner says "hands off" in autonomous mode. Team log buttons are locked. Pressing STOP ends autonomous mode |
| Operators notify the judge when autonomy ends | **End autonomous mode** button, logged |
| A separate computer, not controlling the rover, may be used for planning (tell the judge first) | Not in the GUI. Use another laptop for anything else |
| ArUco markers are 4x4_100, IDs 0-99 | Marker ID boxes accept 0-99 only |
| ArUco results must show automatically on a screen the judge can see. Manual lookup isn't accepted | "Stored marker IDs" panel, filled by software (`show_markers`) |
| Distances are odometer distance over the real terrain | Rover-side: drive N m means N m travelled along the ground, not straight-line |
| Margin of error for each operation is in the rubric | Rubric is "to come", not available yet |

## Operations

Live (phases 1-2):

| Button | op_id | Input |
| --- | --- | --- |
| [Store Home] Stow arm and end effector | `store_home` | none |
| [Drive Forward] Drive forward 1 / 3 / 5 m | `drive_forward_1/3/5` | none (rules allow only 1, 3, 5) |
| [Drive Backwards] Drive backwards N m | `drive_backwards` | N m, > 0 (rules give no fixed values) |
| [Rotate] Rotate N° from current heading (+ = clockwise) | `rotate` | N °, -360 to 360 |
| [Rotate and Drive] Drive N m at M° clockwise of heading | `rotate_and_drive` | N m > 0, M ° 0-360 |
| [Face Bearing] Face absolute bearing N° | `face_bearing` | N °, 0-360, north = 0 |
| [Report Bearing] Report current bearing | `report_bearing` | none, returns ° |
| [Report GNSS] Report current GNSS position | `report_gnss` | none, returns lat/lon ° |
| [Drive to Waypoint] Drive within 3 m of GNSS waypoint | `drive_to_waypoint` | lat °, lon ° |
| [Drive to Marked Waypoint] Drive within 1 m of ArUco #3 tower | `drive_to_marked_waypoint` | lat °, lon ° |

Shown for review, not wired (phases 3-7), all with official names: [Read and Store Markers], [Retrieve Markers], [EFF Max Height], [EFF Min Height], [EFF Position] (height m, distance m), [Store Home], [EFF Open] (N cm, max 10), [EFF Close], [EFF Report], [EFF Pickup], [EFF Squeeze], [EFF Set Down], [EFF Press Marked Button] (marker IDs N and M).

Phases 4, 6 and 7 also need driving to GNSS points. The rules have no separate operation for that, so the drive buttons are reused.

**[STOP]** in the top bar is a team safety control, not a judged operation. The task rules don't mention one.

Bad input (empty, not a number, out of range) is logged as `NOT SENT: ...` and nothing is dispatched.

## Code layout

All in `judge_console.py`:

- `Operation` / `Field`: one entry per button. Adding an operation = adding one `Operation`. Set `live=True` to enable it.
- `Command`: what a click produces (`op_id`, `args`, phrase, timestamp).
- `CommandSink`: where commands go. `PrintSink` prints. **The ROS step is a new sink**, the GUI doesn't change.
- `show_bearing`, `show_gnss`, `show_markers`, `show_eff`: hooks that put returned values on screen with units. ROS subscribers will call these.

Kept separate from `rover_dashboard` (relay/motor telemetry). Same stack (PySide6), so it can be merged in as a tab later.

## Open questions

Not available yet (so not checked): the **Scoring Rubric** (marked "to come"), the **Interventions Guide**, and the **Rover Safety Requirements** pages. The rules list illuminated kill switches and hard stops under safety, which are hardware, not this GUI.

1. Rotate sign: the rules don't say. Positive = clockwise is assumed and printed on the button.
2. Bearing: the rules say geographic north. Magnetic declination for Drumheller must be applied on the rover side.
3. [EFF Pickup] and [EFF Set Down] inputs depend on a diagram the organisers haven't published yet. No text boxes yet.
4. [Retrieve Markers] in phase 4 returns the IDs from the latest [Read and Store Markers]. The GNSS target may depend on them.

## Next steps

1. `Ros2Sink`: publish timed `geometry_msgs/Twist` on `/cmd_vel` (the topic `tasc_drive_2027` drive code uses) for Drive Forward/Backwards and Rotate. Test in simulation.
2. Feed `show_bearing` / `show_gnss` from simulated IMU and GNSS topics.
3. Wire the waypoint buttons to a navigation goal once the sim has localisation.
4. Decide where it lives: new package in `tasc_drive_2027`, or a tab in `rover_dashboard`.
