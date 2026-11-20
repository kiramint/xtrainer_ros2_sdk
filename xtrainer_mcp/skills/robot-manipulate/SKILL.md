---
name: robot-manipulate
description: >
  Pick, place, and reorient objects with the XTrainer arms through the xtrainer
  MCP tools. Use when the user asks to grasp, move, or put an object somewhere,
  including 抓取, 放置, 抓放, pick, place, and /robot-manipulate. Wrist cameras
  are the metric source. The top-camera hand-eye file is not used for positions.
---

# Robot manipulate

Drive the already-running XTrainer through one `xtrainer_mcp` process. Tool names and units are in `xtrainer_mcp/README.md`. Plans exist only inside that process: `plan_pose` and `execute_plan` must share it.

`Arm1` uses gripper `left`. `Arm2` uses gripper `right`. Positions are meters in `base_link`. Joints are radians. Grasp and place contact with `link="tip"`.

## Pose

The jaws open along the flange X axis. Quaternion `(1, 0, 0, 0)` points flange Z straight down and flange X along world +X.

Build a straight-down pose whose jaws close across the object's short axis `s = (sx, sy, 0)`:

- tool X = `(sx, sy, 0)`
- tool Y = `(sy, -sx, 0)`
- tool Z = `(0, 0, -1)`

If that pose has no IK solution, pitch about tool Y so the tip reaches farther horizontally. For jaws along world X, pitching toward +X by `α` degrees is quaternion `(cos(α/2), 0, sin(α/2), 0)`. Try 15°, 25°, 35°, then 45°, and keep the smallest `α` for which `ik` returns `ok` and `self_collision.valid`. Carry the object upright, and apply the pitch only on the last move into the goal.

## Look

`camera_top` may be used as a picture. Do not call `project_pixel` on it, and do not trust its hand-eye transform. Measure with `camera_left` or `camera_right`.

Move a wrist so the object fills the view, then sample the mask with `project_pixel` (`window` 5). Reject depths that belong to the fingers. Record the object's center, short axis, top height, and the support or container floor. A container goal needs the floor height and the rim; the release point is inside the rim, not on it.

## Move

Read `get_status` first. Mode 5 is idle. Mode 9 needs `clear_error` before another motion. Mode 11 means stop and tell the user.

Before a transfer, move the other arm out of the goal volume. `plan_pose`, then `check_plan`. Execute only when `valid` is true and `contacts` is empty. Use `pilz_lin` for a short Cartesian step and `ompl` when the line fails or the orientation change is large. After `execute_plan`, wait until the arm is idle.

Set the gripper torque before closing. `gripper_get_state` reports `OPENED` below 0.05 and `CLOSED` above 0.95; anything between stays `MOVING`. A position that stops between those thresholds while the load sits on the torque limit is a grasp on the object.

## Pick and place

1. Open the gripper. Put the tip above the object center with the jaws across the short axis.
2. Photograph. Shift in the horizontal plane until the object is between the pads, then descend. Stop the tip between the support and the object top, near the support, so the pads meet the sides.
3. Close. If the position returns near 0, the jaws missed: open, look, and correct. If it stalls between 0.05 and 0.95, lift and look again. The object must still be between the pads.
4. Carry it upright to a reachable pose near the goal. Pitch in, with the tip high enough that the object bottom clears the rim, then lower until the bottom is just above the floor.
5. Open. Retreat along the approach before changing orientation, so the jaws do not sweep the object.
6. From a wrist camera, `project_pixel` the object. Its horizontal position is inside the goal and its height matches the floor plus the object.

Do not drive the tip below the support. Do not start a second `move_group` or `robot_state_publisher`.
