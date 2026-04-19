# Hybrid Humble Nav2 Package Summary

This package is refactored for **ROS 2 Humble**, **Jetson Orin**, **LDLIDAR**, **YOLO/Isaac detections**, and **Nav2**.

## What changed in this updated version

- The shared decision logic now includes a **cost-aware rover vs drone score** instead of relying only on fixed if/else rules.
- Fully autonomous mode now **self-approves** rover-to-drone, landing, and drone-to-rover transitions instead of getting stuck in pending states.
- The tracker now exposes **obstacle area ratio** so the decision engine can reason about obstacle severity.
- Rover logic now distinguishes between:
  - short-lived moving obstacles,
  - persistent static blockages,
  - conditions where ground avoidance is still reasonable,
  - conditions where switching to drone is the safer choice.
- Regression tests now cover:
  - manual mode safety gating,
  - invalid route escalation,
  - automatic rover-to-drone switching in full autonomy,
  - automatic landing and return to rover in full autonomy.

## System behavior improvements

### Rover mode
- Continues driving forward when the path is clear.
- Turns away from non-blocking side obstacles.
- Stops for moving obstacles in the forward path.
- Backs up and requests drone mode for persistent static blockage when drone travel has a lower risk/cost score.
- Uses planner validity, lidar blockage, terrain danger, and obstacle size in the switch decision.

### Drone mode
- Ascends when the forward air path is blocked.
- Shifts laterally around side obstacles.
- Waits for a stable clear landing zone before landing.
- Returns to rover mode after landing when the area is safe again.

## Important limitation

Nav2 does not automatically consume arbitrary obstacle topics; it still needs an observation source configured in the costmap. This package publishes `/obstacles` as a filtered `LaserScan`, and `config/nav2_obstacle_layer_example.yaml` shows the intended Nav2 wiring.

## Recommended next upgrades

1. Feed real Nav2 costmap status into `path_blocked` and `route_valid`.
2. Replace the demo control bridge with real rover and drone motor commands.
3. Add altitude state and battery state to the decision score.
4. Add mission objectives such as `goal_reached`, `search_mode`, or `return_home`.
5. Log actions and scores to CSV or ROS bags for evaluation and demos.
