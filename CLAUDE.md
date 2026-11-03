I need an automatic emergency brake in the planner (ros_ws/src/planner_node/src/planner_gap_follow.cpp). Follow CLAUDE.md if it's in the repo.

The problem: when the car hits a wall in the sim, it doesn't stop. It keeps pushing forward slowly, even after contact. When something is closer than a set distance in front of the car, throttle must go to ZERO and stay there.

Step 1: find out why it creeps (report before coding)
Read the code and the sim launch file (ros_ws/src/neo_telemetry/launch/neo_telemetry_sim_launch.py) and tell me what the car actually sends when it's in the Brake state or right after a collision. Check at least these:
- what sim_brake_throttle is set to, and whether it's positive (forward)
- whether sim_throttle_floor or any minimum-throttle logic keeps it above zero
- how readings below range_min or zero/inf/NaN are handled. If the lidar returns nothing when the car is touching a wall and that gets cleaned to "open at 10 m," the car thinks the path is clear. That could be the bug.
Show me the evidence. Then implement the brake below, and fix whatever is causing the creep if the brake alone doesn't cover it.

Step 2: the emergency brake
- Always on, independent of brake_mode or any speed mode. It's the last thing applied in the output layer, so it overrides every other throttle decision.
- Trigger: any lidar point in front of the car, within the car's width (car half-width 0.15 m plus a margin) and closer than estop_distance, measured forward from the lidar.
- For this check, a reading below range_min (or zero) in the forward cone counts as an obstacle right in front, NOT as open space. Use the raw scan for this, not the cleaned ranges.
- When triggered: sim mode sends throttle exactly 0.0. VESC mode uses the existing real brake path (check how the code brakes on the real car; don't invent a new command).
- Latch: stay stopped until the closest in-path point is farther than estop_release_distance continuously for estop_hold_time.
- Steering keeps following the planner, so the car is aimed at open space when it starts moving again.

Parameters (declare in the constructor, log at startup, add to the sim launch file):
- estop_distance: 0.30 m
- estop_release_distance: 0.50 m (must be >= estop_distance; clamp and warn if not)
- estop_lateral_margin: 0.05 m
- estop_hold_time: 0.5 s

Debug:
- publish std_msgs/Bool on /planner/debug/estop
- RCLCPP_WARN_THROTTLE (1 s) when the e-stop is active, with the closest in-path distance

Constraints:
- no per-frame heap allocation in the scan callback
- don't change VESC defaults or the existing e-stop/safety logic besides this
- keep comments short and casual, matching the file's style
- rebuild with `docker compose restart onboard`, check the logs for "Failed <<<", and confirm the new parameters are logged at startup

When done, tell me what caused the creeping, what you changed, and anything you couldn't verify without watching the car drive. and explain the code 