# Nav2 tuning guide (external reference)

Source: <https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/>
Saved 2026-09-19. The author's own values are for a different (differential-drive,
depth-camera-equipped) robot — read as "what this parameter does and roughly
what range is sane," not as settings to copy onto PICAR-2. Cross-check anything
used against `picar2_bringup/config/nav2.yaml`'s own comments, which explain
*our* measured reasons for each value.

The source page was truncated before Smoother Server, Behavior Server,
Recovery Server and SLAM Toolbox sections — not captured here either.

## AMCL

| Parameter | Default | Author | Notes |
|---|---|---|---|
| alpha1-5 | 0.2 each | 0.2 each | process-noise / motion-model uncertainty |
| robot_model_type | DifferentialMotionModel | OmniMotionModel | match wheel config |
| min_particles / max_particles | 500 / 2000 | same | more = better accuracy, more CPU |
| pf_err / pf_z | 0.05 / 0.99 | same | population error/density |
| resample_interval | 1 | 1 | every movement |
| laser_model_type | likelihood_field | same | fastest of the three models |
| max_beams | 60 | 60 | more = accuracy vs CPU |
| laser_max_range / laser_min_range | 100.0 / -1.0 | same | -1.0 = use sensor's own min |
| laser_likelihood_max_dist | 2.0 | 2.0 | obstacle inflation in the likelihood model |
| do_beamskip | false | false | usually unneeded with likelihood_field |
| beam_skip_threshold/distance/error_threshold | 0.3/0.5/0.9 | same | |
| z_hit/z_short/z_max/z_rand | 0.5/0.005/0.05/0.5 | z_short 0.05 | raise z_short if frequent short readings |
| sigma_hit / lambda_short | 0.2 / 0.1 | same | |
| update_min_d | 0.25 m | **0.05 m** | smaller suits slow robots |
| update_min_a | 0.2 rad | **0.05 rad** | smaller = less rotation needed to trigger update |
| save_pose_rate | 0.5 Hz | 0.5 Hz | |
| tf_broadcast | true | true | publishes map→odom |
| transform_tolerance | 1.0 s | 1.0 s | |
| recovery_alpha_fast/slow | 0.0/0.0 | 0.0/0.0 | >0 enables particle-spread recovery (PICAR-2 keeps these 0 deliberately - see `amcl.yaml`) |

## BT Navigator

| Parameter | Default | Author | Notes |
|---|---|---|---|
| odom_topic | odom | /odometry/filtered | use the filtered source |
| bt_loop_duration | 10 ms | 10 ms | lower = more responsive, more CPU |
| default_server_timeout | 20 ms | 20 ms | |
| wait_for_service_timeout | 1000 ms | 1000 ms | |
| action_server_result_timeout | 900 s | 900 s | |
| transform_tolerance | 0.1 s | 0.1 s | |

## Controller Server

| Parameter | Default | Author | Notes |
|---|---|---|---|
| controller_frequency | 20 Hz | **5 Hz** | lower reduces CPU load |
| costmap_update_timeout | 0.30 s | 0.30 s | |
| failure_tolerance | 0.0 s | **0.3 s** | -1.0 = never give up |
| min_x/y/theta_velocity_threshold | 0.0001 | 0.001 | odometry-noise filtering |
| progress_checker required_movement_radius / movement_time_allowance | 0.5 m / 10 s | same | |
| general_goal_checker.xy_goal_tolerance | 0.25 m | **0.35 m** | |
| general_goal_checker.yaw_goal_tolerance | 0.25 rad | **0.50 rad** | too low causes goal "dancing" |
| FollowPath.plugin | dwb_core | MPPI or RotationShim | |

### MPPI controller

| Parameter | Default | Author | Notes |
|---|---|---|---|
| time_steps | 56 | **15** | fewer = less compute |
| model_dt | 0.05 s | **0.2 s** | larger saves CPU |
| batch_size | 1000 | **10000** | more parallel rollouts, better paths |
| vx_std/vy_std/wz_std | 0.2/0.2/0.2 | wz_std **0.4** | |
| vx_max/min, vy_max, wz_max | 0.5/-0.35/0.5/1.9 | vx_min **0.0** (no reverse) | |
| ax/ay_max, az_max | 3.0/3.0/3.5 | same | |
| iteration_count | 1 | 1 | rarely needs >1 with large batch |
| temperature / gamma | 0.3 / 0.015 | same | selection randomness / cost sensitivity |
| Critics: Constraint, Cost, Goal, GoalAngle, PathAlign, PathAngle, PathFollow, PreferForward, Twirling | see upstream defaults | CostCritic consider_footprint=**true**; PathAlign weight **14.0** (was 10); PathAngle weight **2.0** (was 2.2) | footprint-aware collision costs CPU but is more realistic; higher PathAlign weight = stricter path adherence |

### Regulated Pure Pursuit (closest to what PICAR-2 runs — compare against our fork's own tuning notes in nav2.yaml before changing anything)

| Parameter | Default | Author | Notes |
|---|---|---|---|
| desired_linear_vel | 0.5 | 0.4 | |
| lookahead_dist / min / max | 0.6/0.3/0.9 | 0.7/0.5/**0.7** | caps over-anticipation |
| lookahead_time | 1.5 s | 1.5 s | |
| rotate_to_heading_angular_vel | 1.8 rad/s | **0.375** | much lower for stability (n/a to us - Ackermann can't rotate in place) |
| use_velocity_scaled_lookahead_dist | false | **true** | |
| min_approach_linear_velocity | 0.05 | 0.05 | |
| approach_velocity_scaling_dist | 1.0 m | **0.6 m** | starts the approach ramp earlier |
| use_cost_regulated_linear_velocity_scaling | false | **true** | |
| regulated_linear_scaling_min_radius | 0.9 m | 0.85 m | |
| regulated_linear_scaling_min_speed | 0.25 | 0.25 | |
| curvature_lookahead_dist (use_fixed_curvature_lookahead) | 1.0 m | 0.6 m | dynamic adjustment usually preferred instead |
| use_rotate_to_heading / min_angle | true / 0.785 rad | same | n/a to Ackermann |
| max_angular_accel | 3.2 rad/s² | 3.2 rad/s² | compare to our `max_angular_accel: 1.0` — theirs is a holonomic/diff-drive robot with no steering-angle bottleneck, ours is Ackermann and this value interacts with the steering LUT slew |
| max_allowed_time_to_collision_up_to_carrot | 1.0 s | **1.5 s** | more reaction time (we use 1.0, tuned against our own corridor-pinch measurement) |
| use_cancel_deceleration / cancel_deceleration | false / 3.2 | false / 3.2 | |
| max_robot_pose_search_dist | 10.0 m | 10.0 m | |

## Local costmap

| Parameter | Default | Author | Notes |
|---|---|---|---|
| update_frequency / publish_frequency | 5 / 5 Hz | same | |
| global_frame | map | **odom** | local consistency independent of the global map |
| rolling_window | false | **true** | |
| width / height | 5 / 5 m | same | (PICAR-2 uses 3 m, tuned against the #22 raytrace/marking-latency trade-off) |
| resolution | 0.1 m | **0.05 m** | more detail, more CPU |
| robot_radius | 0.1 | 0.15 | |
| layer order | — | obstacle, voxel, range_sensor, denoise, inflation | order matters |
| obstacle_layer scan.raytrace_min_range / obstacle_min_range | 0.0 | **0.20 m** each | compensates for the robot's own body in the scan |
| obstacle_layer scan.clearing | false | **true** | |
| voxel_layer (depth camera) | — | z_voxels 16, obstacle_max_range 1.25 m (D435-specific), obstacle_min_range 0.05 m | tune ranges to the actual sensor |
| range_sensor_layer | enabled | **disabled** in author's setup | |
| denoise_layer minimal_group_size / connectivity | 2 / 8 | same | "helpful but may slow things down with long-range LIDAR or large maps" |
| inflation_radius | 0.55 m | **1.75 m** | author cites this + cost_scaling_factor 2.58 as widely-repeated "magic numbers" for indoor robots — PICAR-2 uses 0.33-0.45 m, deliberately much smaller (narrow corridors); don't import 1.75 m blindly |
| cost_scaling_factor | 1.0 | **2.58** | must match the local costmap's own value in RPP's `inflation_cost_scaling_factor` (PICAR-2 note: ours is 5.0, matched intentionally — see nav2.yaml comment) |

## Global costmap

| Parameter | Default | Author | Notes |
|---|---|---|---|
| publish_frequency | 1 Hz | **5 Hz** | |
| robot_radius | 0.1 | 0.15 | |
| resolution | 0.1 m | **0.05 m** | |
| track_unknown_space | false | **true** | |
| layer order | — | static, obstacle, voxel, range_sensor, inflation | |
| obstacle_layer scan.raytrace_min_range / obstacle_min_range | 0.0 | **0.20 m** each | |
| obstacle_layer scan.clearing | false | **true** | (PICAR-2's #22 fix went the other way on the *max* range instead — see nav2.yaml `raytrace_max_range: 2.5`) |
| voxel_layer (depth camera) | — | raytrace/obstacle ranges 0.05-1.25 m | |
| inflation_radius / cost_scaling_factor | 0.55 / 1.0 | **1.75 / 2.58** | same "magic numbers" as local; same caveat |

## Map saver

| Parameter | Default | Author |
|---|---|---|
| save_map_timeout | 2.0 s | 5.0 s |
| free_thresh_default / occupied_thresh_default | 0.25 / 0.65 | same |

## Planner server

| Parameter | Default | Author | Notes |
|---|---|---|---|
| expected_planner_frequency | 20 Hz | 20 Hz | |
| GridBased.plugin | NavfnPlanner | NavfnPlanner (`::` syntax preferred) | PICAR-2 uses SmacPlannerHybrid instead — Ackermann can't use a grid planner that ignores heading |
| GridBased.tolerance | 0.5 m | 0.5 m | |
| GridBased.use_astar | false | false | Dijkstra fine for most cases |
| GridBased.allow_unknown | true | true | |

## Author's general principles

1. Tuning is "more art than science" — trial-and-error per robot/environment.
2. Changing one component's rate often forces changes elsewhere (e.g. controller
   frequency vs costmap update rate).
3. Sensor-specific parameters (LIDAR min range, depth-camera max range) need
   recalibrating per sensor model, not copied.
4. inflation_radius=1.75 m / cost_scaling_factor=2.58 are repeated across many
   guides as generic "good defaults" for indoor robots — evaluate against your
   own corridor widths before adopting; PICAR-2's corridors are narrower than
   that value assumes.
5. Higher goal tolerances reduce goal-reaching oscillation; too tight causes
   "dancing" at the goal.
6. Combining sensor sources (LIDAR + depth + ultrasonic) with a denoise layer
   improves robustness — n/a to PICAR-2's current sensor set (LD19 + SEN0628 ToF).

## Not relevant to PICAR-2 as written

- Everything holonomic/diff-drive-specific (`rotate_to_heading_angular_vel`,
  `use_rotate_to_heading` at non-trivial angles) — Ackermann cannot spin in
  place, see `CLAUDE.md` Navigation section.
- The RealSense/voxel-layer depth-camera tuning — we use `/sen0628/pointcloud`
  from a matrix ToF, different geometry and range.
- MPPI section — we run RegulatedPurePursuit (our own fork).
