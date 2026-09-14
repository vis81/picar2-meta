# Nav2 upstream patches worth a look

Audit of `navigation2` main vs our base 1.3.12 (Jazzy), done 2026-09-13,
filtered to what touches the PICAR-2 stack. Our fork is `vis81/navigation2`,
branch `picar2/1.3.12`; packages currently built from it: `nav2_smac_planner`,
`nav2_regulated_pure_pursuit_controller`, `nav2_behaviors`. Anything else
needs its `COLCON_IGNORE` removed and a Pi rebuild (`colcon build
--packages-select <pkg>` inside the container, `install-docker`).

Commit hashes are on `origin/main` of the upstream repo (`/home/chia/ros2_src/navigation2`);
`git show <hash>` there, cherry-pick with `git fetch <path> <full sha>` into the fork
(expect conflicts in `parameter_handler.cpp` — main rewrote the parameter pattern;
port by hand, as DWPP was).

## Worth doing when the symptom shows

| upstream | what | why we'd want it | cost |
|---|---|---|---|
| **#5922** `3eba44b5` AMCL: save pose to file, reinitialise at last remembered pose | pose survives an AMCL/Pi restart | ends "set position on the phone after every restart"; a reboot mid-run re-localises itself | `nav2_amcl` into the fork build; small |
| **#5677** `7a6ceaff` RPP: prevent collision check premature termination with scaled lookahead | at low speed the velocity-scaled carrot is close, so the collision projection stops early and misses what is just beyond it | the crawl-into-the-wall case at the pinch | few lines in `collision_checker.cpp`, next to our `max_collision_check_dist` |
| **#6343** `0ae2f598` controller server: goal/plan TF-snapshot skew | plan and robot pose refreshed against one `map→odom` snapshot | our 33 "Unable to transform robot pose into global plan's frame" aborts over 24 bags (each one a `follow_path` abort with no obstacle); `amcl.yaml transform_tolerance 0.5` is the band-aid | `nav2_controller` into the fork build (~5 min on the Pi) |
| #6318 `70fd2f5e` RPP: path handler with small pose search distance | pruning bug when `max_robot_pose_search_dist` is small | only if we ever lower that parameter | small, but main's path handler lives in the controller server (#5446) |
| #5852 `0b93eeb3` AMCL: laser callback on the main thread | latency / CPU | goes with #5922 if AMCL is built anyway | small |

## CPU on the Pi (costmap_2d, larger package to carry)

| upstream | what | why |
|---|---|---|
| #6092 `8917a13e` ObstacleLayer world-distance pre-filter | drops points beyond `obstacle_max_range` before the expensive part | the ToF cloud + lidar marking at 10 Hz; 351 controller loop misses in 22 bags point at the Pi being close to the edge |
| #5933 `46172d71` new inflation layer, optional OpenMP | faster inflation | same |
| #6064 `65b8d22c` StaticLayer bounds for rolling costmaps | correctness/perf of the static layer in the local costmap | we run the static layer in the global costmap only; low |

## Config-only tries (no code)

- Smac `allow_primitive_interpolation: true` (#6183 flipped the default) — smoother
  paths between primitives; untested here.
- RPP `min_distance_to_obstacle` (main) is a **minimum** projection distance,
  not a cap — the opposite of what the corridor needed; our fork has
  `max_collision_check_dist` for that.

## Already ported (for the record)

- Smac analytic-expansion cusp scoring (ours, see `docs/nav2-smac-right-turn.md`).
- RPP `cost_lookahead_dist` (ours, off), `path_curvature_lookahead_dist` (ours,
  off — stalls the sim robot, unresolved), DWPP `use_dynamic_window` (#5783,
  hand-ported; used with `max_linear_accel 0.8` only), `max_collision_check_dist`
  (ours).
- Behaviors: BackUp/DriveOnHeading tolerate start contact for the whole
  backup distance (ours).
- Not ported: Smac `goal_heading_mode` (#4127) — tried both ALL_DIRECTION and a
  ±k-bin tolerance variant; on this map any freedom at wp1 makes the planner
  cut into the corridor and shunt. Dropped.

## Not relevant

Goal-checker family (AxisGoalChecker, progress goal checker, xy hysteresis) —
flow routes never reach the final goal checker. BT/Lyrical refactors, docking,
Smac2D/Lattice fixes.
