#!/usr/bin/env bash
# One entry point for every supervised layer.
#
# supervisord programs have fixed commands, but two of these are parameterised:
# `map` picks between cartographer and slam_toolbox, and `amcl` needs whichever
# saved map is being loaded. Rather than rewriting supervisord.conf and calling
# reread/update on every mode switch, each layer reads its arguments from
# /ws/run/<layer>.args, which the web UI writes immediately before starting it.
#
# The args file holds everything after `ros2 launch picar2_bringup`, launch file
# included, so the same mechanism selects the SLAM backend and passes it options:
#
#   run/map.args      cartographer.launch.py
#                     slam.launch.py
#   run/amcl.args     amcl.launch.py map_yaml:=/ws/maps/F1.yaml
#
# Deliberately not `set -u`: the args are optional and an unset variable here
# would fail a layer that simply has no options.
set -eo pipefail

layer="${1:?usage: run-layer.sh <layer>}"

source /opt/ros/jazzy/setup.bash
source /ws/install-docker/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///ws/cyclonedds.xml
export PICAR_WS=/ws

# robot | sim. The same six programs run either way — only the bottom layer
# changes, from the real hardware to Gazebo — so the profile lives here rather
# than in a second supervisord.conf that would drift from the first.
profile="${PICAR_PROFILE:-}"
if [ -z "$profile" ] && [ -r /ws/run/profile ]; then
    profile="$(cat /ws/run/profile)"
fi
profile="${profile:-robot}"

# Defaults for the layers whose arguments never vary, so a missing args file is
# normal rather than a failure. The parameterised layers have no default: being
# asked to start one without arguments is a bug in the caller, and guessing a
# map would be worse than refusing.
default_args() {
    case "$1" in
        bringup)
            if [ "$profile" = "sim" ]; then
                # No lidar/joy switches here: sim.launch.py takes a world and a
                # spawn pose instead, and sets use_sim_time for the whole graph
                # itself. Override the world through /ws/run/bringup.args.
                echo "sim.launch.py headless:=${PICAR_HEADLESS:-false}"
            else
                echo "picar2.launch.py lidar:=${PICAR_LIDAR:-ld19} use_joy:=${PICAR_USE_JOY:-false}"
            fi ;;
        nav)     echo "nav2.launch.py" ;;
        explore) echo "explore.launch.py" ;;
        *)       echo "" ;;
    esac
}

# cartographer, slam_toolbox, amcl, nav2 and explore all declare use_sim_time;
# picar2.launch.py and sim.launch.py do not — the first is real hardware and the
# second sets it for everything downstream. Appended rather than baked into the
# args file so one saved route or map works in both profiles.
sim_time_arg() {
    case "$1" in
        map|amcl|nav|explore)
            if [ "$profile" = "sim" ]; then echo "use_sim_time:=true"; fi ;;
    esac
    # Always succeed. `[ … ] && echo` would return non-zero on the robot, where
    # the test is false, and under `set -e` the calling command substitution
    # inherits that and kills the script — so nav and explore would exit 1
    # without ever launching. Caught in test; it would not have been obvious
    # on the robot, where the layer simply never appears.
    return 0
}

args_file="/ws/run/${layer}.args"
if [ -r "$args_file" ]; then
    args="$(cat "$args_file")"
else
    args="$(default_args "$layer")"
fi

case "$layer" in
    webui)
        exec python3 /ws/etc/webui/server.py
        ;;
    map|amcl)
        if [ -z "$args" ]; then
            echo "run-layer.sh: $layer needs $args_file (which backend, which map)" >&2
            exit 64                      # EX_USAGE: a caller bug, not a crash
        fi
        ;;
esac

if [ -z "$args" ]; then
    echo "run-layer.sh: unknown layer '$layer'" >&2
    exit 64
fi

extra="$(sim_time_arg "$layer")"
echo "run-layer.sh: [$profile] $layer -> ros2 launch picar2_bringup $args $extra"
# Unquoted on purpose: the args hold several whitespace-separated arguments and
# must word-split.
# shellcheck disable=SC2086
exec ros2 launch picar2_bringup $args $extra
