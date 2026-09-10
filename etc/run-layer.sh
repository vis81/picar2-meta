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

# A simulator on the same DDS domain as the robot is a hazard, not a
# convenience: both are on the same LAN, topic names are identical, and the
# sim's /cmd_vel would drive the real vehicle. Put sim on its own domain, the
# same reasoning the benchmark uses.
if [ "$profile" = "sim" ]; then
    export ROS_DOMAIN_ID="${PICAR_DOMAIN_ID:-1}"
fi

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

# Gazebo needs two things the robot does not: third-party world files usually
# lack the system plugins that make them load at all (scripts/patch_gz_world.py
# injects them and prints a patched copy), and the model path has to be exported
# before launch. Doing it here rather than in a make eval is what lets the sim
# run as a plain service.
if [ "$profile" = "sim" ] && [ "$layer" = "bringup" ]; then
    export GZ_SIM_RESOURCE_PATH="${PICAR_GZ_RESOURCE_PATH:-/ws/install-docker/picar2_description/share}"
    world="${PICAR_WORLD:-/ws/install-docker/picar2_bringup/share/picar2_bringup/worlds/room.sdf}"
    if [ -r "$world" ]; then
        patched="$(python3 /ws/scripts/patch_gz_world.py "$world")" || patched="$world"
        # Only when the args file did not already name a world, so an explicit
        # choice there still wins.
        case "$args" in
            *world:=*) ;;
            *) args="$args world:=$patched" ;;
        esac
    else
        echo "run-layer.sh: world '$world' not readable; letting sim.launch.py pick its default" >&2
    fi
fi

case "$layer" in
    webui)
        # The UI is a ROS node too: it looks up map->base_footprint and reads
        # /map. Under Gazebo those are stamped with /clock, so on the wall
        # clock the lookup never resolves and the UI sits on "no robot
        # position" forever with a perfectly good map on screen.
        # PICAR_PROFILE is already exported; server.py reads it and sets
        # use_sim_time itself. Passing --ros-args here does not work: it has
        # its own argparse and rejects arguments it does not know.
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
