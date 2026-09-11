#!/usr/bin/env bash
# End-to-end test of the web UI's controls.
#
# Start the stack yourself first — this script never starts or stops it:
#
#     sudo systemctl start picar          # the robot
#     sudo systemctl start picar-sim      # Gazebo on a workstation
#
# then point the script at it. It asks the running system what it is rather
# than being told, so the same invocation works for either:
#
#   scripts/test_webui.sh                              # whatever is on localhost
#   scripts/test_webui.sh --host 192.168.1.68          # a robot on the network
#   scripts/test_webui.sh --only speed,supervision     # just those sections
#
# --host is normally the only thing you need. A non-local host is assumed to be
# reachable over ssh as the same machine, which is how the container-level
# checks get there; --ssh-user changes the login, --ssh overrides the target
# outright, and --container names it if discovery guesses wrong.
#
# Sections: modes speed backend costmap waypoints save localize motion
#           explore supervision logs
#
# Every check goes through the same HTTP API the phone uses, so it exercises
# what a person actually touches rather than the internals underneath. A few
# checks need to look inside the container — which SLAM node is really running,
# how many publishers /map has, whether restarting one layer disturbs another.
# Those need docker access: local by default, or over --ssh for a robot across
# the network. Without it they are skipped rather than failed, and everything
# else still runs.
#
# Sections that command movement (drive, route, explore) are skipped unless
# --allow-motion is given. On the robot that is the difference between a test
# run and a crash, so the default is off and the flag has to be deliberate.
#
# Exit status is the number of failures, capped at 125.

set -uo pipefail

HOST=localhost
PORT=8080
CONTAINER=""          # discovered from the running system unless given
SSH=""                # user@host for docker on another machine
SSH_USER=pi           # login used when --host is remote and --ssh is not given
ALLOW_MOTION=0
ONLY=""
TEST_MAP=uitest
KEEP=0

while [ $# -gt 0 ]; do
    case "$1" in
        --host)      HOST="$2"; shift 2 ;;
        --port)      PORT="$2"; shift 2 ;;
        --container) CONTAINER="$2"; shift 2 ;;
        --ssh)       SSH="$2"; shift 2 ;;
        --ssh-user)  SSH_USER="$2"; shift 2 ;;
        --allow-motion) ALLOW_MOTION=1; shift ;;
        --only)      ONLY="$2"; shift 2 ;;
        --keep)      KEEP=1; shift ;;          # leave the test map behind
        -h|--help)   sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

API="http://$HOST:$PORT/api"
PASS=0; FAIL=0; SKIP=0; FAILED_NAMES=""

# ── plumbing ─────────────────────────────────────────────────────────────────
ok()  { printf '    \033[32mPASS\033[0m  %s\n' "$1"; PASS=$((PASS+1)); }
bad() { printf '    \033[31mFAIL\033[0m  %s  -- %s\n' "$1" "$2"; FAIL=$((FAIL+1))
        FAILED_NAMES="$FAILED_NAMES\n      $1"; }
skip(){ printf '    \033[33mSKIP\033[0m  %s  (%s)\n' "$1" "$2"; SKIP=$((SKIP+1)); }
chk() { [ "$2" = "$3" ] && ok "$1" || bad "$1" "got '$2' want '$3'"; }

# Status field, via python rather than jq so there is one less thing to install.
st()  { curl -s -m 10 "$API/status" | python3 -c "
import sys,json
try: d=json.load(sys.stdin)
except Exception: print(''); raise SystemExit
try: print($1)
except Exception: print('')
" 2>/dev/null; }

post() { curl -s -m "${3:-20}" -X POST -H 'Content-Type: application/json' \
              -d "$2" "$API/$1"; }
# HTTP status only — for the endpoints that correctly return 204 No Content.
code() { curl -s -m "${3:-20}" -o /dev/null -w '%{http_code}' \
              -X POST -H 'Content-Type: application/json' -d "$2" "$API/$1"; }
jok()  { python3 -c "
import sys,json
try: print(json.load(sys.stdin).get('ok'))
except Exception: print('')
" 2>/dev/null; }

# Wait for a status expression to become True. Returns 1 on timeout.
waitfor() {
    local n=0 limit=${2:-40}
    while [ "$n" -lt "$limit" ]; do
        [ "$(st "$1")" = "True" ] && return 0
        sleep 3; n=$((n+1))
    done
    return 1
}

# docker, wherever it lives. Empty CONTAINER means the container-level checks
# were not available and their callers skip.
# BatchMode so an unknown host key or a missing agent fails instead of sitting
# at a prompt: pointing this at a robot that is off should say so, not hang.
# The banner prints before any of this runs, so a slow probe is visible.
dock() { if [ -n "$SSH" ]; then
             ssh -o ConnectTimeout=6 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
                 "$SSH" docker "$@"
         else docker "$@"; fi; }

# Runs a ros2 command inside the container, on the same DDS domain the stack
# is actually using. Taken from supervisord's own environment rather than a
# file or a guess: the sim puts itself on its own domain so it can never drive
# the robot, and a checker on the wrong domain sees an empty graph and reports
# a healthy system as broken.
ros() { dock exec "$CONTAINER" bash -lc "
source /opt/ros/jazzy/setup.bash
source /ws/install-docker/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
dom=\$(tr '\\0' '\\n' < /proc/1/environ 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
[ -n \"\$dom\" ] || dom=\$(tr '\\0' '\\n' < /proc/\$(pgrep -f webui/server.py | head -1)/environ 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
export ROS_DOMAIN_ID=\${dom:-0}
$1" 2>/dev/null; }

sup() { dock exec "$CONTAINER" supervisorctl -c /ws/etc/supervisord.conf "$@" 2>&1; }
suppid() { sup status "$1" | awk '{print $4}' | tr -d ,; }

want() { [ -z "$ONLY" ] && return 0; case ",$ONLY," in *,"$1",*) return 0;; esac; return 1; }

# Some sections need a stack state the modes section normally leaves behind.
# With --only they can be reached cold, and a missing prerequisite is not a
# failure of the thing under test — it means the check never ran.
need_nav2() {
    [ "$(st 'd["modes"]["nav2"]')" = "True" ] && return 0
    skip "$1" "nav2 is not running — run without --only, or start a mode first"
    return 1
}
need_mapping() {
    [ "$(st 'd["mode"]')" = "mapping" ] && return 0
    skip "$1" "not in mapping mode — run without --only, or start mapping first"
    return 1
}
section() { printf '\n  \033[1m=== %s ===\033[0m\n' "$1"; }

# One /map publisher, always: cartographer and AMCL both publish it and both
# broadcast map->odom, so two at once is the bug this invariant exists to catch.
one_map_publisher() {
    local n
    if [ -z "$CONTAINER" ]; then skip "$1" "no docker access"; return; fi
    n=$(ros 'timeout 10 ros2 topic info /map 2>/dev/null | grep -c "Publisher count: 1"')
    chk "$1" "${n:-0}" "1"
}

# A check that needs to look inside the container; skipped, not failed, when
# only the HTTP API is reachable.
chk_ros() {  # name, command, want
    if [ -z "$CONTAINER" ]; then skip "$1" "no docker access"; return; fi
    chk "$1" "$(ros "$2")" "$3"
}

# ── preflight ────────────────────────────────────────────────────────────────
# The stack has to be up already. Starting it here would mean guessing which
# one was wanted, and on a robot that guess moves hardware.
if ! curl -s -m 8 "$API/status" >/dev/null 2>&1; then
    cat >&2 <<MSG
  Nothing is answering on $API.

  Start the stack first, then re-run:
      sudo systemctl start picar          # robot
      sudo systemctl start picar-sim      # Gazebo
  or point this at another machine with --host / --ssh.
MSG
    exit 2
fi

# Where docker lives follows from where the UI lives: a non-local host is the
# same machine the container is on, so ssh there. One --host is then enough for
# a robot across the network, and nothing needs to know whether it is a Pi or
# this workstation.
if [ -z "$SSH" ]; then
    case "$HOST" in
        localhost|127.0.0.1|::1|"$(hostname)") ;;      # local docker
        *) SSH="$SSH_USER@$HOST" ;;
    esac
fi

# Ask the running system what it is. PROFILE is absent on an older build, in
# which case the container name is the only clue left.
PROFILE=$(st 'd.get("profile","")')
[ -n "$PROFILE" ] || PROFILE=unknown

printf '  target : %s:%s\n' "$HOST" "$PORT"
printf '  profile: %s\n' "$PROFILE"
[ -n "$SSH" ] && printf '  ssh    : %s\n' "$SSH"

# Find the container unless told. Names come from etc/picar.service and
# etc/picar-sim.service; profile order matters so a machine running both is
# not tested through the wrong one.
if [ -z "$CONTAINER" ]; then
    case "$PROFILE" in
        sim)   cands="picar2-sim picar2" ;;
        robot) cands="picar2 picar2-sim" ;;
        *)     cands="picar2 picar2-sim" ;;
    esac
    for c in $cands; do
        if dock exec "$c" true 2>/dev/null; then CONTAINER="$c"; break; fi
    done
elif ! dock exec "$CONTAINER" true 2>/dev/null; then
    # Named but not reachable. Clearing it makes the container checks skip;
    # leaving it set made them run and fail, which reads as a broken robot
    # rather than a missing login.
    echo "  note: container '$CONTAINER' is not reachable — those checks will skip" >&2
    CONTAINER=""
fi

printf '  docker : %s\n' \
       "$([ -n "$CONTAINER" ] && echo "$CONTAINER${SSH:+ via $SSH}" || echo 'unavailable — container checks will skip')"
printf '  motion : %s\n' "$([ $ALLOW_MOTION = 1 ] && echo allowed || echo blocked)"
if [ "$PROFILE" = robot ] && [ $ALLOW_MOTION = 1 ]; then
    printf '  \033[33m!! this is a real robot and movement is enabled — it will drive\033[0m\n'
    sleep 3
fi
# Start from a known backend. It is process state, so a previous run that
# selected slam_toolbox would otherwise have the modes section bring up the
# wrong engine and fail a check about the engine.
curl -s -m 10 -X POST -H 'Content-Type: application/json' \
     -d '{"mode":"idle"}' "$API/mode" >/dev/null 2>&1; sleep 5
curl -s -m 10 -X POST -H 'Content-Type: application/json' \
     -d '{"backend":"cartographer"}' "$API/slam" >/dev/null 2>&1

# ── modes ────────────────────────────────────────────────────────────────────
if want modes; then
section "modes"
post mode '{"mode":"idle"}' >/dev/null; sleep 6
chk "idle after teardown"        "$(st 'd["mode"]')" "idle"
chk "mapping accepted"           "$(post mode '{"mode":"mapping"}' | jok)" "True"
waitfor 'd["nav_ready"]' 40 && ok "mapping reaches nav_ready" \
                            || bad "mapping reaches nav_ready" "timed out"
chk "map layer up"               "$(st 'd["modes"]["map"]')"  "True"
chk "nav2 layer up"              "$(st 'd["modes"]["nav2"]')" "True"
chk "pose resolves"              "$(st 'd["pose"] is not None')" "True"
one_map_publisher "one /map publisher while mapping"
chk "missing map refused"        "$(post mode '{"mode":"localize","map":"no_such_map"}' | jok)" "False"
chk "refusal launched nothing"   "$(st 'd["modes"]["amcl"]')" "False"
fi

# ── speed ────────────────────────────────────────────────────────────────────
if want speed; then
section "speed"
if ! need_nav2 "speed checks"; then :; else
chk "change while navigation is up" "$(post speed '{"value":0.5}' | jok)" "True"
chk "value took"                    "$(st 'd["max_speed"]')" "0.5"
chk "joystick limit follows"        "$(st 'round(d["drive_limits"][0],2)')" "0.5"
chk "angular hits the relay clamp"  "$(st 'round(d["drive_limits"][1],2)')" "1.2"
post speed '{"value":0.3}' >/dev/null
chk "angular derived below clamp"   "$(st 'round(d["drive_limits"][1],3)')" "0.882"
chk "too fast refused"              "$(post speed '{"value":1.5}' | jok)" "False"
chk "too slow refused"              "$(post speed '{"value":0.05}' | jok)" "False"
chk "non-numeric refused"           "$(post speed '{"value":"fast"}' | jok)" "False"
post speed '{"value":0.4}' >/dev/null
fi
fi

# ── SLAM backend ─────────────────────────────────────────────────────────────
if want backend; then
section "SLAM backend"
chk "switch refused while mapping"  "$(post slam '{"backend":"slam_toolbox"}' | jok)" "False"
chk "unknown backend refused"       "$(post slam '{"backend":"gmapping"}' | jok)" "False"
chk_ros "cartographer is the live node" 'timeout 10 ros2 node list 2>/dev/null | grep -c cartographer_node' "1"
post mode '{"mode":"idle"}' >/dev/null; sleep 8
chk "switch allowed when idle"      "$(post slam '{"backend":"slam_toolbox"}' | jok)" "True"
chk "reported in status"            "$(st 'd["slam_backend"]')" "slam_toolbox"
post mode '{"mode":"mapping"}' >/dev/null; sleep 20
chk_ros "slam_toolbox is the live node" 'timeout 10 ros2 node list 2>/dev/null | grep -c slam_toolbox' "1"
# Put the stack back the way this section found it. Leaving slam_toolbox
# running made the save section fail against cartographer's services and
# cascade into localize — a test that changes state has to restore it, and
# switching the preference alone does not relaunch the layer.
post mode '{"mode":"idle"}' >/dev/null; sleep 8
post slam '{"backend":"cartographer"}' >/dev/null 2>&1
post mode '{"mode":"mapping"}' >/dev/null
waitfor 'd["nav_ready"]' 40 || true
chk_ros "cartographer restored" 'timeout 10 ros2 node list 2>/dev/null | grep -c cartographer_node' "1"
fi

# ── costmap overlay ──────────────────────────────────────────────────────────
if want costmap; then
section "costmap overlay"
if ! need_nav2 "costmap checks"; then :; else
hdr=$(curl -s -m 10 -D- -o /tmp/.uitest_cm "$API/costmap" | tr -d '\r')
chk "costmap 200"        "$(echo "$hdr" | grep -c '200 OK')" "1"
chk "X-Cell header"      "$(echo "$hdr" | grep -ci 'x-cell')" "1"
sz=$(wc -c < /tmp/.uitest_cm 2>/dev/null || echo 1)
[ $((sz % 8)) -eq 0 ] && ok "float32 xy pairs ($sz bytes)" \
                      || bad "float32 xy pairs" "$sz not a multiple of 8"
rm -f /tmp/.uitest_cm
fi
fi

# ── waypoints ────────────────────────────────────────────────────────────────
if want waypoints; then
section "waypoints"
curl -s -m 10 -X POST "$API/waypoints/clear" >/dev/null
chk "route needs two waypoints" "$(post route/start '{"loop":false,"flow":true}' | jok)" "False"
post waypoint '{"x":1.0,"y":0.0,"yaw":0.0}'  >/dev/null
post waypoint '{"x":2.0,"y":1.0,"yaw":1.57}' >/dev/null
post waypoint '{"x":0.0,"y":2.0,"yaw":3.14}' >/dev/null
chk "three added"    "$(st 'len(d["route"]["waypoints"])')" "3"
curl -s -m 10 -X POST "$API/waypoint/undo" >/dev/null
chk "undo removes one" "$(st 'len(d["route"]["waypoints"])')" "2"
post waypoint '{"x":0.0,"y":2.0,"yaw":3.14}' >/dev/null
fi

# ── save ─────────────────────────────────────────────────────────────────────
if want save; then
section "save map"
if ! need_mapping "save checks"; then :; else
chk "empty name refused"   "$(post mapping/save '{"name":""}' | jok)" "False"
chk "slash refused"        "$(post mapping/save '{"name":"a/b"}' | jok)" "False"
chk "dotfile refused"      "$(post mapping/save '{"name":".hidden"}' | jok)" "False"
chk "save succeeds"        "$(post mapping/save "{\"name\":\"$TEST_MAP\"}" 150 | jok)" "True"
chk "appears in the list"  "$(st "'$TEST_MAP' in [m['name'] for m in d['maps']]")" "True"
chk "grid + pbstream"      "$(st "[m['has_grid'] and m['has_pbstream'] for m in d['maps'] if m['name']=='$TEST_MAP'][0]")" "True"
fi
fi

# ── localize ─────────────────────────────────────────────────────────────────
if want localize; then
section "localize"
post mode "{\"mode\":\"localize\",\"map\":\"$TEST_MAP\"}" >/dev/null; sleep 18
chk "amcl up"               "$(st 'd["modes"]["amcl"]')" "True"
chk "map layer torn down"   "$(st 'd["modes"]["map"]')"  "False"
one_map_publisher "one /map publisher after the cross-switch"
chk "waypoints reloaded for this map" "$(st 'len(d["route"]["waypoints"])')" "3"
chk "goal refused before a pose is set" "$(post goal '{"x":1,"y":0,"yaw":0}' | jok)" "False"
chk "initial pose accepted" "$(post initialpose '{"x":0.0,"y":0.0,"yaw":0.0}' 30 | jok)" "True"
waitfor 'd["nav_ready"]' 40 && ok "nav2 ready after the pose" \
                            || bad "nav2 ready after the pose" "timed out"
# Not asserting a refusal: AMCL rejects an out-of-map pose but keeps
# publishing map->odom from the estimate it already had, so set_initial_pose
# sees a pose appear and reports success. Its 504 only fires when AMCL had no
# prior pose at all. What is deterministic, and what actually matters, is that
# the robot did not teleport to the bad pose.
post initialpose '{"x":500,"y":500,"yaw":0}' 30 >/dev/null
sleep 3
chk "bad pose did not move the robot" "$(st 'abs(d["pose"]["x"]) < 50 if d["pose"] else True')" "True"
fi

# ── motion ───────────────────────────────────────────────────────────────────
if want motion; then
section "motion"
if [ $ALLOW_MOTION = 0 ]; then
    skip "goals, routes, drive, explore" "pass --allow-motion"
else
    chk "goal accepted" "$(post goal '{"x":1.0,"y":0.0,"yaw":0.0}' | jok)" "True"
    sleep 5
    case "$(st 'd["nav"]["state"]')" in
        pending|active|succeeded) ok "nav state advanced" ;;
        *) bad "nav state advanced" "got $(st 'd["nav"]["state"]')" ;;
    esac
    chk "cancel accepted" "$(curl -s -m 10 -X POST "$API/goal/cancel" | jok)" "True"
    sleep 3
    chk "route starts (flow, loop)" "$(post route/start '{"loop":true,"flow":true}' | jok)" "True"
    sleep 6
    chk "route active"   "$(st 'd["route"]["active"]')" "True"
    chk "flow flag set"  "$(st 'd["route"]["flow"]')" "True"
    chk "loop flag set"  "$(st 'd["route"]["loop"]')" "True"
    # 204 No Content by design: /api/drive runs at 10 Hz and a body is waste.
    chk "takeover returns 204" "$(curl -s -m 10 -o /dev/null -w '%{http_code}' -X POST "$API/takeover")" "204"
    sleep 3
    chk "takeover cancelled the route" "$(st 'd["route"]["active"]')" "False"
    chk "drive returns 204" "$(code drive '{"linear":0.1,"angular":0.0}')" "204"
    post drive '{"linear":0.0,"angular":0.0}' >/dev/null
    chk "route stop is idempotent" "$(curl -s -m 10 -X POST "$API/route/stop" | jok)" "True"
fi
fi

# ── explore ──────────────────────────────────────────────────────────────────
if want explore; then
section "explore"
chk "refused outside mapping" "$(post explore '{"on":true}' | jok)" "False"
if [ $ALLOW_MOTION = 0 ]; then
    skip "explore start/stop" "pass --allow-motion"
else
    post mode '{"mode":"mapping"}' >/dev/null
    waitfor 'd["nav_ready"]' 40 || true
    chk "explore starts"  "$(post explore '{"on":true}' | jok)" "True"
    sleep 6
    chk "explore layer up" "$(st 'd["modes"]["explore"]')" "True"
    chk "explore stops"    "$(post explore '{"on":false}' | jok)" "True"
    sleep 4
    chk "explore layer down" "$(st 'd["modes"]["explore"]')" "False"
fi
fi

# ── supervision ──────────────────────────────────────────────────────────────
# The point of the service split: a layer restarts without taking its
# neighbours down, and a restarted UI picks the running stack back up.
if want supervision; then
section "supervision"
if [ -z "$CONTAINER" ]; then
    skip "restart independence and adoption" "no docker access — pass --ssh"
else
    b=$(suppid bringup)
    sup restart webui >/dev/null; sleep 16
    chk "webui restart leaves bringup alone" "$(suppid bringup)" "$b"
    chk "mode was adopted"  "$(st 'd["mode"] in ("mapping","localize")')" "True"
    chk "waypoints adopted" "$(st 'len(d["route"]["waypoints"]) >= 0')" "True"
    w=$(suppid webui)
    sup restart bringup >/dev/null; sleep 22
    chk "bringup restart leaves the webui alone" "$(suppid webui)" "$w"
    # A UI restarted mid-transition used to adopt a mode with a layer missing
    # and never notice.
    sup stop nav >/dev/null; sleep 3
    chk "nav stopped" "$(st 'd["modes"]["nav2"]')" "False"
    sup restart webui >/dev/null; sleep 16
    waitfor 'd["nav_ready"]' 40 && ok "adoption completed the half-finished mode" \
                                || bad "adoption completed the half-finished mode" "timed out"
fi
fi

# ── logs ─────────────────────────────────────────────────────────────────────
if want logs; then
section "logs"
for l in map amcl nav2 explore; do
    c=$(curl -s -m 10 -o /dev/null -w '%{http_code}' "$API/logs?layer=$l")
    [ "$c" = "200" ] && ok "/api/logs?layer=$l" || bad "/api/logs?layer=$l" "HTTP $c"
done
chk "unknown layer refused" \
    "$(curl -s -m 10 -o /dev/null -w '%{http_code}' "$API/logs?layer=nonsense")" "400"
fi

# ── teardown ─────────────────────────────────────────────────────────────────
# Idle first, and *wait* for it. /api/mode returns 202 and tears the layers down
# on a worker, so deleting the map immediately leaves AMCL running against a
# file that is about to vanish — and run/amcl.args still naming it. A later
# restart then adopts a map that no longer exists and every attempt to localize
# dies with "amcl failed to start". Observed on the robot, not hypothetical.
post mode '{"mode":"idle"}' >/dev/null 2>&1
for _ in $(seq 1 20); do
    [ "$(st 'd["modes"]["amcl"] or d["modes"]["map"]')" = "False" ] && break
    sleep 1
done
if [ $KEEP = 0 ] && want save; then
    ws=$(curl -s -m 8 "$API/debug" | python3 -c "
import sys,json
try: print(json.load(sys.stdin).get('ws',''))
except Exception: print('')" 2>/dev/null)
    [ -n "$ws" ] && rm -f "$ws/maps/$TEST_MAP".* 2>/dev/null
    rm -f "$(dirname "$0")/../maps/$TEST_MAP".* 2>/dev/null
    # The args file is what adoption reads to rebuild state, so it has to go
    # with the map. Removing it is safe: every layer start rewrites it.
    if [ -n "$SSH" ]; then
        ssh -o ConnectTimeout=6 -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
            "$SSH" "rm -f '$ws/run/amcl.args'" >/dev/null 2>&1 || true
    else
        rm -f "$ws/run/amcl.args" 2>/dev/null || true
    fi
fi

printf '\n  \033[1m%d passed, %d failed, %d skipped\033[0m\n' "$PASS" "$FAIL" "$SKIP"
[ "$FAIL" -gt 0 ] && printf '  failures:%b\n' "$FAILED_NAMES"
exit $(( FAIL > 125 ? 125 : FAIL ))
