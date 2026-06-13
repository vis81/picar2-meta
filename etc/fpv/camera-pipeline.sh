#!/usr/bin/env bash
# rpicam-vid → UDP MPEG-TS → ffmpeg → RTSP push wrapper for MediaMTX.
#
# Why this shape, after several false starts:
#   - MediaMTX's bundled rpiCamera source ships libcamera 0.3.0 (predates
#     IMX500 sensor support) and is unusable here.
#   - rpicam-vid's raw H.264 stdout pipe doesn't carry SPS/PPS in a way
#     ffmpeg can discover on a pipe (probe times out before keyframes).
#   - rpicam-vid's --listen TCP mode allows only one connection slot.
#   - rpicam-vid's --libav-format rtsp errors with "Protocol not found"
#     because the embedded ffmpeg in rpicam-vid lacks the RTSP muxer.
#
# What works: rpicam-vid muxes to MPEG-TS over UDP using its embedded
# libav (MPEG-TS carries codec parameters in-band), and a separate ffmpeg
# reads that UDP socket and pushes RTSP to MediaMTX. Codec is copied; no
# re-encode.
#
# Env vars (defaults applied if unset):
#   FPV_WIDTH, FPV_HEIGHT, FPV_FPS, FPV_BITRATE
#   FPV_UDP_PORT      — bridge UDP port between rpicam-vid and ffmpeg
#   FPV_RTSP_URL      — MediaMTX's local RTSP target

set -uo pipefail

WIDTH="${FPV_WIDTH:-1280}"
HEIGHT="${FPV_HEIGHT:-720}"
FPS="${FPV_FPS:-30}"
BITRATE="${FPV_BITRATE:-2500000}"
# Camera is mounted upside-down on this robot; 180 rotates the image right-way-up.
# rpicam-vid only supports 0 or 180. Override at runtime with FPV_ROTATION=0.
ROTATION="${FPV_ROTATION:-180}"
UDP_PORT="${FPV_UDP_PORT:-9999}"
RTSP_URL="${FPV_RTSP_URL:-rtsp://127.0.0.1:8554/cam}"

log() { printf '[fpv-pipe] %s\n' "$*" >&2; }

CAM_PID=
FF_PID=

cleanup() {
    log "shutdown — killing children"
    [[ -n "$FF_PID"  ]] && kill -TERM "$FF_PID"  2>/dev/null || true
    [[ -n "$CAM_PID" ]] && kill -TERM "$CAM_PID" 2>/dev/null || true
    sleep 0.4
    [[ -n "$FF_PID"  ]] && kill -KILL "$FF_PID"  2>/dev/null || true
    [[ -n "$CAM_PID" ]] && kill -KILL "$CAM_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 1. ffmpeg listens for UDP MPEG-TS and pushes RTSP. Start ffmpeg first so
#    that when rpicam-vid begins sending, the socket is ready and no UDP
#    packets are lost during startup.
log "starting ffmpeg listening on udp://0.0.0.0:$UDP_PORT → $RTSP_URL"
ffmpeg -loglevel info -fflags nobuffer -flags low_delay \
       -probesize 5000000 -analyzeduration 5000000 \
       -i "udp://0.0.0.0:$UDP_PORT?listen=1" \
       -c:v copy -an -f rtsp -rtsp_transport tcp \
       "$RTSP_URL" 2>&1 | sed 's/^/[ffmpeg] /' &
FF_PID=$!

# Give ffmpeg's UDP socket time to be bound before the producer starts.
sleep 0.3

# 2. rpicam-vid produces MPEG-TS-over-UDP. The MPEG-TS muxer embeds
#    width/height/fps in PMT/PAT so ffmpeg can resolve them immediately
#    rather than waiting for a keyframe.
log "starting rpicam-vid → udp://127.0.0.1:$UDP_PORT (rotation=$ROTATION)"
rpicam-vid -t 0 --nopreview \
    --codec libav --libav-format mpegts \
    --width "$WIDTH" --height "$HEIGHT" --framerate "$FPS" \
    --bitrate "$BITRATE" --intra "$FPS" \
    --rotation "$ROTATION" \
    -o "udp://127.0.0.1:$UDP_PORT" \
    2> >(grep -v '^\[' >&2) &
CAM_PID=$!

# 3. Block on whichever child exits first; cleanup trap kills the other.
wait -n "$CAM_PID" "$FF_PID"
exit $?
