#!/usr/bin/env bash
# Install MediaMTX on the Raspberry Pi OS host (NOT inside the picar2 docker)
# so it can talk to libcamera directly. Sets up a systemd service that reads
# the config from this workspace's etc/fpv/mediamtx.yml, plus a self-signed
# TLS cert because WebXR on Quest 3 requires a secure context.
#
# Usage:
#   sudo ./install-mediamtx.sh                    # install + enable (don't start)
#   sudo VERSION=1.9.3 ./install-mediamtx.sh      # pin a version
#   sudo ./install-mediamtx.sh --uninstall        # remove everything
#
# Idempotent. Safe to re-run.

set -euo pipefail

VERSION="${VERSION:-1.9.3}"
WS_DIR="${WS_DIR:-/home/pi/picar_ws}"
CONFIG_SRC="${WS_DIR}/etc/fpv/mediamtx.yml"
PUBLIC_SRC="${WS_DIR}/etc/fpv/public"
CONFIG_DIR=/etc/mediamtx
BINARY=/usr/local/bin/mediamtx
UNIT=/etc/systemd/system/mediamtx.service
CERT="${CONFIG_DIR}/fpv.crt"
KEY="${CONFIG_DIR}/fpv.key"

UI_UNIT=/etc/systemd/system/picar-fpv-ui.service
UI_SCRIPT="${WS_DIR}/etc/fpv/serve.py"

UNINSTALL=0
[[ "${1:-}" = "--uninstall" ]] && UNINSTALL=1

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo." >&2; exit 1
fi

log() { printf '[fpv] %s\n' "$*"; }

uninstall() {
    log "removing mediamtx + UI services"
    systemctl disable --now mediamtx picar-fpv-ui 2>/dev/null || true
    rm -f "$UNIT" "$UI_UNIT" "$BINARY"
    rm -rf "$CONFIG_DIR"
    systemctl daemon-reload
    log "done"
}

if [[ $UNINSTALL -eq 1 ]]; then
    uninstall
    exit 0
fi

if [[ ! -f "$CONFIG_SRC" ]]; then
    echo "Config not found at $CONFIG_SRC — set WS_DIR if running from another path." >&2
    exit 1
fi

# 1. libcamera userland so MediaMTX's rpiCamera source can talk to the sensor.
#    Skip the apt dance entirely if the tooling is already on disk — important
#    because broken third-party APT sources commonly fail `apt-get update` on
#    long-lived Pi systems, and we don't want that to abort the installer.
if command -v rpicam-hello >/dev/null 2>&1 || command -v libcamera-hello >/dev/null 2>&1; then
    log "libcamera tooling already installed — skipping apt"
else
    log "installing libcamera-apps + rpicam-apps (Pi OS Bookworm names both)"
    # Don't fail on unrelated broken sources; we only need the main Pi OS repo.
    apt-get update -qq || log "apt-get update returned non-zero (ignoring; broken third-party source likely)"
    apt-get install -y --no-install-recommends \
        rpicam-apps libcamera-apps libcamera-tools curl ca-certificates >/dev/null \
        || apt-get install -y --no-install-recommends libcamera-apps libcamera-tools curl ca-certificates >/dev/null \
        || { echo "Failed to install libcamera tooling." >&2; exit 1; }
fi
# curl/openssl are needed below — install if missing (cheap, no apt churn if present).
for tool in curl openssl; do
    command -v "$tool" >/dev/null 2>&1 || apt-get install -y --no-install-recommends "$tool" >/dev/null
done

# 2. Download MediaMTX binary if missing or version mismatch.
need_install=1
if [[ -x "$BINARY" ]]; then
    cur="$("$BINARY" --version 2>/dev/null | head -n1 | awk '{print $NF}' | sed 's/^v//')"
    if [[ "$cur" == "$VERSION" ]]; then
        log "mediamtx $VERSION already installed"
        need_install=0
    else
        log "found mediamtx $cur — replacing with $VERSION"
    fi
fi

if [[ $need_install -eq 1 ]]; then
    arch="$(dpkg --print-architecture)"
    case "$arch" in
        arm64) mtx_arch=linux_arm64v8 ;;
        armhf) mtx_arch=linux_armv7 ;;
        amd64) mtx_arch=linux_amd64 ;;
        *) echo "unsupported arch: $arch" >&2; exit 1 ;;
    esac
    url="https://github.com/bluenviron/mediamtx/releases/download/v${VERSION}/mediamtx_v${VERSION}_${mtx_arch}.tar.gz"
    log "downloading $url"
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' EXIT
    curl -fsSL "$url" -o "$tmp/mtx.tgz"
    tar -xzf "$tmp/mtx.tgz" -C "$tmp" mediamtx
    install -m 0755 "$tmp/mediamtx" "$BINARY"
fi

# 3. Self-signed cert for WebXR's secure-context requirement.
mkdir -p "$CONFIG_DIR"
if [[ ! -s "$CERT" || ! -s "$KEY" ]]; then
    log "generating self-signed TLS cert for WebXR (10-year validity)"
    # SAN list: all known Pi IPs at install time, plus a wildcard hostname.
    sans="DNS:rpi4.local,DNS:localhost,IP:127.0.0.1"
    for ip in $(hostname -I); do sans="$sans,IP:$ip"; done
    openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
        -keyout "$KEY" -out "$CERT" \
        -subj "/CN=picar-fpv" \
        -addext "subjectAltName=$sans" 2>/dev/null
    chmod 644 "$CERT"
    chmod 640 "$KEY"
fi

# 4. systemd unit. Runs as root so V4L2 + /dev/video* access just works.
log "writing $UNIT"
cat > "$UNIT" <<EOF
[Unit]
Description=MediaMTX (picar2 FPV WebRTC server)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=$BINARY $CONFIG_SRC
Restart=on-failure
RestartSec=2
# rpiCamera needs access to /dev/video*, /dev/media*, /dev/dma_heap, vchiq.
# Easier than enumerating udev groups across Pi OS releases.
User=root
# Embed our public/ as the static-files root so /index.html etc. are served
# from MediaMTX's built-in HTTP. The path is also bind-readable.
Environment=MTX_PUBLIC_DIR=$PUBLIC_SRC
ProtectSystem=full
ProtectHome=read-only
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

# 5. Static UI server systemd unit (separate process, port 8443).
#    Needed because MediaMTX doesn't serve arbitrary static files; the Quest
#    WebXR client is hosted here.
log "writing $UI_UNIT"
cat > "$UI_UNIT" <<EOF
[Unit]
Description=picar2 FPV static UI server (HTTPS)
After=network-online.target mediamtx.service
Wants=network-online.target

[Service]
Type=simple
ExecStart=$UI_SCRIPT --port 8443
Restart=on-failure
RestartSec=2
User=root
ProtectSystem=full
ProtectHome=read-only
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable mediamtx picar-fpv-ui >/dev/null
log "enabled (not started). Start both with:"
log "  sudo systemctl start mediamtx picar-fpv-ui"
log ""
log "  config:    $CONFIG_SRC"
log "  ui dir:    $PUBLIC_SRC"
log "  cert:      $CERT  (accept once on Quest)"
log "  stream:    https://<pi-ip>:8889/cam/   (MediaMTX built-in viewer)"
log "  quest UI:  https://<pi-ip>:8443/       (custom WebXR client)"
