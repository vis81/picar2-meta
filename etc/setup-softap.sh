#!/usr/bin/env bash
# Set up concurrent AP+STA on Raspberry Pi 4 (BCM43455 / brcmfmac).
#
# Creates a virtual uap0 interface on wlan0 so the Pi can run a NetworkManager
# "shared" hotspot while keeping its upstream WiFi (STA) connection. AP and STA
# must share one channel — the AP follows whatever channel STA is on.
#
# Usage:
#   sudo ./setup-softap.sh                       # uses defaults below
#   sudo SSID=picar PASS=secretpass ./setup-softap.sh
#   sudo ./setup-softap.sh --ssid picar --pass secretpass
#   sudo ./setup-softap.sh --teardown            # remove everything
#
# Idempotent: safe to re-run to change SSID / password.

set -euo pipefail

SSID="${SSID:-picar-ap}"
PASS="${PASS:-changeme1234}"
CON_NAME="${CON_NAME:-picar-ap}"
UAP_IFACE="${UAP_IFACE:-uap0}"
WLAN_IFACE="${WLAN_IFACE:-wlan0}"
COUNTRY="${COUNTRY:-US}"

TEARDOWN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ssid)     SSID="$2";     shift 2 ;;
        --pass)     PASS="$2";     shift 2 ;;
        --con)      CON_NAME="$2"; shift 2 ;;
        --country)  COUNTRY="$2";  shift 2 ;;
        --teardown) TEARDOWN=1;    shift   ;;
        -h|--help)
            sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "Unknown arg: $1" >&2
            exit 2
            ;;
    esac
done

if [[ $EUID -ne 0 ]]; then
    echo "Must run as root (try: sudo $0 $*)" >&2
    exit 1
fi

if [[ ${#PASS} -lt 8 || ${#PASS} -gt 63 ]]; then
    echo "PASS must be 8-63 chars (WPA2 requirement); got ${#PASS}" >&2
    exit 1
fi

log() { printf '[softap] %s\n' "$*"; }

teardown() {
    log "tearing down"
    nmcli connection delete "$CON_NAME" 2>/dev/null || true
    systemctl disable --now "${UAP_IFACE}.service" 2>/dev/null || true
    rm -f "/etc/systemd/system/${UAP_IFACE}.service"
    rm -f "/etc/NetworkManager/conf.d/10-${UAP_IFACE}.conf"
    iw dev "$UAP_IFACE" del 2>/dev/null || true
    systemctl daemon-reload
    systemctl reload NetworkManager 2>/dev/null || true
    log "done"
}

if [[ $TEARDOWN -eq 1 ]]; then
    teardown
    exit 0
fi

# 1. NetworkManager must be the active network backend
if ! systemctl is-active --quiet NetworkManager; then
    echo "NetworkManager is not running — this script targets NM-managed systems" >&2
    exit 1
fi

# 2. Regdomain (do_wifi_country writes wpa_supplicant.conf, harmless if NM-managed)
log "setting country=$COUNTRY"
raspi-config nonint do_wifi_country "$COUNTRY" 2>/dev/null || true
iw reg set "$COUNTRY" 2>/dev/null || true

# 3. Tell NM to manage uap0 before creating it
NM_CONF="/etc/NetworkManager/conf.d/10-${UAP_IFACE}.conf"
if [[ ! -f "$NM_CONF" ]]; then
    log "writing $NM_CONF"
    cat > "$NM_CONF" <<EOF
[device-${UAP_IFACE}]
match-device=interface-name:${UAP_IFACE}
managed=true
EOF
    systemctl reload NetworkManager
fi

# 4. systemd unit so uap0 is recreated at boot
UNIT="/etc/systemd/system/${UAP_IFACE}.service"
if [[ ! -f "$UNIT" ]]; then
    log "writing $UNIT"
    cat > "$UNIT" <<EOF
[Unit]
Description=Create ${UAP_IFACE} virtual AP interface on ${WLAN_IFACE}
After=sys-subsystem-net-devices-${WLAN_IFACE}.device
Before=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
ExecStart=/sbin/iw dev ${WLAN_IFACE} interface add ${UAP_IFACE} type __ap
ExecStartPost=/sbin/ip link set ${UAP_IFACE} up
ExecStop=/sbin/iw dev ${UAP_IFACE} del
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable "${UAP_IFACE}.service"
fi

# 5. Ensure uap0 exists right now (start the unit; tolerate "already exists")
if ! ip link show "$UAP_IFACE" >/dev/null 2>&1; then
    log "creating $UAP_IFACE"
    systemctl start "${UAP_IFACE}.service" || iw dev "$WLAN_IFACE" interface add "$UAP_IFACE" type __ap
    ip link set "$UAP_IFACE" up
fi

# 6. Read STA channel — AP must match (single-channel concurrency on bcm43455)
STA_CHAN="$(iw dev "$WLAN_IFACE" info 2>/dev/null | awk '/channel/ {print $2; exit}' || true)"
if [[ -z "${STA_CHAN}" ]]; then
    log "warning: $WLAN_IFACE not on a channel yet — AP will pick at activation time"
    BAND_ARG=()
    CHAN_ARG=()
else
    log "STA on channel $STA_CHAN — AP will match"
    if [[ "$STA_CHAN" -ge 36 ]]; then
        BAND_ARG=(802-11-wireless.band a)
    else
        BAND_ARG=(802-11-wireless.band bg)
    fi
    CHAN_ARG=(802-11-wireless.channel "$STA_CHAN")
fi

# 7. Create or update the AP connection
if nmcli -t -f NAME connection show | grep -Fxq "$CON_NAME"; then
    log "updating existing connection '$CON_NAME'"
else
    log "creating connection '$CON_NAME'"
    nmcli connection add \
        type wifi \
        ifname "$UAP_IFACE" \
        con-name "$CON_NAME" \
        autoconnect yes \
        ssid "$SSID"
fi

nmcli connection modify "$CON_NAME" \
    connection.interface-name "$UAP_IFACE" \
    802-11-wireless.mode ap \
    802-11-wireless.ssid "$SSID" \
    "${BAND_ARG[@]}" \
    "${CHAN_ARG[@]}" \
    ipv4.method shared \
    ipv6.method disabled \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$PASS" \
    802-11-wireless-security.proto rsn \
    802-11-wireless-security.pairwise ccmp \
    802-11-wireless-security.group ccmp

# 8. (Re)activate
log "activating '$CON_NAME'"
nmcli connection down "$CON_NAME" 2>/dev/null || true
nmcli connection up "$CON_NAME"

# 9. Status
log "interfaces:"
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status \
    | awk -F: -v w="$WLAN_IFACE" -v u="$UAP_IFACE" '$1==w || $1==u {print "  " $0}'
log "AP address:"
ip -4 -o addr show "$UAP_IFACE" | awk '{print "  " $4}'
log "ssid=$SSID  channel=$(iw dev "$UAP_IFACE" info | awk '/channel/ {print $2; exit}')"
log "done"
