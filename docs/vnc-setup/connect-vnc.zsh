#!/bin/zsh
# -----------------------------------------------
# OFFICE VNC CONNECTOR — TurboVNC virtual display :4
# -----------------------------------------------
HOST="ubuntu-server"        # SSH alias in ~/.ssh/config
DISPLAY_NUM=4
PORT=$((5900 + DISPLAY_NUM))   # 5904
SOCK="/tmp/ssh-vnc-abeldg@10.2.219.100:22"
VIEWER="/Applications/TurboVNC/TurboVNC Viewer.app/Contents/MacOS/TurboVNC Viewer"

# Kill any stale master socket and free local port
ssh -O exit -S "${SOCK}" "${HOST}" 2>/dev/null || true
lsof -ti tcp:${PORT} | xargs kill -9 2>/dev/null || true
sleep 0.5

echo "\n[1/3] Opening SSH master + tunnel on localhost:${PORT}..."
ssh -fNM -S "${SOCK}" \
    -o ExitOnForwardFailure=yes \
    -L "${PORT}:127.0.0.1:${PORT}" \
    "${HOST}"

echo "\n[2/3] Ensuring TurboVNC is running on display :${DISPLAY_NUM}..."
ssh -S "${SOCK}" "${HOST}" "
  ss -tlnp | grep -q ':${PORT}' \
    && echo '  Already running.' \
    || ~/bin/vnc-start-clean ${DISPLAY_NUM}
"

echo "\n[3/3] Launching TurboVNC Viewer..."
"${VIEWER}" -securitytypes VNC "localhost::${PORT}" &

sleep 2
echo "\nDone. GNOME desktop on display :${DISPLAY_NUM} via localhost:${PORT}"
