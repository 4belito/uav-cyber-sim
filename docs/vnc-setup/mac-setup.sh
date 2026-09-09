#!/bin/zsh
# =============================================================
# VNC Mac-side Bootstrap
# Sets up SSH key, ~/.ssh/config entry, and connect-vnc.zsh
# Run once on a fresh Mac to restore the VNC client setup.
# =============================================================
set -euo pipefail

SERVER_USER="abeldg"
SERVER_HOST="10.2.219.100"
KEY_FILE="$HOME/.ssh/id_ed25519_vnc"

# ── 1. SSH key ────────────────────────────────────────────────
echo "[1/3] SSH key..."
if [[ ! -f "$KEY_FILE" ]]; then
  ssh-keygen -t ed25519 -f "$KEY_FILE" -C "mac-vnc" -N ""
  echo "  Generated $KEY_FILE"
else
  echo "  Key already exists, skipping."
fi

echo ""
echo "  Copying public key to server (will prompt for password once):"
DISPLAY= ssh-copy-id -i "${KEY_FILE}.pub" "${SERVER_USER}@${SERVER_HOST}"

# ── 2. SSH config ─────────────────────────────────────────────
echo "[2/3] SSH config..."
SSH_CONFIG="$HOME/.ssh/config"
if grep -q "Host ubuntu-server" "$SSH_CONFIG" 2>/dev/null; then
  echo "  ubuntu-server entry already in $SSH_CONFIG, skipping."
else
  cat >> "$SSH_CONFIG" << SSH
Host ubuntu-server
  HostName ${SERVER_HOST}
  User ${SERVER_USER}
  IdentityFile ${KEY_FILE}
  ServerAliveInterval 30
  ServerAliveCountMax 3
  TCPKeepAlive yes
  ControlMaster auto
  ControlPath /tmp/ssh-vnc-%r@%h:%p
  ControlPersist 10m
SSH
  echo "  Added ubuntu-server to $SSH_CONFIG"
fi

# ── 3. connect-vnc.zsh ────────────────────────────────────────
echo "[3/3] Installing ~/connect-vnc.zsh..."
cat > "$HOME/connect-vnc.zsh" << 'SCRIPT'
#!/bin/zsh
# -----------------------------------------------
# OFFICE VNC CONNECTOR — TurboVNC virtual display :4
# -----------------------------------------------
HOST="ubuntu-server"
DISPLAY_NUM=4
PORT=$((5900 + DISPLAY_NUM))
SOCK="/tmp/ssh-vnc-abeldg@10.2.219.100:22"
VIEWER="/Applications/TurboVNC/TurboVNC Viewer.app/Contents/MacOS/TurboVNC Viewer"

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
SCRIPT
chmod +x "$HOME/connect-vnc.zsh"

echo ""
echo "=== Mac setup complete ==="
echo "  Run: ~/connect-vnc.zsh"
