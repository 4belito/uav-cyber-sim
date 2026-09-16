#!/bin/bash
# =============================================================
# VNC Server Bootstrap — run as abeldg on 10.2.219.100
# Reproduces the TurboVNC + GNOME session on display :4
# =============================================================
set -euo pipefail

DISPLAY_NUM=4
VNC_PORT=$((5900 + DISPLAY_NUM))
USER=abeldg
HOME_DIR="/home/${USER}"
VNC_BIN="/opt/TurboVNC/bin/vncserver"

# ── 1. Prerequisites ──────────────────────────────────────────
echo "[1/7] Checking TurboVNC..."
if [[ ! -x "$VNC_BIN" ]]; then
  echo "TurboVNC not found at $VNC_BIN"
  echo "Download from https://turbovnc.org and install, then re-run."
  exit 1
fi
"$VNC_BIN" -version 2>&1 | head -1

# ── 2. VNC password ───────────────────────────────────────────
echo "[2/7] Setting VNC password..."
mkdir -p "${HOME_DIR}/.vnc"
if [[ ! -f "${HOME_DIR}/.vnc/passwd" ]]; then
  "${VNC_BIN%vncserver}vncpasswd" "${HOME_DIR}/.vnc/passwd"
else
  echo "  passwd already exists, skipping."
fi

# ── 3. vnc-start-clean script ─────────────────────────────────
echo "[3/7] Installing vnc-start-clean..."
mkdir -p "${HOME_DIR}/bin"
cat > "${HOME_DIR}/bin/vnc-start-clean" << 'SCRIPT'
#!/bin/bash
set -e

DISPLAY_NUM="${1:-4}"
GEOMETRY="1920x1080"
VNC_BIN="/opt/TurboVNC/bin/vncserver"

echo "[VNC] Cleaning stale session for :$DISPLAY_NUM ..."
$VNC_BIN -kill :$DISPLAY_NUM 2>/dev/null || true
rm -f /tmp/.X${DISPLAY_NUM}-lock
rm -rf /tmp/.X11-unix/X${DISPLAY_NUM}

echo "[VNC] Starting fresh session on :$DISPLAY_NUM ..."
$VNC_BIN :$DISPLAY_NUM \
  -xstartup /home/abeldg/.vnc/xstartup.turbovnc \
  -localhost \
  -securitytypes VNC \
  -geometry "$GEOMETRY"

echo "[VNC] Ready on :$DISPLAY_NUM"
SCRIPT
chmod +x "${HOME_DIR}/bin/vnc-start-clean"

# ── 4. xstartup ───────────────────────────────────────────────
echo "[4/7] Writing xstartup.turbovnc..."
cat > "${HOME_DIR}/.vnc/xstartup.turbovnc" << 'XSTART'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP=ubuntu:GNOME
export GNOME_SHELL_SESSION_MODE=ubuntu
export DESKTOP_SESSION=ubuntu
export XDG_SESSION_DESKTOP=ubuntu

# Isolate VNC config from physical GDM session — prevents dconf/D-Bus conflicts
export XDG_CONFIG_HOME=/home/abeldg/clean_home/.config
export XDG_DATA_HOME=/home/abeldg/clean_home/.local
export XDG_CACHE_HOME=/home/abeldg/clean_home/.cache

# Disable lock screen before session starts — prevents gnome-shell crash on VNC unlock
eval "$(dbus-launch --sh-syntax)"
gsettings set org.gnome.desktop.lockdown disable-lock-screen true
gsettings set org.gnome.desktop.screensaver lock-enabled false
kill "$DBUS_SESSION_BUS_PID" 2>/dev/null
unset DBUS_SESSION_BUS_ADDRESS DBUS_SESSION_BUS_PID

# Pre-unlock gnome-keyring with empty password inside the session D-Bus so
# gnome-shell sees an already-unlocked daemon and skips the dialog
exec dbus-launch --exit-with-session sh -c '
  eval "$(echo "" | /usr/bin/gnome-keyring-daemon --unlock --daemonize --components=pkcs11,secrets,ssh 2>/dev/null)"
  export GNOME_KEYRING_CONTROL SSH_AUTH_SOCK
  exec gnome-session --session=ubuntu
'
XSTART
chmod +x "${HOME_DIR}/.vnc/xstartup.turbovnc"

# ── 5. systemd service ────────────────────────────────────────
echo "[5/7] Installing vncserver@.service..."
sudo tee /etc/systemd/system/vncserver@.service > /dev/null << 'SERVICE'
[Unit]
Description=TurboVNC Server for display :%i
After=network.target

[Service]
Type=oneshot
User=abeldg
PAMName=gdm-autologin
WorkingDirectory=/home/abeldg
RemainAfterExit=yes
KillMode=none
Environment=DISPLAY=:%i
ExecStart=/home/abeldg/bin/vnc-start-clean %i
ExecStop=/opt/TurboVNC/bin/vncserver -kill :%i

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable "vncserver@${DISPLAY_NUM}.service"
sudo systemctl restart "vncserver@${DISPLAY_NUM}.service"

# ── 6. Linger (session survives logout) ───────────────────────
echo "[6/7] Enabling linger for ${USER}..."
sudo loginctl enable-linger "${USER}"

# ── 7. Ubuntu appearance via dconf ───────────────────────────
echo "[7/7] Applying Ubuntu/Yaru appearance settings..."
# Requires a running VNC session — wait for Xvnc to be ready
for i in $(seq 1 10); do
  ss -tlnp | grep -q ":${VNC_PORT}" && break
  echo "  waiting for Xvnc on port ${VNC_PORT}... (${i}/10)"
  sleep 2
done

if ! ss -tlnp | grep -q ":${VNC_PORT}"; then
  echo "  Xvnc not listening yet; skipping gsettings (re-run after VNC starts)"
else
  # Find the dbus session bus address from gsd-xsettings of the VNC session
  DBUS_ADDR=""
  for i in $(seq 1 8); do
    GSD_PID=$(pgrep -u "${USER}" -n gsd-xsettings 2>/dev/null || true)
    if [[ -n "$GSD_PID" ]]; then
      DBUS_ADDR=$(cat /proc/${GSD_PID}/environ 2>/dev/null | tr '\0' '\n' \
                  | grep DBUS_SESSION_BUS_ADDRESS | cut -d= -f2-)
      [[ -n "$DBUS_ADDR" ]] && break
    fi
    sleep 2
  done

  if [[ -z "$DBUS_ADDR" ]]; then
    echo "  Could not find dbus address; skipping gsettings (run apply-appearance.sh manually)"
  else
    export DISPLAY=:${DISPLAY_NUM}
    export DBUS_SESSION_BUS_ADDRESS="$DBUS_ADDR"

    gsettings set org.gnome.desktop.interface gtk-theme             'Yaru'
    gsettings set org.gnome.desktop.interface icon-theme            'Yaru'
    gsettings set org.gnome.desktop.interface cursor-theme          'Yaru'
    gsettings set org.gnome.desktop.interface font-name             'Ubuntu 11'
    gsettings set org.gnome.desktop.interface monospace-font-name   'Ubuntu Mono 13'
    gsettings set org.gnome.desktop.interface document-font-name    'Sans 11'
    gsettings set org.gnome.desktop.interface enable-hot-corners    false
    gsettings set org.gnome.desktop.interface clock-format          '24h'
    gsettings set org.gnome.desktop.background picture-uri          'file:///usr/share/backgrounds/warty-final-ubuntu.png'
    gsettings set org.gnome.desktop.background picture-options      'zoom'
    gsettings set org.gnome.desktop.background show-desktop-icons   true
    gsettings set org.gnome.desktop.wm.preferences button-layout              ':minimize,maximize,close'
    gsettings set org.gnome.desktop.wm.preferences titlebar-font              'Ubuntu Bold 11'
    gsettings set org.gnome.desktop.wm.preferences titlebar-uses-system-font  false
    gsettings set org.gnome.desktop.wm.preferences action-middle-click-titlebar 'lower'
    gsettings set org.gnome.mutter attach-modal-dialogs       true
    gsettings set org.gnome.mutter edge-tiling                true
    gsettings set org.gnome.mutter dynamic-workspaces         true
    gsettings set org.gnome.mutter workspaces-only-on-primary true
    gsettings set org.gnome.settings-daemon.plugins.xsettings antialiasing 'rgba'
    gsettings set org.gnome.desktop.screensaver picture-uri         'file:///usr/share/backgrounds/warty-final-ubuntu.png'
    echo "  Appearance settings applied."
  fi
fi

echo ""
echo "=== Setup complete ==="
echo "  VNC display  : :${DISPLAY_NUM}"
echo "  VNC port     : ${VNC_PORT} (localhost only)"
echo "  Connect from Mac: ~/connect-vnc.zsh"
