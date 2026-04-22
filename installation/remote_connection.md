# Remote Connection

How to connect to a remote Ubuntu simulation server and run GUI simulations (Gazebo, QGroundControl) inside Docker.

---

## Connection Methods

| Method | Gazebo/QGC in Docker | Notes |
|---|---|---|
| TurboVNC + `make vnc-run` | ✅ | Recommended |
| Physical access at server | ✅ | Use `make run` normally |
| SSH only (`ssh -Y`) | ❌ | OpenGL not supported over SSH X11 |

Gazebo and QGC require OpenGL rendered on the server. SSH X11 forwarding cannot provide this — only a real desktop session (VNC) works remotely.

---

## TurboVNC Setup

VNC runs a full GNOME desktop on the server using software rendering. You connect via an SSH tunnel so the VNC port is never exposed to the network.

### Prerequisites

| Where | What |
|---|---|
| Server | [TurboVNC](https://turbovnc.org) installed at `/opt/TurboVNC/` |
| Server | GNOME desktop packages (`ubuntu-desktop` or equivalent) |
| Your machine | [TurboVNC Viewer](https://turbovnc.org/Downloads) |

### Server: one-time setup

**1. Create `~/bin/vnc-start-clean`** — a script that kills stale locks and starts a fresh session:

```bash
mkdir -p ~/bin
cat > ~/bin/vnc-start-clean << 'EOF'
#!/bin/bash
DISPLAY_NUM="${1:-4}"
VNC_BIN="/opt/TurboVNC/bin/vncserver"
$VNC_BIN -kill :$DISPLAY_NUM 2>/dev/null || true
rm -f /tmp/.X${DISPLAY_NUM}-lock
rm -rf /tmp/.X11-unix/X${DISPLAY_NUM}
$VNC_BIN :$DISPLAY_NUM \
  -xstartup ~/.vnc/xstartup.turbovnc \
  -localhost \
  -securitytypes VNC \
  -geometry 1920x1080
EOF
chmod +x ~/bin/vnc-start-clean
```

**2. Create `~/.vnc/xstartup.turbovnc`** — launches a GNOME session with dbus:

```bash
mkdir -p ~/.vnc
cat > ~/.vnc/xstartup.turbovnc << 'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP=ubuntu:GNOME
export GNOME_SHELL_SESSION_MODE=ubuntu

# Disable lock screen (prevents gnome-shell crash on VNC unlock)
eval "$(dbus-launch --sh-syntax)"
gsettings set org.gnome.desktop.lockdown disable-lock-screen true
gsettings set org.gnome.desktop.screensaver lock-enabled false
kill "$DBUS_SESSION_BUS_PID" 2>/dev/null
unset DBUS_SESSION_BUS_ADDRESS DBUS_SESSION_BUS_PID

exec dbus-launch --exit-with-session gnome-session --session=ubuntu
EOF
chmod +x ~/.vnc/xstartup.turbovnc
```

**3. Set a VNC password:**

```bash
/opt/TurboVNC/bin/vncpasswd ~/.vnc/passwd
```

**4. (Optional) Auto-start on boot via systemd:**

```bash
sudo tee /etc/systemd/system/vncserver@.service > /dev/null << EOF
[Unit]
Description=TurboVNC Server for display :%i
After=network.target

[Service]
Type=oneshot
User=$USER
RemainAfterExit=yes
KillMode=none
ExecStart=$HOME/bin/vnc-start-clean %i
ExecStop=/opt/TurboVNC/bin/vncserver -kill :%i

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable vncserver@4.service
sudo systemctl start vncserver@4.service

# Keep session alive after logout
sudo loginctl enable-linger $USER
```

### Your machine: connecting

**1. Add the server to `~/.ssh/config`** (replace values to match your setup):

```
Host ubuntu-server
  HostName <server-ip>
  User <username>
  IdentityFile ~/.ssh/id_ed25519
  ServerAliveInterval 30
  ControlMaster auto
  ControlPath /tmp/ssh-vnc-%r@%h:%p
  ControlPersist 10m
```

**2. Create a connect script** — saves typing each session:

```zsh
#!/bin/zsh
HOST="ubuntu-server"       # SSH alias above
DISPLAY_NUM=4
PORT=$((5900 + DISPLAY_NUM))
SOCK="/tmp/ssh-vnc-${HOST}"
VIEWER="/path/to/TurboVNC Viewer"   # adjust for your OS

ssh -O exit -S "${SOCK}" "${HOST}" 2>/dev/null || true
lsof -ti tcp:${PORT} | xargs kill -9 2>/dev/null || true
sleep 0.5

ssh -fNM -S "${SOCK}" -o ExitOnForwardFailure=yes \
    -L "${PORT}:127.0.0.1:${PORT}" "${HOST}"

ssh -S "${SOCK}" "${HOST}" "
  ss -tlnp | grep -q ':${PORT}' \
    && echo 'VNC already running.' \
    || ~/bin/vnc-start-clean ${DISPLAY_NUM}
"

"${VIEWER}" -securitytypes VNC "localhost::${PORT}" &
```

Save it as `~/connect-vnc.zsh`, `chmod +x`, and run it to connect.

---

## Running the Simulation via Docker

Once the VNC viewer is open, run from any terminal on the server:

```bash
cd uav-cyber-sim
make vnc-run                    # default display :4
make vnc-run VNC_DISPLAY=:5    # override display
```

Inside the container:

```bash
python run.py --visualizer novis          # headless, works everywhere
python run.py --visualizer gazebo         # 3D view in VNC window
python run.py --visualizer QGroundControl # QGC in VNC window
```

---

## SSH X11 Forwarding (Diagnostics Only)

Use `ssh -Y` only to verify the X11 forwarding chain works — not for running Gazebo or QGC.

```bash
ssh -Y <user>@<server>
cd uav-cyber-sim && make run
# inside the container:
xterm    # should open on your local screen
```

For this to work, `/etc/ssh/sshd_config` on the server must have:

```
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost yes
```

Works from any client OS — macOS (XQuartz), Windows (VcXsrv / MobaXterm), Linux (built-in).

---

## Troubleshooting

**VNC session not starting or black screen**
```bash
~/bin/vnc-start-clean 4
```

**`cannot open display` inside container**
- Confirm the VNC session is running: `ss -tlnp | grep 5904`
- Re-run `make vnc-run` — it regenerates the xauth file each time.

**Gazebo crashes immediately inside container**
- Use `make vnc-run`, not `make run` — the required env vars are only set in `vnc-run`.

**Gazebo is slow**
- Expected with software rendering. Limit to ≤ 3 UAVs when using Gazebo.

**GNOME lock screen freezes the session**
- The xstartup script disables the lock screen. If it reappears, re-run the xstartup setup step.
