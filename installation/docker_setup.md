# Docker Setup

Two ways to get the simulation environment running inside Docker.

---

## Option 1: Pull the Pre-Built Image (Recommended)

Download the latest image from Docker Hub — no compilation required.

```bash
make pull
make run
```

---

## Option 2: Build the Image Locally

Use this when you need to modify the `Dockerfile` (e.g., add packages, change the repo branch).

```bash
make build
```

This compiles ArduPilot SITL, the Gazebo plugin, QGroundControl, and the Python environment. Expect **10–15 minutes**.

After building, push to Docker Hub:

```bash
make login   # one-time Docker Hub authentication
make push
```

---

## Visualizer Support by Context

| Context | `novis` | `gazebo` | `QGroundControl` |
|---|---|---|---|
| Docker, physically at server | ✅ | ✅ | ✅ |
| Docker, SSH only (no desktop) | ✅ | ❌ | ❌ |
| Docker, via VNC (`make vnc-run`) | ✅ | ✅ | ✅ |

Gazebo and QGC require a real desktop session — see `remote_connection.md` for VNC setup.

---

## Makefile Targets

| Target | Description |
|---|---|
| `make pull` | Pull latest image from Docker Hub |
| `make run` | Launch container (local display or SSH X11) |
| `make vnc-run` | Launch container targeting a TurboVNC session (default display `:4`) |
| `make build` | Build image locally from `Dockerfile` |
| `make push` | Push local image to Docker Hub |
| `make stop` | Stop and remove the running container |

Override the VNC display number:

```bash
make vnc-run VNC_DISPLAY=:5
```

---

## Keeping the Image Up to Date

The Docker image is a snapshot built at a specific commit. It does **not** update automatically when the repo changes.

### When to rebuild (`make build` + `make push`)

Only needed when the compiled toolchain or system dependencies change:

| Change | Rebuild needed? |
|---|---|
| Python code (`simulator/`, notebooks, `run.py`) | ❌ |
| New Python package in `pyproject.toml` | ❌ |
| Dockerfile changes (new apt package, etc.) | ✅ |
| ArduPilot SITL changes | ✅ |
| Gazebo plugin changes | ✅ |
| QGroundControl version change | ✅ |

### Updating Python code without rebuilding

For Python-only changes, just pull inside the running container — no rebuild required:

```bash
# inside the container
git pull
uv sync   # only needed if pyproject.toml changed
```

This is instant vs 10–15 minutes for a full rebuild.

---

## Troubleshooting

**`cannot open display` inside container**
- Check `echo $DISPLAY` on the host — should be set.
- Re-run `make run` or `make vnc-run` — the auth file is regenerated each time.

**`xterm: command not found` inside container**
- The image may be stale. Run `make pull` to refresh.

**Gazebo or QGC fails to start (SSH-only session)**
- GUI apps require a desktop session. Switch to `make vnc-run` with a VNC session open.
