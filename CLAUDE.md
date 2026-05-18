# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UAV-CYBER-SIM is a modular, distributed multi-UAV simulation framework for coordination and cybersecurity research. It integrates ArduPilot SITL, Gazebo, QGroundControl, and PyMAVLink to enable realistic multi-drone mission execution, adversarial testing, and Remote ID spoofing experiments.

## Environment Setup

The project uses [uv](https://docs.astral.sh/uv/) for Python environment management (Python 3.11):

```bash
uv sync
source .venv/bin/activate  # or prefix commands with: uv run
```

Dev container is available in [.devcontainer/Dockerfile](.devcontainer/Dockerfile) with full ArduPilot/Gazebo toolchain pre-built.

## Running Simulations

```bash
# Headless (fastest, supports 60+ UAVs)
python run.py --visualizer novis

# With Gazebo 3D visualization (max ~3 UAVs)
python run.py --visualizer gazebo

# With QGroundControl (max ~25 UAVs)
python run.py --visualizer QGroundControl
```

Jupyter notebooks in the root are the primary way to run simulations — numbered by complexity (`1-*` through `7-*`).

## Linting and Type Checking

```bash
ruff check .
ruff format .
mypy simulator/
```

Config in [pyproject.toml](pyproject.toml): line-length 88, targets py311, `ignore_missing_imports=true`.

### Strict Type Checking Requirements

Pylance runs in **strict mode** (`python.analysis.typeCheckingMode: strict` in [.vscode/settings.json](.vscode/settings.json)). All code must satisfy strict type checking. Follow these rules in every change:

- **Annotate all function signatures** — every parameter and return type must be explicitly typed. No bare `def f(x)` or missing `-> ReturnType`.
- **No implicit `Any`** — avoid `Any` unless interfacing with an untyped third-party library (e.g. `pymavlink`). When you must use `Any`, narrow it as soon as possible with a cast or isinstance check.
- **Use precise container types** — prefer `list[int]` over `List[int]`, `dict[str, float]` over `Dict`, `tuple[int, ...]` over `Tuple`. Use `Sequence`, `Mapping`, or `Iterable` for read-only parameters.
- **Narrow `Optional` / `X | None`** — always guard with `if x is not None` before use; never assume a value is present.
- **Dataclasses and TypedDicts** — use `@dataclass` or `TypedDict` for structured data instead of untyped `dict` literals.
- **Treat warnings as errors** — `reportMissingParameterType`, `reportMissingReturnType`, `reportMissingTypeArgument`, `reportUnknownParameterType`, and `reportUnknownVariableType` are all active warnings; do not introduce new instances of these.
- **Run `mypy simulator/` before finishing** — confirm zero new errors before considering a task done.

## Architecture

### Process Model

Each simulated vehicle spawns multiple OS processes:

- **SITL**: ArduPilot software-in-the-loop firmware (TCP port `5760 + offset`)
- **Logic** (`simulator/logic.py`): Bidirectional MAVLink proxy; executes the vehicle's mission plan
- **ADS-B Injector** (`simulator/adsb_injector.py`): Injects nearby traffic into ArduPilot
- **GCS** (`simulator/gcs.py`): Monitors telemetry for a group of vehicles

A single **Oracle** (`simulator/oracle.py`) coordinates all vehicles via ZMQ ROUTER/DEALER sockets — collecting positions, distributing Remote ID beacons, and detecting mission completion.

### Communication Stack

```text
ArduPilot SITL <--TCP--> Logic <--UDP--> GCS
                                |
                              ZMQ PUB/SUB --> Oracle --> ZMQ ROUTER/DEALER
```

Port allocation is managed in [simulator/config.py](simulator/config.py) with per-vehicle offsets:

- ARP: `5760 + offset`, ADSB: `5761 + offset`, RID: `5764-5765 + offset`, GCS: `5766 + offset`

### Key Classes

| Class | File | Role |
| :--- | :--- | :--- |
| `Simulator` | `simulator/sim.py` | Orchestrates all processes; call `.launch()` |
| `Oracle` | `simulator/oracle.py` | Global coordinator; call `.run()` to execute missions |
| `SimVehicle` | `simulator/entities/simvehicle.py` | Vehicle definition with plan |
| `GCS` | `simulator/gcs.py` | Ground control station monitoring |
| `Logic` | `simulator/logic.py` | Per-vehicle MAVLink proxy + plan executor |

### Mission Planning

Plans live in [simulator/planner/plans/](simulator/planner/plans/):

- `AutoPlan` — autonomous waypoint missions
- `GuidedPlan` — guided mode (manual waypoint injection)
- `PursuitPlan` — pursuit-evasion game scenario

Plans are composed of `Action` sequences (arm, takeoff, upload_mission, start_mission, land, etc.) defined in [simulator/planner/actions/](simulator/planner/actions/).

### Coordinate Systems

Two coordinate representations (see [simulator/helpers/coordinates.py](simulator/helpers/coordinates.py)):

- `ENUPose` — East-North-Up local frame (meters from origin)
- `GRAPose` — Geodetic (lat°, lon°, alt m)

### Visualizers

Abstract `Visualizer` base in [simulator/visualizer/visualizer.py](simulator/visualizer/visualizer.py), with three backends in [simulator/visualizer/](simulator/visualizer/): `gazebo/`, `QGroundControl/`, `novisualizer/`. Gazebo models and worlds are generated at runtime into `runtime_models/` and `runtime_worlds/`.

## Submodules

- `ardupilot/` — ArduPilot firmware (built with `waf --board sitl`)
- `ardupilot_gazebo/` — Gazebo plugin bridging SITL to physics engine

After cloning: `git submodule update --init --recursive`

## Extended Context

Detailed architecture notes, confirmed behaviors, known bugs and fixes live in [`docs/context/`](docs/context/):

- [`mavlink-routing.md`](docs/context/mavlink-routing.md) — port map, QGC connection model, ArduPilot MissionItemProtocol behavior
- [`mission-planning.md`](docs/context/mission-planning.md) — AutoPlan order, upload protocol, PushMissionToGCS; GuidedPlan ArduPlane (PlaneTakeOff/PlaneGoTo/PlaneLand, AUTOLAND_DIR_OFF, origin_heading threading)
- [`ardupilot-sitl.md`](docs/context/ardupilot-sitl.md) — parm files, Gazebo Zephyr axis alignment, TECS/L1 tuning, plane-zephyr SITL physics model, frame-string routing, JSON model ROMFS gotcha, AUTOLAND parameters (WP_DIST/WP_ALT/DIR_OFF)
- [`copter-iris-sitl.md`](docs/context/copter-iris-sitl.md) — copter-iris frame: iris.json physics, copter-iris.parm, ROMFS auto-rebuild, trajectory equivalence vs gazebo-iris
- [`gazebo-models.md`](docs/context/gazebo-models.md) — color template system (Jinja2), OGRE material scripts, physics/ardupilot/color_template layout, URI replacement pitfall
- [`known-issues.md`](docs/context/known-issues.md) — fixed bugs and their root causes
