# ArduPilot SITL & Gazebo Zephyr Model

## Parameter Files

| File | Purpose |
|---|---|
| `ardupilot/Tools/autotest/default_params/gazebo-zephyr.parm` | Upstream tuned params for the Gazebo Zephyr model |
| `ardupilot/Tools/autotest/default_params/plane-zephyr.parm` | Project-specific params for the (modified) Gazebo Zephyr |
| `simulator/params/vehicle.parm` | Additional per-vehicle params loaded at runtime |

`plane-zephyr.parm` is derived from `gazebo-zephyr.parm` but with modifications for the project's axis-aligned Gazebo model.

> **Confidence:** Confirmed from repo

## Gazebo Zephyr Model Structure

```
ardupilot_gazebo/models/gazebo-zephyr/
  base/       ← physics model (SDF), meshes, inertia
  template/   ← ArduPilot plugin + LiftDrag aerodynamics
  red/        ← color variant
```

The **template** SDF includes the base model and adds:
- `libLiftDragPlugin.so` plugins for wing, elevons, rudders, propeller blades
- `libArduPilotPlugin.so` bridge (FDM ports 9002/9003)

> **Confidence:** Confirmed from repo

## Axis Alignment (Project-Specific Change)

The project modified the Gazebo Zephyr model to align Gazebo and ArduPilot coordinate frames:

**In `template/model.sdf`:**
- Model pose: `<pose>0 0 0 0 0 3.141593</pose>` — 180° yaw rotation
- `<modelXYZToAirplaneXForwardZDown>0 0 0 3.141593 0 0</modelXYZToAirplaneXForwardZDown>`
- `<gazeboXYZToNED>0 0 0 3.141593 0 1.57079632</gazeboXYZToNED>`

**Consequence for `plane-zephyr.parm`:** the servo function mapping must be swapped vs upstream:
```
SERVO1_FUNCTION 78   # ElevonRight → flap_left_joint (channel 0, multiplier=-1)
SERVO2_FUNCTION 77   # ElevonLeft  → flap_right_joint (channel 1, multiplier=-1)
```
vs. `gazebo-zephyr.parm` which uses `77/78`. The 180° model rotation causes what Gazebo labels "left" to be physically "right" from ArduPilot's NED perspective.

> **Confidence:** Confirmed from current chat (template SDF read + channel mapping verified)

## Critical `plane-zephyr.parm` Parameters

### IMU Calibration (SITL requirement)
```
INS_ACCOFFS_* 0.001   # small non-zero so SITL sees IMU as calibrated
INS_ACCSCAL_* 1.001
INS_ACC2OFFS_* 0.001
INS_ACC2SCAL_* 1.001
INS_GYR_CAL   0      # skip gyro calibration
```
Without these, SITL may refuse to arm — the IMU is not recognised as calibrated.

> **Confidence:** Confirmed from repo (`gazebo-zephyr.parm` comment, confirmed in practice)

### Airspeed (disabled)
```
ARSPD_TYPE 0
ARSPD_USE  0
TECS_SYNAIRSPEED 1   # use GPS groundspeed as synthetic airspeed for TECS
```
No physical airspeed sensor in Gazebo. TECS still needs speed feedback; `SYNAIRSPEED` provides it via GPS.

> **Confidence:** Confirmed from repo

### TECS Tuning
```
TECS_PTCH_DAMP 0.0   # MUST be 0 — any positive value causes self-exciting phugoid oscillation
TECS_SPDWEIGHT 0.5   # pitch for altitude, throttle for speed
TECS_CLMB_MAX  5.0   # realistic climb rate limit for Zephyr
TECS_INTEG_GAIN 0.15
```

**`TECS_PTCH_DAMP` note:** values like 3.0 were tested and caused oscillation. At 20 deg/s pitch rate, `3.0 × (20×π/180)` ≈ 1.05 rad = 60° overcorrection — self-exciting, not damping.

> **Confidence:** Confirmed from current chat (comment in `gazebo-zephyr.parm`)

### Throttle Channel
```
SERVO3_FUNCTION 70   # throttle
SERVO3_MIN 1000
SERVO3_TRIM 1000
SERVO3_MAX 2000
THR_MIN 0            # no idle spin when armed
THR_PASS_STAB 0      # suppress throttle on ground
TKOFF_THR_SLEW -1    # unrestricted throttle during takeoff
TKOFF_THR_MINACC 0   # no minimum acceleration needed before throttle
```

> **Confidence:** Confirmed from repo

### Navigation (L1 Guidance)
```
NAVL1_PERIOD  12     # vs ArduPilot default 17-25 s
NAVL1_DAMPING 0.75
WP_RADIUS     20     # vs ArduPilot default 30 m
```

**Rationale for Zephyr at 11 m/s:**
- Minimum turn radius at 45° bank: `V²/(g·tan45°) ≈ 12 m`
- L1 lookahead: `NAVL1_PERIOD × speed / (2π)`
  - Default 17 s → ~30 m lookahead = 2.5× turn radius → wide waypoint transitions
  - 12 s → ~21 m lookahead = 1.7× turn radius → tighter, less overshoot
- `WP_RADIUS 20` ≈ 1.6× minimum turn radius (default 30 m caused overshoot on WP2)

> **Confidence:** Confirmed from current chat (physics-derived, tested behavior)

## Headless Equivalent of `gazebo-zephyr`

The `gazebo-zephyr` frame has `external: True` in ArduPilot's `vehicleinfo.py` — it requires a running Gazebo instance (FDM ports 9002/9003). It cannot be used with pure ArduPilot SITL.

**Headless equivalent:** `plane-elevon` — same elevon servo layout, uses ArduPilot's internal physics (no Gazebo required).

To use: change the `--vehicle` SITL arg from `gazebo-zephyr` to `plane-elevon`. Load `plane-zephyr.parm` (or the equivalent parm file) separately via `--add-param-file` as usual.

> **Confidence:** Confirmed from ArduPilot source (`vehicleinfo.py`)

---

## `eeprom.bin` Parameter Caching

ArduPilot SITL persists parameters to `simulator/ardupilot_logs/veh_N/eeprom.bin` between runs. If you change `.parm` files but SITL still uses old values, delete `eeprom.bin`:

```bash
rm simulator/ardupilot_logs/veh_*/eeprom.bin
```

This forces ArduPilot to reload all parameters from scratch on next launch.

> **Confidence:** Confirmed from current session

---

## `_dump_critical_params` (logic.py)

Function at `simulator/logic.py:70–116`.

Requests named parameters from ArduPilot after startup and logs them between `=== PARAM DUMP START ===` and `=== PARAM DUMP END ===`. Used for post-run verification that the correct parm file was loaded. Results appear in `simulator/logs/logics/logic_N.log`.

> **Confidence:** Confirmed from repo
