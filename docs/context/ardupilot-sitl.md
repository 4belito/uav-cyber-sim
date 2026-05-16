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
  physics/          ← physics model (SDF), meshes, textures
  ardupilot/        ← ArduPilot plugin + LiftDrag aerodynamics (includes physics/ at runtime)
  color_template/   ← Jinja2 templates; rendered per-color into runtime_models/
```

The **ardupilot** SDF includes the physics model and adds:
- `libLiftDragPlugin.so` plugins for wing, elevons, rudders, propeller blades
- `libArduPilotPlugin.so` bridge (FDM ports patched at runtime)

See `docs/context/gazebo-models.md` for the full color template system.

> **Confidence:** Confirmed from repo (refactored in plane-model branch)

## Axis Alignment (Project-Specific Change)

The project modified the Gazebo Zephyr model to align Gazebo and ArduPilot coordinate frames:

**In `ardupilot/model.sdf`:**
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

### Navigation (L1 Guidance) and Roll Controller

These must match the firmware defaults that `gazebo-zephyr` uses implicitly (it loads no `models/plane.parm`). `plane-zephyr` loads `models/plane.parm` first, so the values below must be explicitly overridden:

```
NAVL1_PERIOD  17     # firmware default (AP_L1_Control.cpp); matches gazebo-zephyr
NAVL1_DAMPING 0.75   # firmware default
WP_RADIUS     90     # firmware default (config.h: WP_RADIUS_DEFAULT=90)
                     # models/plane.parm sets 50 — must override to 90
RLL2SRV_TCONST 0.5  # firmware default; models/plane.parm sets 0.25 (2× too fast)
RLL2SRV_RMAX   0    # firmware default (unlimited); models/plane.parm sets 90
```

**`models/plane.parm` trap:** This file is loaded for `plane-zephyr` but NOT for `gazebo-zephyr`. It sets `WP_RADIUS=50`, `RLL2SRV_TCONST=0.25`, `RLL2SRV_RMAX=90`, and `NAVL1_PERIOD=15` — all of which diverge from Gazebo's firmware defaults. Every value must be explicitly overridden in `plane-zephyr.parm`.

**L1 lookahead at 11 m/s:** `17 × 11 / (2π) ≈ 30 m` (1.7× Zephyr min turn radius of ~12 m at 45° bank). Period=12 caused cross-track orbit at final approach; 17 converges cleanly.

**WP_RADIUS=90 on small missions:** With 100 m legs, 90 m acceptance radius leaves only 10 m of effective straight flight per leg — TECS never stabilises altitude or speed. For trajectory comparison between SITL and Gazebo, use legs ≥ 300 m.

> **Confidence:** Confirmed — firmware defaults read from source (`AP_L1_Control.cpp`, `AP_RollController.cpp`, `ArduPlane/config.h`); WP_RADIUS mismatch verified from log analysis

## Headless Equivalent of `gazebo-zephyr` (`plane-zephyr`)

The `gazebo-zephyr` frame has `external: True` in `vehicleinfo.py` — requires a running Gazebo instance. For pure SITL (no Gazebo), use the **`plane-zephyr`** frame defined in `vehicleinfo.py`:

```python
# ardupilot/Tools/autotest/pysim/vehicleinfo.py
"plane-zephyr": {
    "model": "plane-zephyr-elevon",   # triggers -elevon AND -zephyr in SIM_Plane.cpp
    "waf_target": "bin/arduplane",
    "default_params_filename": [
        "models/plane.parm",           # loaded first; most values overridden below
        "default_params/plane-zephyr.parm",
    ],
},
```

> **Confidence:** Confirmed from repo (`vehicleinfo.py`)

## Frame-String Routing: How SITL Selects the Physics Model

`SITL_cmdline.cpp` uses **prefix matching** (`strncasecmp(name, model_str, strlen(name))`) against its model table:

| Model string | Prefix match | Physics class |
|---|---|---|
| `plane-zephyr-elevon` | `plane` | `Plane::create` → `SIM_Plane.cpp` |
| `gazebo-zephyr` | `gazebo` | `Gazebo::create` → external physics |

**Consequence:** `SIM_Plane.cpp` is **never executed** during a Gazebo run. Changes to `SIM_Plane.cpp` (including the `-zephyr` branch) cannot affect `gazebo-zephyr` behaviour.

Within `SIM_Plane.cpp`, the frame string is checked with `strstr`:
- `-elevon` → enables elevon mixing
- `-zephyr` → overrides aerodynamic coefficients (see section below)
- `-heavy`, `-jet`, `-soaring` → other presets (unrelated)

> **Confidence:** Confirmed from `SITL_cmdline.cpp` and `SIM_Plane.cpp`

---

## SITL Physics Model: Zephyr Aerodynamic Coefficients

`SIM_Plane.cpp` defaults to **Skywalker 2013** aerodynamics (hardcoded in `SIM_Plane.h`). The `-zephyr` branch added to `SIM_Plane.cpp` overrides these with values derived from the Gazebo Zephyr LiftDragPlugin:

```cpp
// SIM_Plane.cpp — added after the -soaring block
if (strstr(frame_str, "-zephyr")) {
    mass = 1.9f;                       // model.sdf: wing(1.5)+prop(0.05)+flaps(0.2)+imu(0.15)
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    coefficient.s           = 0.50f;   // LiftDragPlugin main wing area
    coefficient.b           = 1.50f;   // wingspan from model.sdf joint positions
    coefficient.c           = 0.333f;  // mean chord = s/b
    coefficient.c_lift_0    = 0.48f;   // cla*a0 = 3.7*0.13 (see formula below)
    coefficient.c_lift_a    = 3.7f;    // main-wing cla from LiftDragPlugin
    coefficient.c_drag_p    = 0.064f;  // cda from LiftDragPlugin
    coefficient.alpha_stall = 0.3391f; // stall angle (rad) from LiftDragPlugin
    coefficient.oswald      = 0.80f;   // lower efficiency for delta-wing AR≈4.5
}
```

**Skywalker vs Zephyr comparison:**

| Coefficient | Skywalker 2013 (SITL default) | Zephyr (Gazebo) |
|---|---|---|
| mass | 2.0 kg | 1.9 kg |
| s (wing area) | 0.45 m² | 0.50 m² |
| c_lift_a | 6.9 | 3.7 |
| c_lift_0 | 0.56 | 0.48 |
| c_drag_p | 0.10 | 0.064 |
| alpha_stall | 0.471 rad (27°) | 0.339 rad (19.4°) |
| AR | 7.85 | ~4.5 (delta wing) |

**Gazebo → SITL coefficient conversion:**

The Gazebo `LiftDragPlugin` formula is `CL = cla * (alpha + a0)` (a0 is *added*, not subtracted). The SITL `last_letter` formula is `CL = c_lift_0 + c_lift_a * alpha`. Mapping:
```
c_lift_0 = cla × a0  =  3.7 × 0.13  =  0.481 ≈ 0.48
c_lift_a = cla       =  3.7
```

**What TECS compensates for automatically:** differences in c_lift_0 and c_lift_a shift the trim AoA and pitch angle, but TECS adjusts throttle/pitch to maintain AIRSPEED_CRUISE regardless. The gross trajectory shape is the same; only fine details (cruise pitch angle, throttle set-point) differ.

**What TECS cannot compensate for:** moment coefficients (`c_l_*`, `c_m_*`, `c_n_*`) remain at Skywalker values — no Zephyr-specific data. These affect turn dynamics and stability margins but not the gross waypoint-to-waypoint path.

> **Confidence:** Confirmed — coefficients read from `ardupilot_gazebo/models/gazebo-zephyr/template/model.sdf` and `SIM_Plane.h`; SITL binary rebuilt and verified

---

## `eeprom.bin` Parameter Caching

ArduPilot SITL persists parameters to `simulator/ardupilot_logs/veh_N/eeprom.bin` between runs. If you change `.parm` files but SITL still uses old values, delete `eeprom.bin`:

```bash
rm simulator/ardupilot_logs/veh_*/eeprom.bin
```

This forces ArduPilot to reload all parameters from scratch on next launch.

> **Confidence:** Confirmed from current session

---

## JSON Model Files and ROMFS (Multicopter Frames)

Multicopter SITL physics can be overridden via a JSON file without modifying C++ source — unlike `SIM_Plane.cpp` which requires recompilation for aerodynamic coefficient changes.

**Model string format:** `"<frame_type>:@ROMFS/models/<name>.json"` e.g. `"x:@ROMFS/models/iris.json"`

**Critical gotcha — ROMFS is compiled in:** `@ROMFS/` paths are embedded into the binary at build time via `ap_romfs_embedded.h`. Adding a new JSON file after the binary was compiled will cause a `PANIC: <name>.json failed to load` crash.

**Absolute paths don't work as a bypass:** `AP_Filesystem_posix.cpp:map_filename()` strips the leading `/` from all paths in SITL mode, turning `/abs/path/file.json` into a relative path that fails to resolve.

**Auto-rebuild fix** (implemented in `simulator/external/sitl.py`): `_romfs_json_newer_than()` compares the mtime of every `*.json` in `ardupilot/Tools/autotest/models/` against the binary. `ensure_sitl_built` triggers a waf rebuild automatically when any JSON is newer. No manual action needed after adding or editing JSON model files.

JSON models live in `ardupilot/Tools/autotest/models/` (same directory as `Callisto.json`, `freestyle.json`).

> **Confidence:** Confirmed — crash reproduced, POSIX map_filename source read, auto-rebuild implemented and verified

---

## `_dump_critical_params` (logic.py)

Function at `simulator/logic.py:70–116`.

Requests named parameters from ArduPilot after startup and logs them between `=== PARAM DUMP START ===` and `=== PARAM DUMP END ===`. Used for post-run verification that the correct parm file was loaded. Results appear in `simulator/logs/logics/logic_N.log`.

> **Confidence:** Confirmed from repo
