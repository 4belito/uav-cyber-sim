# ArduCopter SITL: copter-iris Frame

SITL-only equivalent of `gazebo-iris` (no Gazebo required). Follows the same pattern as `plane-zephyr` for the fixed-wing case.

## Files Created

| File | Purpose |
|---|---|
| `ardupilot/Tools/autotest/models/iris.json` | Iris physics model (mass, inertia, geometry) |
| `ardupilot/Tools/autotest/default_params/copter-iris.parm` | ArduCopter params loaded after `copter.parm` |
| vehicleinfo.py entry | Registers frame as `copter-iris` |

## vehicleinfo.py Registration

```python
"copter-iris": {
    "model": "x:@ROMFS/models/iris.json",
    "waf_target": "bin/arducopter",
    "default_params_filename": [
        "default_params/copter.parm",
        "default_params/copter-iris.parm",
    ],
},
```

Usage: `model="copter-iris"`, `firmware="ArduCopter"` in `SimVehicle`.

## iris.json Physics Parameters

Derived from `ardupilot_gazebo/models/gazebo-iris/base/model.sdf`:

```json
{
    "mass": 1.9,
    "diagonal_size": 0.494,
    "hoverThrOut": 0.35,
    "disc_area": 0.181,
    "propExpo": 0.65,
    "mdrag_coef": 0.2,
    "moment_inertia": [0.008, 0.015, 0.017]
}
```

- `mass`: 1.5 (base) + 0.15 (imu) + 0.15 (odometry) + 4×0.025 (rotors) = 1.9 kg (SITL default was 3.0 kg)
- `diagonal_size`: rotor_0 to rotor_1 tip-to-tip = sqrt(0.26²+0.42²) = 0.494 m
- `disc_area`: 4 × π × 0.12² = 0.181 m²
- `hoverThrOut`: 0.35 (observed 34% throttle in Gazebo log)
- `moment_inertia`: Ixx=0.008, Iyy=0.015, Izz=0.017 kg·m² from SDF

## Critical copter-iris.parm Parameters

```
# FRAME_TYPE: copter.parm sets 0 (PLUS); Iris is X-frame — must override
FRAME_CLASS 1
FRAME_TYPE  1

# MOT_THST_HOVER: Gazebo hovers at ~34%; copter.parm default is 0.39
MOT_THST_HOVER  0.35
MOT_THST_EXPO   0.65

# Battery: 3S LiPo
MOT_BAT_VOLT_MAX 12.6
MOT_BAT_VOLT_MIN 9.6

# EKF drag: mass=1.9, disc_area=0.181, Cd≈0.6 → bcoef=1.9/(0.181×0.6)≈17.7
EK3_DRAG_BCOEF_X  17.7
EK3_DRAG_BCOEF_Y  17.7
EK3_DRAG_MCOEF    0.180

# Suppress geofence circle in QGC (copter.parm sets FENCE_RADIUS=150)
FENCE_RADIUS 0
```

## What NOT to include (Gazebo-specific)

`gazebo-iris.parm` has `PLND_ENABLED 1`, `PLND_TYPE 3`, `RNGFND1_TYPE 1`, `SIM_SONAR_SCALE 10`, `RC8_OPTION 39`. These are for IRLock precision landing using Gazebo's camera/beacon plugin — they have no effect in SITL and `RNGFND1_TYPE 1` causes a moving rangefinder-coverage circle in QGC's fly view.

## ROMFS: Binary Must Be Rebuilt After Adding iris.json

`@ROMFS/` data is compiled into the binary at build time via `ap_romfs_embedded.h`. SITL's POSIX filesystem **strips the leading `/`** from absolute paths (`map_filename()` in `AP_Filesystem_posix.cpp:58`), so absolute-path bypasses don't work.

Fix implemented in `simulator/external/sitl.py`: `_romfs_json_newer_than()` checks if any `*.json` in `ardupilot/Tools/autotest/models/` is newer than the binary. If so, `ensure_sitl_built` triggers a rebuild automatically — no manual intervention needed.

```python
def _romfs_json_newer_than(binary_path: Path) -> bool:
    models_dir = ARDUPILOT_PATH / "Tools" / "autotest" / "models"
    binary_mtime = binary_path.stat().st_mtime
    return any(
        json_file.stat().st_mtime > binary_mtime
        for json_file in models_dir.glob("*.json")
    )
```

## Trajectory Equivalence: copter-iris vs gazebo-iris

| Metric | Gazebo | SITL | Δ |
|---|---|---|---|
| AUTO duration | 95.0 s | 92.0 s | −3 s |
| Altitude mean | 7.90 m | 7.88 m | −0.01 m |
| Speed mean | 4.15 m/s | 4.31 m/s | +0.16 m/s |
| Throttle mean | 32% | 34% | +2% |
| xtrack mean | 0.09 m | 0.14 m | +0.05 m |
| xtrack max | 9.3 m | 5.2 m | −4.1 m |
| WP5 timing lag | — | +2.9 s | cumulative |

Remaining differences are fundamental simulation gaps (not fixable by parameters):
1. **Throttle/speed offset**: SITL linear thrust vs Gazebo RPM+PID motor loop
2. **Waypoint timing drift**: same motor response difference (same pattern as plane-zephyr)
3. **Altitude/xtrack**: essentially identical

## eeprom.bin Cache

Same rule as for plane: delete after changing `.parm` files:
```bash
rm simulator/ardupilot_logs/veh_*/eeprom.bin
```
