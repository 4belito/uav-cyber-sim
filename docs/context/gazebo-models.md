# Gazebo Model Color Template System

## Directory Convention

Every model in `ardupilot_gazebo/models/<model>/` follows this layout:

```
<model>/
  physics/          ← static assets: SDF, meshes, textures (checked in)
  ardupilot/        ← model.sdf + model.config; references physics/ via <include>
  color_template/   ← Jinja2 .j2 templates + static assets; rendered per color at runtime
```

At runtime, `Gazebo._render_color_model()` renders `color_template/` into
`runtime_models/<model>/<color>/`, then `_generate_vehicle_models_from_bases()` copies
`ardupilot/` into `runtime_models/vehicle_N/` and patches:
- Model name: `<model name="...">` → `vehicle_N`
- Physics URI: `model://<model>/physics` → `model://<model>/<color>`
- FDM ports: `<fdm_port_in>` / `<fdm_port_out>`

## Jinja2 Rendering

`_render_color_model(model_name, color)` in `simulator/visualizer/gazebo/gazebo.py`:
- Walks `color_template/` recursively
- `.j2` files → rendered with `{"color": color.value}` → output strips `.j2` suffix
- Non-`.j2` files (e.g. `model.config`) → copied as-is
- Output directory: `RUNTIME_GAZEBO_MODELS / model_name / color.value`

Template variable: `{{ color }}` is always lowercase (e.g. `"red"`, `"blue"`).
Use `{{ color | capitalize }}` inside templates when Gazebo material names need title-case
(e.g. iris uses `Gazebo/{{ color | capitalize }}`).

## gazebo-iris: Gazebo Built-in Materials

`color_template/model.sdf.j2` uses:
- Rotors + legs: `<name>Gazebo/{{ color | capitalize }}</name>` (e.g. `Gazebo/Green`)
- Body: `<name>Gazebo/DarkGrey</name>` (fixed)
- Mesh URIs: `model://gazebo-iris/physics/meshes/*.dae`

No `.material` file needed — these are Gazebo 11 built-in OGRE materials.

## gazebo-zephyr: PNG Texture Materials

`color_template/model.sdf.j2` applies a `<material><script>` override to all 4 visuals
(wing, propeller, flap_left, flap_right):

```xml
<material>
  <script>
    <uri>model://gazebo-zephyr/{{ color }}/materials/scripts</uri>
    <uri>model://gazebo-zephyr/physics/materials/textures</uri>
    <name>zephyr_wing_{{ color }}</name>
  </script>
</material>
```

`color_template/materials/scripts/wing.material.j2` generates the OGRE script:
```
material zephyr_wing_{{ color }}
{
    receive_shadows off
    technique { pass { texture_unit {
        texture wing_{{ color }}.png
        filtering trilinear
    } } }
}
```

The `<uri>model://gazebo-zephyr/{{ color }}/materials/scripts</uri>` resolves to the
runtime-rendered directory; the texture URI resolves to `physics/materials/textures/`
where the PNGs live statically.

## Adding a New Color to gazebo-zephyr

1. Drop `wing_<color>.png` (512×512 RGB) into
   `ardupilot_gazebo/models/gazebo-zephyr/physics/materials/textures/`.
2. Add the color to `Color` enum in `simulator/config.py` if not present.
3. No code changes needed — the template renders `zephyr_wing_<color>` and
   `wing_<color>.png` automatically.

## GAZEBO_MODEL_PATH Search Order

`RUNTIME_GAZEBO_MODELS:ARDUPILOT_GAZEBO_MODELS`

Runtime dir is first, so `model://gazebo-zephyr/red/` resolves to the rendered runtime
color dir; `model://gazebo-zephyr/physics/` resolves to the static source dir.

## URI Replacement (Critical Detail)

`ardupilot/model.sdf` has:
```xml
<include>
  <uri>model://<model>/physics</uri>
  <pose>...</pose>   ← sibling element may be present
</include>
```

The replacement in `_generate_vehicle_models_from_bases` uses **targeted string replace**,
not regex, to avoid silently skipping when `<pose>` is present:

```python
sdf = sdf.replace(
    f"<uri>model://{veh.model}/physics</uri>",
    f"<uri>model://{veh.model}/{veh.color.value}</uri>",
)
```

A regex like `<include>\s*<uri>...</uri>\s*</include>` fails when there are sibling
elements inside `<include>` (the zephyr's `<pose>` child). This was the root cause of
the white-plane bug.
