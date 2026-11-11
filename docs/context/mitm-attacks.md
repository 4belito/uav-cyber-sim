# MITM Attack Taxonomy

The adversarial scenarios the simulator ships, how they are wired, and which
notebook runs each one. The attack logic lives in two places: Remote ID spoof
profiles (beacon forgery) and pluggable man-in-the-middle (MITM) strategies
(link tampering). The user-facing summary is the **Cyberattack Scenarios**
section of the top-level `README.md`; this file is the implementation-level
reference.

## Where the code lives

| Piece | File | Role |
|---|---|---|
| MITM proxy process | `simulator/mitm.py` | Interposed on one vehicle's GCS↔Logic links when its config enables MITM. |
| Strategy base + registry | `simulator/runtime/mitm/strategies.py` | `MITMStrategy` with `on_downlink` / `on_uplink` hooks; subclasses register by name. |
| Strategy exports | `simulator/runtime/mitm/__init__.py` | Public list of strategy classes. |
| RID sniffer | `simulator/runtime/mitm/rid_sniffer.py` | Passive overhearing of the Remote ID feed, used by pursuit attacks. |
| Spoof profile | `simulator/entities/spoof_profile.py` | Remote ID beacon forgery (not MITM). |

A `MITMStrategy` sees every intercepted message twice over: `on_downlink`
(telemetry Logic → GCS) and `on_uplink` (commands GCS → Logic). Each hook
returns the message to forward it, a modified message, or `None` to drop it.
Strategies may also inject their own messages through the `MITMContext` handed
to `bind()` at startup; injected commands are spoofed as the GCS (srcSystem
255) over the MITM's Logic-facing command connection.

## Strategy catalog

Registered name → class → behavior. The registry name is what a vehicle config
uses; notebooks usually construct the class directly and assign it to
`vehicle.mitm`.

| Registry name | Class | Behavior |
|---|---|---|
| `passthrough` | `PassthroughStrategy` | Forwards everything unchanged. Baseline / proof the tap is transparent; control case for the rest. |
| `blackout` | `BlackoutStrategy` | Drops all telemetry and all commands. Keeps only the keep-alive `HEARTBEAT` and the `LOGIC_DONE` / `LOGIC_FAILED` completion signal so the sim does not deadlock. Denial of service. |
| `intervention` | `InterventionStrategy` | The core hijack. Runs a real GCS `Intervention` (a `Trigger` plus a guided `InterventionPlan`) from the MITM position, driven by the same `InterventionRunner` the GCS uses. `MissionTrigger` gives the classic visible takeover; `ProximityTrigger` hands control back when the condition clears. `stealth=True` attaches a `StealthModeMask`. |
| `hijack_pursuit` | `HijackPursuitStrategy` | Hijacks, then streams `SET_POSITION_TARGET_GLOBAL_INT` to steer the victim after another vehicle. Learns the target position passively via a `RIDSniffer` on the victim's own RID feed, so the Oracle needs no changes. Target RID only arrives while the two are within the Oracle's transmission range. |
| `spoof_gcs` | `SpoofGCSStrategy` | Once `seq >= trigger_seq`, injects fake position reports toward the GCSs selected by `target_mask`, so the operator sees the drone where it is not. |
| `spoof_owner_gcs` | `SpoofOwnerGCSStrategy` | `SpoofGCSStrategy` fixed to target only the owner GCS (index 0); no `target_mask`. |
| `cloak_position` | `CloakPositionStrategy` | Passes real telemetry until a `Trigger` holds, then freezes the GCS's view at that fix, `altitude_offset` metres higher (reads as "climbed over the obstacle"). The vehicle is untouched — Logic reads SITL directly, so the real AUTO mission flies on. Reuses the same `Trigger` type a GCS guard would, so "near" matches exactly. |
| `ghost_mission` | `GhostMissionStrategy` | Stealth hijack plus fabricated position: after the hijack engages, every `GLOBAL_POSITION_INT` is replaced with a fake fix dead-reckoned along the planned route. The map shows the drone flying its mission while it is driven elsewhere. Naive by design — only position is faked; `MISSION_CURRENT`, `NAV_CONTROLLER_OUTPUT`, `VFR_HUD` still carry the truth, and a bent route is cut-cornered. |

### `StealthModeMask` (helper, not a strategy)

`StealthModeMask` hides an attacker-induced flight-mode change from the GCS.
The giveaway of a hijack is the downlink `HEARTBEAT` reporting a new
`custom_mode` (e.g. AUTO `3` → GUIDED `4`). Dropping heartbeats would stall the
GCS (it blocks on `wait_heartbeat`), so the mask instead **rewrites** the mode
fields back to the last pre-hijack mode, and **drops** any `STATUSTEXT` that
names the hijack mode (a firmware-dependent second tell — ArduPlane announces
mode names; default ArduCopter SITL does not). Engagement is a one-way latch:
`observe()` keeps the cover mode current while inactive, `engage()` freezes it
the moment the hijack fires, `mask()` scrubs every message thereafter.

## Attack families → notebooks

| Family | Notebooks |
|---|---|
| Adversarial autonomy (pursuit-evasion) | `7-attack_pursuit` |
| Remote ID spoofing (beacon forgery) | `8-attack_RID_spoofing`, `8-attack_RID_spoofing_city` |
| MITM baseline / reconnaissance | `9-attack_mitm_passthrough` |
| Denial of service | `10-attack_mitm_blackout` |
| Command injection / takeover | `11-attack-mitm_hijack`, `11-attack_mitm_hijack_fleet`, `11-attack_mitm_hijack_fleet_random` |
| False data injection (GCS position view) | `12-mitm_spoof_gcs`, `12-mitm_spoof_gcs_proximity` |
| Takeover + weaponization (hijack → pursuit) | `13-mitm_hijack_pursuit`, `14-mitm_hijack_pursuit_fleet` |
| Takeover + evasion (stealth hijack) | `15-mitm_hijack_stealth` |
| False data injection (ghost mission) | `16-mitm_naive_data_injection_to_GCS` |

> `cloak_position` (`CloakPositionStrategy`) is exercised by the proximity
> spoof notebook alongside `spoof_gcs`; it shares the family with the GCS
> position-spoof scenarios above.

## Scope

All of this runs against ArduPilot SITL inside the simulator. No real aircraft
or live radio links are involved; the scenarios exist to study detection,
deconfliction, and system-level resilience for multi-UAV operations.
