# Configured body sensor mapping

FOHEART physical identity semantics remain `UNKNOWN`. The software maps stable
session-local logical slots to anatomical roles only after operator selection.
This is `CONFIGURED`, never hardware-discovered anatomy.

| Concept | Meaning |
|---|---|
| transport/header key | opaque source key; real `0x15` bytes 1..4 are a candidate with `UNKNOWN` semantics |
| logical slot | registry-owned `slot_N`, stable for that registry/session |
| body role | operator-confirmed wearing location |
| physical sensor label | optional sticker/name typed by a human |
| skeleton segment/joint | downstream FK semantic, not a sensor ID |

## Profiles

Upper mode remains backward compatible and requires seven roles:

```text
torso
left_upper_arm, left_forearm, left_hand
right_upper_arm, right_forearm, right_hand
```

Full mode requires exactly 17:

```text
head
left_shoulder, right_shoulder
torso
pelvis
left_upper_arm, right_upper_arm
left_forearm, right_forearm
left_hand, right_hand
left_thigh, right_thigh
left_lower_leg, right_lower_leg
left_foot, right_foot
```

Unknown roles, missing required roles, duplicate roles/YAML keys, duplicate
slots, and duplicate saved transport-key bindings are rejected. A full mapping
cannot be loaded as upper, or vice versa. Legacy version-1 upper files without a
`profile` field load as `profile: upper`.

## Logical slots and collision behavior

`LogicalSlotRegistry` allocates the first unused `slot_N` for a new opaque key
and retains the binding; packet sequence never determines identity. The
architecture is not capped at seven and is tested through `slot_16`.

Diagnostics cover:

- conflicting data under the same candidate key (`collision`, hard error);
- actual slot reassignment (`refused`, hard error);
- a new key after a session is marked running (warning);
- a new key after registry freeze (hard error);
- saved keys that never reappear;
- timestamp-based disappearance/staleness.

Two real devices with byte-identical unknown keys cannot be separated from the
current wire evidence. They yield fewer unique slots than required, so the
17-slot monitor gate, mapping, calibration, and full viewer fail. The software
does not manufacture physical IDs.

## Interactive mapping

Software exercise:

```bash
python -m foheart.tools.map_body_sensors \
  --mode full --synthetic \
  --output /tmp/foheart-full-map.yaml
```

Future real run:

```bash
python -m foheart.tools.map_body_sensors \
  --mode full \
  --output config/my_full_body_mapping.yaml
```

The command first requires 17 stable slots, prints them, then prompts for every
role in the order above. An unknown or already-used slot is refused. The output
path must not exist.

Upper mode remains:

```bash
python -m foheart.tools.map_body_sensors \
  --mode upper --output config/my_upper_mapping.yaml
```

## Motion-assisted assignment

Add `--motion-assisted`. For each role, the command asks the operator to move
only that body segment, captures a bounded window, and reports a unique
strongest motion-energy slot when available:

```text
Move only LEFT THIGH now.
Press ENTER to begin detection.

Strongest moving slot: slot_11 (energy ...)
Assign left_thigh -> slot_11 ? [y/N]
```

Only `y`/`yes` confirms. Rejection, ambiguity, a used slot, or no motion returns
to manual choice. Motion magnitude never silently determines anatomy.

## Versioned full-body YAML

```yaml
version: 1
profile: full
status: CONFIGURED

body_mapping:
  head: slot_0
  left_shoulder: slot_1
  right_shoulder: slot_2
  torso: slot_3
  pelvis: slot_4
  left_upper_arm: slot_5
  right_upper_arm: slot_6
  left_forearm: slot_7
  right_forearm: slot_8
  left_hand: slot_9
  right_hand: slot_10
  left_thigh: slot_11
  right_thigh: slot_12
  left_lower_leg: slot_13
  right_lower_leg: slot_14
  left_foot: slot_15
  right_foot: slot_16

logical_slots:
  slot_0:
    transport_key:
      kind: hid_0x15_header_bytes_1_4
      value: 00dd0314
      evidence_status: UNKNOWN
    physical_label: optional-head-sticker
```

The example slot numbers are illustrative only. The interactive tool records
the actual session allocation. `logical_slots` metadata remains separate from
`body_mapping`; a sticker does not change transport or anatomy semantics.

## Manual-derived placement guide

The approximate wearing locations below come from FOHEART documentation and are
labelled `MANUAL_DERIVED`, not project physical validation:

- head: head-mounted sensor;
- shoulders: left/right shoulder locations;
- torso: back/upper torso;
- pelvis: waist/crotch/pelvis location;
- upper arms: outside upper arm;
- forearms: outside forearm near the wrist, not on the wrist joint;
- hands: back of each hand;
- thighs: outside middle thigh;
- lower legs: below the knee where muscle influence is minimized;
- feet: on shoes/feet according to the wearing layout; keep the documented
  guide-column/orientation direction consistent.

Do not infer these roles from proprietary sensor numbers.

## Exact future mapping gate

1. Connect C1 and power all 17 sensors.
2. Run `python -m foheart.tools.live_sensor_monitor --mode full
   --debug-transport`.
3. Verify 17 stable rows. Stop if fewer, stale, colliding, swapping, or newly
   appearing keys are observed.
4. Put sensors at the manual-derived locations.
5. Run `map_body_sensors --mode full`.
6. Move only the requested location when using assistance.
7. Confirm every assignment manually.
8. Save the `CONFIGURED` YAML and inspect its 17 unique mappings before
   calibration.
