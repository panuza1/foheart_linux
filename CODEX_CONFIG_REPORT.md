# FOHEART runtime configuration report

This change adds validated YAML configuration, shared CLI overrides, descriptor-
checked USB selection, protocol auto-selection, and raw/read-only fallback. No
USB write path was added or enabled. Hardware was not connected.

## Files changed

Created:

- `config/default.yaml`
- `src/foheart/config.py`
- `tests/test_config.py`
- `docs/configuration.md`
- `CODEX_CONFIG_REPORT.md`

Updated:

- `pyproject.toml`
- `README.md`
- `CODEX_IMPLEMENTATION_REPORT.md`
- `src/foheart/usb/descriptors.py`
- `src/foheart/usb/c1_device.py`
- `src/foheart/protocol/parser.py`
- `src/foheart/protocol/__init__.py`
- `src/foheart/tools/discover.py`
- `src/foheart/tools/usb_dump.py`
- `src/foheart/tools/monitor.py`
- `tests/test_discovery.py`
- `tests/test_protocol.py`

## Config structure

`RuntimeConfig` contains frozen dataclasses for:

```text
USBConfig
  mode
  pid
  interface
  in_endpoint
  out_endpoint
  timeout_ms
  read_size

ProtocolConfig
  outer_frame
  sensor_id_mode

StreamConfig
  mode

MonitorConfig
  show_raw
  show_euler
  show_quaternion
  show_imu
```

Runtime built-ins mirror `config/default.yaml`. YAML uses `yaml.safe_load` and
rejects unknown sections/keys, invalid types, unsupported choices, non-positive
timeouts/sizes, and non-router PIDs. Decimal and hexadecimal integers are
accepted. Internal `None` represents YAML/CLI `auto` for optional numeric
fields.

PyYAML `>=6` was added as the only new dependency.

## CLI options added

All three runtime tools—`discover`, `usb_dump`, and `monitor`—accept:

```text
--config
--usb-mode
--pid
--interface
--in-endpoint
--out-endpoint
--timeout-ms
--read-size
--outer-frame
--sensor-id-mode
--stream-mode
```

The monitor additionally accepts boolean pairs such as
`--show-raw`/`--no-show-raw` for all monitor display fields. Existing dump
options (`--vid`, `--count`, `--output`, `--hex`) and monitor mock options remain
compatible.

## Precedence behavior

Resolution order is:

```text
explicit CLI option > selected YAML file > built-in defaults
```

Common argparse options use suppressed defaults, so omitted CLI fields do not
overwrite YAML. Explicit `--pid auto`, `--interface auto`, endpoint `auto`, or
`--read-size auto` can intentionally reset a fixed YAML value.

The default file is not required at runtime; it is a checked, user-editable
mirror of the built-in defaults. A test ensures both remain identical.

## Auto-detection behavior

- Only router PIDs `0x5751` and `0x5851` are eligible for `C1Device` open.
- `usb.mode: auto` chooses BULK or INTERRUPT only from the active descriptors.
- `bulk` prefers PID `0x5751`; `hid` prefers PID `0x5851`.
- PID preference does not bypass descriptor checks: bulk still requires a bulk
  IN/OUT set and HID still requires an interrupt IN set.
- More than one remaining device or endpoint candidate produces a diagnostic
  error instead of first-match selection.
- Explicit interface and endpoint values filter descriptor candidates. Errors
  include both the requested and available sets.
- Explicit read size changes the transfer buffer capacity; automatic read size
  retains the descriptor-derived default.

Discovery may display known out-of-scope FOHEART products, but marks them out of
scope and never passes them to the C1 runtime.

## Fallback behavior

`resolve_outer_frame()` selects `fixed_0x13` only when a transfer has all three
recovered properties:

```text
byte 0 = 0x13
length = 0x88e
format byte 5 = 0
```

`auto` and configured `fixed_0x13` both fall back to raw on any mismatch.
`raw` never invokes the parser. A parser error in live monitor mode is also
caught and converted to labelled raw output.

Sensor ID `auto` resolves to provisional `loop_index`, the safest currently
implemented identity. `decoded_index` explicitly applies the recovered packed
index expression. `unknown` retains the slot only. Monitor output labels each
case and does not call it a confirmed physical sensor ID.

## Read-only enforcement

The only enabled stream mode is `read_only`. `experimental` is accepted by the
schema and CLI solely so every tool can refuse it with:

```text
Experimental USB write path is disabled because
the C1 start-stream command is not validated.
```

This check runs before device discovery/open in `discover`, `usb_dump`, and
`monitor`. No tool sends `0x70`, `0x73`, RTTRANS, control transfers, or any other
USB write. The existing low-level transport write method was not expanded or
called.

## Tests added

Thirteen new test cases cover:

- built-in/default YAML equality;
- YAML overlay loading;
- CLI precedence and explicit reset to `auto`;
- hexadecimal PID, interface, endpoint, and read-size parsing;
- invalid USB mode;
- invalid outer-frame mode;
- invalid sensor-ID mode;
- rejection of ChargePlate/MC1507 PIDs;
- descriptor override application and mismatch rejection;
- automatic bulk and interrupt selection;
- exact fixed-frame recognition and raw fallback;
- decoded packed-index behavior;
- experimental stream refusal.

CLI smoke checks also confirmed every required option appears on all three tools,
an experimental dump refuses before device access, and PID `0x5752` is rejected.

## Actual pytest result

Command:

```bash
env -u PYTHONPATH .venv/bin/pytest -q
```

Actual result:

```text
..............................                                           [100%]
30 passed in 0.13s
```

## Example commands

Use the supplied defaults:

```bash
python -m foheart.tools.monitor --config config/default.yaml
```

Override YAML values:

```bash
python -m foheart.tools.monitor \
  --config config/default.yaml \
  --usb-mode bulk \
  --pid 0x5751 \
  --outer-frame raw
```

Inspect descriptors under the same selection policy:

```bash
python -m foheart.tools.discover \
  --config config/default.yaml \
  --interface auto \
  --in-endpoint auto
```

Read-only raw capture with the statically observed bulk-HS buffer capacity:

```bash
python -m foheart.tools.usb_dump \
  --config config/default.yaml \
  --usb-mode bulk \
  --pid 0x5751 \
  --read-size 0x1400 \
  --outer-frame auto \
  --count 100 \
  --output samples/c1_capture.bin
```

## Known limitations

- No C1 was attached, so device preference, active descriptors, transfers, and
  parser fallback are not hardware validated.
- Automatic descriptor read size is `wMaxPacketSize`; a larger logical transfer
  buffer such as statically observed `0x1400` must be selected explicitly until
  real captures establish a safe device-specific default.
- Fixed `0x13` parsing is partial and not treated as universal.
- Physical sensor identity remains unvalidated.
- Monitor without `--count` prints resolved status and exits; positive `--count`
  performs that many read-only attempts.
- Start-stream initialization remains UNKNOWN. A silent read-only C1 may only
  time out, and no write fallback exists.

=== INFORMATION FOR CHATGPT ===

Config implemented: YES
Default config path: config/default.yaml
USB modes: auto, bulk, hid
Outer frame modes: auto, fixed_0x13, raw
Sensor ID modes: auto, loop_index, decoded_index, unknown
Stream modes: read_only; experimental recognized but disabled

CLI precedence: CLI override > config file > built-in/default config
Descriptor validation: YES; explicit values must match an unambiguous descriptor candidate
Raw fallback: YES; non-matching or rejected fixed_0x13 transfers remain raw
Experimental writes enabled: NO

Tests: 30 passed in 0.13s with env -u PYTHONPATH .venv/bin/pytest -q
Current blocker: no connected C1; start-stream command remains UNKNOWN and disabled
Ready for real C1 validation: YES, for descriptor inspection and read-only capture only
