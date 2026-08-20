# Configuration

All tools use the same validated YAML model. Runtime defaults are built into the
package and mirrored by `config/default.yaml`; passing `--config PATH` overlays
that file, then explicitly supplied CLI values overlay both.

```text
CLI override > selected YAML file > built-in defaults
```

Unknown sections, unknown keys, unsupported modes, non-positive sizes/timeouts,
and unsupported PIDs are rejected.

## YAML keys

### `usb`

| Key | Default | Accepted values | Behavior |
|---|---|---|---|
| `mode` | `auto` | `auto`, `bulk`, `hid` | `auto` trusts descriptors; `bulk` requires a bulk endpoint set and prefers PID `5751`; `hid` requires interrupt endpoints and prefers PID `5851` |
| `pid` | `auto` | `auto`, `0x5751`, `0x5851` | Select either supported router or one explicit router; ChargePlate and MC1507 sensor PIDs are rejected |
| `interface` | `auto` | `auto`, integer | Descriptor-derived unless explicitly constrained |
| `in_endpoint` | `auto` | `auto`, integer/hex | Descriptor-derived unless explicitly constrained; direction and transfer type must match |
| `out_endpoint` | `auto` | `auto`, integer/hex | Descriptor-derived unless explicitly constrained; interrupt OUT may be absent when automatic |
| `timeout_ms` | `1000` | positive integer | Read timeout only |
| `read_size` | `auto` | `auto`, positive integer/hex | Automatic uses the descriptor packet size; `0x1400` is the statically observed high-speed bulk buffer capacity, not a universal default |

If multiple devices, interfaces, alternate settings, or endpoint sets remain
after filtering, the runtime stops with diagnostics. It never selects the first
ambiguous descriptor silently.

### `protocol`

| Key | Default | Accepted values | Behavior |
|---|---|---|---|
| `outer_frame` | `auto` | `auto`, `fixed_0x13`, `raw` | `auto` uses the fixed decoder only for exact code `0x13`, length `0x88e`, format `0`; `fixed_0x13` requests the same validated shape; any mismatch falls back to raw; `raw` never decodes |
| `sensor_id_mode` | `auto` | `auto`, `loop_index`, `decoded_index`, `unknown` | `auto` resolves to provisional loop index; `decoded_index` applies the recovered packed-index calculation; `unknown` labels only the record slot |

`loop_index`, `decoded_index`, and slot values are not claimed as validated
physical sensor identities. Monitor output labels their evidence level.

### `stream`

| Key | Default | Accepted values | Behavior |
|---|---|---|---|
| `mode` | `read_only` | `read_only`, `experimental` | Only `read_only` runs. `experimental` is recognized solely to produce a clear refusal before USB open |

No configuration enables writes. The tools do not send `0x70`, `0x73`,
RTTRANS, control transfers, or a start-stream command.

### `monitor`

| Key | Default | Accepted values |
|---|---:|---|
| `show_raw` | `false` | boolean |
| `show_euler` | `true` | boolean |
| `show_quaternion` | `true` | boolean |
| `show_imu` | `true` | boolean |

The monitor equivalents are `--show-raw`/`--no-show-raw`,
`--show-euler`/`--no-show-euler`, `--show-quaternion`/`--no-show-quaternion`,
and `--show-imu`/`--no-show-imu`.

## Shared CLI overrides

`discover`, `usb_dump`, and `monitor` accept:

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

Integer USB values accept decimal or Python-style hexadecimal notation. `pid`,
`interface`, endpoint, and read-size overrides also accept `auto`, allowing a
CLI argument to reset a fixed YAML value.

Examples:

```bash
python -m foheart.tools.discover --config config/default.yaml

python -m foheart.tools.monitor \
  --config my-c1.yaml \
  --usb-mode bulk \
  --pid 0x5751 \
  --outer-frame auto

python -m foheart.tools.usb_dump \
  --config my-c1.yaml \
  --outer-frame raw \
  --count 100 \
  --output samples/c1_capture.bin
```

## Auto-detection and fallback

The runtime first restricts discovery to C1 router PIDs. `bulk` prefers `5751`
and `hid` prefers `5851`, but the actual transport is still required to match
the active descriptor. `auto` does not infer transport type from PID.

Endpoint overrides filter the descriptor candidates. A missing or wrong
interface, direction, transfer type, or endpoint produces an error containing
the requested and available descriptor sets.

For each incoming transfer, protocol `auto` checks only the recovered fixed
shape. A non-matching or parser-rejected transfer stays raw. This fallback is
read-only and does not attempt initialization.

## Evidence status

| Area | State |
|---|---|
| libusb interface/endpoints and transfer sizes in the audited DLL | confirmed statically; hardware validation pending |
| descriptor selection and override validation | implemented; hardware validation pending |
| fixed `0x13` parser | partial static recovery; not universal |
| loop/decoded sensor identity | partial; physical identity not validated |
| start-stream command | UNKNOWN and disabled |
| USB writes | disabled by every CLI configuration path |
