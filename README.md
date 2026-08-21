# foheart-linux

Linux-native USB discovery, raw capture, replay, and normalized sensor models
for the FOHEART C1 / MC1508 router. The decoder supports only the statically
recovered bulk-HS `0x13`/format-0 block; real hardware validation is still
required.

## Ubuntu setup

```bash
sudo apt install libusb-1.0-0 libusb-1.0-0-dev
python -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
pytest
```

## Commands

```bash
python -m foheart.tools.discover
python -m foheart.tools.discover --config config/default.yaml
python -m foheart.tools.usb_dump --config config/default.yaml --pid 0x5751 --count 100 --read-size 0x1400 --output samples/c1_capture.bin --hex
python -m foheart.tools.replay samples/c1_capture.bin
python -m foheart.tools.monitor --mock --count 10
python -m foheart.tools.monitor --config config/default.yaml
```

No stream-start command is sent. `usb_dump` only performs reads after selecting
one unambiguous endpoint set from the active descriptors. Real sensor values are
never synthesized; generated samples are available only behind `--mock` and are
prominently labelled.

## Configuration

The built-in defaults mirror `config/default.yaml`. A user YAML file overrides
those defaults, and explicitly supplied CLI values override the YAML file:

```text
CLI > config file > defaults
```

For example:

```bash
python -m foheart.tools.monitor \
  --config config/default.yaml \
  --usb-mode bulk \
  --outer-frame raw
```

USB mode `auto` selects from descriptors. Explicit interface and endpoint
values are accepted only when they match the selected descriptor. Protocol mode
`auto` decodes only an exact recovered `0x13`/`0x88e` transfer and otherwise
falls back to raw output. `stream.mode: experimental` is recognized but always
refused before the device is opened; this project sends no start-stream or
polling command.

See `docs/configuration.md` for every key and CLI option. For a connected device,
`monitor --count N` performs `N` read-only attempts; without `--count`, it prints
resolved status and exits.

## Permissions

Copy and edit `99-foheart.rules.example` only after confirming the actual ID with
`lsusb`. Do not install the template blindly. After installing a verified rule,
reload udev rules and reconnect the router.

Protocol evidence and unresolved fields are tracked in
`docs/protocol_status.md`.
