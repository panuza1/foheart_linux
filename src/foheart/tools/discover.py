from __future__ import annotations

import argparse

from foheart.config import (
    ConfigError,
    add_config_arguments,
    load_config_from_args,
    require_read_only,
)
from foheart.constants import FOHEART_VID, PRODUCTS, ROUTER_PIDS
from foheart.usb.discovery import discover_foheart_devices
from foheart.usb.descriptors import DescriptorSelectionError, select_endpoints


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Discover FOHEART USB devices")
    add_config_arguments(parser)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_config_from_args(args)
        require_read_only(config)
    except ConfigError as exc:
        print(f"Configuration error: {exc}")
        return 2

    result = discover_foheart_devices()
    for error in result.errors:
        print(error)
    if not result.devices:
        print("No FOHEART device found.\n\nSupported C1 routers:")
        for pid in sorted(ROUTER_PIDS):
            print(f"{FOHEART_VID:04x}:{pid:04x} {PRODUCTS[pid]}")
        print("\nKnown but out-of-scope FOHEART products:")
        for pid, name in PRODUCTS.items():
            if pid in ROUTER_PIDS:
                continue
            print(f"{FOHEART_VID:04x}:{pid:04x} {name}")
        return 0

    devices = [
        device
        for device in result.devices
        if config.usb.pid is None or device.pid == config.usb.pid
    ]
    if not devices:
        print("No device matched the configured C1 router selection.")
        return 0

    for device in devices:
        print(f"Device {device.vid:04x}:{device.pid:04x} {device.known_product_name}")
        print(f"  bus={device.bus} address={device.address} port_path={device.port_path}")
        print(
            f"  manufacturer={device.manufacturer!r} product={device.product!r} "
            f"serial={device.serial!r}"
        )
        for error in device.access_errors:
            print(f"  access error: {error}")
        if device.pid in ROUTER_PIDS:
            try:
                selection = select_endpoints(
                    device.configurations,
                    transfer_type={"bulk": "BULK", "hid": "INTERRUPT"}.get(
                        config.usb.mode
                    ),
                    interface_number=config.usb.interface,
                    in_endpoint=config.usb.in_endpoint,
                    out_endpoint=config.usb.out_endpoint,
                    read_size=config.usb.read_size,
                )
                print(
                    f"  mode={selection.transfer_type} interface={selection.interface_number} "
                    f"alternate={selection.alternate_setting} "
                    f"IN=0x{selection.in_endpoint:02x} OUT="
                    f"{f'0x{selection.out_endpoint:02x}' if selection.out_endpoint is not None else 'none'}"
                )
            except DescriptorSelectionError as exc:
                print(f"  transport selection: {exc}")
        else:
            print("  runtime support: OUT OF SCOPE (not a C1 router)")
        for configuration in device.configurations:
            print(f"  configuration {configuration.value}")
            for interface in configuration.interfaces:
                print(
                    f"    interface {interface.number} alternate setting "
                    f"{interface.alternate_setting} class=0x{interface.interface_class:02x}"
                )
                for endpoint in interface.endpoints:
                    print(
                        f"      endpoint address=0x{endpoint.address:02x} "
                        f"direction={endpoint.direction} type={endpoint.transfer_type} "
                        f"wMaxPacketSize={endpoint.max_packet_size}"
                    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
