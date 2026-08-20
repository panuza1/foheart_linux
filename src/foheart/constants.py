FOHEART_VID = 0x1483

PRODUCTS = {
    0x5751: "FOHEART C1 Router",
    0x5851: "FOHEART C1 HID Router",
    0x5752: "FOHEART C1 ChargePlate",
    0x5753: "FOHEART C1 MC1507 Sensor",
    0x5853: "FOHEART C1 HID MC1507 Sensor",
}

ROUTER_PIDS = frozenset((0x5751, 0x5851))


def is_foheart_product(vid: int, pid: int) -> bool:
    return vid == FOHEART_VID and pid in PRODUCTS

