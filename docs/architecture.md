# Architecture

```text
FOHEART sensors -> C1 router -> libusb/PyUSB -> C1Device/transport
                                             -> C1ProtocolParser
                                             -> normalized SensorFrame
                                             -> calibration (later)
                                             -> skeleton solver (later)
                                             -> G1 retargeting (out of scope)
```

## This phase

- **USB transport:** discovers devices, reads real descriptors, rejects ambiguous
  interfaces, claims one interface, and performs bulk or interrupt transfers.
- **C1 protocol:** one parser boundary receives complete raw transfer payloads.
  It fail-closed decodes only the statically recovered exact-length bulk-HS
  `0x13`/format-0 variant; all other layouts remain unsupported.
- **Sensor normalization:** protocol-independent dataclasses represent sensor
  quaternions and IMU vectors. Quaternion order remains optional for unknown
  formats; the recovered fixed record is explicitly `wxyz`.

## Later phases

- **Calibration** consumes normalized samples; it does not know USB offsets.
- **Skeleton solver** consumes calibrated sensor orientations.
- **G1 retargeting** consumes solved human poses. It is not part of this package.

Mock samples are created after the parser boundary and are always labelled. Raw
recordings preserve transfer timestamps and endpoint addresses so parser work can
be repeated without hardware.
