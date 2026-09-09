# Validation, architecture and maintenance

## Source baseline

The standalone repository is a curated source snapshot of OpenFrameTap's
`ESP-Mosaico` branch at
`74dbd1ca44625f5ba1e7b66db6e273ec2adc09e7`.
The original repository and historical/private evidence stay intact.
A fresh Git history intentionally excludes ROCK/Linux code and historical
captures; it is not a claim of independently reinventing the protocols.

The original checkout's current ESP README and firmware README were refreshed
during extraction. Firmware C/C++ sources, manifests, sdkconfig defaults and
sanitized tests are preserved. Local paths and historical logs are not public
runtime dependencies.

## Evidence inherited from the baseline

The latest recorded firmware delivery reports 78 passing on-board cases,
validated camera session/keepalives, settings ACK plus state readback, and a
central-hold shadow test. Earlier owner acceptance covers physical four-way
motion, release-to-stop, Center/flip, relative head tracking, Function Button
photo and recording start/stop/save.

The latest settings tests restored the initial camera mode/format/zoom. They
did not create photo/video files to validate every selectable format. Full
fresh-device pairing confirmation, all firmware versions, sustained preview
frame rate and post-correction full-discharge gauge accuracy are not claimed.

The split changes repository packaging and documentation, not firmware behavior.
Existing hardware results are inherited evidence, not tests rerun from this
repository. No new flash, camera operation, or legacy Windows pytest run is
needed for documentation-only extraction.

## Component boundaries

- `oft_protocol`: independent DUML/CRC/reassembly, UDP envelope, typed profile,
  orientation and safety helpers; bounded parsers retain unknown data.
- `main/oft_ble.c` and `oft_network.c`: discovery, session, per-session credential
  transfer and STA ownership. Credentials are erased from temporary buffers.
- `main/oft_udp.c`: single writer, ACK/heartbeat scheduling, telemetry, motion,
  typed camera settings and media queueing.
- `main/oft_touch.c`, `oft_motion.c`, `oft_ui.c`: fresh physical input,
  attitude acquisition, LVGL menus and state snapshots.
- `main/oft_video.cpp`: low-priority software preview decode. Keep the accepted
  FRESH policy and touch isolation unless a separate measured change is intended.

The current pairing request contains reference application identity/PIN constants
in `oft_pocket3.c`; these are not a user's Wi-Fi password or account token.
This inherited shared identity is an interoperability limitation, not a
production per-device enrollment design. Do not change it casually or assume
that it grants authorization on a new camera.

## Tests and diagnostics

Unity cases live under `components/oft_protocol/test/` and run on the MCU.
Tests cover framing, CRC, resynchronization, sequence wrap, command bounds,
watchdog, input, settings and related state guards.
Do not substitute successful compilation for physical control acceptance.

The explicit `mosaico-serial.py --selftest` path requires a disconnected camera
and released decoder resources. Prepare that state before running it; it is
not a general-purpose command for an active recording session.

Maintain 250 ms input watchdog, single-writer ownership, bounded queues,
release/cancel stopping and fail-closed behavior. A failed radio cannot deliver
a guaranteed stop. Keep diagnostics bounded and avoid automatic shooting or
arbitrary opcode experiments.

## Publication hygiene

Do not commit captures, screenshots of private scenes, packet dumps, Wi-Fi
credentials, NVS/flash reads, device-specific logs or generated binaries.
Only small sanitized wire fixtures belong in Git. Preserve their hashes,
direction and source metadata.

Before distributing binaries, assemble third-party license notices for the exact
SDK/components and review codec distribution requirements. The root MIT license
does not replace these licenses.

The local battery profile is specific to the confirmed 65 mAh board battery;
RAM correction requires backups, readback and rollback. Never write OTP,
change charger hardware or force a deep discharge as a test.
