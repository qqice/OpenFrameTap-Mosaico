# OpenFrameTap Mosaico

This repository is the independent ESP-Mosaico V1.0 / ESP32-S31 firmware.
Read README.md, docs/build.md and docs/development.md before changes.
Do not deploy to ROCK 4D or import its Linux runtime as a dependency.

Use the pinned ESP-IDF 6.1 preview target esp32s31; do not substitute esp32s3,
upgrade the SDK or patch SDK sources. Bound local builds to two jobs and run
firmware tests on the positively identified board. Do not run legacy Python
pytest or broad Windows media analysis.

Hardware actions require the current owner's authorization; historical lab
authorizations do not automatically authorize actions on another user's board.
Before flashing verify USB target, chip and partition layout. Preserve NVS,
bootloader, partition table, NAND and security configuration. Never erase all
flash or alter eFuses, OTP, charger hardware or camera calibration.

Preserve watchdog, bounded queues, single writer, typed commands and stop paths.
No automatic shooting. Restore camera settings after authorized bounded tests.
Keep preview changes separate from UI/control maintenance. Battery RAM correction
is specific to a confirmed 65 mAh cell and requires backup/readback/rollback.

Keep artifacts private and ignored; preserve vendor licenses and dependency locks.
Distinguish compilation, MCU tests, protocol replies and physically observed
behavior. Do not label inherited hardware evidence as a fresh test.
