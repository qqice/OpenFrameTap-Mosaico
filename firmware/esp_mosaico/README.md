# ESP-Mosaico firmware

Native ESP-IDF 6.1 firmware for the esp32s31 preview target.
See the [project README](../../README.md), [build guide](../../docs/build.md)
and [validation notes](../../docs/development.md) for current functionality.

The project includes normal-mode Pocket 3 control, relative yaw/pitch head tracking,
physical shutter, radial settings menus and experimental low-frame-rate H.264 preview.
The preview is software decoded, not a hardware H.264 path.

Build from the repository root with `./scripts/mosaico.ps1 build`.
Initialize the pinned OpenH264 submodule first. Never substitute esp32s3 for esp32s31.
The flash helper writes only the application at 0x20000 and assumes a separately
verified compatible factory partition layout. See the build guide before flashing.
