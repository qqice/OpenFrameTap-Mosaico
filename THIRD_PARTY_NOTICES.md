# License scope and third-party notices

The root MIT license applies to project-owned ESP-Mosaico firmware, Mosaico
development scripts and their documentation, except files carrying another
license. It does **not** relicense third-party code or the historical
ROCK/Linux implementation remaining in the original OpenFrameTap repository.

## Source included or referenced

| Component | Version / source | License handling |
| --- | --- | --- |
| Cisco OpenH264 | v2.6.0, commit `652bdb7719f30b52b08e506645a7322ff1b2cc6f`; [upstream](https://github.com/cisco/openh264) | BSD-2-Clause; keep the submodule LICENSE and notices when redistributing source or binaries |
| Experimental CABAC adaptations | `firmware/esp_mosaico/components/oft_openh264/compat/cabac32.cpp` and `cabac_compact.cpp` | Derived portions retain their Cisco notices and BSD-2-Clause headers; not relicensed MIT |
| Bosch BMM150 SensorAPI | commit `0dce0617873cda1f6d51f6b7b961fdc2641e0c7c`; [upstream](https://github.com/boschsensortec/BMM150_SensorAPI) | BSD-3-Clause; LICENSE is included beside the vendor files |

OpenH264's checked-out source is unmodified. The adapter can generate explicitly
instrumented build copies; no claim that the resulting binary is an official
Cisco-distributed binary is made. This repository does not assess codec patent
rights or offer a patent license; review upstream distribution terms before
shipping binaries.

## Components fetched at build time

ESP-IDF, TinyUSB, LVGL, the CO5300 and CST9217-compatible drivers, BMI270 and their
transitive components retain their respective licenses. Exact registry versions
and hashes are recorded in `firmware/esp_mosaico/dependencies.lock`; direct
requirements are in `main/idf_component.yml`. Their source is fetched by the
ESP-IDF component manager, not copied into this repository.

When distributing firmware binaries, collect the applicable LICENSE / NOTICE
files from the exact SDK and managed component versions used for that build.
This source repository does not include a binary-release license bundle.

## Protocol and board references

The Mosaico implementation grew from the original
[OpenFrameTap](https://github.com/qqice/openframetap) Pocket 3 work.
Protocol research also consulted
[OpenPocketCine](https://github.com/erik-sutton95/OpenPocketCine) (camera settings
reference revision `9c4e7334`),
[lib-osmo-ble](https://github.com/yigitkonur/lib-osmo-ble),
[djictl](https://github.com/xaionaro-go/djictl), and
[reverse-engineering-dji](https://github.com/xaionaro/reverse-engineering-dji).
These repositories are references, not bundled dependencies or blanket grants
to redistribute their implementations.

Board wiring follows Espressif's Mosaico V1.0 configuration; actual display
initialization is provided by the pinned public driver. Keep existing provenance
comments and sanitized test-fixture metadata when modifying the code.
