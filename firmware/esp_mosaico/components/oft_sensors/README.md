# Sensor dependency provenance

BMM150 SensorAPI files are unmodified Bosch BSD-3-Clause sources, pinned to
`0dce0617873cda1f6d51f6b7b961fdc2641e0c7c` from
https://github.com/boschsensortec/BMM150_SensorAPI . Retain the supplied LICENSE.
Each physical BMM150 uses its own device/context and factory trim compensation;
the board wrapper does not use a process-global I2C address adapter.

BMI270 uses the pinned `espressif/bmi270`1.1.0 registry component. Sensor config
loading is the normal volatile BMI270 initialization, not camera firmware update
or sensor OTP calibration. ESP-IDF itself is not changed.
