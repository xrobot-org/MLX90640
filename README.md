# MLX90640

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


Melexis MLX90640 I2C 32x24 thermal IR array sensor module for XRobot.

This module initializes the MLX90640 over I2C, loads EEPROM calibration data,
configures refresh rate and chess / interleaved mode, processes thermal frames
in a worker thread, and publishes calibrated temperature, raw thermal image, and
frame statistics topics.

The I2C name and address are constructor arguments, so projects may use other
hardware aliases or non-default sensor addresses if needed.

## Required Hardware

- `mlx90640_i2c`

## Constructor Arguments

- `refresh_rate`: default `MLX90640::RefreshRate::HZ_8`
- `emissivity`: default `0.95`
- `reflected_temperature_shift`: default `8.0`
- `use_chess_mode`: default `true`
- `temperature_topic_name`: default `"mlx90640_temperature"`
- `image_topic_name`: default `"mlx90640_image"`
- `stats_topic_name`: default `"mlx90640_stats"`
- `i2c_name`: default `"mlx90640_i2c"`
- `i2c_address`: default `0x33`

## Published Topics

- `temperature_topic_name`: `MLX90640::ThermalFrame`, 32x24 calibrated temperature frame
- `image_topic_name`: `MLX90640::ThermalImage`, raw thermal image frame
- `stats_topic_name`: `MLX90640::ThermalStats`, frame counter, ambient / reflected temperature, min / max / average / center temperature, and bad-pixel count

## Shell Commands

The module registers `mlx90640` in `RamFS` when `ramfs` is available.

- `mlx90640` or `mlx90640 help`: print command usage
- `mlx90640 stats`: print current frame statistics
- `mlx90640 rate <0-7>`: change refresh rate
- `mlx90640 mode chess|interleaved`: change readout mode

## XRobot Configuration Example

```yaml
- id: thermal_camera
  name: MLX90640
  constructor_args:
    refresh_rate: MLX90640::RefreshRate::HZ_8
    emissivity: 0.95
    reflected_temperature_shift: 8.0
    use_chess_mode: true
    temperature_topic_name: "mlx90640_temperature"
    image_topic_name: "mlx90640_image"
    stats_topic_name: "mlx90640_stats"
    i2c_name: "mlx90640_i2c"
    i2c_address: 0x33
```
