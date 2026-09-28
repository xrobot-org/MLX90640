# MLX90640

XRobot Module for the Melexis MLX90640 32x24 thermal IR array sensor (I2C).

During construction the module sends an I2C general-call reset, reads the
EEPROM at 100 kHz, extracts the calibration parameters (including the broken
and outlier pixel lists), sets the readout mode (chess or interleaved) and the
refresh rate, switches the I2C bus to 400 kHz and reads frames until both
subpages have been seen. EEPROM, calibration or configuration failures and a
failed warm-up stop with `ASSERT`.

The `mlx90640` worker thread (16 KiB stack, `MEDIUM` priority) then repeatedly
waits for data-ready (polling every 10 ms, manual trigger after 1 s, timeout
after 3 s), reads each subpage, drops subpages that fail the frame / auxiliary
data validation, and once both subpages of a frame have been read computes the
temperatures and image, corrects the bad pixels and publishes the three topics.
A bus error or a frame that cannot be assembled in 8 attempts stops with
`ASSERT`. The module logs a statistics line every second.

Note: the module changes the I2C bus clock, which affects every device on that bus.

The calibration and temperature calculation are adapted from the Melexis
MLX90640 driver library; see [NOTICE](NOTICE) for third-party attribution.

## Published topics

| Topic (default name) | Type | Content |
| --- | --- | --- |
| `mlx90640_temperature` | `MLX90640::ThermalFrame` | Frame counter, ambient and reflected temperature (°C), emissivity, last subpage, mode (1 = chess, 0 = interleaved) and 768 pixel temperatures (°C, row-major 32x24) |
| `mlx90640_image` | `MLX90640::ThermalImage` | Frame counter and the 768 compensated IR image values (Melexis "image", relative, not °C) with their min / max |
| `mlx90640_stats` | `MLX90640::ThermalStats` | Frame counter, ambient / reflected temperature, supply voltage (V), min / max (with pixel index) / average / center temperature, number of EEPROM-flagged bad pixels, `ready` |

The reflected temperature is the ambient temperature minus
`reflected_temperature_shift`.

## Shell command

If `ramfs` is not `nullptr`, the module adds the command `mlx90640` to it.

```sh
mlx90640 [help]                      # usage
mlx90640 stats                       # print the current statistics
mlx90640 show <count> <interval_ms>  # print them count times (interval clamped to 10..5000 ms)
mlx90640 refresh <0-7>               # set the refresh rate (RefreshRate value)
mlx90640 emissivity <0.1-1.0>        # set the emissivity
```

## Dependencies

No other Modules; LibXR only.

## Constructor

```cpp
struct Param
{
  RefreshRate refresh_rate;
  float emissivity;
  float reflected_temperature_shift;
  bool use_chess_mode;
  const char* temperature_topic_name;
  const char* image_topic_name;
  const char* stats_topic_name;
  uint8_t i2c_address;
};

MLX90640(LibXR::I2C& i2c, LibXR::RamFS* ramfs,
         const Param& param = {.refresh_rate = MLX90640::RefreshRate::HZ_8,
                               .emissivity = 0.95f,
                               .reflected_temperature_shift = 8.0f,
                               .use_chess_mode = true,
                               .temperature_topic_name = "mlx90640_temperature",
                               .image_topic_name = "mlx90640_image",
                               .stats_topic_name = "mlx90640_stats",
                               .i2c_address = 0x33});
```

Dependencies:

- `i2c`: the I2C bus the sensor is on (its clock is set to 100 kHz, then 400 kHz).
- `ramfs`: RamFS that receives the `mlx90640` command; `nullptr` registers no command.

Configuration (`Param`):

- `refresh_rate`: `MLX90640::RefreshRate`, `HZ_0_5`, `HZ_1`, `HZ_2`, `HZ_4`,
  `HZ_8`, `HZ_16`, `HZ_32`, `HZ_64` (register values 0..7), default `HZ_8`.
- `emissivity`: object emissivity, clamped to 0.1..1.0, default 0.95.
- `reflected_temperature_shift`: reflected temperature = ambient minus this
  value, °C, default 8.0.
- `use_chess_mode`: `true` chess readout, `false` interleaved, default `true`.
- `temperature_topic_name`, `image_topic_name`, `stats_topic_name`: topic names,
  defaults `mlx90640_temperature`, `mlx90640_image`, `mlx90640_stats`.
- `i2c_address`: 7-bit sensor address, default `0x33`.

## Use

```sh
xrobot module add xrobot-org/MLX90640
xrobot setup
xrobot instance add xrobot-org/MLX90640
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/MLX90640
    id: mlx90640_0
    args:
      - i2c: i2c1
      - ramfs: ramfs
      - param:
          refresh_rate: MLX90640::RefreshRate::HZ_8
          emissivity: 0.95f
          reflected_temperature_shift: 8.0f
          use_chess_mode: 'true'
          temperature_topic_name: '"mlx90640_temperature"'
          image_topic_name: '"mlx90640_image"'
          stats_topic_name: '"mlx90640_stats"'
          i2c_address: '0x33'
```

BSP side:

```cpp
XR_REGISTER(i2c1, LibXR::I2C);
XR_REGISTER(ramfs, LibXR::RamFS);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/MLX90640` in a BSP, prints the current
constructor.
