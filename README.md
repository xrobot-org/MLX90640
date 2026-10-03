# MLX90640

Melexis MLX90640 32x24 红外热成像传感器驱动模块 / Driver module for the Melexis MLX90640 32x24 thermal IR array sensor

## 1. 模块作用 / Purpose

构造时，MLX90640 先发送 I2C 广播复位，以 100 kHz 读取 EEPROM 并解析校准参数（包括坏点与离群点列表），设置读出模式（棋盘或交错）和刷新率，再把 I2C 总线切换到 400 kHz，读取帧直到两个子页都已出现。EEPROM、校准、配置读写失败或预热失败时以 `ASSERT` 终止。I2C 总线时钟的修改对该总线上的所有设备生效。

随后工作线程 `mlx90640`（栈 16 KiB，优先级 `MEDIUM`）循环等待数据就绪（每 10 ms 轮询，1 s 后手动触发，3 s 超时），读取各子页，丢弃帧校验或辅助数据校验失败的子页；两个子页都读到后计算温度与图像，修正坏点，并发布三个 Topic。总线错误或连续 8 次无法组成完整帧时以 `ASSERT` 终止。每秒输出一行统计日志。

校准与温度计算改编自 Melexis MLX90640 驱动库，第三方声明见 [NOTICE](NOTICE)。

Upon construction, MLX90640 sends an I2C general-call reset, reads the EEPROM at 100 kHz and parses the calibration parameters (including the broken and outlier pixel lists), sets the readout mode (chess or interleaved) and the refresh rate, switches the I2C bus to 400 kHz and reads frames until both subpages have been seen. EEPROM, calibration or configuration failures and a failed warm-up stop with `ASSERT`. The change of the I2C bus clock applies to every device on that bus.

The `mlx90640` worker thread (16 KiB stack, `MEDIUM` priority) then repeatedly waits for data-ready (polling every 10 ms, manual trigger after 1 s, timeout after 3 s) and reads each subpage, dropping subpages that fail the frame or auxiliary-data validation. Once both subpages of a frame have been read, it computes the temperatures and the image, corrects the bad pixels and publishes the three Topics. A bus error or a frame that cannot be assembled in 8 attempts stops with `ASSERT`. A statistics line is logged every second.

The calibration and temperature calculation are adapted from the Melexis MLX90640 driver library; see [NOTICE](NOTICE) for third-party attribution.

## 2. Shell 命令 / Shell Command

`ramfs` 非 `nullptr` 时，模块向其中添加命令 `mlx90640`。

```sh
mlx90640 [help]                      # 用法 / usage
mlx90640 stats                       # 打印当前统计 / print the current statistics
mlx90640 show <count> <interval_ms>  # 打印 count 次，间隔限制在 10..5000 ms / print count times, interval clamped to 10..5000 ms
mlx90640 refresh <0-7>               # 设置刷新率（RefreshRate 值） / set the refresh rate (RefreshRate value)
mlx90640 emissivity <0.1-1.0>        # 设置发射率 / set the emissivity
```

If `ramfs` is not `nullptr`, the module adds the command `mlx90640` to it.

## 3. 构造接口 / Constructor

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

依赖：

- `i2c`：传感器所在的 I2C 总线，时钟先设为 100 kHz，再设为 400 kHz。
- `ramfs`：接收 `mlx90640` 命令的 RamFS；为 `nullptr` 时不注册命令。

配置参数（`Param`）：

- `refresh_rate`：`MLX90640::RefreshRate`，可取 `HZ_0_5`、`HZ_1`、`HZ_2`、`HZ_4`、`HZ_8`、`HZ_16`、`HZ_32`、`HZ_64`（寄存器值 0 到 7），默认 `HZ_8`。
- `emissivity`：物体发射率，限制在 0.1 到 1.0，默认 0.95。
- `reflected_temperature_shift`：反射温度等于环境温度减去该值，单位 ℃，默认 8.0。
- `use_chess_mode`：`true` 为棋盘读出，`false` 为交错读出，默认 `true`。
- `temperature_topic_name`、`image_topic_name`、`stats_topic_name`：三个 Topic 的名称，默认 `mlx90640_temperature`、`mlx90640_image`、`mlx90640_stats`。
- `i2c_address`：传感器 7 位地址，默认 `0x33`。

Dependencies:

- `i2c`: the I2C bus the sensor is on; its clock is set to 100 kHz, then 400 kHz.
- `ramfs`: the RamFS that receives the `mlx90640` command; no command is registered when it is `nullptr`.

Configuration parameters (`Param`):

- `refresh_rate`: `MLX90640::RefreshRate`, one of `HZ_0_5`, `HZ_1`, `HZ_2`, `HZ_4`, `HZ_8`, `HZ_16`, `HZ_32`, `HZ_64` (register values 0 to 7), default `HZ_8`.
- `emissivity`: object emissivity, clamped to 0.1 to 1.0, default 0.95.
- `reflected_temperature_shift`: the reflected temperature equals the ambient temperature minus this value, in °C, default 8.0.
- `use_chess_mode`: `true` for chess readout, `false` for interleaved readout, default `true`.
- `temperature_topic_name`, `image_topic_name`, `stats_topic_name`: names of the three Topics, defaults `mlx90640_temperature`, `mlx90640_image`, `mlx90640_stats`.
- `i2c_address`: 7-bit sensor address, default `0x33`.

## 4. Topic

| Topic（默认名称） | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `mlx90640_temperature` | 发布 | `MLX90640::ThermalFrame` | 帧计数、环境温度与反射温度（℃）、发射率、最近子页、模式（1 为棋盘，0 为交错）和 768 个像素温度（℃，行优先 32x24） |
| `mlx90640_image` | 发布 | `MLX90640::ThermalImage` | 帧计数、768 个补偿后的红外图像值（Melexis 的 image，相对量）及其最小值与最大值 |
| `mlx90640_stats` | 发布 | `MLX90640::ThermalStats` | 帧计数、环境与反射温度、供电电压（V）、最低温（含像素索引）、最高温（含像素索引）、平均温度、中心温度、EEPROM 标记的坏点数、`ready` |

反射温度等于环境温度减去 `reflected_temperature_shift`。

| Topic (default name) | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `mlx90640_temperature` | Publish | `MLX90640::ThermalFrame` | Frame counter, ambient and reflected temperature (°C), emissivity, last subpage, mode (1 chess, 0 interleaved) and 768 pixel temperatures (°C, row-major 32x24) |
| `mlx90640_image` | Publish | `MLX90640::ThermalImage` | Frame counter and the 768 compensated IR image values (the Melexis "image", relative values) with their minimum and maximum |
| `mlx90640_stats` | Publish | `MLX90640::ThermalStats` | Frame counter, ambient and reflected temperature, supply voltage (V), lowest temperature (with pixel index), highest temperature (with pixel index), average temperature, center temperature, number of bad pixels flagged in the EEPROM, `ready` |

The reflected temperature equals the ambient temperature minus `reflected_temperature_shift`.

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/MLX90640` 写入的实例，`i2c` 与 `ramfs` 填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/MLX90640`, with `i2c` and `ramfs` set to names registered by the BSP's `XR_REGISTER` (Registration):

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
          use_chess_mode: true
          temperature_topic_name: "mlx90640_temperature"
          image_topic_name: "mlx90640_image"
          stats_topic_name: "mlx90640_stats"
          i2c_address: 0x33
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片 MLX90640 红外热成像传感器，接入支持 100 kHz 与 400 kHz 的 I2C 总线，7 位地址默认 `0x33`。

Dependencies: LibXR.

Hardware: one MLX90640 thermal IR array sensor on an I2C bus that supports 100 kHz and 400 kHz, with the 7-bit address defaulting to `0x33`.
