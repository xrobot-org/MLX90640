#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: XRobot Module for Melexis MLX90640 32x24 thermal IR array sensor
depends: []
=== END MANIFEST === */
// clang-format on

#include <array>
#include <cstddef>
#include <cstdint>

#include "i2c.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

class MLX90640
{
 public:
  static constexpr std::size_t WIDTH = 32;
  static constexpr std::size_t HEIGHT = 24;
  static constexpr std::size_t PIXEL_COUNT = WIDTH * HEIGHT;

  enum class RefreshRate : uint8_t
  {
    HZ_0_5 = 0,
    HZ_1 = 1,
    HZ_2 = 2,
    HZ_4 = 3,
    HZ_8 = 4,
    HZ_16 = 5,
    HZ_32 = 6,
    HZ_64 = 7,
  };

  struct ThermalFrame
  {
    uint32_t frame_counter = 0;
    float ambient_temperature = 0.0f;
    float reflected_temperature = 0.0f;
    float emissivity = 0.95f;
    uint8_t subpage = 0;
    uint8_t mode = 1;
    std::array<float, PIXEL_COUNT> temperature = {};
  };

  struct ThermalImage
  {
    uint32_t frame_counter = 0;
    float min_value = 0.0f;
    float max_value = 0.0f;
    std::array<float, PIXEL_COUNT> image = {};
  };

  struct ThermalStats
  {
    uint32_t frame_counter = 0;
    float ambient_temperature = 0.0f;
    float reflected_temperature = 0.0f;
    float supply_voltage = 0.0f;
    float min_temperature = 0.0f;
    float max_temperature = 0.0f;
    float average_temperature = 0.0f;
    float center_temperature = 0.0f;
    uint16_t min_index = 0;
    uint16_t max_index = 0;
    uint8_t bad_pixel_count = 0;
    bool ready = false;
  };

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

  MLX90640(
      LibXR::I2C& i2c,
      LibXR::RamFS* ramfs = nullptr,
      const Param& param = {.refresh_rate = MLX90640::RefreshRate::HZ_8, .emissivity = 0.95f, .reflected_temperature_shift = 8.0f, .use_chess_mode = true, .temperature_topic_name = "mlx90640_temperature", .image_topic_name = "mlx90640_image", .stats_topic_name = "mlx90640_stats", .i2c_address = 0x33});

  void OnMonitor() {}

 private:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x33;
  static constexpr std::size_t AUX_COUNT = 64U;
  static constexpr std::size_t FRAME_COUNT = PIXEL_COUNT + AUX_COUNT + 2U;
  static constexpr std::size_t EEPROM_COUNT = 832U;
  static constexpr std::size_t PIXEL_WORD_COUNT = PIXEL_COUNT;
  static constexpr std::size_t AUX_WORD_COUNT = AUX_COUNT;
  static constexpr std::size_t EEPROM_WORD_COUNT = EEPROM_COUNT;
  static constexpr std::size_t BAD_PIXEL_TABLE_SIZE = 5U;

  static constexpr int EEPROM_I2C_FREQ = 100000;
  static constexpr int RUNTIME_I2C_FREQ = 400000;

  static constexpr uint16_t STATUS_REGISTER = 0x8000;
  static constexpr uint16_t CONTROL_REGISTER = 0x800D;
  static constexpr uint16_t EEPROM_START_ADDRESS = 0x2400;
  static constexpr uint16_t PIXEL_DATA_START_ADDRESS = 0x0400;
  static constexpr uint16_t AUX_DATA_START_ADDRESS = 0x0700;
  static constexpr uint16_t INIT_STATUS_VALUE = 0x0030;

  static constexpr uint16_t STATUS_FRAME_MASK = 0x0001;
  static constexpr uint16_t STATUS_DATA_READY_MASK = 0x0008;
  static constexpr uint16_t CONTROL_TRIGGER_MASK = 0x8000;
  static constexpr uint16_t CONTROL_REFRESH_MASK = static_cast<uint16_t>(~(0x7U << 7U));
  static constexpr uint16_t CONTROL_RESOLUTION_MASK =
      static_cast<uint16_t>(~(0x3U << 10U));
  static constexpr uint16_t CONTROL_MODE_MASK = 0x1000;

  static constexpr std::size_t CONTROL_WORD_INDEX = PIXEL_COUNT + AUX_COUNT;
  static constexpr std::size_t SUBPAGE_WORD_INDEX = CONTROL_WORD_INDEX + 1U;

  static constexpr std::size_t COMPENSATION_PIXEL0_INDEX = 776U;
  static constexpr std::size_t GAIN_INDEX = 778U;
  static constexpr std::size_t COMPENSATION_PIXEL1_INDEX = 808U;
  static constexpr std::size_t VDD_INDEX = 810U;
  static constexpr std::size_t PTAT_ART_INDEX = 768U;
  static constexpr std::size_t PTAT_INDEX = 800U;

  static constexpr uint32_t WORKER_STACK_BYTES = 16384;
  static constexpr uint32_t DATA_READY_TIMEOUT_MS = 3000;
  static constexpr uint32_t MANUAL_TRIGGER_DELAY_MS = 1000;
  static constexpr uint32_t DATA_READY_POLL_DELAY_MS = 10;
  static constexpr uint8_t SUBPAGE_MASK_COMPLETE = 0x03;
  static constexpr uint32_t CENTER_PIXEL_INDEX = (HEIGHT / 2U) * WIDTH + (WIDTH / 2U);
  static_assert(PIXEL_COUNT == WIDTH * HEIGHT, "MLX90640 pixel count mismatch");
  static_assert(FRAME_COUNT == 834U, "MLX90640 frame size mismatch");
  static_assert(EEPROM_COUNT == 832U, "MLX90640 EEPROM size mismatch");

  struct CalibrationData
  {
    std::array<uint16_t, EEPROM_COUNT> words = {};
    int16_t k_vdd = 0;
    int16_t vdd_25 = 0;
    float kv_ptat = 0.0f;
    float kt_ptat = 0.0f;
    uint16_t v_ptat_25 = 0;
    float alpha_ptat = 0.0f;
    int16_t gain_ee = 0;
    float tgc = 0.0f;
    float cp_kv = 0.0f;
    float cp_kta = 0.0f;
    uint8_t resolution_ee = 0;
    uint8_t calibration_mode_ee = 0;
    float ks_ta = 0.0f;
    std::array<float, 5> ks_to = {};
    std::array<int16_t, 5> ct = {};
    std::array<uint16_t, PIXEL_COUNT> alpha = {};
    uint8_t alpha_scale = 0;
    std::array<int16_t, PIXEL_COUNT> offset = {};
    std::array<int8_t, PIXEL_COUNT> kta = {};
    uint8_t kta_scale = 0;
    std::array<int8_t, PIXEL_COUNT> kv = {};
    uint8_t kv_scale = 0;
    std::array<float, 2> cp_alpha = {};
    std::array<int16_t, 2> cp_offset = {};
    std::array<float, 3> il_chess_c = {};
    std::array<uint16_t, BAD_PIXEL_TABLE_SIZE> broken_pixels = {0xFFFF, 0xFFFF, 0xFFFF,
                                                                0xFFFF, 0xFFFF};
    std::array<uint16_t, BAD_PIXEL_TABLE_SIZE> outlier_pixels = {0xFFFF, 0xFFFF, 0xFFFF,
                                                                 0xFFFF, 0xFFFF};
    bool valid = false;
    bool calibrated_temperature = false;
  };

  struct FrameBuffer
  {
    std::array<uint16_t, FRAME_COUNT> words = {};
    uint16_t status_register = 0;
    uint16_t control_register = 0;
    uint8_t subpage = 0;
    bool valid = false;
  };

  struct ProcessingResult
  {
    float ambient_temperature = 25.0f;
    float reflected_temperature = 17.0f;
    float supply_voltage = 3.3f;
    bool calibrated_temperature = false;
    std::array<float, PIXEL_COUNT> temperature = {};
    std::array<float, PIXEL_COUNT> image = {};
  };

  static bool IsDataReady(uint16_t status_register);
  static uint8_t GetSubpage(uint16_t status_register);
  static uint16_t SetRefreshRateBits(uint16_t control_register, uint8_t refresh_rate);
  static uint16_t SetTriggerBit(uint16_t control_register);
  static uint16_t SetModeBits(uint16_t control_register, bool use_chess_mode);
  static uint8_t GetMode(uint16_t control_register);
  static uint8_t GetModeRaw(uint16_t control_register);
  static uint8_t GetResolution(uint16_t control_register);

  static float Pow2(int exponent);
  static int16_t ReadSignedWord(uint16_t word);
  static uint8_t Byte(uint16_t value, uint8_t index);
  static uint8_t Nibble(uint16_t value, uint8_t index);
  static LibXR::ErrorCode CheckAdjacentPixels(uint16_t pixel_a, uint16_t pixel_b);
  static float GetMedian(std::array<float, 4> values);

  static void ExtractVddParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                   CalibrationData& calibration);
  static void ExtractPtatParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                    CalibrationData& calibration);
  static void ExtractGainParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                    CalibrationData& calibration);
  static void ExtractTgcParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                   CalibrationData& calibration);
  static void ExtractResolutionParameters(
      const std::array<uint16_t, EEPROM_COUNT>& ee_data, CalibrationData& calibration);
  static void ExtractKsTaParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                    CalibrationData& calibration);
  static void ExtractKsToParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                    CalibrationData& calibration);
  static void ExtractCpParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                  CalibrationData& calibration);
  static void ExtractAlphaParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                     CalibrationData& calibration);
  static void ExtractOffsetParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                      CalibrationData& calibration);
  static void ExtractKtaPixelParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                        CalibrationData& calibration);
  static void ExtractKvPixelParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                       CalibrationData& calibration);
  static void ExtractCilcParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                    CalibrationData& calibration);
  static LibXR::ErrorCode ExtractDeviatingPixels(
      const std::array<uint16_t, EEPROM_COUNT>& ee_data, CalibrationData& calibration);
  static bool HasUsableEepromDump(const std::array<uint16_t, EEPROM_COUNT>& words);

  static void WorkerEntry(MLX90640* self);
  void WorkerLoop();

  bool WarmUpFrameBuffers();
  bool ReadNextSubpage(uint8_t& subpage);
  bool AcquireAndPublishFrame();
  void ApplyCurrentFrame(uint8_t subpage);
  void UpdateStats();
  void LogStatsIfDue();
  float ComputeReflectedTemperature() const;
  uint8_t CountBadPixels() const;
  int PrintStatsLocked() const;

  bool TrySetRefreshRate(RefreshRate refresh_rate);
  bool ConfigureMode(bool use_chess_mode);
  void PrintUsage() const;
  int HandleCommand(int argc, char** argv);
  static int CommandFunc(MLX90640* self, int argc, char** argv);

  LibXR::ErrorCode ReadWords(uint8_t slave_addr, uint16_t start_address,
                             uint16_t word_count, uint16_t* data);
  LibXR::ErrorCode WriteWord(uint8_t slave_addr, uint16_t write_address, uint16_t data);
  void SetFrequency(int freq);
  LibXR::ErrorCode GeneralReset();
  bool DumpEeprom();
  bool ParseCalibration();
  bool CaptureSubpage(FrameBuffer& frame);
  void ProcessFrame(const FrameBuffer& frame, float emissivity,
                    float reflected_temperature_shift, ProcessingResult& result) const;
  bool SetRefreshRate(uint8_t refresh_rate);
  bool SetMode(bool use_chess_mode);
  bool SynchronizeFrameStatus();
  bool WaitForDataReady(uint16_t& status_register);
  bool TriggerMeasurement();
  bool ValidateFrame(const FrameBuffer& frame) const;
  bool ValidateAuxData(const FrameBuffer& frame) const;
  float GetAmbientTemperature(const FrameBuffer& frame) const;
  float GetSupplyVoltage(const FrameBuffer& frame) const;
  void CalculateImage(const FrameBuffer& frame, ProcessingResult& result) const;
  void CalculateTemperature(const FrameBuffer& frame, float emissivity,
                            float reflected_temperature, ProcessingResult& result) const;
  void CorrectBadPixels(const std::array<uint16_t, BAD_PIXEL_TABLE_SIZE>& pixels,
                        std::array<float, PIXEL_COUNT>& field, uint8_t mode) const;
  bool IsPixelBad(uint16_t pixel) const;

  RefreshRate refresh_rate_;
  float emissivity_;
  float reflected_temperature_shift_;
  bool use_chess_mode_;
  uint8_t i2c_address_ = DEFAULT_ADDRESS;

  LibXR::Topic topic_temperature_;
  LibXR::Topic topic_image_;
  LibXR::Topic topic_stats_;
  LibXR::I2C* i2c_ = nullptr;

  LibXR::RamFS::File cmd_file_;
  bool cmd_registered_ = false;

  LibXR::Thread worker_;
  LibXR::Mutex mutex_;
  LibXR::Semaphore i2c_sem_;
  LibXR::ReadOperation i2c_read_block_;
  LibXR::WriteOperation i2c_write_block_;

  uint8_t current_subpage_ = 0;
  uint8_t current_mode_ = 1;
  LibXR::ErrorCode last_error_ = LibXR::ErrorCode::OK;
  uint32_t frame_counter_ = 0;
  uint32_t last_stats_log_ms_ = 0;

  std::array<uint8_t, FRAME_COUNT * 2U> raw_buffer_ = {};
  CalibrationData calibration_ = {};
  FrameBuffer frame_buffer_ = {};
  ProcessingResult processing_result_ = {};

  ThermalFrame thermal_frame_ = {};
  ThermalImage image_frame_ = {};
  ThermalStats stats_ = {};
};
