#include "MLX90640.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "i2c.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

static int ErrorCodeValue(LibXR::ErrorCode error) { return static_cast<int>(error); }

bool MLX90640::IsDataReady(uint16_t status_register)
{
  return (status_register & STATUS_DATA_READY_MASK) != 0U;
}

uint8_t MLX90640::GetSubpage(uint16_t status_register)
{
  return static_cast<uint8_t>(status_register & STATUS_FRAME_MASK);
}

uint16_t MLX90640::SetRefreshRateBits(uint16_t control_register, uint8_t refresh_rate)
{
  const auto refresh_bits = static_cast<uint16_t>((refresh_rate & 0x07U) << 7U);
  return static_cast<uint16_t>((control_register & CONTROL_REFRESH_MASK) | refresh_bits);
}

uint16_t MLX90640::SetTriggerBit(uint16_t control_register)
{
  return static_cast<uint16_t>(control_register | CONTROL_TRIGGER_MASK);
}

uint16_t MLX90640::SetModeBits(uint16_t control_register, bool use_chess_mode)
{
  if (use_chess_mode)
  {
    return static_cast<uint16_t>(control_register | CONTROL_MODE_MASK);
  }
  return static_cast<uint16_t>(control_register & ~CONTROL_MODE_MASK);
}

uint8_t MLX90640::GetMode(uint16_t control_register)
{
  return static_cast<uint8_t>((control_register & CONTROL_MODE_MASK) >> 12U);
}

uint8_t MLX90640::GetModeRaw(uint16_t control_register)
{
  return static_cast<uint8_t>((control_register & CONTROL_MODE_MASK) >> 5U);
}

uint8_t MLX90640::GetResolution(uint16_t control_register)
{
  return static_cast<uint8_t>((control_register & ~CONTROL_RESOLUTION_MASK) >> 10U);
}

float MLX90640::Pow2(int exponent) { return std::ldexp(1.0f, exponent); }

int16_t MLX90640::ReadSignedWord(uint16_t word) { return static_cast<int16_t>(word); }

constexpr float SCALE_ALPHA = 0.000001f;

uint8_t MLX90640::Byte(uint16_t value, uint8_t index)
{
  return static_cast<uint8_t>((value >> (index * 8U)) & 0xFFU);
}

uint8_t MLX90640::Nibble(uint16_t value, uint8_t index)
{
  return static_cast<uint8_t>((value >> (index * 4U)) & 0x0FU);
}

void MLX90640::ExtractVddParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                    CalibrationData& calibration)
{
  const auto k_vdd = static_cast<int8_t>(Byte(ee_data[51], 1U));
  auto vdd_25 = static_cast<int16_t>(Byte(ee_data[51], 0U));
  vdd_25 = static_cast<int16_t>(((vdd_25 - 256) << 5) - 8192);

  calibration.k_vdd = static_cast<int16_t>(32 * k_vdd);
  calibration.vdd_25 = vdd_25;
}

void MLX90640::ExtractPtatParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                     CalibrationData& calibration)
{
  float kv_ptat = static_cast<float>((ee_data[50] & 0xFC00U) >> 10U);
  if (kv_ptat > 31.0f)
  {
    kv_ptat -= 64.0f;
  }
  kv_ptat /= 4096.0f;

  float kt_ptat = static_cast<float>(ee_data[50] & 0x03FFU);
  if (kt_ptat > 511.0f)
  {
    kt_ptat -= 1024.0f;
  }
  kt_ptat /= 8.0f;

  calibration.kv_ptat = kv_ptat;
  calibration.kt_ptat = kt_ptat;
  calibration.v_ptat_25 = ee_data[49];
  calibration.alpha_ptat = static_cast<float>(ee_data[16] & 0xF000U) / Pow2(14) + 8.0f;
}

void MLX90640::ExtractGainParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                     CalibrationData& calibration)
{
  calibration.gain_ee = static_cast<int16_t>(ee_data[48]);
}

void MLX90640::ExtractTgcParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                    CalibrationData& calibration)
{
  calibration.tgc =
      static_cast<float>(static_cast<int8_t>(Byte(ee_data[60], 0U))) / 32.0f;
}

void MLX90640::ExtractResolutionParameters(
    const std::array<uint16_t, EEPROM_COUNT>& ee_data, CalibrationData& calibration)
{
  calibration.resolution_ee = static_cast<uint8_t>((ee_data[56] & 0x3000U) >> 12U);
}

void MLX90640::ExtractKsTaParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                     CalibrationData& calibration)
{
  calibration.ks_ta =
      static_cast<float>(static_cast<int8_t>(Byte(ee_data[60], 1U))) / 8192.0f;
}

void MLX90640::ExtractKsToParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                     CalibrationData& calibration)
{
  const int8_t step = static_cast<int8_t>(((ee_data[63] & 0x3000U) >> 12U) * 10);
  calibration.ct[0] = -40;
  calibration.ct[1] = 0;
  calibration.ct[2] = static_cast<int16_t>(Nibble(ee_data[63], 1U));
  calibration.ct[3] = static_cast<int16_t>(Nibble(ee_data[63], 2U));

  calibration.ct[2] = static_cast<int16_t>(calibration.ct[2] * step);
  calibration.ct[3] = static_cast<int16_t>(calibration.ct[2] + calibration.ct[3] * step);
  calibration.ct[4] = 400;

  const int32_t ks_to_scale = 1L << (Nibble(ee_data[63], 0U) + 8U);
  calibration.ks_to[0] = static_cast<float>(static_cast<int8_t>(Byte(ee_data[61], 0U))) /
                         static_cast<float>(ks_to_scale);
  calibration.ks_to[1] = static_cast<float>(static_cast<int8_t>(Byte(ee_data[61], 1U))) /
                         static_cast<float>(ks_to_scale);
  calibration.ks_to[2] = static_cast<float>(static_cast<int8_t>(Byte(ee_data[62], 0U))) /
                         static_cast<float>(ks_to_scale);
  calibration.ks_to[3] = static_cast<float>(static_cast<int8_t>(Byte(ee_data[62], 1U))) /
                         static_cast<float>(ks_to_scale);
  calibration.ks_to[4] = -0.0002f;
}

void MLX90640::ExtractCpParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                   CalibrationData& calibration)
{
  const uint8_t alpha_scale = static_cast<uint8_t>(Nibble(ee_data[32], 3U) + 27U);

  auto offset_sp0 = static_cast<int16_t>(ee_data[58] & 0x03FFU);
  if (offset_sp0 > 511)
  {
    offset_sp0 = static_cast<int16_t>(offset_sp0 - 1024);
  }

  auto offset_sp1 = static_cast<int16_t>((ee_data[58] & 0xFC00U) >> 10U);
  if (offset_sp1 > 31)
  {
    offset_sp1 = static_cast<int16_t>(offset_sp1 - 64);
  }
  offset_sp1 = static_cast<int16_t>(offset_sp1 + offset_sp0);

  auto alpha_sp0 = static_cast<int16_t>(ee_data[57] & 0x03FFU);
  if (alpha_sp0 > 511)
  {
    alpha_sp0 = static_cast<int16_t>(alpha_sp0 - 1024);
  }
  const float cp_alpha0 = static_cast<float>(alpha_sp0) / Pow2(alpha_scale);

  auto alpha_sp1 = static_cast<int16_t>((ee_data[57] & 0xFC00U) >> 10U);
  if (alpha_sp1 > 31)
  {
    alpha_sp1 = static_cast<int16_t>(alpha_sp1 - 64);
  }
  const float cp_alpha1 = (1.0f + static_cast<float>(alpha_sp1) / 128.0f) * cp_alpha0;

  const auto cp_kta = static_cast<int8_t>(Byte(ee_data[59], 0U));
  const auto kta_scale1 = static_cast<uint8_t>(Nibble(ee_data[56], 1U) + 8U);
  calibration.cp_kta = static_cast<float>(cp_kta) / Pow2(kta_scale1);

  const auto cp_kv = static_cast<int8_t>(Byte(ee_data[59], 1U));
  const auto kv_scale = Nibble(ee_data[56], 2U);
  calibration.cp_kv = static_cast<float>(cp_kv) / Pow2(kv_scale);

  calibration.cp_alpha[0] = cp_alpha0;
  calibration.cp_alpha[1] = cp_alpha1;
  calibration.cp_offset[0] = offset_sp0;
  calibration.cp_offset[1] = offset_sp1;
}

void MLX90640::ExtractAlphaParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                      CalibrationData& calibration)
{
  std::array<int, 24> acc_row = {};
  std::array<int, 32> acc_column = {};
  std::array<float, PIXEL_COUNT> alpha_temp = {};

  const uint8_t acc_rem_scale = Nibble(ee_data[32], 0U);
  const uint8_t acc_column_scale = Nibble(ee_data[32], 1U);
  const uint8_t acc_row_scale = Nibble(ee_data[32], 2U);
  uint8_t alpha_scale = static_cast<uint8_t>(Nibble(ee_data[32], 3U) + 30U);
  const int alpha_ref = ee_data[33];

  for (int i = 0; i < 6; ++i)
  {
    const int p = i * 4;
    acc_row[p + 0] = Nibble(ee_data[34 + i], 0U);
    acc_row[p + 1] = Nibble(ee_data[34 + i], 1U);
    acc_row[p + 2] = Nibble(ee_data[34 + i], 2U);
    acc_row[p + 3] = Nibble(ee_data[34 + i], 3U);
  }

  for (int& row : acc_row)
  {
    if (row > 7)
    {
      row -= 16;
    }
  }

  for (int i = 0; i < 8; ++i)
  {
    const int p = i * 4;
    acc_column[p + 0] = Nibble(ee_data[40 + i], 0U);
    acc_column[p + 1] = Nibble(ee_data[40 + i], 1U);
    acc_column[p + 2] = Nibble(ee_data[40 + i], 2U);
    acc_column[p + 3] = Nibble(ee_data[40 + i], 3U);
  }

  for (int& column : acc_column)
  {
    if (column > 7)
    {
      column -= 16;
    }
  }

  for (int row = 0; row < 24; ++row)
  {
    for (int column = 0; column < 32; ++column)
    {
      const int pixel = 32 * row + column;
      float value = static_cast<float>((ee_data[64 + pixel] & 0x03F0U) >> 4U);
      if (value > 31.0f)
      {
        value -= 64.0f;
      }

      value *= static_cast<float>(1U << acc_rem_scale);
      value = static_cast<float>(alpha_ref + (acc_row[row] << acc_row_scale) +
                                 (acc_column[column] << acc_column_scale)) +
              value;
      value /= Pow2(alpha_scale);
      value -=
          calibration.tgc * (calibration.cp_alpha[0] + calibration.cp_alpha[1]) / 2.0f;
      alpha_temp[pixel] = SCALE_ALPHA / value;
    }
  }

  float max_value = alpha_temp[0];
  for (float value : alpha_temp)
  {
    if (value > max_value)
    {
      max_value = value;
    }
  }

  alpha_scale = 0;
  while (max_value > 0.0f && max_value < 32767.4f && alpha_scale < 31U)
  {
    max_value *= 2.0f;
    ++alpha_scale;
  }

  for (std::size_t i = 0; i < PIXEL_COUNT; ++i)
  {
    calibration.alpha[i] =
        static_cast<uint16_t>(alpha_temp[i] * Pow2(alpha_scale) + 0.5f);
  }

  calibration.alpha_scale = alpha_scale;
}

LibXR::ErrorCode MLX90640::CheckAdjacentPixels(uint16_t pixel_a, uint16_t pixel_b)
{
  const auto line_a = static_cast<uint16_t>(pixel_a >> 5U);
  const auto line_b = static_cast<uint16_t>(pixel_b >> 5U);
  const auto column_a = static_cast<uint16_t>(pixel_a - (line_a << 5U));
  const auto column_b = static_cast<uint16_t>(pixel_b - (line_b << 5U));

  const int line_delta = static_cast<int>(line_a) - static_cast<int>(line_b);
  if (line_delta > -2 && line_delta < 2)
  {
    const int column_delta = static_cast<int>(column_a) - static_cast<int>(column_b);
    if (column_delta > -2 && column_delta < 2)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }
  }

  return LibXR::ErrorCode::OK;
}

void MLX90640::ExtractOffsetParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                       CalibrationData& calibration)
{
  std::array<int, 24> occ_row = {};
  std::array<int, 32> occ_column = {};

  const uint8_t occ_rem_scale = Nibble(ee_data[16], 0U);
  const uint8_t occ_column_scale = Nibble(ee_data[16], 1U);
  const uint8_t occ_row_scale = Nibble(ee_data[16], 2U);
  const auto offset_ref = static_cast<int16_t>(ee_data[17]);

  for (int i = 0; i < 6; ++i)
  {
    const int p = i * 4;
    occ_row[p + 0] = Nibble(ee_data[18 + i], 0U);
    occ_row[p + 1] = Nibble(ee_data[18 + i], 1U);
    occ_row[p + 2] = Nibble(ee_data[18 + i], 2U);
    occ_row[p + 3] = Nibble(ee_data[18 + i], 3U);
  }

  for (int& row : occ_row)
  {
    if (row > 7)
    {
      row -= 16;
    }
  }

  for (int i = 0; i < 8; ++i)
  {
    const int p = i * 4;
    occ_column[p + 0] = Nibble(ee_data[24 + i], 0U);
    occ_column[p + 1] = Nibble(ee_data[24 + i], 1U);
    occ_column[p + 2] = Nibble(ee_data[24 + i], 2U);
    occ_column[p + 3] = Nibble(ee_data[24 + i], 3U);
  }

  for (int& column : occ_column)
  {
    if (column > 7)
    {
      column -= 16;
    }
  }

  for (int row = 0; row < 24; ++row)
  {
    for (int column = 0; column < 32; ++column)
    {
      const int pixel = 32 * row + column;
      auto value = static_cast<int16_t>((ee_data[64 + pixel] & 0xFC00U) >> 10U);
      if (value > 31)
      {
        value = static_cast<int16_t>(value - 64);
      }
      value = static_cast<int16_t>(value * static_cast<int16_t>(1U << occ_rem_scale));
      calibration.offset[pixel] =
          static_cast<int16_t>(offset_ref + (occ_row[row] << occ_row_scale) +
                               (occ_column[column] << occ_column_scale) + value);
    }
  }
}

void MLX90640::ExtractKtaPixelParameters(
    const std::array<uint16_t, EEPROM_COUNT>& ee_data, CalibrationData& calibration)
{
  std::array<float, PIXEL_COUNT> kta_temp = {};
  std::array<int8_t, 4> kta_rc = {static_cast<int8_t>((ee_data[54] & 0xFF00U) >> 8U),
                                  static_cast<int8_t>((ee_data[55] & 0xFF00U) >> 8U),
                                  static_cast<int8_t>(ee_data[54] & 0x00FFU),
                                  static_cast<int8_t>(ee_data[55] & 0x00FFU)};

  const uint8_t kta_scale1_base = static_cast<uint8_t>(Nibble(ee_data[56], 1U) + 8U);
  const uint8_t kta_scale2 = Nibble(ee_data[56], 0U);

  for (int row = 0; row < 24; ++row)
  {
    for (int column = 0; column < 32; ++column)
    {
      const int pixel = 32 * row + column;
      const uint8_t split =
          static_cast<uint8_t>(2 * (pixel / 32 - (pixel / 64) * 2) + pixel % 2);

      float value = static_cast<float>((ee_data[64 + pixel] & 0x000EU) >> 1U);
      if (value > 3.0f)
      {
        value -= 8.0f;
      }
      value *= static_cast<float>(1U << kta_scale2);
      value = static_cast<float>(kta_rc[split]) + value;
      kta_temp[pixel] = value / Pow2(kta_scale1_base);
    }
  }

  float max_value = std::fabs(kta_temp[0]);
  for (float value : kta_temp)
  {
    max_value = std::max(max_value, std::fabs(value));
  }

  uint8_t kta_scale1 = 0;
  while (max_value > 0.0f && max_value < 63.4f && kta_scale1 < 31U)
  {
    max_value *= 2.0f;
    ++kta_scale1;
  }

  for (std::size_t i = 0; i < PIXEL_COUNT; ++i)
  {
    const float value = kta_temp[i] * Pow2(kta_scale1);
    calibration.kta[i] = static_cast<int8_t>(value < 0.0f ? value - 0.5f : value + 0.5f);
  }

  calibration.kta_scale = kta_scale1;
}

void MLX90640::ExtractKvPixelParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                        CalibrationData& calibration)
{
  std::array<float, PIXEL_COUNT> kv_temp = {};
  std::array<int8_t, 4> kv_terms = {};

  kv_terms[0] = static_cast<int8_t>(Nibble(ee_data[52], 3U));
  kv_terms[1] = static_cast<int8_t>(Nibble(ee_data[52], 1U));
  kv_terms[2] = static_cast<int8_t>(Nibble(ee_data[52], 2U));
  kv_terms[3] = static_cast<int8_t>(Nibble(ee_data[52], 0U));

  for (auto& term : kv_terms)
  {
    if (term > 7)
    {
      term = static_cast<int8_t>(term - 16);
    }
  }

  const uint8_t kv_scale_base = Nibble(ee_data[56], 2U);
  for (int row = 0; row < 24; ++row)
  {
    for (int column = 0; column < 32; ++column)
    {
      const int pixel = 32 * row + column;
      const uint8_t split =
          static_cast<uint8_t>(2 * (pixel / 32 - (pixel / 64) * 2) + pixel % 2);
      kv_temp[pixel] = static_cast<float>(kv_terms[split]) / Pow2(kv_scale_base);
    }
  }

  float max_value = std::fabs(kv_temp[0]);
  for (float value : kv_temp)
  {
    max_value = std::max(max_value, std::fabs(value));
  }

  uint8_t kv_scale = 0;
  while (max_value > 0.0f && max_value < 63.4f && kv_scale < 31U)
  {
    max_value *= 2.0f;
    ++kv_scale;
  }

  for (std::size_t i = 0; i < PIXEL_COUNT; ++i)
  {
    const float value = kv_temp[i] * Pow2(kv_scale);
    calibration.kv[i] = static_cast<int8_t>(value < 0.0f ? value - 0.5f : value + 0.5f);
  }

  calibration.kv_scale = kv_scale;
}

void MLX90640::ExtractCilcParameters(const std::array<uint16_t, EEPROM_COUNT>& ee_data,
                                     CalibrationData& calibration)
{
  calibration.calibration_mode_ee =
      static_cast<uint8_t>(((ee_data[10] & 0x0800U) >> 4U) ^ 0x80U);

  float il_chess_c0 = static_cast<float>(ee_data[53] & 0x003FU);
  if (il_chess_c0 > 31.0f)
  {
    il_chess_c0 -= 64.0f;
  }
  calibration.il_chess_c[0] = il_chess_c0 / 16.0f;

  float il_chess_c1 = static_cast<float>((ee_data[53] & 0x07C0U) >> 6U);
  if (il_chess_c1 > 15.0f)
  {
    il_chess_c1 -= 32.0f;
  }
  calibration.il_chess_c[1] = il_chess_c1 / 2.0f;

  float il_chess_c2 = static_cast<float>((ee_data[53] & 0xF800U) >> 11U);
  if (il_chess_c2 > 15.0f)
  {
    il_chess_c2 -= 32.0f;
  }
  calibration.il_chess_c[2] = il_chess_c2 / 8.0f;
}

LibXR::ErrorCode MLX90640::ExtractDeviatingPixels(
    const std::array<uint16_t, EEPROM_COUNT>& ee_data, CalibrationData& calibration)
{
  calibration.broken_pixels.fill(0xFFFFU);
  calibration.outlier_pixels.fill(0xFFFFU);

  uint16_t pixel = 0;
  uint16_t broken_count = 0;
  uint16_t outlier_count = 0;

  while (pixel < PIXEL_COUNT && broken_count < BAD_PIXEL_TABLE_SIZE &&
         outlier_count < BAD_PIXEL_TABLE_SIZE)
  {
    const auto ee_word = ee_data[pixel + 64U];
    if (ee_word == 0)
    {
      calibration.broken_pixels[broken_count++] = pixel;
    }
    else if ((ee_word & 0x0001U) != 0U)
    {
      calibration.outlier_pixels[outlier_count++] = pixel;
    }
    ++pixel;
  }

  if (broken_count > 4U)
  {
    return LibXR::ErrorCode::CHECK_ERR;
  }
  if (outlier_count > 4U)
  {
    return LibXR::ErrorCode::CHECK_ERR;
  }
  if ((broken_count + outlier_count) > 4U)
  {
    return LibXR::ErrorCode::CHECK_ERR;
  }

  for (uint16_t i = 0; i < broken_count; ++i)
  {
    for (uint16_t j = i + 1; j < broken_count; ++j)
    {
      const LibXR::ErrorCode error =
          CheckAdjacentPixels(calibration.broken_pixels[i], calibration.broken_pixels[j]);
      if (error != LibXR::ErrorCode::OK)
      {
        return error;
      }
    }
  }

  for (uint16_t i = 0; i < outlier_count; ++i)
  {
    for (uint16_t j = i + 1; j < outlier_count; ++j)
    {
      const LibXR::ErrorCode error = CheckAdjacentPixels(calibration.outlier_pixels[i],
                                                         calibration.outlier_pixels[j]);
      if (error != LibXR::ErrorCode::OK)
      {
        return error;
      }
    }
  }

  for (uint16_t i = 0; i < broken_count; ++i)
  {
    for (uint16_t j = 0; j < outlier_count; ++j)
    {
      const LibXR::ErrorCode error = CheckAdjacentPixels(calibration.broken_pixels[i],
                                                         calibration.outlier_pixels[j]);
      if (error != LibXR::ErrorCode::OK)
      {
        return error;
      }
    }
  }

  return LibXR::ErrorCode::OK;
}

bool MLX90640::HasUsableEepromDump(const std::array<uint16_t, EEPROM_COUNT>& words)
{
  bool has_non_zero = false;
  bool has_not_erased = false;

  for (uint16_t word : words)
  {
    has_non_zero = has_non_zero || (word != 0U);
    has_not_erased = has_not_erased || (word != 0xFFFFU);
  }

  return has_non_zero && has_not_erased;
}

bool MLX90640::ParseCalibration()
{
  calibration_.valid = false;
  calibration_.calibrated_temperature = false;

  if (!HasUsableEepromDump(calibration_.words))
  {
    last_error_ = LibXR::ErrorCode::CHECK_ERR;
    return false;
  }

  ExtractVddParameters(calibration_.words, calibration_);
  ExtractPtatParameters(calibration_.words, calibration_);
  ExtractGainParameters(calibration_.words, calibration_);
  ExtractTgcParameters(calibration_.words, calibration_);
  ExtractResolutionParameters(calibration_.words, calibration_);
  ExtractKsTaParameters(calibration_.words, calibration_);
  ExtractKsToParameters(calibration_.words, calibration_);
  ExtractCpParameters(calibration_.words, calibration_);
  ExtractAlphaParameters(calibration_.words, calibration_);
  ExtractOffsetParameters(calibration_.words, calibration_);
  ExtractKtaPixelParameters(calibration_.words, calibration_);
  ExtractKvPixelParameters(calibration_.words, calibration_);
  ExtractCilcParameters(calibration_.words, calibration_);

  const LibXR::ErrorCode error = ExtractDeviatingPixels(calibration_.words, calibration_);
  if (error != LibXR::ErrorCode::OK)
  {
    last_error_ = error;
    return false;
  }

  calibration_.valid = true;
  calibration_.calibrated_temperature = true;
  last_error_ = LibXR::ErrorCode::OK;
  return true;
}

constexpr float KELVIN_OFFSET = 273.15f;
constexpr float NOMINAL_VDD = 3.3f;
constexpr float NOMINAL_TA = 25.0f;
constexpr float PROCESSING_SCALE_ALPHA = 0.000001f;

float MLX90640::GetSupplyVoltage(const FrameBuffer& frame) const
{
  if (calibration_.k_vdd == 0)
  {
    return NOMINAL_VDD;
  }

  const auto resolution_ram = GetResolution(frame.control_register);
  const float resolution_correction =
      Pow2(calibration_.resolution_ee) / Pow2(resolution_ram);
  return (resolution_correction *
              static_cast<float>(ReadSignedWord(frame.words[VDD_INDEX])) -
          static_cast<float>(calibration_.vdd_25)) /
             static_cast<float>(calibration_.k_vdd) +
         NOMINAL_VDD;
}

float MLX90640::GetAmbientTemperature(const FrameBuffer& frame) const
{
  if (std::fabs(calibration_.kt_ptat) < 1e-6f)
  {
    return NOMINAL_TA;
  }

  const float vdd = GetSupplyVoltage(frame);
  const auto ptat = ReadSignedWord(frame.words[PTAT_INDEX]);
  const float denominator =
      static_cast<float>(ptat) * calibration_.alpha_ptat +
      static_cast<float>(ReadSignedWord(frame.words[PTAT_ART_INDEX]));
  if (std::fabs(denominator) < 1e-6f)
  {
    return NOMINAL_TA;
  }

  const float ptat_art = (static_cast<float>(ptat) / denominator) * Pow2(18);
  return (ptat_art / (1.0f + calibration_.kv_ptat * (vdd - NOMINAL_VDD)) -
          static_cast<float>(calibration_.v_ptat_25)) /
             calibration_.kt_ptat +
         NOMINAL_TA;
}

void MLX90640::CalculateTemperature(const FrameBuffer& frame, float emissivity,
                                    float reflected_temperature,
                                    ProcessingResult& result) const
{
  const uint16_t sub_page = frame.subpage;
  const float vdd = result.supply_voltage;
  const float ta = result.ambient_temperature;
  const float ta4 = std::pow(ta + KELVIN_OFFSET, 4.0f);
  const float tr4 = std::pow(reflected_temperature + KELVIN_OFFSET, 4.0f);
  const float ta_tr = tr4 - (tr4 - ta4) / emissivity;
  const float kta_scale = Pow2(calibration_.kta_scale);
  const float kv_scale = Pow2(calibration_.kv_scale);
  const float alpha_scale = Pow2(calibration_.alpha_scale);

  std::array<float, 4> alpha_corr = {};
  alpha_corr[0] = 1.0f / (1.0f + calibration_.ks_to[0] * 40.0f);
  alpha_corr[1] = 1.0f;
  alpha_corr[2] = 1.0f + calibration_.ks_to[1] * calibration_.ct[2];
  alpha_corr[3] = alpha_corr[2] * (1.0f + calibration_.ks_to[2] *
                                              (calibration_.ct[3] - calibration_.ct[2]));

  const float gain = static_cast<float>(calibration_.gain_ee) /
                     static_cast<float>(ReadSignedWord(frame.words[GAIN_INDEX]));
  const uint8_t mode = GetModeRaw(frame.control_register);

  std::array<float, 2> ir_data_cp = {
      static_cast<float>(ReadSignedWord(frame.words[COMPENSATION_PIXEL0_INDEX])) * gain,
      static_cast<float>(ReadSignedWord(frame.words[COMPENSATION_PIXEL1_INDEX])) * gain};

  ir_data_cp[0] -= calibration_.cp_offset[0] *
                   (1.0f + calibration_.cp_kta * (ta - 25.0f)) *
                   (1.0f + calibration_.cp_kv * (vdd - NOMINAL_VDD));
  if (mode == calibration_.calibration_mode_ee)
  {
    ir_data_cp[1] -= calibration_.cp_offset[1] *
                     (1.0f + calibration_.cp_kta * (ta - 25.0f)) *
                     (1.0f + calibration_.cp_kv * (vdd - NOMINAL_VDD));
  }
  else
  {
    ir_data_cp[1] -= (calibration_.cp_offset[1] + calibration_.il_chess_c[0]) *
                     (1.0f + calibration_.cp_kta * (ta - 25.0f)) *
                     (1.0f + calibration_.cp_kv * (vdd - NOMINAL_VDD));
  }

  for (int pixel = 0; pixel < static_cast<int>(PIXEL_COUNT); ++pixel)
  {
    const int8_t il_pattern = static_cast<int8_t>(pixel / 32 - (pixel / 64) * 2);
    const int8_t chess_pattern =
        static_cast<int8_t>(il_pattern ^ (pixel - (pixel / 2) * 2));
    const int8_t conversion_pattern = static_cast<int8_t>(
        ((pixel + 2) / 4 - (pixel + 3) / 4 + (pixel + 1) / 4 - pixel / 4) *
        (1 - 2 * il_pattern));

    const int8_t pattern = (mode == 0U) ? il_pattern : chess_pattern;
    if (pattern != static_cast<int8_t>(sub_page))
    {
      continue;
    }

    float ir_data = static_cast<float>(ReadSignedWord(frame.words[pixel])) * gain;
    const float kta = calibration_.kta[pixel] / kta_scale;
    const float kv = calibration_.kv[pixel] / kv_scale;
    ir_data -= calibration_.offset[pixel] * (1.0f + kta * (ta - 25.0f)) *
               (1.0f + kv * (vdd - NOMINAL_VDD));

    if (mode != calibration_.calibration_mode_ee)
    {
      ir_data += calibration_.il_chess_c[2] * (2 * il_pattern - 1) -
                 calibration_.il_chess_c[1] * conversion_pattern;
    }

    ir_data -= calibration_.tgc * ir_data_cp[sub_page];
    ir_data /= emissivity;

    const float alpha_compensated = (PROCESSING_SCALE_ALPHA * alpha_scale /
                                     static_cast<float>(calibration_.alpha[pixel])) *
                                    (1.0f + calibration_.ks_ta * (ta - 25.0f));

    float sx = alpha_compensated * alpha_compensated * alpha_compensated *
               (ir_data + alpha_compensated * ta_tr);
    sx = std::sqrt(std::sqrt(sx)) * calibration_.ks_to[1];

    float temperature =
        std::sqrt(std::sqrt(ir_data / (alpha_compensated * (1.0f - calibration_.ks_to[1] *
                                                                       KELVIN_OFFSET) +
                                       sx) +
                            ta_tr)) -
        KELVIN_OFFSET;

    int8_t range = 0;
    if (temperature < calibration_.ct[1])
    {
      range = 0;
    }
    else if (temperature < calibration_.ct[2])
    {
      range = 1;
    }
    else if (temperature < calibration_.ct[3])
    {
      range = 2;
    }
    else
    {
      range = 3;
    }

    temperature = std::sqrt(std::sqrt(
                      ir_data / (alpha_compensated * alpha_corr[range] *
                                 (1.0f + calibration_.ks_to[range] *
                                             (temperature - calibration_.ct[range]))) +
                      ta_tr)) -
                  KELVIN_OFFSET;

    result.temperature[pixel] = temperature;
  }
}

void MLX90640::CalculateImage(const FrameBuffer& frame, ProcessingResult& result) const
{
  const uint16_t sub_page = frame.subpage;
  const float vdd = result.supply_voltage;
  const float ta = result.ambient_temperature;
  const float kta_scale = Pow2(calibration_.kta_scale);
  const float kv_scale = Pow2(calibration_.kv_scale);
  const float gain = static_cast<float>(calibration_.gain_ee) /
                     static_cast<float>(ReadSignedWord(frame.words[GAIN_INDEX]));
  const uint8_t mode = GetModeRaw(frame.control_register);

  std::array<float, 2> ir_data_cp = {
      static_cast<float>(ReadSignedWord(frame.words[COMPENSATION_PIXEL0_INDEX])) * gain,
      static_cast<float>(ReadSignedWord(frame.words[COMPENSATION_PIXEL1_INDEX])) * gain};

  ir_data_cp[0] -= calibration_.cp_offset[0] *
                   (1.0f + calibration_.cp_kta * (ta - 25.0f)) *
                   (1.0f + calibration_.cp_kv * (vdd - NOMINAL_VDD));
  if (mode == calibration_.calibration_mode_ee)
  {
    ir_data_cp[1] -= calibration_.cp_offset[1] *
                     (1.0f + calibration_.cp_kta * (ta - 25.0f)) *
                     (1.0f + calibration_.cp_kv * (vdd - NOMINAL_VDD));
  }
  else
  {
    ir_data_cp[1] -= (calibration_.cp_offset[1] + calibration_.il_chess_c[0]) *
                     (1.0f + calibration_.cp_kta * (ta - 25.0f)) *
                     (1.0f + calibration_.cp_kv * (vdd - NOMINAL_VDD));
  }

  for (int pixel = 0; pixel < static_cast<int>(PIXEL_COUNT); ++pixel)
  {
    const int8_t il_pattern = static_cast<int8_t>(pixel / 32 - (pixel / 64) * 2);
    const int8_t chess_pattern =
        static_cast<int8_t>(il_pattern ^ (pixel - (pixel / 2) * 2));
    const int8_t conversion_pattern = static_cast<int8_t>(
        ((pixel + 2) / 4 - (pixel + 3) / 4 + (pixel + 1) / 4 - pixel / 4) *
        (1 - 2 * il_pattern));

    const int8_t pattern = (mode == 0U) ? il_pattern : chess_pattern;
    if (pattern != static_cast<int8_t>(sub_page))
    {
      continue;
    }

    float ir_data = static_cast<float>(ReadSignedWord(frame.words[pixel])) * gain;
    const float kta = calibration_.kta[pixel] / kta_scale;
    const float kv = calibration_.kv[pixel] / kv_scale;
    ir_data -= calibration_.offset[pixel] * (1.0f + kta * (ta - 25.0f)) *
               (1.0f + kv * (vdd - NOMINAL_VDD));

    if (mode != calibration_.calibration_mode_ee)
    {
      ir_data += calibration_.il_chess_c[2] * (2 * il_pattern - 1) -
                 calibration_.il_chess_c[1] * conversion_pattern;
    }

    ir_data -= calibration_.tgc * ir_data_cp[sub_page];
    result.image[pixel] = ir_data * calibration_.alpha[pixel];
  }
}

float MLX90640::GetMedian(std::array<float, 4> values)
{
  std::sort(values.begin(), values.end());
  return (values[1] + values[2]) / 2.0f;
}

bool MLX90640::ValidateAuxData(const FrameBuffer& frame) const
{
  const auto aux = frame.words.data() + PIXEL_WORD_COUNT;
  if (aux[0] == 0x7FFFU)
  {
    XR_LOG_WARN("MLX90640: aux validation failed: aux[0]=0x%04X", aux[0]);
    return false;
  }

  for (std::size_t i = 8; i < 19; ++i)
  {
    if (aux[i] == 0x7FFFU)
    {
      XR_LOG_WARN("MLX90640: aux validation failed: aux[%u]=0x%04X",
                  static_cast<unsigned>(i), aux[i]);
      return false;
    }
  }
  for (std::size_t i = 20; i < 23; ++i)
  {
    if (aux[i] == 0x7FFFU)
    {
      XR_LOG_WARN("MLX90640: aux validation failed: aux[%u]=0x%04X",
                  static_cast<unsigned>(i), aux[i]);
      return false;
    }
  }
  for (std::size_t i = 24; i < 33; ++i)
  {
    if (aux[i] == 0x7FFFU)
    {
      XR_LOG_WARN("MLX90640: aux validation failed: aux[%u]=0x%04X",
                  static_cast<unsigned>(i), aux[i]);
      return false;
    }
  }
  for (std::size_t i = 40; i < 51; ++i)
  {
    if (aux[i] == 0x7FFFU)
    {
      XR_LOG_WARN("MLX90640: aux validation failed: aux[%u]=0x%04X",
                  static_cast<unsigned>(i), aux[i]);
      return false;
    }
  }
  for (std::size_t i = 52; i < 55; ++i)
  {
    if (aux[i] == 0x7FFFU)
    {
      XR_LOG_WARN("MLX90640: aux validation failed: aux[%u]=0x%04X",
                  static_cast<unsigned>(i), aux[i]);
      return false;
    }
  }
  for (std::size_t i = 56; i < 64; ++i)
  {
    if (aux[i] == 0x7FFFU)
    {
      XR_LOG_WARN("MLX90640: aux validation failed: aux[%u]=0x%04X",
                  static_cast<unsigned>(i), aux[i]);
      return false;
    }
  }

  return true;
}

bool MLX90640::ValidateFrame(const FrameBuffer& frame) const
{
  uint8_t line = 0;
  for (std::size_t i = 0; i < PIXEL_WORD_COUNT; i += 32U)
  {
    if (frame.words[i] == 0x7FFFU && (line % 2U) == frame.subpage)
    {
      XR_LOG_WARN(
          "MLX90640: pixel validation failed: line=%u "
          "word[%u]=0x%04X subpage=%u",
          line, static_cast<unsigned>(i), frame.words[i], frame.subpage);
      return false;
    }
    ++line;
  }

  return ValidateAuxData(frame);
}

void MLX90640::CorrectBadPixels(const std::array<uint16_t, BAD_PIXEL_TABLE_SIZE>& pixels,
                                std::array<float, PIXEL_COUNT>& field, uint8_t mode) const
{
  for (uint16_t pixel : pixels)
  {
    if (pixel == 0xFFFFU)
    {
      break;
    }

    const uint8_t line = static_cast<uint8_t>(pixel >> 5U);
    const uint8_t column = static_cast<uint8_t>(pixel - (line << 5U));
    if (mode == 1U)
    {
      if (line == 0U)
      {
        if (column == 0U)
        {
          field[pixel] = field[33];
        }
        else if (column == 31U)
        {
          field[pixel] = field[62];
        }
        else
        {
          field[pixel] = (field[pixel + 31U] + field[pixel + 33U]) / 2.0f;
        }
      }
      else if (line == 23U)
      {
        if (column == 0U)
        {
          field[pixel] = field[705];
        }
        else if (column == 31U)
        {
          field[pixel] = field[734];
        }
        else
        {
          field[pixel] = (field[pixel - 33U] + field[pixel - 31U]) / 2.0f;
        }
      }
      else if (column == 0U)
      {
        field[pixel] = (field[pixel - 31U] + field[pixel + 33U]) / 2.0f;
      }
      else if (column == 31U)
      {
        field[pixel] = (field[pixel - 33U] + field[pixel + 31U]) / 2.0f;
      }
      else
      {
        const std::array<float, 4> neighbours = {field[pixel - 33U], field[pixel - 31U],
                                                 field[pixel + 31U], field[pixel + 33U]};
        field[pixel] = GetMedian(neighbours);
      }
    }
    else
    {
      if (column == 0U)
      {
        field[pixel] = field[pixel + 1U];
      }
      else if (column == 1U || column == 30U)
      {
        field[pixel] = (field[pixel - 1U] + field[pixel + 1U]) / 2.0f;
      }
      else if (column == 31U)
      {
        field[pixel] = field[pixel - 1U];
      }
      else if (!IsPixelBad(static_cast<uint16_t>(pixel - 2U)) &&
               !IsPixelBad(static_cast<uint16_t>(pixel + 2U)))
      {
        const float delta_pos = field[pixel + 1U] - field[pixel + 2U];
        const float delta_neg = field[pixel - 1U] - field[pixel - 2U];
        if (std::fabs(delta_pos) > std::fabs(delta_neg))
        {
          field[pixel] = field[pixel - 1U] + delta_neg;
        }
        else
        {
          field[pixel] = field[pixel + 1U] + delta_pos;
        }
      }
      else
      {
        field[pixel] = (field[pixel - 1U] + field[pixel + 1U]) / 2.0f;
      }
    }
  }
}

bool MLX90640::IsPixelBad(uint16_t pixel) const
{
  for (uint16_t bad_pixel : calibration_.outlier_pixels)
  {
    if (pixel == bad_pixel)
    {
      return true;
    }
  }
  for (uint16_t bad_pixel : calibration_.broken_pixels)
  {
    if (pixel == bad_pixel)
    {
      return true;
    }
  }
  return false;
}

bool MLX90640::CaptureSubpage(FrameBuffer& frame)
{
  uint16_t status_register = 0;
  if (!SynchronizeFrameStatus())
  {
    XR_LOG_ERROR("MLX90640: frame status sync failed: %d", ErrorCodeValue(last_error_));
    return false;
  }

  if (!WaitForDataReady(status_register))
  {
    XR_LOG_ERROR("MLX90640: wait data-ready failed: %d", ErrorCodeValue(last_error_));
    return false;
  }

  last_error_ = WriteWord(i2c_address_, STATUS_REGISTER, INIT_STATUS_VALUE);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    XR_LOG_ERROR("MLX90640: clear status failed: %d", ErrorCodeValue(last_error_));
    return false;
  }

  last_error_ = ReadWords(i2c_address_, PIXEL_DATA_START_ADDRESS,
                          static_cast<uint16_t>(PIXEL_WORD_COUNT), frame.words.data());
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    XR_LOG_ERROR("MLX90640: pixel data read failed: %d", ErrorCodeValue(last_error_));
    return false;
  }

  last_error_ = ReadWords(i2c_address_, AUX_DATA_START_ADDRESS,
                          static_cast<uint16_t>(AUX_WORD_COUNT),
                          frame.words.data() + PIXEL_WORD_COUNT);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    XR_LOG_ERROR("MLX90640: aux data read failed: %d", ErrorCodeValue(last_error_));
    return false;
  }

  last_error_ = ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &frame.control_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    XR_LOG_ERROR("MLX90640: control register read failed: %d",
                 ErrorCodeValue(last_error_));
    return false;
  }

  frame.status_register = status_register;
  frame.subpage = GetSubpage(status_register);
  frame.words[CONTROL_WORD_INDEX] = frame.control_register;
  frame.words[SUBPAGE_WORD_INDEX] = frame.subpage;
  current_mode_ = GetMode(frame.control_register);
  frame.valid = ValidateFrame(frame);
  if (!frame.valid)
  {
    last_error_ = LibXR::ErrorCode::CHECK_ERR;
    XR_LOG_WARN(
        "MLX90640: frame validation failed: status=0x%04X "
        "control=0x%04X subpage=%u",
        frame.status_register, frame.control_register, frame.subpage);
  }
  return frame.valid;
}

void MLX90640::ProcessFrame(const FrameBuffer& frame, float emissivity,
                            float reflected_temperature_shift,
                            ProcessingResult& result) const
{
  result.ambient_temperature = GetAmbientTemperature(frame);
  result.reflected_temperature = result.ambient_temperature - reflected_temperature_shift;
  result.supply_voltage = GetSupplyVoltage(frame);
  result.calibrated_temperature = calibration_.calibrated_temperature;

  CalculateImage(frame, result);
  CalculateTemperature(frame, emissivity, result.reflected_temperature, result);

  const uint8_t mode = GetMode(frame.control_register);
  CorrectBadPixels(calibration_.broken_pixels, result.temperature, mode);
  CorrectBadPixels(calibration_.outlier_pixels, result.temperature, mode);
  CorrectBadPixels(calibration_.broken_pixels, result.image, mode);
  CorrectBadPixels(calibration_.outlier_pixels, result.image, mode);
}

bool MLX90640::SetRefreshRate(uint8_t refresh_rate)
{
  uint16_t control_register = 0;
  last_error_ = ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &control_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  control_register = SetRefreshRateBits(control_register, refresh_rate);
  const uint16_t updated_control = control_register;
  last_error_ = WriteWord(i2c_address_, CONTROL_REGISTER, control_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  last_error_ = ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &control_register);
  XR_LOG_INFO("MLX90640: refresh configured: wrote=0x%04X readback=0x%04X",
              updated_control, control_register);
  return last_error_ == LibXR::ErrorCode::OK;
}

bool MLX90640::SetMode(bool use_chess_mode)
{
  uint16_t control_register = 0;
  last_error_ = ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &control_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  control_register = SetModeBits(control_register, use_chess_mode);
  const uint16_t updated_control = control_register;
  last_error_ = WriteWord(i2c_address_, CONTROL_REGISTER, control_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  last_error_ = ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &control_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }
  XR_LOG_INFO("MLX90640: mode configured: wrote=0x%04X readback=0x%04X", updated_control,
              control_register);

  current_mode_ = GetMode(control_register);
  return true;
}

bool MLX90640::DumpEeprom()
{
  last_error_ =
      ReadWords(i2c_address_, EEPROM_START_ADDRESS,
                static_cast<uint16_t>(EEPROM_WORD_COUNT), calibration_.words.data());
  return last_error_ == LibXR::ErrorCode::OK;
}

bool MLX90640::SynchronizeFrameStatus()
{
  uint16_t status_register = 0;
  last_error_ = ReadWords(i2c_address_, STATUS_REGISTER, 1U, &status_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  last_error_ = WriteWord(i2c_address_, STATUS_REGISTER, INIT_STATUS_VALUE);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  uint16_t control_register = 0;
  const LibXR::ErrorCode control_error =
      ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &control_register);
  if (control_error == LibXR::ErrorCode::OK)
  {
    XR_LOG_DEBUG(
        "MLX90640: frame sync: status before clear=0x%04X, "
        "control=0x%04X",
        status_register, control_register);
  }
  else
  {
    XR_LOG_WARN("MLX90640: frame sync: control readback failed: %d",
                ErrorCodeValue(control_error));
  }

  return true;
}

bool MLX90640::WaitForDataReady(uint16_t& status_register)
{
  const uint32_t start_ms = LibXR::Thread::GetTime();
  bool triggered = false;
  int poll_count = 0;

  while (static_cast<uint32_t>(LibXR::Thread::GetTime() - start_ms) <
         DATA_READY_TIMEOUT_MS)
  {
    ++poll_count;
    last_error_ = ReadWords(i2c_address_, STATUS_REGISTER, 1U, &status_register);
    if (last_error_ != LibXR::ErrorCode::OK)
    {
      return false;
    }

    if (IsDataReady(status_register))
    {
      XR_LOG_DEBUG("MLX90640: data-ready after %d polls: status=0x%04X", poll_count,
                   status_register);
      return true;
    }

    if ((poll_count == 1) || ((poll_count % 50) == 0))
    {
      uint16_t control_register = 0;
      const LibXR::ErrorCode control_error =
          ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &control_register);
      if (control_error == LibXR::ErrorCode::OK)
      {
        XR_LOG_DEBUG(
            "MLX90640: waiting data-ready: poll=%d "
            "status=0x%04X control=0x%04X",
            poll_count, status_register, control_register);
      }
      else
      {
        XR_LOG_WARN("MLX90640: waiting data-ready: control read failed: %d",
                    ErrorCodeValue(control_error));
      }
    }

    if (!triggered && static_cast<uint32_t>(LibXR::Thread::GetTime() - start_ms) >=
                          MANUAL_TRIGGER_DELAY_MS)
    {
      triggered = true;
      if (!TriggerMeasurement())
      {
        XR_LOG_WARN("MLX90640: manual trigger failed: %d", ErrorCodeValue(last_error_));
      }
    }

    LibXR::Thread::Yield();
    LibXR::Thread::Sleep(DATA_READY_POLL_DELAY_MS);
  }

  last_error_ = LibXR::ErrorCode::TIMEOUT;
  XR_LOG_ERROR("MLX90640: data-ready timeout: last status=0x%04X", status_register);
  return false;
}

bool MLX90640::TriggerMeasurement()
{
  uint16_t control_register = 0;
  last_error_ = ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &control_register);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  const uint16_t triggered_control = SetTriggerBit(control_register);
  last_error_ = WriteWord(i2c_address_, CONTROL_REGISTER, triggered_control);
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    return false;
  }

  XR_LOG_WARN("MLX90640: manual trigger: control 0x%04X -> 0x%04X", control_register,
              triggered_control);

  uint16_t readback_control = 0;
  const LibXR::ErrorCode readback_error =
      ReadWords(i2c_address_, CONTROL_REGISTER, 1U, &readback_control);
  if (readback_error == LibXR::ErrorCode::OK)
  {
    XR_LOG_WARN("MLX90640: manual trigger readback: control=0x%04X", readback_control);
  }
  else
  {
    XR_LOG_WARN("MLX90640: manual trigger readback failed: %d",
                ErrorCodeValue(readback_error));
  }

  return true;
}

constexpr uint32_t kStatsLogIntervalMs = 1000;

static const char* RefreshRateToString(MLX90640::RefreshRate refresh_rate)
{
  switch (refresh_rate)
  {
    case MLX90640::RefreshRate::HZ_0_5:
      return "0.5";
    case MLX90640::RefreshRate::HZ_1:
      return "1";
    case MLX90640::RefreshRate::HZ_2:
      return "2";
    case MLX90640::RefreshRate::HZ_4:
      return "4";
    case MLX90640::RefreshRate::HZ_8:
      return "8";
    case MLX90640::RefreshRate::HZ_16:
      return "16";
    case MLX90640::RefreshRate::HZ_32:
      return "32";
    case MLX90640::RefreshRate::HZ_64:
      return "64";
    default:
      return "?";
  }
}

MLX90640::MLX90640(LibXR::I2C& external_i2c_name, LibXR::RamFS* external_ramfs,
                   RefreshRate refresh_rate, float emissivity,
                   float reflected_temperature_shift, bool use_chess_mode,
                   const char* temperature_topic_name, const char* image_topic_name,
                   const char* stats_topic_name, uint8_t i2c_address)
    : refresh_rate_(refresh_rate),
      emissivity_(std::clamp(emissivity, 0.1f, 1.0f)),
      reflected_temperature_shift_(reflected_temperature_shift),
      use_chess_mode_(use_chess_mode),
      i2c_address_(i2c_address),
      topic_temperature_(
          LibXR::Topic::CreateTopic<ThermalFrame>(temperature_topic_name, nullptr, true)),
      topic_image_(
          LibXR::Topic::CreateTopic<ThermalImage>(image_topic_name, nullptr, true)),
      topic_stats_(
          LibXR::Topic::CreateTopic<ThermalStats>(stats_topic_name, nullptr, true)),
      i2c_(std::addressof(external_i2c_name)),
      cmd_file_(LibXR::RamFS::CreateFile("mlx90640", CommandFunc, this)),
      i2c_read_block_(i2c_sem_),
      i2c_write_block_(i2c_sem_)
{
  if (auto* ramfs = external_ramfs; ramfs != nullptr)
  {
    ramfs->Add(cmd_file_);
    cmd_registered_ = true;
  }

  LibXR::Mutex::LockGuard lock(mutex_);
  XR_LOG_INFO("MLX90640: initialize start");

  last_error_ = GeneralReset();
  if (last_error_ != LibXR::ErrorCode::OK)
  {
    XR_LOG_WARN("MLX90640: general-call reset failed: %d, continuing",
                ErrorCodeValue(last_error_));
  }

  SetFrequency(EEPROM_I2C_FREQ);
  if (!DumpEeprom())
  {
    XR_LOG_ERROR("MLX90640: EEPROM dump failed: %d", ErrorCodeValue(last_error_));
    ASSERT(false);
  }

  if (!ParseCalibration())
  {
    XR_LOG_ERROR("MLX90640: calibration parse failed: %d", ErrorCodeValue(last_error_));
    ASSERT(false);
  }

  if (!SetMode(use_chess_mode_))
  {
    XR_LOG_ERROR("MLX90640: set mode failed: %d", ErrorCodeValue(last_error_));
    ASSERT(false);
  }

  if (!SetRefreshRate(static_cast<uint8_t>(refresh_rate_)))
  {
    XR_LOG_ERROR("MLX90640: set refresh rate failed: %d", ErrorCodeValue(last_error_));
    ASSERT(false);
  }

  SetFrequency(RUNTIME_I2C_FREQ);

  XR_LOG_INFO("MLX90640: sensor initialize ok");

  thermal_frame_.temperature.fill(0.0f);
  image_frame_.image.fill(0.0f);
  stats_.ready = false;
  last_stats_log_ms_ = 0;

  const bool warmed_up = WarmUpFrameBuffers();
  ASSERT(warmed_up);

  thermal_frame_.frame_counter = 0;
  image_frame_.frame_counter = 0;
  stats_.frame_counter = 0;
  UpdateStats();

  XR_LOG_INFO("MLX90640 initialized at 0x%02X, refresh=%sHz, mode=%s", i2c_address_,
              RefreshRateToString(refresh_rate_),
              use_chess_mode_ ? "chess" : "interleaved");

  XR_LOG_INFO("MLX90640: creating MLX90640 worker");
  worker_.Create(this, WorkerEntry, "mlx90640", WORKER_STACK_BYTES,
                 LibXR::Thread::Priority::MEDIUM);
  XR_LOG_INFO("MLX90640: MLX90640 worker registered");
}

void MLX90640::WorkerEntry(MLX90640* self)
{
  ASSERT(self != nullptr);
  XR_LOG_INFO("MLX90640: worker entry");
  self->WorkerLoop();
}

void MLX90640::WorkerLoop()
{
  while (true)
  {
    ASSERT(AcquireAndPublishFrame());
    LibXR::Thread::Yield();
  }
}

bool MLX90640::WarmUpFrameBuffers()
{
  uint8_t seen_subpages = 0;
  for (int attempt = 0; attempt < 12 && seen_subpages != SUBPAGE_MASK_COMPLETE; ++attempt)
  {
    uint8_t subpage = 0;
    if (!ReadNextSubpage(subpage))
    {
      if (last_error_ == LibXR::ErrorCode::CHECK_ERR)
      {
        XR_LOG_WARN("MLX90640: warm-up ignored invalid frame on attempt %d", attempt + 1);
        LibXR::Thread::Yield();
        continue;
      }
      return false;
    }

    ApplyCurrentFrame(subpage);
    seen_subpages = static_cast<uint8_t>(seen_subpages | (1U << subpage));
  }

  if (seen_subpages != SUBPAGE_MASK_COMPLETE)
  {
    XR_LOG_ERROR("MLX90640 warm-up did not observe both subpages, mask=0x%02X",
                 seen_subpages);
    return false;
  }

  return true;
}

bool MLX90640::ReadNextSubpage(uint8_t& subpage)
{
  if (!CaptureSubpage(frame_buffer_))
  {
    if (last_error_ != LibXR::ErrorCode::CHECK_ERR)
    {
      XR_LOG_ERROR("MLX90640 frame fetch failed: %d", ErrorCodeValue(last_error_));
    }
    return false;
  }

  subpage = frame_buffer_.subpage;
  return true;
}

void MLX90640::ApplyCurrentFrame(uint8_t subpage)
{
  current_subpage_ = subpage;
  ProcessFrame(frame_buffer_, emissivity_, reflected_temperature_shift_,
               processing_result_);
  thermal_frame_.ambient_temperature = processing_result_.ambient_temperature;
  thermal_frame_.reflected_temperature = processing_result_.reflected_temperature;
  thermal_frame_.emissivity = emissivity_;
  thermal_frame_.subpage = subpage;
  thermal_frame_.mode = current_mode_;
  thermal_frame_.temperature = processing_result_.temperature;

  image_frame_.frame_counter = thermal_frame_.frame_counter;
  image_frame_.image = processing_result_.image;

  const auto image_minmax =
      std::minmax_element(image_frame_.image.begin(), image_frame_.image.end());
  image_frame_.min_value = *image_minmax.first;
  image_frame_.max_value = *image_minmax.second;
}

bool MLX90640::AcquireAndPublishFrame()
{
  LibXR::Mutex::LockGuard lock(mutex_);

  uint8_t seen_subpages = 0;
  for (int attempt = 0; attempt < 8 && seen_subpages != SUBPAGE_MASK_COMPLETE; ++attempt)
  {
    uint8_t subpage = 0;
    if (!ReadNextSubpage(subpage))
    {
      if (last_error_ == LibXR::ErrorCode::CHECK_ERR)
      {
        XR_LOG_WARN("MLX90640: ignored invalid frame on acquisition attempt %d",
                    attempt + 1);
        LibXR::Thread::Yield();
        continue;
      }
      return false;
    }

    ApplyCurrentFrame(subpage);
    seen_subpages = static_cast<uint8_t>(seen_subpages | (1U << subpage));
  }

  if (seen_subpages != SUBPAGE_MASK_COMPLETE)
  {
    XR_LOG_ERROR("MLX90640 could not assemble a whole frame, mask=0x%02X", seen_subpages);
    return false;
  }

  thermal_frame_.frame_counter = ++frame_counter_;
  image_frame_.frame_counter = thermal_frame_.frame_counter;

  UpdateStats();

  topic_temperature_.Publish(thermal_frame_);
  topic_image_.Publish(image_frame_);
  topic_stats_.Publish(stats_);
  return true;
}

void MLX90640::UpdateStats()
{
  auto minmax = std::minmax_element(thermal_frame_.temperature.begin(),
                                    thermal_frame_.temperature.end());

  float sum = 0.0f;
  for (float value : thermal_frame_.temperature)
  {
    sum += value;
  }

  stats_.frame_counter = thermal_frame_.frame_counter;
  stats_.ambient_temperature = thermal_frame_.ambient_temperature;
  stats_.reflected_temperature = thermal_frame_.reflected_temperature;
  stats_.supply_voltage = processing_result_.supply_voltage;
  stats_.min_temperature = *minmax.first;
  stats_.max_temperature = *minmax.second;
  stats_.average_temperature = sum / static_cast<float>(PIXEL_COUNT);
  stats_.center_temperature = thermal_frame_.temperature[CENTER_PIXEL_INDEX];
  stats_.min_index = static_cast<uint16_t>(
      std::distance(thermal_frame_.temperature.begin(), minmax.first));
  stats_.max_index = static_cast<uint16_t>(
      std::distance(thermal_frame_.temperature.begin(), minmax.second));
  stats_.bad_pixel_count = CountBadPixels();
  stats_.ready = true;
  LogStatsIfDue();
}

void MLX90640::LogStatsIfDue()
{
  const uint32_t now_ms = LibXR::Thread::GetTime();
  if (last_stats_log_ms_ != 0 &&
      static_cast<uint32_t>(now_ms - last_stats_log_ms_) < kStatsLogIntervalMs)
  {
    return;
  }

  last_stats_log_ms_ = now_ms;
  XR_LOG_INFO("MLX90640 stats: frame=%" PRIu32
              " Ta(sensor)=%.2fC Vdd=%.3fV min=%.2fC[%u] max=%.2fC[%u] "
              "avg=%.2fC center=%.2fC bad=%u",
              stats_.frame_counter, stats_.ambient_temperature, stats_.supply_voltage,
              stats_.min_temperature, stats_.min_index, stats_.max_temperature,
              stats_.max_index, stats_.average_temperature, stats_.center_temperature,
              stats_.bad_pixel_count);

  const auto raw_minmax = std::minmax_element(
      frame_buffer_.words.begin(), frame_buffer_.words.begin() + PIXEL_WORD_COUNT);
  XR_LOG_INFO(
      "MLX90640 raw: sp=%u status=0x%04X ctrl=0x%04X "
      "pix[min=0x%04X max=0x%04X p0=0x%04X pc=0x%04X] "
      "aux[gain=0x%04X ptat=0x%04X ptatArt=0x%04X vdd=0x%04X] "
      "image[min=%.6f max=%.6f]",
      frame_buffer_.subpage, frame_buffer_.status_register,
      frame_buffer_.control_register, *raw_minmax.first, *raw_minmax.second,
      frame_buffer_.words[0], frame_buffer_.words[CENTER_PIXEL_INDEX],
      frame_buffer_.words[GAIN_INDEX], frame_buffer_.words[PTAT_INDEX],
      frame_buffer_.words[PTAT_ART_INDEX], frame_buffer_.words[VDD_INDEX],
      image_frame_.min_value, image_frame_.max_value);
}

float MLX90640::ComputeReflectedTemperature() const
{
  return processing_result_.reflected_temperature;
}

uint8_t MLX90640::CountBadPixels() const
{
  uint8_t count = 0;
  for (uint16_t pixel : calibration_.broken_pixels)
  {
    if (pixel != 0xFFFFU)
    {
      ++count;
    }
  }
  for (uint16_t pixel : calibration_.outlier_pixels)
  {
    if (pixel != 0xFFFFU)
    {
      ++count;
    }
  }
  return count;
}

bool MLX90640::TrySetRefreshRate(RefreshRate refresh_rate)
{
  if (!SetRefreshRate(static_cast<uint8_t>(refresh_rate)))
  {
    XR_LOG_ERROR("MLX90640 set refresh rate failed: %d", ErrorCodeValue(last_error_));
    return false;
  }

  refresh_rate_ = refresh_rate;
  return true;
}

bool MLX90640::ConfigureMode(bool use_chess_mode)
{
  if (!SetMode(use_chess_mode))
  {
    XR_LOG_ERROR("MLX90640 set mode failed: %d", ErrorCodeValue(last_error_));
    return false;
  }

  use_chess_mode_ = use_chess_mode;
  return true;
}

static const char* ShellRefreshRateToString(MLX90640::RefreshRate refresh_rate)
{
  switch (refresh_rate)
  {
    case MLX90640::RefreshRate::HZ_0_5:
      return "0.5";
    case MLX90640::RefreshRate::HZ_1:
      return "1";
    case MLX90640::RefreshRate::HZ_2:
      return "2";
    case MLX90640::RefreshRate::HZ_4:
      return "4";
    case MLX90640::RefreshRate::HZ_8:
      return "8";
    case MLX90640::RefreshRate::HZ_16:
      return "16";
    case MLX90640::RefreshRate::HZ_32:
      return "32";
    case MLX90640::RefreshRate::HZ_64:
      return "64";
    default:
      return "?";
  }
}

void MLX90640::PrintUsage() const
{
  LibXR::STDIO::Printf<"Usage:\r\n">();
  LibXR::STDIO::Printf<
      "  stats                       - Show current frame statistics.\r\n">();
  LibXR::STDIO::Printf<
      "  show [count] [interval_ms]  - Print statistics repeatedly.\r\n">();
  LibXR::STDIO::Printf<"  refresh [0-7]               - Set sensor refresh rate.\r\n">();
  LibXR::STDIO::Printf<"  emissivity [0.1-1.0]        - Set emissivity.\r\n">();
}

int MLX90640::PrintStatsLocked() const
{
  if (!stats_.ready)
  {
    LibXR::STDIO::Printf<"MLX90640 frame is not ready yet.\r\n">();
    return 0;
  }

  LibXR::STDIO::Printf<
      "Frame=%u Mode=%u Subpage=%u Ta=%.2fC Tr=%.2fC "
      "Vdd=%.2fV Min=%.2fC(idx=%u) "
      "Max=%.2fC(idx=%u) Avg=%.2fC Center=%.2fC Bad=%u\r\n">(
      static_cast<unsigned int>(stats_.frame_counter), thermal_frame_.mode,
      thermal_frame_.subpage, stats_.ambient_temperature, stats_.reflected_temperature,
      stats_.supply_voltage, stats_.min_temperature, stats_.min_index,
      stats_.max_temperature, stats_.max_index, stats_.average_temperature,
      stats_.center_temperature, stats_.bad_pixel_count);
  return 0;
}

int MLX90640::HandleCommand(int argc, char** argv)
{
  if (argc == 1 || (argc == 2 && std::strcmp(argv[1], "help") == 0))
  {
    PrintUsage();
    return 0;
  }

  if (argc == 2 && std::strcmp(argv[1], "stats") == 0)
  {
    LibXR::Mutex::LockGuard lock(mutex_);
    return PrintStatsLocked();
  }

  if (argc == 4 && std::strcmp(argv[1], "show") == 0)
  {
    int count = std::max(1, std::atoi(argv[2]));
    int interval_ms = std::clamp(std::atoi(argv[3]), 10, 5000);
    while (count-- > 0)
    {
      {
        LibXR::Mutex::LockGuard lock(mutex_);
        PrintStatsLocked();
      }
      LibXR::Thread::Sleep(static_cast<uint32_t>(interval_ms));
    }
    return 0;
  }

  if (argc == 3 && std::strcmp(argv[1], "refresh") == 0)
  {
    const int rate = std::atoi(argv[2]);
    if (rate < 0 || rate > 7)
    {
      LibXR::STDIO::Printf<"Error: refresh rate must be in [0, 7].\r\n">();
      return -1;
    }

    LibXR::Mutex::LockGuard lock(mutex_);
    if (!TrySetRefreshRate(static_cast<RefreshRate>(rate)))
    {
      LibXR::STDIO::Printf<"Error: failed to update refresh rate.\r\n">();
      return -1;
    }

    LibXR::STDIO::Printf<"Refresh rate updated to %sHz.\r\n">(
        ShellRefreshRateToString(refresh_rate_));
    return 0;
  }

  if (argc == 3 && std::strcmp(argv[1], "emissivity") == 0)
  {
    char* end = nullptr;
    const float value = std::strtof(argv[2], &end);
    if (end == argv[2] || *end != '\0' || value < 0.1f || value > 1.0f)
    {
      LibXR::STDIO::Printf<"Error: emissivity must be in [0.1, 1.0].\r\n">();
      return -1;
    }

    LibXR::Mutex::LockGuard lock(mutex_);
    emissivity_ = value;
    LibXR::STDIO::Printf<"Emissivity updated to %.3f.\r\n">(emissivity_);
    return 0;
  }

  LibXR::STDIO::Printf<"Error: invalid arguments.\r\n">();
  PrintUsage();
  return -1;
}

int MLX90640::CommandFunc(MLX90640* self, int argc, char** argv)
{
  return (self != nullptr) ? self->HandleCommand(argc, argv) : -1;
}

LibXR::ErrorCode MLX90640::ReadWords(uint8_t slave_addr, uint16_t start_address,
                                     uint16_t word_count, uint16_t* data)
{
  const std::size_t byte_count = static_cast<std::size_t>(word_count) * 2U;
  if (byte_count > raw_buffer_.size() || data == nullptr)
  {
    return (data == nullptr) ? LibXR::ErrorCode::PTR_NULL : LibXR::ErrorCode::SIZE_ERR;
  }

  constexpr std::size_t kReadChunkWords = 14U;
  std::size_t word_offset = 0U;
  while (word_offset < word_count)
  {
    const std::size_t chunk_words =
        std::min<std::size_t>(word_count - word_offset, kReadChunkWords);
    const std::size_t chunk_bytes = chunk_words * 2U;
    const std::size_t byte_offset = word_offset * 2U;
    const uint16_t address = static_cast<uint16_t>(start_address + word_offset);

    const auto ec =
        i2c_->MemRead(slave_addr, address,
                      LibXR::RawData(raw_buffer_.data() + byte_offset, chunk_bytes),
                      i2c_read_block_, LibXR::I2C::MemAddrLength::BYTE_16);
    if (ec != LibXR::ErrorCode::OK)
    {
      XR_LOG_ERROR("MLX90640 bus: read failed: addr=0x%02X reg=0x%04X words=%u ec=%d",
                   slave_addr, address, static_cast<unsigned>(chunk_words),
                   static_cast<int>(ec));
      return ec;
    }

    for (std::size_t i = 0; i < chunk_words; ++i)
    {
      const std::size_t raw_index = byte_offset + i * 2U;
      const auto msb = static_cast<uint16_t>(raw_buffer_[raw_index]);
      const auto lsb = static_cast<uint16_t>(raw_buffer_[raw_index + 1U]);
      data[word_offset + i] = static_cast<uint16_t>((msb << 8U) | lsb);
    }

    word_offset += chunk_words;
  }

  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode MLX90640::WriteWord(uint8_t slave_addr, uint16_t write_address,
                                     uint16_t data)
{
  uint8_t bytes[2] = {static_cast<uint8_t>((data >> 8U) & 0xFFU),
                      static_cast<uint8_t>(data & 0xFFU)};

  const auto ec =
      i2c_->MemWrite(slave_addr, write_address, LibXR::ConstRawData(bytes, 2U),
                     i2c_write_block_, LibXR::I2C::MemAddrLength::BYTE_16);
  if (ec != LibXR::ErrorCode::OK)
  {
    XR_LOG_ERROR("MLX90640 bus: write failed: addr=0x%02X reg=0x%04X data=0x%04X ec=%d",
                 slave_addr, write_address, data, static_cast<int>(ec));
  }
  return ec;
}

void MLX90640::SetFrequency(int freq)
{
  if (freq > 0)
  {
    const auto ec = i2c_->SetConfig({static_cast<uint32_t>(freq)});
    if (ec != LibXR::ErrorCode::OK)
    {
      XR_LOG_WARN("MLX90640 failed to set I2C frequency to %d Hz", freq);
    }
  }
}

LibXR::ErrorCode MLX90640::GeneralReset()
{
  constexpr uint8_t general_call_reset = 0x06;
  const auto ec =
      i2c_->Write(0x00, LibXR::ConstRawData(&general_call_reset, 1U), i2c_write_block_);
  if (ec != LibXR::ErrorCode::OK)
  {
    XR_LOG_WARN("MLX90640 bus: general-call reset failed: ec=%d", static_cast<int>(ec));
  }
  return ec;
}
