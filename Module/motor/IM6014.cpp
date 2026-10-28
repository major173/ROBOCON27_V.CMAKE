/** 
*@file IM6014.cpp
*@author FN
*@brief IM6014 控制数据 反馈数据 和函数接口
*@version 0.1
*@date 2026-10-27
*/
#include "IM6014.hpp"

#include<cmath>
#include <cstdint>
#include<cstring>

//辅助函数 完成函数读取 写入和数值范围检查
namespace IM6014 {
namespace {
  constexpr double kRatio = 38.0 / 3.0; 
  constexpr double kTwoPi = 6.28318530717958647692;
  // 按小端顺序读取整数，不要求数据地址对齐。
  uint16_t readU16(const uint8_t *data) {
  return static_cast<uint16_t>(
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8));
  }

  uint32_t readU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
  }

  // 按补码还原有符号整数。
  int16_t readI16(const uint8_t *data) {
  const uint16_t value = readU16(data);
  return static_cast<int16_t>(
      (value & 0x8000U) != 0U
          ? static_cast<int32_t>(value) - 65536
          : static_cast<int32_t>(value));
  }

  int32_t readI32(const uint8_t *data) {
  const uint32_t value = readU32(data);
  return (value & 0x80000000U) != 0U
             ? -1 - static_cast<int32_t>(~value)
             : static_cast<int32_t>(value);
  }

  // 按小端顺序写入整数。
  void writeU16(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  }

  void writeU32(uint8_t *data, uint32_t value) {
  for (std::size_t i = 0; i < 4; ++i) {
    data[i] = static_cast<uint8_t>(value >> (8U * i));
  }
}

  // 转换为整数前，检查数值有限且在字段范围内。
  bool inRange(double value, double min, double max) {
  return std::isfinite(value) && value >= min && value <= max;
  }

  } // namespace

  //CRC实现
uint32_t crc(const uint8_t *data, std::size_t size) {
  if ((size % 4U) != 0U || (data == nullptr && size != 0U)) {
    return 0;
  }

  uint32_t result = 0xFFFFFFFFU;

  for (std::size_t offset = 0; offset < size; offset += 4) {
    // 每次读取一个小端 32 位字，从最高位开始处理。
    result ^= readU32(data + offset);

    for (int bit = 0; bit < 32; ++bit) {
      result = (result & 0x80000000U) != 0U
                   ? (result << 1) ^ 0x04C11DB7U
                   : result << 1;
    }
  }

  return result;
}

bool encodeCommand(const Command &command,uint8_t *data,std::size_t capacity){  
  // 检查缓冲区、目标 ID 和控制模式。
  if (data == nullptr || capacity < kTxSize ||
    command.id > 14U ||
    (command.mode != Mode::Stop && command.mode != Mode::Foc)) {
  return false;
  }
  //输出端的物理量转化为转子端协议数值
  const double torque = command.torque / kRatio * 2560.0;
  const double speed = command.speed * kRatio * 64.0 / kTwoPi;
  const double position = command.position * kRatio * 32768.0 / kTwoPi;
  const double kp = command.kp / (kRatio * kRatio) * 12800.0;
  const double kd = command.kd / (kRatio * kRatio) * 51200.0;
  // 检查数值范围
  if (!inRange(torque, -32768.0, 32767.0) ||
    !inRange(speed, -32768.0, 32767.0) ||
    !inRange(position, -2147483648.0, 2147483647.0) ||
    !inRange(kp, 0.0, 32767.0) ||
    !inRange(kd, 0.0, 32767.0)) {
  return false;
  }
  // 将数值转换为整数并写入数据缓冲区
  std::memset(data,0,kTxSize);
  //帧头
  data[0] = 0XFE;
  data[1] = 0xEE;
  //将ID、模式、超时保护放到同一个字节
  data[2] = static_cast<uint8_t>
      (command.id |
      (static_cast<uint8_t>(command.mode) << 4) |
      (command.timeout_enabled ? 0x80U : 0U));
  writeU16(data + 4,
           static_cast<uint16_t>(static_cast<int16_t>(torque)));
  writeU16(data + 6,
           static_cast<uint16_t>(static_cast<int16_t>(speed)));
  writeU32(data + 8,
           static_cast<uint32_t>(static_cast<int32_t>(position)));
  writeU16(data + 12, static_cast<uint16_t>(kp));
  writeU16(data + 14, static_cast<uint16_t>(kd));

  // 发送 CRC 包含帧头，覆盖前 16 字节。
  writeU32(data + 16, crc(data, 16));
  return true;
}

bool decodeFeedback(const uint8_t *data, std::size_t size,uint8_t expected_id,Feedback &feedback){
  // 检查数据有效性
  if (data == nullptr || size != kRxSize || expected_id > 14U ||
  data[0] != 0xFC || data[1] != 0xEE ||
  (data[2] & 0x0FU) != expected_id ||
  crc(data + 2, 20) != readU32(data + 22)) {
  return false;
  }
  //创建一个临时反馈对象
  Feedback decoded{};
  decoded.id = data[2] & 0x0FU;
  decoded.mode = (data[2] >> 4) & 0x07U;
  decoded.timed_out = (data[2] & 0x80U) != 0U;

  decoded.driver_temperature = static_cast<int8_t>(
      data[3] >= 128U ? static_cast<int16_t>(data[3]) - 256 : data[3]);
  decoded.winding_temperature = data[4];
  decoded.voltage = data[5] * 0.5F;

  // 转子端协议数值还原为输出端物理量。
  decoded.torque =
      static_cast<float>(readI16(data + 6) / 2560.0 * kRatio);
  decoded.speed =
      static_cast<float>(readI16(data + 8) * kTwoPi / 64.0 / kRatio);
  decoded.raw_position = readI32(data + 10);
  decoded.position =
      static_cast<float>(decoded.raw_position * kTwoPi / 32768.0 / kRatio);

  decoded.error = readU32(data + 14);
  decoded.warning =
      static_cast<uint8_t>((readU16(data + 18) >> 13) & 0x07U);

  // 有效反馈按原样保存，包括停机、超时和错误状态。
  feedback = decoded;
  return true;
}
// 解析器实现
bool StreamParser::feed(uint8_t byte, Feedback &feedback) {
  // 等待帧头的第一个字节。
  if (size_ == 0) {
    if (byte == 0xFC) {
      buffer_[size_++] = byte;
    }
    return false;
  }

  // 确认帧头的第二个字节；连续 FC 时保留最后一个 FC。
  if (size_ == 1) {
    if (byte == 0xEE) {
      buffer_[size_++] = byte;
    } else if (byte != 0xFC) {
      size_ = 0;
    }
    return false;
  }

  buffer_[size_++] = byte;
  if (size_ < kRxSize) {
    return false;
  }

  if (decodeFeedback(buffer_, kRxSize, expected_id_, feedback)) {
    reset();
    return true;
  }

  if (crc(buffer_ + 2, 20) != readU32(buffer_ + 22)) {
    ++bad_crc;
  } else {
    ++wrong_id;
  }

  // 坏帧中可能包含下一帧的开头，保留它并继续接收。
  for (std::size_t i = 1; i + 1 < kRxSize; ++i) {
    if (buffer_[i] == 0xFC && buffer_[i + 1] == 0xEE) {
      size_ = kRxSize - i;
      std::memmove(buffer_, buffer_ + i, size_);
      return false;
    }
  }

  // 没找到完整帧头时，保留末尾单独的 FC。
  size_ = buffer_[kRxSize - 1] == 0xFC ? 1U : 0U;
  if (size_ != 0U) {
    buffer_[0] = 0xFC;
  }
  return false;
}

} // namespace IM6014

