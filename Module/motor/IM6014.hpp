
 /**
  * @file IM6014.hpp
  * @author FN
  * @brief IM6014 声明控制数据 反馈数据 和函数接口
  *        1.使用的宇树电机IM6014 发送20字节 反馈26字节
  *        2.输入的五个控制量 分别为 
  *          a.torque前馈力矩
  *          b.position   目标位置
  *          c.velocity   目标速度
  *          d.kp         位置刚度
  *          e.kd         速度刚度/阻尼
  * @version 0.1
  * @date 2026-04-21
  */
  #pragma once

  #include <cstddef>
  #include <cstdint>

  namespace IM6014{


    //协议帧的固定长度,单位：字节。
    constexpr std::size_t kTxSize =20;
    constexpr std::size_t kRxSize =26;

    //电机命令控制支持的控制模式
    enum class Mode : uint8_t {
      Stop = 0 ,//停机
      Foc=1,//FOC混合控制
    };

    //IM6014发送端的控制参数指令 简化为身份+控制量
    struct Command{
      //身份
      uint8_t id{0};                           //电机ID:0~14
      Mode mode{Mode::Stop};                //默认停机
      bool timeout_enabled{true};           //电机通信超时开关
      //控制量
      float torque{0};                      //前馈力矩，单位：N.m
      float speed{0};                       //目标速度，单位：rad/s
      float position{0};                    //目标位置，单位：rad
      float kp{0};                          //位置刚度，单位：N.m/rad
      float kd{0};                          //速度刚度/阻尼，单位：N.m/(rad/s)
    };

    //IM6014接收端反馈参数
    struct Feedback{
      uint8_t id{0};                          //反馈的电机ID
      uint8_t mode{0};                        //原样保存反馈模式
      uint8_t timed_out{0};                   //电机是否超时

      int8_t driver_temperature{0};           // 驱动温度，℃
      uint8_t winding_temperature{0};         // 绕组温度，℃
      float voltage{0};                       // 电机端电压，V

      float torque{0};                        // 输出端力矩，Nm
      float speed{0};                         // 输出端角速度，rad/s
      float position{0};                      // 输出端位置，rad，不代表机构机械零位
      int32_t raw_position{0};                // 协议中的原始位置整数

      uint32_t error{0};                      // 错误位，可同时包含多个错误
      uint8_t warning{0};                     // 警告位
    };
    
    uint32_t crc(const uint8_t *data, std::size_t size);
    
    // 将控制量编码为协议帧,计算并写入CRC
    bool encodeCommand(const Command &command,uint8_t *data,std::size_t capacity);
    // 将协议帧解码为反馈量,并校验CRC
    bool decodeFeedback(const uint8_t *data, std::size_t size,uint8_t expected_id,Feedback &feedback);

    // 数据流解析器
    class StreamParser {
    public:
      explicit StreamParser(uint8_t expected_id = 0): expected_id_(expected_id) {}
      //输入一个字节
      //返回true->成功解析出一帧有效反馈 返回false->未解析出完整帧
      bool feed(uint8_t byte, Feedback &feedback);
      void reset() { size_ = 0; }
      // 辅助诊断计数。
      uint32_t bad_crc{0};           // CRC 校验失败次数
      uint32_t wrong_id{0};          // CRC 正确但 ID 不匹配的次数

    private:
      uint8_t expected_id_;         // 当前解析器对应的电机 ID
      uint8_t buffer_[kRxSize]{};   // 暂存尚未完成解析的一帧
      std::size_t size_{0};         // 当前已收集的字节数
};

} // namespace IM6014


