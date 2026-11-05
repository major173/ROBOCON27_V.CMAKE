#pragma once

#include <cstdint>

struct ButtonEvents {
  bool short_press{false};
  bool long_press{false};
};

// 每个布尔按键使用一个实例；收到新按键数据时更新。
class Button {
public:
  ButtonEvents update(bool down, uint32_t now_ms) {
    ButtonEvents events{};
    if (down && !down_) {
      pressed_at_ms_ = now_ms;
      long_reported_ = false;
    }

    // 满 3 秒只报一次长按；短按在松手时报出。
    if (down_ && !long_reported_) {
      if (uint32_t(now_ms - pressed_at_ms_) >= 3000U) {
        events.long_press = true;
        long_reported_ = true;
      } else if (!down) {
        events.short_press = true;
      }
    }
    down_ = down;
    return events;
  }

private:
  uint32_t pressed_at_ms_{0};
  bool down_{false};
  bool long_reported_{false};
};
