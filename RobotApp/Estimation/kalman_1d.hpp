#pragma once

#include <cstdint>

namespace robotapp::estimation {

// Lightweight 1D Kalman filter for smoothing a scalar signal.
struct Kalman1D {
  float x = 0.0f;
  float p = 1.0f;
  float q_base = 0.001f;
  float r = 0.05f;
  bool initialized = false;

  void reset(float x0 = 0.0f)
  {
    x = x0;
    p = 1.0f;
    initialized = false;
  }

  void set_tunings(float q_in, float r_in)
  {
    q_base = q_in;
    r = r_in;
  }

  float update(float z, float dt_s)
  {
    if (!initialized)
    {
      x = z;
      p = 1.0f;
      initialized = true;
      return x;
    }

    const float q = (dt_s > 0.0f && dt_s < 1.0f) ? (q_base * dt_s) : q_base;
    p += q;
    const float k = p / (p + r);
    x = x + k * (z - x);
    p = (1.0f - k) * p;
    return x;
  }
};

}  // namespace robotapp::estimation
