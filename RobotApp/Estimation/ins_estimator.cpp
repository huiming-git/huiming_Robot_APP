#include "RobotApp/Estimation/ins_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "RobotApp/Domain/config.hpp"

namespace robotapp::estimation {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kGravity = 9.80665f;
constexpr float kMaxDtS = 0.05f;

inline float clampf(float v, float lo, float hi) { return std::clamp(v, lo, hi); }

inline float wrap_pi(float a)
{
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

inline float norm3(const float v[3])
{
  return std::sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

inline bool normalize3(float v[3])
{
  const float n = norm3(v);
  if (!(n > 0.0f) || !std::isfinite(n)) return false;
  v[0] /= n;
  v[1] /= n;
  v[2] /= n;
  return true;
}

inline void cross3(const float a[3], const float b[3], float out[3])
{
  out[0] = (a[1] * b[2]) - (a[2] * b[1]);
  out[1] = (a[2] * b[0]) - (a[0] * b[2]);
  out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

inline void quat_normalize(float q[4])
{
  const float n = std::sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
  if (!(n > 0.0f) || !std::isfinite(n))
  {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }
  q[0] /= n;
  q[1] /= n;
  q[2] /= n;
  q[3] /= n;
}

inline void quat_rotate_world_from_body(const float q[4], const float v_b[3], float out_w[3])
{
  // world = q * v * conj(q), q = (w,x,y,z), v = pure quaternion (0,v)
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float vx = v_b[0], vy = v_b[1], vz = v_b[2];

  // t = 2 * cross(q_vec, v)
  const float tx = 2.0f * ((y * vz) - (z * vy));
  const float ty = 2.0f * ((z * vx) - (x * vz));
  const float tz = 2.0f * ((x * vy) - (y * vx));

  // out = v + w*t + cross(q_vec, t)
  out_w[0] = vx + (w * tx) + ((y * tz) - (z * ty));
  out_w[1] = vy + (w * ty) + ((z * tx) - (x * tz));
  out_w[2] = vz + (w * tz) + ((x * ty) - (y * tx));
}

inline void quat_rotate_body_from_world(const float q[4], const float v_w[3], float out_b[3])
{
  // body = conj(q) * v * q
  const float qc[4] = {q[0], -q[1], -q[2], -q[3]};
  quat_rotate_world_from_body(qc, v_w, out_b);
}

inline void quat_integrate(float q[4], const float omega_rps[3], float dt_s)
{
  const float wx = omega_rps[0], wy = omega_rps[1], wz = omega_rps[2];
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float half_dt = 0.5f * dt_s;

  q[0] = w + (-x * wx - y * wy - z * wz) * half_dt;
  q[1] = x + (w * wx + y * wz - z * wy) * half_dt;
  q[2] = y + (w * wy - x * wz + z * wx) * half_dt;
  q[3] = z + (w * wz + x * wy - y * wx) * half_dt;
  quat_normalize(q);
}

inline void quat_to_rpy(const float q[4], float rpy[3])
{
  const float w = q[0], x = q[1], y = q[2], z = q[3];

  const float sinr_cosp = 2.0f * ((w * x) + (y * z));
  const float cosr_cosp = 1.0f - 2.0f * ((x * x) + (y * y));
  rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * ((w * y) - (z * x));
  rpy[1] = (std::abs(sinp) >= 1.0f) ? std::copysign(kPi / 2.0f, sinp) : std::asin(sinp);

  const float siny_cosp = 2.0f * ((w * z) + (x * y));
  const float cosy_cosp = 1.0f - 2.0f * ((y * y) + (z * z));
  rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}

inline uint32_t age_us(uint64_t now_us, uint64_t src_us)
{
  if (now_us == 0U || src_us == 0U) return 0xFFFFFFFFU;
  if (now_us < src_us) return 0U;
  const uint64_t d = now_us - src_us;
  return (d > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : static_cast<uint32_t>(d);
}

}  // namespace

void InsEstimator::reset(uint64_t ts_us)
{
  last_ts_us_ = ts_us;
  q_wxyz_[0] = 1.0f;
  q_wxyz_[1] = 0.0f;
  q_wxyz_[2] = 0.0f;
  q_wxyz_[3] = 0.0f;
  vel_mps_[0] = vel_mps_[1] = vel_mps_[2] = 0.0f;
  pos_m_[0] = pos_m_[1] = pos_m_[2] = 0.0f;
  gyro_bias_rps_[0] = gyro_bias_rps_[1] = gyro_bias_rps_[2] = 0.0f;
  accel_bias_mps2_[0] = accel_bias_mps2_[1] = accel_bias_mps2_[2] = 0.0f;
  err_int_[0] = err_int_[1] = err_int_[2] = 0.0f;
}

domain::StateEstimate InsEstimator::step(const domain::ImuState& imu, const domain::MagState* mag,
                                         const domain::MotorFeedback* motor, uint64_t ts_us)
{
  domain::StateEstimate out{};

  if (ts_us == 0U || imu.ts_us == 0U)
  {
    out.ts_us = ts_us;
    out.valid = 0U;
    return out;
  }

  if (age_us(ts_us, imu.ts_us) > domain::config::kImuTimeoutUs)
  {
    out.ts_us = ts_us;
    out.valid = 0U;
    return out;
  }

  if (last_ts_us_ == 0U)
  {
    last_ts_us_ = ts_us;
    out.ts_us = ts_us;
    out.valid = 0U;
    return out;
  }

  const uint64_t dt_us_64 = (ts_us >= last_ts_us_) ? (ts_us - last_ts_us_) : 0ULL;
  last_ts_us_ = ts_us;
  const float dt_s = static_cast<float>(dt_us_64) * 1.0e-6f;
  if (!(dt_s > 0.0f) || !std::isfinite(dt_s) || (dt_s > kMaxDtS))
  {
    out.ts_us = ts_us;
    out.valid = 0U;
    return out;
  }

  // Wheel odometry measurement (if available and fresh).
  bool odom_ok = false;
  float odom_v_fwd = 0.0f;
  float odom_yaw_rate = 0.0f;
  const auto& wc = wheel_odom_cfg_;
  if ((motor != nullptr) && (motor->ts_us != 0U) &&
      (age_us(ts_us, motor->ts_us) <= domain::config::kOdomTimeoutUs) && (wc.wheel_radius_m > 0.0f) &&
      (wc.wheel_gear_ratio > 0.0f))
  {
    const float rpm_l =
        static_cast<float>(motor->wheels[0].rpm) * static_cast<float>(wc.rpm_sign_left);
    const float rpm_r =
        static_cast<float>(motor->wheels[1].rpm) * static_cast<float>(wc.rpm_sign_right);
    const float omega_l =
        (rpm_l * 2.0f * kPi / 60.0f) / wc.wheel_gear_ratio;  // rad/s at wheel
    const float omega_r =
        (rpm_r * 2.0f * kPi / 60.0f) / wc.wheel_gear_ratio;

    const float v_l = omega_l * wc.wheel_radius_m;
    const float v_r = omega_r * wc.wheel_radius_m;
    odom_v_fwd = 0.5f * (v_l + v_r);
    if (wc.wheel_track_m > 0.0f)
    {
      odom_yaw_rate = (v_r - v_l) / wc.wheel_track_m;  // rad/s
    }
    odom_ok = std::isfinite(odom_v_fwd) && std::isfinite(odom_yaw_rate);
  }

  // Gyro rad/s.
  float omega_rps[3] = {
      imu.gyro_dps[0] * kDegToRad,
      imu.gyro_dps[1] * kDegToRad,
      imu.gyro_dps[2] * kDegToRad,
  };

  // Subtract bias estimate.
  omega_rps[0] -= gyro_bias_rps_[0];
  omega_rps[1] -= gyro_bias_rps_[1];
  omega_rps[2] -= gyro_bias_rps_[2];

  // Optional yaw-rate correction from wheel odometry (disabled by default).
  if (odom_ok && (gains_.odom_yaw_gain > 0.0f))
  {
    const float yaw_err = odom_yaw_rate - omega_rps[2];
    omega_rps[2] += gains_.odom_yaw_gain * yaw_err;
  }

  // Accelerometer correction (gravity direction).
  float acc_b[3] = {
      imu.accel_mps2[0] - accel_bias_mps2_[0],
      imu.accel_mps2[1] - accel_bias_mps2_[1],
      imu.accel_mps2[2] - accel_bias_mps2_[2],
  };
  const float acc_norm = norm3(acc_b);
  if (std::isfinite(acc_norm) && (acc_norm > 0.5f * kGravity) && (acc_norm < 2.0f * kGravity))
  {
    float acc_unit[3] = {acc_b[0], acc_b[1], acc_b[2]};
    if (normalize3(acc_unit))
    {
      // Reference specific-force direction at rest in world frame: +Z (up).
      const float g_ref_w[3] = {0.0f, 0.0f, 1.0f};
      float g_est_b[3] = {0.0f, 0.0f, 0.0f};
      quat_rotate_body_from_world(q_wxyz_, g_ref_w, g_est_b);
      (void)normalize3(g_est_b);

      float err[3] = {0.0f, 0.0f, 0.0f};
      cross3(acc_unit, g_est_b, err);

      // Integrate error for bias estimation (clamped to avoid windup).
      err_int_[0] = clampf(err_int_[0] + err[0] * dt_s, -10.0f, 10.0f);
      err_int_[1] = clampf(err_int_[1] + err[1] * dt_s, -10.0f, 10.0f);
      err_int_[2] = clampf(err_int_[2] + err[2] * dt_s, -10.0f, 10.0f);

      omega_rps[0] += gains_.kp_acc * err[0] + gains_.ki_acc * err_int_[0];
      omega_rps[1] += gains_.kp_acc * err[1] + gains_.ki_acc * err_int_[1];
      omega_rps[2] += gains_.kp_acc * err[2] + gains_.ki_acc * err_int_[2];

      // Expose bias estimate derived from integral term.
      gyro_bias_rps_[0] = -gains_.ki_acc * err_int_[0];
      gyro_bias_rps_[1] = -gains_.ki_acc * err_int_[1];
      gyro_bias_rps_[2] = -gains_.ki_acc * err_int_[2];
    }
  }

  // Magnetometer yaw correction (tilt-compensated heading).
  if ((mag != nullptr) && (mag->ts_us != 0U) && (age_us(ts_us, mag->ts_us) <= domain::config::kMagTimeoutUs))
  {
    float m_b[3] = {mag->mag_uT[0], mag->mag_uT[1], mag->mag_uT[2]};
    if (normalize3(m_b))
    {
      float rpy[3] = {0.0f, 0.0f, 0.0f};
      quat_to_rpy(q_wxyz_, rpy);
      const float roll = rpy[0];
      const float pitch = rpy[1];
      const float yaw_est = rpy[2];

      const float cr = std::cos(roll);
      const float sr = std::sin(roll);
      const float cp = std::cos(pitch);
      const float sp = std::sin(pitch);

      const float mxh = (m_b[0] * cp) + (m_b[2] * sp);
      const float myh = (m_b[0] * sr * sp) + (m_b[1] * cr) - (m_b[2] * sr * cp);
      const float yaw_meas = std::atan2(myh, mxh);

      const float yaw_err = wrap_pi(yaw_meas - yaw_est);
      const float omega_yaw_w[3] = {0.0f, 0.0f, gains_.kp_mag_yaw * yaw_err};
      float omega_yaw_b[3] = {0.0f, 0.0f, 0.0f};
      quat_rotate_body_from_world(q_wxyz_, omega_yaw_w, omega_yaw_b);
      omega_rps[0] += omega_yaw_b[0];
      omega_rps[1] += omega_yaw_b[1];
      omega_rps[2] += omega_yaw_b[2];
    }
  }

  // Integrate attitude.
  quat_integrate(q_wxyz_, omega_rps, dt_s);

  // Dead-reckoning velocity/position (drifts without external references).
  float acc_w[3] = {0.0f, 0.0f, 0.0f};
  quat_rotate_world_from_body(q_wxyz_, acc_b, acc_w);
  acc_w[2] -= kGravity;
  vel_mps_[0] += acc_w[0] * dt_s;
  vel_mps_[1] += acc_w[1] * dt_s;
  vel_mps_[2] += acc_w[2] * dt_s;

  // Wheel-odometry correction (horizontal velocity observation).
  if (odom_ok && (gains_.odom_vel_gain > 0.0f))
  {
    // Convert estimated vel to body frame, correct forward component, then rotate back to world.
    float vel_b[3] = {0.0f, 0.0f, 0.0f};
    quat_rotate_body_from_world(q_wxyz_, vel_mps_, vel_b);
    const float k = clampf(gains_.odom_vel_gain, 0.0f, 1.0f);
    vel_b[0] = vel_b[0] + k * (odom_v_fwd - vel_b[0]);
    float vel_w_new[3] = {0.0f, 0.0f, 0.0f};
    quat_rotate_world_from_body(q_wxyz_, vel_b, vel_w_new);
    vel_mps_[0] = vel_w_new[0];
    vel_mps_[1] = vel_w_new[1];
    // Keep z from IMU integration (wheel odom doesn't observe vertical).
  }

  pos_m_[0] += vel_mps_[0] * dt_s;
  pos_m_[1] += vel_mps_[1] * dt_s;
  pos_m_[2] += vel_mps_[2] * dt_s;

  // Output snapshot.
  out.q_wxyz[0] = q_wxyz_[0];
  out.q_wxyz[1] = q_wxyz_[1];
  out.q_wxyz[2] = q_wxyz_[2];
  out.q_wxyz[3] = q_wxyz_[3];
  quat_to_rpy(q_wxyz_, out.rpy_rad);
  out.pos_m[0] = pos_m_[0];
  out.pos_m[1] = pos_m_[1];
  out.pos_m[2] = pos_m_[2];
  out.vel_mps[0] = vel_mps_[0];
  out.vel_mps[1] = vel_mps_[1];
  out.vel_mps[2] = vel_mps_[2];
  out.gyro_bias_rps[0] = gyro_bias_rps_[0];
  out.gyro_bias_rps[1] = gyro_bias_rps_[1];
  out.gyro_bias_rps[2] = gyro_bias_rps_[2];
  out.accel_bias_mps2[0] = accel_bias_mps2_[0];
  out.accel_bias_mps2[1] = accel_bias_mps2_[1];
  out.accel_bias_mps2[2] = accel_bias_mps2_[2];
  out.ts_us = ts_us;
  out.valid = 1U;
  return out;
}

}  // namespace robotapp::estimation
