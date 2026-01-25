#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "RobotApp/Algo/chassis_controller.hpp"
#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Drivers/motors.hpp"
#include "RobotApp/Logic/logic_bus.hpp"
#include "RobotApp/Platform/can_port.hpp"
#include "RobotApp/Safety/safety_policy.hpp"

namespace robotapp::tasks {

class ControlLoop {
 public:
  using ControllerFn = algo::ChassisControllerFn;

  void init();
  void tick(uint64_t ts_us);
  void set_logic_bus(const logic::LogicBus* bus) { logic_bus_ = bus; }
  void set_chassis_controller(ControllerFn fn)
  {
    controller_.store(fn != nullptr ? fn : &algo::chassis_controller_step, std::memory_order_relaxed);
  }

  const domain::ControlTelemetry& telemetry() const { return telemetry_; }
  const domain::ChassisCommand& last_command() const { return last_cmd_; }
  bool last_command_valid() const { return last_cmd_valid_; }
  uint64_t last_command_ts_us() const { return last_cmd_ts_us_; }
  const domain::StateEstimate& last_filtered_estimate() const { return last_est_filtered_; }
  const domain::LqrState& last_lqr_state() const { return lqr_state_; }

 private:
  domain::ChassisSensors sensors_{};
  domain::ControlTelemetry telemetry_{};
  uint64_t last_ts_us_ = 0;
  uint32_t jitter_samples_ = 0;
  domain::ChassisCommand last_cmd_{};
  bool last_cmd_valid_ = false;
  uint64_t last_cmd_ts_us_ = 0;
  uint64_t last_wheels_fb_ts_us_ = 0;
  domain::StateEstimate last_est_filtered_{};
  domain::LqrState lqr_state_{};
  std::size_t dm_bringup_index_ = 0;
  bool dm_bringup_done_ = false;
  struct JointDebugState {
    bool has_home = false;
    std::array<float, 4> home{};
    uint64_t start_ts_us = 0;
  };
  JointDebugState joint_debug_{};

  struct WheelOdomState {
    bool valid = false;
    int32_t ticks = 0;
    uint16_t last_enc = 0;
    float vel_mps = 0.0f;
  };
  std::array<WheelOdomState, 2> wheel_odom_{};

 platform::AppCanPort can_port_{};
 drivers::StaticMotorManager<8> motor_mgr_{};
 drivers::Dji3508Group dji_group_{};
 std::array<drivers::Dm4310MotorWithBringup, 4> dm_motors_{};
 const logic::LogicBus* logic_bus_ = nullptr;  // injected
  std::atomic<ControllerFn> controller_{&algo::chassis_controller_step};
  safety::SafetyPolicy safety_{};
};

}  // namespace robotapp::tasks
