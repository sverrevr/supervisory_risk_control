#pragma once

namespace topic_indices
{
constexpr auto actuator_controls_roll = 0;
constexpr auto actuator_controls_pitch = 1;
constexpr auto actuator_controls_yaw = 2;
constexpr auto actuator_controls_throttle = 3;
constexpr auto actuator_outputs_0 = 4;
constexpr auto actuator_outputs_1 = 5;
constexpr auto actuator_outputs_2 = 6;
constexpr auto actuator_outputs_3 = 7;
constexpr auto motor_max = 8;
constexpr auto motor_diff = 9;
constexpr auto vxvy_error_component = 10;
constexpr auto vz_error_component = 11;
constexpr auto yawrate_error_component = 12;
constexpr auto rp_rate_deviation = 13;
constexpr auto vxvy_integral = 14;
constexpr auto vz_integral = 15;
constexpr auto yawrate_integral = 16;
constexpr auto js_derivative = 17;
}