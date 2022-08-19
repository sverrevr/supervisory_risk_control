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
constexpr auto rollspeed_integ = 8;
constexpr auto pitchspeed_integ = 9;
constexpr auto yawspeed_integ = 10;
constexpr auto vx_integ = 11;
constexpr auto vy_integ = 12;
constexpr auto vz_integ = 13;
constexpr auto motor_max = 14;
constexpr auto roll_pitch_js_error = 15;
constexpr auto yaw_moment = 16;
constexpr auto height_over_ground = 17;
constexpr auto roll_pitch_ref_error = 18;
constexpr auto roll = 19;
constexpr auto pitch = 20;
constexpr auto yaw = 21;
constexpr auto roll_setpoint = 22;
constexpr auto pitch_setpoint = 23;
constexpr auto roll_error = 22;
constexpr auto pitch_error = 23;
}