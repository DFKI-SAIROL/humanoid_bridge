// gates.hpp ------------------------------------------------------------------
// Target gates between /robot_cmd and /lowcmd. No ROS: tests/test_gates.cpp
// covers them. Design: docs/psi0_gates.md.
#pragma once

#include "joint.hpp"

namespace sairol_bridge::gates
{
    // Gates 1 and 2: the joint range, then at most q_rate_limit·dt away from
    // q_prev (the last target sent). q_rate_limit 0: range only.
    float_t limitTarget(float_t q_target, float_t q_prev, const Joint &joint, float_t dt);

    // Gate 4: the torque-implied window, so kp·(q - q_meas) + kd·(dq_cmd - dq_meas) + tau stays within ±tau_limit.
    // At kp ~ 0 q has no effect, so q_meas: never a division by zero, never inf or NaN.
    float_t torqueWindow(float_t q, float_t q_meas, float_t dq_meas, float_t dq_cmd,
                         float_t kp, float_t kd, float_t tau, float_t tau_limit);

    // The last check of a tracking frame: false if any field is NaN or Inf.
    bool finiteCommand(float_t q, float_t dq, float_t kp, float_t kd, float_t tau);
} // namespace sairol_bridge::gates
