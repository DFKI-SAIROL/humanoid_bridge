// joint.hpp ------------------------------------------------------------------
// One joint's limits and default gains. No ROS, so pure code and tests use it.
#pragma once

namespace sairol_bridge
{
    typedef float float_t;

    struct Joint
    {
        int idx;
        float_t q_min{0};
        float_t q_max{0};
        float_t dq_limit{0};
        float_t tau_limit{0};
        float_t kp{0};
        float_t kd{0};
        bool if_strong_joint{false}; // Whether the joint is a strong joint
        bool if_parallel_joint{false}; // Whether the joint is a parallel joint
        float_t q_rate_limit{0}; // max target speed, rad/s; 0 = none (docs/psi0_gates.md)
    };
} // namespace sairol_bridge
