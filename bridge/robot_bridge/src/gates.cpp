#include "gates.hpp"

#include <algorithm>
#include <cmath>

namespace sairol_bridge::gates
{
    float_t limitTarget(float_t q_target, float_t q_prev, const Joint &joint, float_t dt)
    {
        const float_t q = std::clamp(q_target, joint.q_min, joint.q_max);
        if (!(joint.q_rate_limit > 0) || !std::isfinite(q_prev) || !(dt > 0))
        {
            return q;
        }
        const float_t step = joint.q_rate_limit * dt;
        return q_prev + std::clamp(q - q_prev, -step, step);
    }

    float_t torqueWindow(float_t q, float_t q_meas, float_t dq_meas, float_t dq_cmd,
                         float_t kp, float_t kd, float_t tau, float_t tau_limit)
    {
        if (!(kp > 1e-6f))
        {
            return q_meas;
        }
        const float_t bias = kd * (dq_meas - dq_cmd) - tau;   // the feedforward uses up part of tau_limit
        return std::clamp(q, (bias - tau_limit) / kp + q_meas, (bias + tau_limit) / kp + q_meas);
    }

    bool finiteCommand(float_t q, float_t dq, float_t kp, float_t kd, float_t tau)
    {
        return std::isfinite(q) && std::isfinite(dq) && std::isfinite(kp) && std::isfinite(kd) &&
               std::isfinite(tau);
    }
} // namespace sairol_bridge::gates
