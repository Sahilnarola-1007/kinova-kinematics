/**
 * @file SafetyFilter.cpp
 * @brief Three-site safety filter (Step 6.4). Layer: below the contract, ACTION side,
 *        1 kHz path — no allocation, no logging, no locks, no exceptions.
 *        Header block and frames: SafetyFilter.hpp. Rationale: safety_filter.md.
 *
 * @author Sahil Narola
 * @date   August 2026
 */

#include <kinova_kinematics/SafetyFilter.hpp>
#include <algorithm>
#include <cmath>

// ── Construction ─────────────────────────────────────────────────────────────

SafetyFilter::SafetyFilter(const SafetyBounds& bounds)
    : bounds_(bounds)
    , counter_{}
{
}

// ── Site 1: clipForce ────────────────────────────────────────────────────────

ForceFilterResult SafetyFilter::clipForce(double f_n_desired) const
{

    ForceFilterResult result;

    if (!std::isfinite(f_n_desired))
    {
        result.value  = 0.0;
        result.scale  = 0.0;   // value == scale * input cannot hold for NaN; broken on purpose
        result.reason = FilterReason::NON_FINITE_REJECTED;
        ++counter_.non_finite_rejected;
        return result;
    }
    const double mag = std::abs(f_n_desired);

    if (mag <= bounds_.f_n_max)
    {
        result.value  = f_n_desired;
        result.scale  = 1.0;
        result.reason = FilterReason::PASS;
    }
    else
    {
        result.scale  = bounds_.f_n_max / mag;
        result.value  = f_n_desired * result.scale;
        result.reason = FilterReason::FORCE_SCALED;
        ++counter_.force_scaled;
    }
    return result;
}

// ── Site 2: clipCartesian ────────────────────────────────────────────────────

CartesianFilterResult SafetyFilter::clipCartesian(
    const Eigen::Matrix<double, 6, 1>& twist_task,
    const Eigen::Vector3d& ee_pos_base) const
{
    CartesianFilterResult result;

        if (!twist_task.allFinite() || !ee_pos_base.allFinite())
    {
        result.twist.setZero();
        result.alpha = 0.0;
        result.beta  = 0.0;
        result.reasons[0] = FilterReason::NON_FINITE_REJECTED;
        result.num_interventions = 1;
        ++counter_.non_finite_rejected;
        return result;
    }

    result.twist = twist_task;
    int reason_idx = 0;

    // Group A: tangential [vx, vy] only — vz is reflex-owned and never scaled here
    const double vx = twist_task(0);
    const double vy = twist_task(1);
    const double v_tan_mag = std::sqrt(vx * vx + vy * vy);

    if (v_tan_mag > bounds_.v_tan_max)
    {
        result.alpha = bounds_.v_tan_max / v_tan_mag;
        result.twist(0) = vx * result.alpha;
        result.twist(1) = vy * result.alpha;
        result.reasons[reason_idx++] = FilterReason::V_TAN_SCALED;
        ++result.num_interventions;
        ++counter_.v_tan_scaled;
    }

    // Group B: angular velocity [wx, wy, wz]
    const double wx = twist_task(3);
    const double wy = twist_task(4);
    const double wz = twist_task(5);
    const double omega_mag = std::sqrt(wx * wx + wy * wy + wz * wz);

    if (omega_mag > bounds_.omega_max)
    {
        result.beta = bounds_.omega_max / omega_mag;
        result.twist(3) = wx * result.beta;
        result.twist(4) = wy * result.beta;
        result.twist(5) = wz * result.beta;
        result.reasons[reason_idx++] = FilterReason::OMEGA_SCALED;
        ++result.num_interventions;
        ++counter_.omega_scaled;
    }

    // Workspace box: REFUSAL, per axis — zero the outward component, let inward pass
    // (otherwise an arm that drifts out is trapped). Position is BASE frame, twist is
    // TASK frame: valid only near axis alignment (safety_filter.md, Limitations).
    bool ws_hit = false;

    if (ee_pos_base.x() < bounds_.ws_x_min && result.twist(0) < 0.0)
        { result.twist(0) = 0.0; ws_hit = true; }
    else if (ee_pos_base.x() > bounds_.ws_x_max && result.twist(0) > 0.0)
        { result.twist(0) = 0.0; ws_hit = true; }

    if (ee_pos_base.y() < bounds_.ws_y_min && result.twist(1) < 0.0)
        { result.twist(1) = 0.0; ws_hit = true; }
    else if (ee_pos_base.y() > bounds_.ws_y_max && result.twist(1) > 0.0)
        { result.twist(1) = 0.0; ws_hit = true; }

    // The only place vz is touched: workspace refusal is last-resort on ALL axes
    if (ee_pos_base.z() < bounds_.ws_z_min && result.twist(2) < 0.0)
        { result.twist(2) = 0.0; ws_hit = true; }
    else if (ee_pos_base.z() > bounds_.ws_z_max && result.twist(2) > 0.0)
        { result.twist(2) = 0.0; ws_hit = true; }

   
    if (ws_hit)
    {
        result.reasons[reason_idx++] = FilterReason::WORKSPACE_REFUSED;
        ++result.num_interventions;
        ++counter_.workspace_refused;
    }

    return result;
}

// ── Site 3: clipJoint ────────────────────────────────────────────────────────

JointFilterResult SafetyFilter::clipJoint(
    const Eigen::Matrix<double, 7, 1>& qdot,
    const Eigen::Matrix<double, 7, 1>& q_send) const
{
    JointFilterResult result;

    if (!qdot.allFinite() || !q_send.allFinite())
    {
        result.qdot.setZero();
        result.vel_scale        = 0.0;
        result.pos_refused_mask = 0;   // no joint hit a limit — the input was garbage; keep the two distinguishable in logs
        result.reasons[0]       = FilterReason::NON_FINITE_REJECTED;
        result.num_interventions = 1;
        ++counter_.non_finite_rejected;
        return result;
    }
    result.qdot = qdot;
    int reason_idx = 0;

    // Velocity: ONE scale for the whole vector, set by the worst joint ratio.
    // Per-joint clipping would change the joint-space direction the DLS solved for.
    double alpha = 1.0;
    for (int j = 0; j < 7; ++j)
    {
        const double mag = std::abs(qdot(j));
        if (mag > bounds_.joint_vel_max[j])
            alpha = std::min(alpha, bounds_.joint_vel_max[j] / mag);
    }

    if (alpha < 1.0)
    {
        result.qdot = qdot * alpha;
        result.vel_scale = alpha;
        result.reasons[reason_idx++] = FilterReason::JOINT_VEL_SCALED;
        ++result.num_interventions;
        ++counter_.joint_vel_scaled;
    }

    // Position: refusal on the post-scaling rate. dt is a single-step lookahead;
    // a τ-based stopping horizon is an open item (SafetyFilter.hpp).
    const double dt = bounds_.pos_lookahead_dt;
    bool pos_hit = false;

    for (int j = 0; j < 7; ++j)
    {
        const double v      = result.qdot(j);          // post-velocity-scaling
        const double q_next = q_send(j) + v * dt;

        // Directional: refuse only motion driving FURTHER out; inward must pass or a
        // joint that overshot is stuck until a power cycle.
        const bool over_high = (q_next > bounds_.joint_pos_max[j]) && (v > 0.0);
        const bool over_low  = (q_next < bounds_.joint_pos_min[j]) && (v < 0.0);

        if (over_high || over_low)
        {
            result.qdot(j) = 0.0;
            result.pos_refused_mask |= (1u << j);
            pos_hit = true;
        }
    }

    if (pos_hit)
    {
        result.reasons[reason_idx++] = FilterReason::JOINT_POS_REFUSED;
        ++result.num_interventions;
        ++counter_.joint_pos_refused;
    }

    return result;
}
