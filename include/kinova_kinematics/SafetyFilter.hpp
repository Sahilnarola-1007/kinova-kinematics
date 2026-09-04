/**
 * @file SafetyFilter.hpp
 * @brief Three-site safety filter for the force-conditioned manipulation skill.
 *        Layer: BELOW THE CONTRACT — adapter, 1 kHz path.
 *        Architecture and design rationale: see safety_filter.md
 *
 * @author Sahil Narola
 * @date   August 2026
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <array>
#include <cstdint>

// ── Reason codes (one per bound that can fire) ───────────────────────────────

enum class FilterReason : uint8_t
{
    PASS              = 0,
    FORCE_SCALED      = 1,   // Site 1
    V_TAN_SCALED      = 2,   // Site 2
    OMEGA_SCALED      = 3,   // Site 2
    WORKSPACE_REFUSED = 4,   // Site 2
    JOINT_VEL_SCALED  = 5,   // Site 3
    JOINT_POS_REFUSED = 6,   // Site 3
    NON_FINITE_REJECTED = 7  // Sites 1, 2, 3 — input was NaN/inf
};

// ── Result types ─────────────────────────────────────────────────────────────

struct ForceFilterResult
{
    double        value  = 0.0;                  ///< Clamped F*n [N]
    double        scale  = 1.0;                  ///< 1.0 = no clip
    FilterReason  reason = FilterReason::PASS;
};

struct CartesianFilterResult
{
    Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
    double alpha = 1.0;   ///< Scale on [vx, vy]
    double beta  = 1.0;   ///< Scale on [wx, wy, wz]

    // idx=0: V_TAN_SCALED, idx=1: OMEGA_SCALED, idx=2: WORKSPACE_REFUSED
    static constexpr int MAX_REASONS = 3;
    std::array<FilterReason, MAX_REASONS> reasons = {
        FilterReason::PASS, FilterReason::PASS, FilterReason::PASS
    };
    int num_interventions = 0;
};

struct JointFilterResult
{
    Eigen::Matrix<double, 7, 1> qdot = Eigen::Matrix<double, 7, 1>::Zero();
    double  vel_scale         = 1.0;
    uint8_t pos_refused_mask  = 0;     ///< Bit j set = joint j position-refused

    // Reason 1 = JOINT_VEL_SCALED, Reason 2 = JOINT_POS_REFUSED
    static constexpr int MAX_REASONS = 2;
    std::array<FilterReason, MAX_REASONS> reasons = {
        FilterReason::PASS, FilterReason::PASS
    };
    int num_interventions = 0;
};

// ── Bounds (from manifest, values are D-09) ──────────────────────────────────

struct SafetyBounds
{
    // Site 1
    double f_n_max   = 15.0;    ///< [N]

    // Site 2
    double v_tan_max = 0.15;    ///< [m/s]
    double omega_max = 0.50;    ///< [rad/s]

    double ws_x_min = -0.5;     ///< Workspace box [m], base frame
    double ws_x_max =  0.5;
    double ws_y_min = -0.5;
    double ws_y_max =  0.5;
    double ws_z_min =  0.05;
    double ws_z_max =  0.70;

    // Site 3
    /// Per-joint velocity caps [rad/s]. [SPEC] Kortex actuator spec, verified Aug 2026.
    /// High-level library enforces lower limits (Tables 40-41); not applied in low-level servoing.
    std::array<double, 7> joint_vel_max = {{
        2.0944, 2.0944, 2.0944, 2.0944,   // joints 1-4: 120 deg/s
        3.4907, 3.4907, 3.4907             // joints 5-7: 200 deg/s
    }};

    /// Per-joint position limits [rad]. [SPEC] User Guide Table 39.
    /// Joints 1,3,5,7 are continuous rotation — no position limit.
    std::array<double, 7> joint_pos_min = {{
        -1e9, -2.2515, -1e9, -2.5800, -1e9, -2.0996, -1e9
    }};
    std::array<double, 7> joint_pos_max = {{
         1e9,  2.2515,  1e9,  2.5800,  1e9,  2.0996,  1e9
    }};

    double pos_lookahead_dt = 0.001;  ///< [s] — single step at 1 kHz
};

// ── Intervention counter (O3 telemetry) ──────────────────────────────────────

struct InterventionCounter
{
    uint64_t force_scaled      = 0;
    uint64_t v_tan_scaled      = 0;
    uint64_t omega_scaled      = 0;
    uint64_t workspace_refused = 0;
    uint64_t joint_vel_scaled  = 0;
    uint64_t joint_pos_refused = 0;
    uint64_t non_finite_rejected = 0;

    uint64_t total() const
    {
        return force_scaled + v_tan_scaled + omega_scaled
             + workspace_refused + joint_vel_scaled + joint_pos_refused
             + non_finite_rejected;
    }

    void reset()
    {
        force_scaled = v_tan_scaled = omega_scaled = 0;
        workspace_refused = joint_vel_scaled = joint_pos_refused = 0;
        non_finite_rejected = 0;
    }
};

// ── SafetyFilter ─────────────────────────────────────────────────────────────

class SafetyFilter
{
public:
    explicit SafetyFilter(const SafetyBounds& bounds);

    /// Site 1: clamp F*n. ACTION side, pre-reflex.
    ForceFilterResult clipForce(double f_n_desired) const;

    /// Site 2: clamp twist. Task frame, post-assembly. vz excluded from norms.
    CartesianFilterResult clipCartesian(
        const Eigen::Matrix<double, 6, 1>& twist_task,
        const Eigen::Vector3d& ee_pos_base) const;

    /// Site 3: clamp q̇. Post-DLS. Uniform velocity scaling, position refusal.
    JointFilterResult clipJoint(
        const Eigen::Matrix<double, 7, 1>& qdot,
        const Eigen::Matrix<double, 7, 1>& q_send) const;

    const SafetyBounds& bounds() const { return bounds_; }
    const InterventionCounter& interventions() const { return counter_; }
    InterventionCounter& interventions() { return counter_; }

private:
    SafetyBounds                bounds_;
    mutable InterventionCounter counter_;
};
