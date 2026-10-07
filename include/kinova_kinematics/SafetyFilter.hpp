/**
 * @file SafetyFilter.hpp
 * @brief Three-site engineered clipping with intervention counting (D-07).
 *
 * ── SafetyFilter ─────────────────────────────────────────────────────────────
 * Layer:   Below the contract (adapter), 1 kHz path
 * Side:    ACTION (transforms commanded quantities; never feeds back to the policy)
 * In:      Site 1 F*n [N], tool z · Site 2 twist [m/s, rad/s] TASK frame + ee_pos [m] BASE
 *          · Site 3 q̇ [rad/s] + q_send [rad], joint space
 * Out:     same quantity, clipped; reason codes; per-bound counters (O3 telemetry)
 * Frames:  Site 2 mixes a task-frame twist with a base-frame position — valid only
 *          near axis alignment (safety_filter.md, Limitations)
 * Fails:   non-finite input → zero output, NON_FINITE_REJECTED, counted once per site
 * Ref:     safety_filter.md, design.md §5.6, D-04, D-09, D-27
 * ─────────────────────────────────────────────────────────────────────────────
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

// ── Bounds ───────────────────────────────────────────────────────────────────
// Header defaults [DESIGN, provisional]: final values come from the trained action
// distribution (D-09) and will be sourced from the manifest once it exists (D-04).

struct SafetyBounds
{
    // Site 1 — tool-z desired normal force. Meaningful only with the external 6-axis
    // wrist F/T sensor mounted and calibrated (stated precondition of the stack).
    double f_n_max   = 15.0;    ///< [N] [DESIGN] within the 40 N Cartesian hard limit [SPEC]

    // Site 2 — task frame
    double v_tan_max = 0.15;    ///< [m/s] [DESIGN] within the 0.5 m/s hard limit [SPEC]
    double omega_max = 0.50;    ///< [rad/s] [DESIGN] within the 0.8727 rad/s hard limit [SPEC]

    double ws_x_min =  0.1;     ///< Workspace box [m], BASE frame [DESIGN]
    double ws_x_max =  0.8;
    double ws_y_min = -0.15;
    double ws_y_max =  0.8;
    double ws_z_min =  0.02;
    double ws_z_max =  0.9;

    // Site 3 — joint space
    /// Per-joint velocity caps [rad/s]. [SPEC] Kortex actuator spec, Aug 2026; not confirmed
    /// by hardware test. User Guide Tables 40-41 (high-level library) are read as not
    /// applying in low-level servoing [UNVERIFIED by test].
    std::array<double, 7> joint_vel_max = {{
        2.0944, 2.0944, 2.0944, 2.0944,   // joints 1-4: 120 deg/s
        3.4907, 3.4907, 3.4907             // joints 5-7: 200 deg/s
    }};

    /// Per-joint position limits [rad]. [SPEC] Kinova Gen3 User Guide Table 39.
    /// Joints 1,3,5,7 are continuous: sentinel ±1e9 (never refuses). D-27: differs from
    /// the ±2π sentinel and 4th-decimal values in KinovaKinematics.cpp; the measured
    /// pos_refused_mask == 42 invariant holds only under this sentinel.
    std::array<double, 7> joint_pos_min = {{
        -1e9, -2.2515, -1e9, -2.5800, -1e9, -2.0996, -1e9
    }};
    std::array<double, 7> joint_pos_max = {{
         1e9,  2.2515,  1e9,  2.5800,  1e9,  2.0996,  1e9
    }};

    /// [s] single 1 kHz step [DESIGN]. The refusal bounds q_send, so servo lag cannot cause
    /// a violation. Overshoot could — not measured; Step 3. Remedy if any: a margin ε on the
    /// limit, not a larger dt.
    double pos_lookahead_dt = 0.001;
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

    /// Site 1: scale |F*n| to f_n_max, sign preserved. ACTION, pre-reflex. [N], tool z.
    ForceFilterResult clipForce(double f_n_desired) const;

    /// Site 2: post-reflex twist, TASK frame [m/s, rad/s]. α on [vx,vy], β on [wx,wy,wz];
    /// vz excluded from both norms (the reflex owns it). Workspace refusal on all axes
    /// uses ee_pos_base, BASE frame [m].
    CartesianFilterResult clipCartesian(
        const Eigen::Matrix<double, 6, 1>& twist_task,
        const Eigen::Vector3d& ee_pos_base) const;

    /// Site 3: post-DLS q̇ [rad/s]. UNIFORM scaling (preserves Cartesian direction), then
    /// directional single-step position refusal against q_send [rad] (commanded anchor).
    JointFilterResult clipJoint(
        const Eigen::Matrix<double, 7, 1>& qdot,
        const Eigen::Matrix<double, 7, 1>& q_send) const;

    const SafetyBounds& bounds() const { return bounds_; }
    const InterventionCounter& interventions() const { return counter_; }
    InterventionCounter& interventions() { return counter_; }

private:
    SafetyBounds                bounds_;
    mutable InterventionCounter counter_;   // mutated from const clip*; single 1 kHz thread only, not thread-safe [DESIGN]
};
