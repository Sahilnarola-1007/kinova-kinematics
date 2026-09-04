/**
 * @file benchmark_safety_filter.cpp
 * @brief O3 latency benchmark for the three-site SafetyFilter on the 1 kHz path.
 *        Times one full filter CYCLE — clipForce -> clipCartesian -> clipJoint —
 *        under four input scenarios, because this code (unlike jointVelocityDLS)
 *        has data-dependent branches and therefore has no single worst case.
 *
 * @author Sahil Narola — Phase 0, Step 6.4 (safety filter), O3 telemetry
 * @date   August 2026
 *
 * Layer: BELOW THE CONTRACT (adapter). Command/action side.
 *   Site 1  ACTION      : F*n [N], tool-z
 *   Site 2  COMMAND     : task-frame twist [m/s, rad/s]; ee_pos_base [m] is OBSERVED
 *   Site 3  COMMAND     : joint rates [rad/s]; q_send [rad] is the COMMANDED anchor (P19)
 *
 * Run pinned + real-time — host scheduling is owned by the invocation, not this file,
 * and the protocol must match benchmark_dls exactly or the two numbers cannot be added:
 *   chrt -f 80 taskset -c 2 ./benchmark_safety_filter filter_latency
 *
 * BUILD TYPE IS PART OF THE PROTOCOL.
 *   Eigen relies on inlining to collapse expression templates; at -O0 that collapse
 *   does not happen and the number is pessimistic by a large, unknown factor.
 *   Required — and it must be RelWithDebInfo, NOT Release, so this matches the flags
 *   benchmark_dls was measured under. Release is -O3 with no -g; mixing the two makes
 *   the numbers non-additive, which defeats the purpose of matching the protocol:
 *     colcon build --packages-select kinova_kinematics \
 *       --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_KORTEX_MOCK=ON
 *
 *   [VERIFIED 4 Sep 2026] CMAKE_BUILD_TYPE:STRING=RelWithDebInfo in
 *   build/kinova_kinematics/CMakeCache.txt; compile_commands.json shows this target
 *   and benchmark_dls both received:  -O2 -g -DNDEBUG -std=gnu++17
 *   Re-verify after any toolchain or CMake change:
 *     python3 -c "import json; print(*[e['command'] for e in json.load(open( \
 *       'build/kinova_kinematics/compile_commands.json')) \
 *       if 'benchmark_safety_filter' in e['file']], sep='\n')"
 *
 *   NOTE ON THE NAME: "O3" in this file is the project REQUIREMENT ID (zero bound
 *   violations across every reported run). It is not the -O3 compiler flag. The two
 *   must never be conflated in the paper or in design.md.
 *
 * Method notes (deltas from benchmark_dls are deliberate; everything else is identical):
 *  - FOUR scenarios, not one. NOMINAL is the deployment path (D-09: bounds sit just
 *    outside the trained action distribution, so the filter should be silent).
 *    ALL_TRIP is maximum work. ALTERNATING defeats the branch predictor and may beat
 *    both. NON_FINITE exercises the reject path added with FilterReason::
 *    NON_FINITE_REJECTED, which is currently green but untimed.
 *  - The three sites are timed TOGETHER. The 1 kHz budget is spent per cycle, and all
 *    three run every cycle; per-site numbers would have to be re-added anyway and each
 *    would carry its own clock-read overhead.
 *  - Inputs are cycled from a small pre-built table rather than being loop constants,
 *    so the compiler cannot hoist the call out of the loop. The table is sized to stay
 *    in L1d.  [UNVERIFIED] table footprint vs this CPU's L1d — check with lscpu if the
 *    tail looks wrong.
 *  - SELF-CHECK ON THE COUNTERS. A benchmark that believes it is exercising the worst
 *    case but silently isn't is worse than no benchmark. Each scenario asserts the
 *    InterventionCounter deltas it expects and fails loudly if they disagree.
 *  - Batched (amortised) mean is reported alongside the per-iteration distribution.
 *    A single filter cycle is expected to land near the cost of two steady_clock reads,
 *    so the per-iteration mean is clock-contaminated; the batched mean is not. The
 *    percentiles still come from the per-iteration run — batching destroys the tail.
 *  - steady_clock is correct here: this measures an ELAPSED INTERVAL. The project rule
 *    preferring an absolute cycle anchor applies to COMMAND generation, not measurement.
 *
 * ── MEASURED  [4 Sep 2026, RelWithDebInfo, chrt -f 80 taskset -c 2, N = 1e6] ──────
 *
 *   scenario       mean   batched    p50    p99   p99.9     max      (all ns)
 *   NOMINAL        21.0     10.0      21     27      35    5391
 *   ALL_TRIP       30.2     15.0      30     36      39    1311
 *   ALTERNATING    25.6     12.6      28     35      40    1142
 *   NON_FINITE     12.6      5.1      13     14      21     785
 *   clock floor (min of 1e5 paired reads): 9 ns
 *
 *   Reportable: worst p99.9 across scenarios <= 40 ns = 0.004 % of the 1 kHz budget.
 *   The BATCHED column is the true cycle cost (5-15 ns). The per-iteration mean sits
 *   ~1 clock floor above it, exactly as predicted above — do not report the mean.
 *
 *   Do NOT claim ALTERNATING is the worst case. Its p99.9 beats ALL_TRIP by 1 ns at a
 *   9 ns clock resolution; the four scenarios are indistinguishable in the tail. The
 *   branch-predictor hypothesis is neither supported nor refuted by this data.
 *
 *   NOMINAL's 5391 ns max is a lone outlier and the only one of its size across the
 *   four rows. Still 0.5 % of budget. Presumed scheduling/first-touch, not filter work.
 */

#include <kinova_kinematics/SafetyFilter.hpp>

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

// ── Configuration ────────────────────────────────────────────────────────────
// 1e6 iterations: p99.9 over the 1e4 floor mandated by the checklist is only the
// 10th-worst sample, which is not a stable tail. 1e6 costs a few seconds of CPU.
constexpr std::size_t kIterations = 1'000'000;
constexpr std::size_t kWarmup     =    10'000;

// Power of two so the cycle index is a mask, not a division.
constexpr std::size_t kTableSize  = 64;
constexpr std::size_t kTableMask  = kTableSize - 1;

// Inner repetitions for the amortised-mean run (clock read cost divided by kBatch).
constexpr std::size_t kBatch      = 64;

constexpr double kCycleBudgetNs   = 1'000'000.0;   // 1 kHz

// ── One cycle's worth of filter inputs ───────────────────────────────────────
struct FilterInput
{
    double                      f_n         = 0.0;                                   // [N]
    Eigen::Matrix<double, 6, 1> twist_task  = Eigen::Matrix<double, 6, 1>::Zero();   // [m/s, rad/s]
    Eigen::Vector3d             ee_pos_base = Eigen::Vector3d::Zero();               // [m]
    Eigen::Matrix<double, 7, 1> qdot        = Eigen::Matrix<double, 7, 1>::Zero();   // [rad/s]
    Eigen::Matrix<double, 7, 1> q_send      = Eigen::Matrix<double, 7, 1>::Zero();   // [rad]
};

enum class Scenario { NOMINAL, ALL_TRIP, ALTERNATING, NON_FINITE };

const char* name(Scenario s)
{
    switch (s)
    {
        case Scenario::NOMINAL:     return "NOMINAL     (no bound fires)";
        case Scenario::ALL_TRIP:    return "ALL_TRIP    (every bound fires)";
        case Scenario::ALTERNATING: return "ALTERNATING (predictor defeated)";
        case Scenario::NON_FINITE:  return "NON_FINITE  (NaN/inf reject path)";
    }
    return "?";
}

const char* shortName(Scenario s)
{
    switch (s)
    {
        case Scenario::NOMINAL:     return "nominal";
        case Scenario::ALL_TRIP:    return "all_trip";
        case Scenario::ALTERNATING: return "alternating";
        case Scenario::NON_FINITE:  return "non_finite";
    }
    return "unknown";
}

// ── Input construction ───────────────────────────────────────────────────────
// Every value is derived from the bounds, never hard-coded against them, so the
// scenarios stay valid if D-09 moves the bounds or if pos_lookahead_dt is renamed
// to stop_horizon_s and retuned to tau.

/// All inputs comfortably inside every bound. jitter varies the data without
/// changing which branches are taken.
FilterInput makeNominal(const SafetyBounds& b, double jitter)
{
    FilterInput in;
    const double s = 1.0 + 0.001 * jitter;

    in.f_n = 0.30 * b.f_n_max * s;                      // ~30 % of the force cap

    in.twist_task(0) = 0.20 * b.v_tan_max * s;          // vx
    in.twist_task(1) = 0.10 * b.v_tan_max * s;          // vy
    in.twist_task(2) = -0.02 * s;                       // vz — reflex-owned, never scaled
    in.twist_task(3) = 0.10 * b.omega_max * s;          // wx
    in.twist_task(4) = 0.10 * b.omega_max * s;          // wy
    in.twist_task(5) = 0.10 * b.omega_max * s;          // wz

    // Centre of the workspace box: no face can refuse.
    in.ee_pos_base.x() = 0.5 * (b.ws_x_min + b.ws_x_max);
    in.ee_pos_base.y() = 0.5 * (b.ws_y_min + b.ws_y_max);
    in.ee_pos_base.z() = 0.5 * (b.ws_z_min + b.ws_z_max);

    for (int j = 0; j < 7; ++j)
    {
        in.qdot(j)   = 0.15 * b.joint_vel_max[j] * s;   // well under the cap
        in.q_send(j) = 0.0;                             // mid-range for every joint
    }
    return in;
}

/// Every bound fires: force scaled, v_tan scaled, omega scaled, all three workspace
/// faces refused, joint velocity scaled, and position refused on the three limited
/// joints (2, 4, 6 — indices 1, 3, 5; the rest are continuous-rotation sentinels).
FilterInput makeAllTrip(const SafetyBounds& b, double jitter)
{
    FilterInput in;
    const double s = 1.0 + 0.001 * jitter;

    in.f_n = 3.0 * b.f_n_max * s;                       // 3x over the cap

    in.twist_task(0) =  4.0 * b.v_tan_max * s;          // +x, drives out of the box
    in.twist_task(1) =  4.0 * b.v_tan_max * s;          // +y, drives out of the box
    in.twist_task(2) =  0.05 * s;                       // +z, drives out of the box
    in.twist_task(3) =  3.0 * b.omega_max * s;
    in.twist_task(4) =  3.0 * b.omega_max * s;
    in.twist_task(5) =  3.0 * b.omega_max * s;

    // Outside the upper face on all three axes, with the (post-scaling) twist still
    // pointing outward — so this trips the directional refusal, not just the box test.
    in.ee_pos_base.x() = b.ws_x_max + 0.10;
    in.ee_pos_base.y() = b.ws_y_max + 0.10;
    in.ee_pos_base.z() = b.ws_z_max + 0.10;

    for (int j = 0; j < 7; ++j)
    {
        in.qdot(j) = 3.0 * b.joint_vel_max[j] * s;      // positive: drives toward q_max

        // Sitting exactly ON the upper limit means q_next exceeds it for ANY dt > 0
        // and any surviving positive rate. Deliberately dt-independent: the scenario
        // must not silently stop tripping when pos_lookahead_dt is retuned.
        // Continuous joints keep their sentinel and simply never refuse — which is
        // the true worst case, since only 3 of 7 joints CAN refuse.
        const bool limited = std::isfinite(b.joint_pos_max[j]) &&
                             b.joint_pos_max[j] < 1e8;
        in.q_send(j) = limited ? b.joint_pos_max[j] : 0.0;
    }
    return in;
}

/// NaN on even entries, +inf on odd — both take the reject branch, so the branch
/// pattern is uniform while both flavours of non-finite input get timed. inf is the
/// one that mattered: inf > bound is TRUE, which is how phantom interventions used
/// to reach the O3 count before the guard went in.
FilterInput makeNonFinite(const SafetyBounds& b, std::size_t k)
{
    FilterInput in = makeNominal(b, static_cast<double>(k));
    const double bad = (k % 2 == 0) ? std::numeric_limits<double>::quiet_NaN()
                                    : std::numeric_limits<double>::infinity();
    in.f_n           = bad;   // Site 1
    in.twist_task(0) = bad;   // Site 2
    in.qdot(0)       = bad;   // Site 3
    return in;
}

std::vector<FilterInput> buildTable(Scenario sc, const SafetyBounds& b)
{
    std::vector<FilterInput> t;
    t.reserve(kTableSize);
    for (std::size_t k = 0; k < kTableSize; ++k)
    {
        const double j = static_cast<double>(k);
        switch (sc)
        {
            case Scenario::NOMINAL:     t.push_back(makeNominal(b, j));  break;
            case Scenario::ALL_TRIP:    t.push_back(makeAllTrip(b, j));  break;
            case Scenario::ALTERNATING: t.push_back((k % 2 == 0) ? makeNominal(b, j)
                                                                 : makeAllTrip(b, j)); break;
            case Scenario::NON_FINITE:  t.push_back(makeNonFinite(b, k)); break;
        }
    }
    return t;
}

// ── Optimiser barriers (GCC/Clang) ───────────────────────────────────────────
template <typename T>
inline void doNotOptimize(const T& value) { asm volatile("" : : "r,m"(value) : "memory"); }
inline void clobber() { asm volatile("" : : : "memory"); }

/// Nearest-rank percentile on an ascending-sorted vector.
std::int64_t percentile(const std::vector<std::int64_t>& sorted, double p)
{
    const std::size_t n = sorted.size();
    std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(n - 1) + 0.5);
    if (idx >= n) idx = n - 1;
    return sorted[idx];
}

struct Report
{
    Scenario     scenario;
    double       mean_ns        = 0.0;   ///< per-iteration (clock-contaminated)
    double       batched_ns     = 0.0;   ///< amortised over kBatch (clock-free)
    std::int64_t p50_ns = 0, p99_ns = 0, p999_ns = 0, max_ns = 0;
    bool         counters_ok    = true;
    std::uint8_t pos_mask       = 0;     ///< OR of jr.pos_refused_mask over warmup
    InterventionCounter delta{};
};

/// Verify the scenario actually exercised the branches it claims to.
///
/// The counters are reset immediately before the timed loop (see runScenario), so the
/// deltas belong to exactly kIterations calls — warmup and the batched run are NOT
/// included. That makes exact counts checkable, not just their direction.
///
/// Two expectations are asserted EXACTLY because they encode design invariants that a
/// refactor could silently break:
///   - NON_FINITE must report 3 rejections per iteration, one per site. A value of N
///     instead of 3N means a site stopped guarding its input.
///   - ALL_TRIP / ALTERNATING must refuse joint POSITION on the three limited joints
///     only (2, 4, 6 -> bit indices 1, 3, 5 -> mask 0b0101010 = 42). Joints 1/3/5/7
///     are continuous-rotation sentinels and can never refuse; a mask above 42 means
///     a sentinel was treated as a real limit.
/// The mask is observed during warmup (untimed), so checking it costs no latency.
bool checkCounters(Scenario         sc,
                   const InterventionCounter& d,
                   std::uint8_t     pos_mask,
                   std::size_t      n_iter,
                   std::ostream&    err)
{
    bool ok = true;
    auto require = [&](bool cond, const char* what)
    {
        if (!cond) { err << "  COUNTER CHECK FAILED: " << what << '\n'; ok = false; }
    };

    switch (sc)
    {
        case Scenario::NOMINAL:
            require(d.total() == 0,
                    "NOMINAL fired a bound — inputs are not inside the bounds, "
                    "so this is not the silent path");
            break;

        case Scenario::ALL_TRIP:
        case Scenario::ALTERNATING:
            require(d.force_scaled      > 0, "force bound never fired");
            require(d.v_tan_scaled      > 0, "v_tan bound never fired");
            require(d.omega_scaled      > 0, "omega bound never fired");
            require(d.workspace_refused > 0, "workspace refusal never fired");
            require(d.joint_vel_scaled  > 0, "joint velocity bound never fired");
            require(d.joint_pos_refused > 0, "joint position refusal never fired");
            require(d.non_finite_rejected == 0,
                    "a finite scenario reached the non-finite reject path");
            require(pos_mask == 0b0101010,
                    "joint position refusal mask != 42 — either a limited joint (2,4,6) "
                    "failed to refuse, or a continuous-rotation sentinel (1,3,5,7) was "
                    "treated as a real limit");
            break;

        case Scenario::NON_FINITE:
            require(d.non_finite_rejected > 0, "non-finite reject path never fired");
            require(d.non_finite_rejected == 3ULL * n_iter,
                    "non_finite_rejected != 3 x iterations — the reject path is expected "
                    "to fire once per SITE per cycle (Sites 1, 2 and 3 each receive a "
                    "poisoned input); a different ratio means a site stopped guarding");
            break;
    }
    return ok;
}

Report runScenario(Scenario sc, const SafetyBounds& bounds, const std::string& prefix)
{
    using clock = std::chrono::steady_clock;

    SafetyFilter filter(bounds);
    const std::vector<FilterInput> table = buildTable(sc, bounds);

    double sink = 0.0;

    // Warmup: cold cache, page faults, first-touch of the sample vector.
    // The position-refusal mask is OR-accumulated here rather than in the timed loop —
    // observing it costs nothing because this pass is untimed, and the inputs are the
    // same table the timed loop will cycle through.
    std::uint8_t pos_mask = 0;
    for (std::size_t i = 0; i < kWarmup; ++i)
    {
        const FilterInput& in = table[i & kTableMask];
        const ForceFilterResult     fr = filter.clipForce(in.f_n);
        const CartesianFilterResult cr = filter.clipCartesian(in.twist_task, in.ee_pos_base);
        const JointFilterResult     jr = filter.clipJoint(in.qdot, in.q_send);
        pos_mask |= jr.pos_refused_mask;
        sink += fr.scale + cr.alpha + jr.vel_scale;
        doNotOptimize(sink);
    }

    // Amortised mean: kBatch cycles between two clock reads. Removes clock overhead
    // from the mean; says nothing about the tail.
    double batched_ns = 0.0;
    {
        constexpr std::size_t kBatchRuns = 10'000;
        long double total = 0.0L;
        for (std::size_t r = 0; r < kBatchRuns; ++r)
        {
            const auto t0 = clock::now();
            for (std::size_t i = 0; i < kBatch; ++i)
            {
                const FilterInput& in = table[i & kTableMask];
                const ForceFilterResult     fr = filter.clipForce(in.f_n);
                const CartesianFilterResult cr = filter.clipCartesian(in.twist_task,
                                                                      in.ee_pos_base);
                const JointFilterResult     jr = filter.clipJoint(in.qdot, in.q_send);
                sink += fr.scale + cr.alpha + jr.vel_scale;
                doNotOptimize(sink);
            }
            const auto t1 = clock::now();
            total += static_cast<long double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
        }
        batched_ns = static_cast<double>(total /
                     static_cast<long double>(kBatchRuns * kBatch));
    }

    // Counters are reset immediately before the timed run so the delta belongs to it.
    filter.interventions().reset();

    std::vector<std::int64_t> samples_ns;
    samples_ns.reserve(kIterations);          // pre-allocated: no growth while timing

    for (std::size_t i = 0; i < kIterations; ++i)
    {
        const FilterInput& in = table[i & kTableMask];
        const FilterInput* in_ptr = &in;
        doNotOptimize(in_ptr);          // barrier on the address, not the 200-byte struct

        const auto t0 = clock::now();
        clobber();
        const ForceFilterResult     fr = filter.clipForce(in.f_n);
        const CartesianFilterResult cr = filter.clipCartesian(in.twist_task, in.ee_pos_base);
        const JointFilterResult     jr = filter.clipJoint(in.qdot, in.q_send);
        clobber();
        const auto t1 = clock::now();

        sink += fr.scale + cr.alpha + jr.vel_scale;
        doNotOptimize(sink);
        samples_ns.push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    }

    Report rep;
    rep.scenario   = sc;
    rep.batched_ns = batched_ns;
    rep.delta      = filter.interventions();

    long double sum = 0.0L;
    for (const auto v : samples_ns) sum += static_cast<long double>(v);
    rep.mean_ns = static_cast<double>(sum / static_cast<long double>(kIterations));

    std::vector<std::int64_t> sorted = samples_ns;
    std::sort(sorted.begin(), sorted.end());
    rep.p50_ns  = percentile(sorted, 0.50);
    rep.p99_ns  = percentile(sorted, 0.99);
    rep.p999_ns = percentile(sorted, 0.999);
    rep.max_ns  = sorted.back();

    rep.pos_mask    = pos_mask;
    rep.counters_ok = checkCounters(sc, rep.delta, pos_mask, kIterations, std::cerr);

    const std::string path = prefix + "_" + shortName(sc) + ".csv";
    std::ofstream csv(path);                                   // RAII
    if (!csv) { std::cerr << "error: cannot open " << path << '\n'; }
    else
    {
        csv << "iteration,latency_ns\n";
        for (std::size_t i = 0; i < samples_ns.size(); ++i)
            csv << i << ',' << samples_ns[i] << '\n';
    }

    doNotOptimize(sink);   // sink is NaN by design in NON_FINITE; just deny elision
    return rep;
}

}  // namespace

int main(int argc, char** argv)
{
    using clock = std::chrono::steady_clock;
    const std::string prefix = (argc > 1) ? argv[1] : "filter_latency";

    // Measurement floor: two back-to-back clock reads with the same barriers as the
    // timed region. A filter cycle may be the same order of magnitude as this.
    std::int64_t floor_min_ns = std::numeric_limits<std::int64_t>::max();
    for (std::size_t i = 0; i < 100'000; ++i)
    {
        const auto t0 = clock::now();
        clobber();
        const auto t1 = clock::now();
        floor_min_ns = std::min<std::int64_t>(
            floor_min_ns,
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    }

    const SafetyBounds bounds;   // manifest defaults; values are D-09, still open

    const Scenario order[] = { Scenario::NOMINAL, Scenario::ALL_TRIP,
                               Scenario::ALTERNATING, Scenario::NON_FINITE };

    std::vector<Report> reports;
    reports.reserve(4);
    for (const Scenario sc : order) reports.push_back(runScenario(sc, bounds, prefix));

    std::cout.setf(std::ios::fixed);
    std::cout.precision(1);
    std::cout << "\nSafetyFilter cycle latency  (clipForce + clipCartesian + clipJoint)\n"
              << "  iterations/scenario : " << kIterations << '\n'
              << "  clock floor (min)   : " << floor_min_ns << " ns\n"
              << "  cycle budget        : " << kCycleBudgetNs << " ns (1 kHz)\n\n";

    // Column widths are explicit: name() strings differ in length, so unpadded output
    // does not line up and the table cannot be read at a glance or pasted into a doc.
    std::cout << "  " << std::left << std::setw(34) << "scenario" << std::right
              << std::setw(9)  << "mean"
              << std::setw(9)  << "batched"
              << std::setw(8)  << "p50"
              << std::setw(8)  << "p99"
              << std::setw(8)  << "p99.9"
              << std::setw(9)  << "max"
              << "   (ns)\n"
              << "  " << std::string(85, '-') << '\n';

    for (const Report& r : reports)
    {
        std::cout << "  " << std::left << std::setw(34) << name(r.scenario) << std::right
                  << std::setw(9)  << r.mean_ns
                  << std::setw(9)  << r.batched_ns
                  << std::setw(8)  << r.p50_ns
                  << std::setw(8)  << r.p99_ns
                  << std::setw(8)  << r.p999_ns
                  << std::setw(9)  << r.max_ns
                  << (r.counters_ok ? "" : "   [COUNTER CHECK FAILED]")
                  << '\n';
    }

    // Counter deltas, printed not just asserted. A benchmark that checks an invariant
    // silently gives you nothing to paste into design.md when the check passes.
    std::cout << "\n  Intervention counters over the timed run (" << kIterations
              << " cycles/scenario)\n"
              << "  " << std::left << std::setw(34) << "scenario" << std::right
              << std::setw(9) << "force"
              << std::setw(9) << "v_tan"
              << std::setw(9) << "omega"
              << std::setw(8) << "ws"
              << std::setw(9) << "j_vel"
              << std::setw(9) << "j_pos"
              << std::setw(12) << "nonfinite"
              << std::setw(7) << "mask"
              << '\n'
              << "  " << std::string(106, '-') << '\n';

    for (const Report& r : reports)
    {
        std::cout << "  " << std::left << std::setw(34) << name(r.scenario) << std::right
                  << std::setw(9)  << r.delta.force_scaled
                  << std::setw(9)  << r.delta.v_tan_scaled
                  << std::setw(9)  << r.delta.omega_scaled
                  << std::setw(8)  << r.delta.workspace_refused
                  << std::setw(9)  << r.delta.joint_vel_scaled
                  << std::setw(9)  << r.delta.joint_pos_refused
                  << std::setw(12) << r.delta.non_finite_rejected
                  << std::setw(7)  << static_cast<unsigned>(r.pos_mask)
                  << '\n';
    }
    std::cout << "\n  expected: NON_FINITE nonfinite == 3 x " << kIterations
              << " = " << (3ULL * kIterations)
              << " (one rejection per site);  ALL_TRIP/ALTERNATING mask == 42"
                 " (joints 2,4,6 only)\n";

    // The reportable O3 number is the worst p99.9 across scenarios, not the nominal one.
    std::int64_t worst_p999 = 0;
    Scenario     worst_sc   = Scenario::NOMINAL;
    bool         all_ok     = true;
    for (const Report& r : reports)
    {
        if (r.p999_ns > worst_p999) { worst_p999 = r.p999_ns; worst_sc = r.scenario; }
        all_ok = all_ok && r.counters_ok;
    }

    std::cout << "\n  worst-case p99.9    : " << worst_p999 << " ns  (" << name(worst_sc) << ")\n"
              << "  share of 1 kHz cycle: "
              << (100.0 * static_cast<double>(worst_p999) / kCycleBudgetNs) << " %\n"
              << "  csv prefix          : " << prefix << "_<scenario>.csv\n";

    if (!all_ok)
    {
        std::cerr << "\nFAIL: at least one scenario did not exercise the branches it claims.\n"
                     "The latency numbers above do not mean what the scenario name says.\n";
        return 1;
    }
    return 0;
}
