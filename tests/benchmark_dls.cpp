/**
 * @file benchmark_dls.cpp
 * @brief Latency benchmark for KinovaKinematics::jointVelocityDLS on the 1 kHz path.
 *        Times the full loop-path call (internal computeJacobian + solveDLS),
 *        reports mean and tail percentiles over N iterations, and writes every
 *        per-iteration sample to CSV.
 * @author Sahil (ABL) — Phase 0, Step 6.2 (differential IK latency)
 *
 * Layer: below the contract (differential IK). Action/command side.
 * Run pinned + real-time (host scheduling owned by the invocation, not this file):
 *   chrt -f 80 taskset -c 2 ./benchmark_dls dls_latency.csv
 *
 * Method notes:
 *  - steady_clock (monotonic) is the correct choice for interval measurement.
 *    The project rule "use cycle-count time, not steady_clock" is for COMMAND
 *    generation (smooth setpoints under jitter); a benchmark measures elapsed
 *    time, so it must read a real monotonic clock. Different job.
 *  - doNotOptimize()/clobber() defeat dead-code elimination and loop-invariant
 *    hoisting so the compiler cannot compute the solve once and reuse it.
 *  - Per-iteration timestamps (not batch timing) are required: batch timing
 *    yields only a mean and destroys the tail distribution p99.9 needs.
 *  - The empty-loop floor (back-to-back clock reads) is measured and printed so
 *    a sub-microsecond result stays honest about clock read overhead.
 *  - A tight p99.9-vs-mean gap is the evidence that the fixed-size Eigen path
 *    does not allocate on the hot path (the reason D-21 fixed the Jacobian size).
 */

#include <kinova_kinematics/KinovaKinematics.hpp>  
#include<eigen3/Eigen/Dense>   
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace {

// --- Benchmark configuration (knobs you own) --------------------------------
// p99.9 over 1e4 is only the 10th-worst sample; 1e6 gives a stable tail and
// still runs in a few seconds of CPU. Warmup covers cold cache / page faults.
constexpr std::size_t kIterations = 1'000'000;
constexpr std::size_t kWarmup     =    10'000;

// [UNVERIFIED] Arbitrary NON-SINGULAR configuration (rad). Replace with your
// measured home config if you want a pose-specific number — but timing is
// essentially config-independent here: the numerical Jacobian does the same
// fixed count of computeFK evaluations and solveDLS runs a fixed-size solve,
// regardless of the joint values (checked by comparing to another config).
constexpr std::array<double, 7> kQFixedRad = {0.10, 0.40, 0.00, 1.30, 0.00, 0.90, 0.50};

// Representative small twist, BASE frame: rows 0-2 linear [m/s], 3-5 angular [rad/s].
// Value does not affect compute time (no data-dependent branching in the solve).
const Eigen::Matrix<double, 6, 1> kTwistDes =
    (Eigen::Matrix<double, 6, 1>() << 0.05, 0.0, 0.0, 0.0, 0.0, 0.0).finished();

constexpr double kLambda = 0.05;  // damping; does not affect latency

// --- Optimizer barriers (GCC/Clang) -----------------------------------------
// Force `value` to be treated as read/written so the surrounding computation
// cannot be elided or hoisted out of the timed loop.
template <typename T>
inline void doNotOptimize(const T& value) {
    asm volatile("" : : "r,m"(value) : "memory");
}
inline void clobber() { asm volatile("" : : : "memory"); }

// Nearest-rank percentile on an already-sorted ascending vector.
std::int64_t percentile(const std::vector<std::int64_t>& sorted, double p) {
    const std::size_t n = sorted.size();
    std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(n - 1) + 0.5);
    if (idx >= n) idx = n - 1;
    return sorted[idx];
}

}  // namespace

int main(int argc, char** argv) {
    using clock = std::chrono::steady_clock;
    const char* out_path = (argc > 1) ? argv[1] : "dls_latency.csv";

    const KinovaKinematics kin;

    // --- Measurement floor: cost of the two clock reads + barriers -----------
    std::int64_t floor_min_ns = std::numeric_limits<std::int64_t>::max();
    for (std::size_t i = 0; i < 100'000; ++i) {
        const auto t0 = clock::now();
        clobber();
        const auto t1 = clock::now();
        const auto ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        floor_min_ns = std::min<std::int64_t>(floor_min_ns, ns);
    }

    // --- Warmup (not recorded) ----------------------------------------------
    double sink = 0.0;
    for (std::size_t i = 0; i < kWarmup; ++i) {
        doNotOptimize(kQFixedRad);
        const DlsResult r = kin.jointVelocityDLS(kQFixedRad, kTwistDes, kLambda);
        sink += r.qdot(0);
        doNotOptimize(sink);
    }

    // --- Timed run: per-iteration latency -----------------------------------
    std::vector<std::int64_t> samples_ns;
    samples_ns.reserve(kIterations);  // pre-allocate: no growth during timing

    for (std::size_t i = 0; i < kIterations; ++i) {
        doNotOptimize(kQFixedRad);
        const auto t0 = clock::now();
        const DlsResult r = kin.jointVelocityDLS(kQFixedRad, kTwistDes, kLambda);
        const auto t1 = clock::now();
        sink += r.qdot(0);
        doNotOptimize(sink);
        samples_ns.push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    }

    // --- Stats ---------------------------------------------------------------
    long double sum = 0.0L;
    for (const auto v : samples_ns) sum += static_cast<long double>(v);
    const double mean_ns = static_cast<double>(sum / static_cast<long double>(kIterations));

    std::vector<std::int64_t> sorted = samples_ns;
    std::sort(sorted.begin(), sorted.end());

    std::cout.setf(std::ios::fixed);
    std::cout.precision(1);
    std::cout << "jointVelocityDLS latency  (N=" << kIterations << ")\n"
              << "  clock floor (min) : " << floor_min_ns          << " ns\n"
              << "  mean              : " << mean_ns               << " ns\n"
              << "  p50               : " << percentile(sorted, 0.50)   << " ns\n"
              << "  p99               : " << percentile(sorted, 0.99)   << " ns\n"
              << "  p99.9             : " << percentile(sorted, 0.999)  << " ns\n"
              << "  max               : " << sorted.back()         << " ns\n"
              << "  budget            : 1'000'000 ns (1 kHz cycle)\n";

    // --- CSV out (RAII stream) ----------------------------------------------
    std::ofstream csv(out_path);
    if (!csv) {
        std::cerr << "error: cannot open " << out_path << " for writing\n";
        return 1;
    }
    csv << "iteration,latency_ns\n";
    for (std::size_t i = 0; i < samples_ns.size(); ++i) {
        csv << i << ',' << samples_ns[i] << '\n';
    }
    csv.close();

    // Consume sink so it cannot be optimized away.
    if (std::isnan(sink)) std::cerr << "unreachable\n";
    std::cout << "  wrote             : " << out_path << '\n';
    return 0;
}
