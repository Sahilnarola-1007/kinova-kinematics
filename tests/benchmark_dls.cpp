/**
 * @file benchmark_dls.cpp
 * @brief Per-call latency of KinovaKinematics::jointVelocityDLS (computeJacobian +
 *        solveDLS) on the 1 kHz path. Reports mean and tail percentiles over N calls,
 *        writes every sample to CSV. ACTION side, below the contract. Step 6.2.
 * @author Sahil (ABL)
 *
 * Protocol (scheduling is owned by the invocation, not this file):
 *   chrt -f 80 taskset -c 2 ./benchmark_dls dls_latency.csv
 * Build type is part of the number: RelWithDebInfo (-O2 -g -DNDEBUG); Eigen's expression
 * templates only collapse under inlining. Results and protocol: design.md §8.2.
 *
 * Method: steady_clock is correct for an elapsed interval (the cycle-anchor rule is for
 * COMMAND generation); doNotOptimize/clobber defeat hoisting; per-iteration samples keep
 * the tail (batch timing destroys p99.9); the clock floor is measured and printed.
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

// p99.9 over 1e4 is the 10th-worst sample; 1e6 gives a stable tail in seconds [DESIGN].
constexpr std::size_t kIterations = 1'000'000;
constexpr std::size_t kWarmup     =    10'000;

// Arbitrary configuration [rad], believed non-singular [UNVERIFIED — SVD would check].
// Timing is config-independent: fixed FK count, fixed-size solve, no data-dependent
// branch [MEASURED by comparing against a second config, run not recorded].
constexpr std::array<double, 7> kQFixedRad = {0.10, 0.40, 0.00, 1.30, 0.00, 0.90, 0.50};

// BASE-frame twist, rows 0-2 [m/s], 3-5 [rad/s]. Value does not affect timing.
const Eigen::Matrix<double, 6, 1> kTwistDes =
    (Eigen::Matrix<double, 6, 1>() << 0.05, 0.0, 0.0, 0.0, 0.0, 0.0).finished();

constexpr double kLambda = 0.05;  // provisional [DESIGN]; does not affect latency

// Optimizer barriers (GCC/Clang): the timed call cannot be elided or hoisted.
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

    // Clock floor: two back-to-back reads + barriers. Conflicting readings exist
    // across runs (9 vs 13 ns) — do not cite until resolved (Step 9).
    std::int64_t floor_min_ns = std::numeric_limits<std::int64_t>::max();
    for (std::size_t i = 0; i < 100'000; ++i) {
        const auto t0 = clock::now();
        clobber();
        const auto t1 = clock::now();
        const auto ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        floor_min_ns = std::min<std::int64_t>(floor_min_ns, ns);
    }

    // Warmup (not recorded): cold cache, page faults.
    double sink = 0.0;
    for (std::size_t i = 0; i < kWarmup; ++i) {
        doNotOptimize(kQFixedRad);
        const DlsResult r = kin.jointVelocityDLS(kQFixedRad, kTwistDes, kLambda);
        sink += r.qdot(0);
        doNotOptimize(sink);
    }

    // Timed run, per-iteration samples.
    std::vector<std::int64_t> samples_ns;
    samples_ns.reserve(kIterations);  // no growth during timing

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

    // Stats
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

    // CSV out
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

    // Consume sink.
    if (std::isnan(sink)) std::cerr << "unreachable\n";
    std::cout << "  wrote             : " << out_path << '\n';
    return 0;
}
