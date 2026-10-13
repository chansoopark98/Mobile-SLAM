// Independent arithmetic fixture, not a Vuforia SDK implementation or ABI stub.
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

int main() {
    using Point = std::array<double, 3>;
    // Analytical Rz(90deg), camera position (1,2,3), CV output axes.
    const auto world_to_camera_cv = [](Point p) -> Point {
        return {p[1] - 2.0, p[0] - 1.0, 3.0 - p[2]};
    };
    int checks = 0;
    const auto expect = [&checks](bool valid) {
        if (!valid) throw std::runtime_error("Independent published contract mismatch");
        ++checks;
    };
    expect(world_to_camera_cv({1, 2, 2}) == Point({0, 0, 1}));
    expect(world_to_camera_cv({1, 3, 1}) == Point({1, 0, 2}));
    const std::array<double, 16> column_major{{0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1}};
    expect(column_major[12] == 1 && column_major[13] == 2 && column_major[14] == 3);
    const std::uint64_t pose_ns = 1'000'000'000;
    expect(pose_ns == std::uint64_t{1'000'000'000});
    expect(pose_ns != std::uint64_t{1'000'000'001});
    expect(pose_ns - std::uint64_t{20'000'000} == std::uint64_t{980'000'000});
    expect(pose_ns - std::uint64_t{10'000'000} == std::uint64_t{990'000'000});
    expect(std::abs(180.0 * 0.02 - 3.6) < 1e-12);
    std::cout << "{\"evidence_scope\":\"independent_arithmetic_executable\",\"checks\":" << checks
              << ",\"vuforia_calls\":0,\"sdk_abi_verified\":false}\n";
}
