#include "tests/test_support.h"
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "systems/robot_factory.h"
using namespace rrtpff;
int main(int argc, char **argv) {
    return runTest([&] {
        require(argc == 2, "source directory required");
        const std::filesystem::path root = argv[1];
        for (const char *name : {"uni_bugtrap", "uni_wall", "quad2d_bugtrap", "quad2d_hole",
                                 "quad3d_block", "quad3d_window"}) {
            const auto file = root / "benchmark/config" / (std::string(name) + ".yaml");
            const auto c = YAML::LoadFile(file.string());
            require(c["profile"].as<std::string>() == "paper_fair", "explicit fair profile");
            require(dfMode(c) == "paper_r2", "r2 default");
            require(c["sampling"]["guidance_probability"].as<double>() == 0, "no map guidance");
            validatePaperFairConfig(c);
            const auto full = YAML::LoadFile((root / "benchmark/config/full_df" / file.filename()).string());
            require(dfMode(full) == "full_df", "retained full mode");
            validatePaperFairConfig(full);
            require(!c["simd"]["width"] && !c["simd"]["enabled"],
                    "fixed AVX2 float8 backend is not a per-case option");
            require(c["pff"]["defer_trajectories"].as<bool>(), "all six deferred curves");
            require(c["pff"]["additional_time_samples"].as<int>() == 12, "candidate set preserved");
            require(c["validation"]["max_step"].as<double>() == .005,
                    "specified discrete resolution preserved");
            require(!c["planner"]["top_k"], "Top-K removed");
            const std::string system = c["system"].as<std::string>();
            if (system == "unicycle") {
                require(c["dynobench"]["model_file"].IsDefined(), "paper unicycle override");
                const auto m = YAML::LoadFile(
                    (file.parent_path() / c["dynobench"]["model_file"].as<std::string>()).string());
                close(m["max_vel"].as<double>(), 1, 0, "unicycle speed");
                close(m["max_angular_vel"].as<double>(), 1.5, 0, "unicycle turn rate");
            } else if (system == "quad3d") {
                require(c["dynobench"]["model_file"].IsDefined(), "paper quad3d override");
                require(std::isinf(c["df_limits"]["max_acceleration"].as<double>()) &&
                            std::isinf(c["df_limits"]["max_angular_acceleration"].as<double>()),
                        "unpublished extra acceleration caps explicitly disabled");
                const auto m = YAML::LoadFile(
                    (file.parent_path() / c["dynobench"]["model_file"].as<std::string>()).string());
                close(m["m"].as<double>(), system == "quad3d" ? 1 : .034, 1e-12, "paper mass");
                require(!m["motor_control"].as<bool>(), "wrench mode");
                close(m["J_v"][2].as<double>(), .2, 1e-12, "yaw inertia");
                close(m["u_ub"][1].as<double>() * 20, 2, 1e-12, "torque normalization");
            } else {
                require(!c["dynobench"]["model_file"].IsDefined(),
                        "quad2d uses DynoBench source model");
                require(!c["dynobench"]["models_directory"].IsDefined(),
                        "models_directory removed");
            }
        }
    });
}
