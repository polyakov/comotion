// End-to-end / integration coverage for `--conflict-record-dir` on
// apps/mobile_robot_2d_crossing.cpp (feature/conflict-record-collection).
// This exercises the actual compiled CLI binary as a subprocess -- the app's
// CLI validation and `writeSubproblemRecords` glue live entirely inside
// mobile_robot_2d_crossing.cpp's own translation unit (not a shared header),
// so there is no library-level entry point to unit test directly; running
// the real binary is the only way to observe this behavior.
//
// Cross-checks captured SubproblemRecords against the SAME run's real ARC
// telemetry (via --metrics-json's repair_attempt_events) to verify
// findValidWindow()'s independent window-search replay actually agrees with
// what ARC's live resolution loop did, across every conflict in the run --
// not just a single hand-picked example.

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#ifndef MOBILE_ROBOT_2D_CROSSING_BINARY
#error "MOBILE_ROBOT_2D_CROSSING_BINARY must be defined by CMake"
#endif

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

int g_failures = 0;

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "conflict_record_cli_regression: " << label
                  << " expected true\n";
        ++g_failures;
    }
    return value;
}

bool expectEq(const std::string &label, std::int64_t actual,
              std::int64_t expected) {
    if (actual != expected) {
        std::cerr << "conflict_record_cli_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        ++g_failures;
        return false;
    }
    return true;
}

bool expectHasKey(const std::string &label, const json &object,
                  const std::string &key) {
    if (!object.is_object() || !object.contains(key)) {
        std::cerr << "conflict_record_cli_regression: " << label
                  << " missing key '" << key << "'\n";
        ++g_failures;
        return false;
    }
    return true;
}

std::string readFile(const fs::path &p) {
    std::ifstream in(p);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

struct RunResult {
    int exit_code = -1;
    std::string combined_output;
};

// Runs the real mobile_robot_2d_crossing binary as a subprocess. Args are
// passed through verbatim (all call sites below use simple flag/value
// tokens with no embedded whitespace, so no quoting is required).
RunResult runApp(const std::vector<std::string> &args, const fs::path &log_path) {
    std::ostringstream cmd;
    cmd << "\"" << MOBILE_ROBOT_2D_CROSSING_BINARY << "\"";
    for (const auto &arg : args)
        cmd << " " << arg;
    cmd << " > \"" << log_path.string() << "\" 2>&1";
    const int status = std::system(cmd.str().c_str());
    RunResult result;
    result.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    result.combined_output = readFile(log_path);
    return result;
}

fs::path makeScratchDir(const std::string &label) {
    const auto dir = fs::temp_directory_path() /
        ("comotion_conflict_record_test_" + label + "_" +
         std::to_string(static_cast<long>(getpid())));
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

std::vector<fs::path> sortedConflictFiles(const fs::path &dir) {
    std::vector<fs::path> files;
    if (fs::exists(dir)) {
        for (const auto &entry : fs::directory_iterator(dir)) {
            if (entry.path().filename().string().rfind("conflict_", 0) == 0)
                files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

// ---------------------------------------------------------------------------
// 1. CLI validation.
// ---------------------------------------------------------------------------
bool testRequiresAlgorithmArc() {
    const auto dir = makeScratchDir("requires_arc");
    const auto result = runApp(
        {"--scenario", "random_crossing", "--num-robots", "4", "--seed", "1",
         "--scenario-generation-seed", "1", "--algorithm", "stcbs",
         "--conflict-record-dir", dir.string()},
        dir / "log.txt");
    bool ok = expectTrue("missing --algorithm arc exits non-zero",
                         result.exit_code != 0);
    ok &= expectTrue(
        "missing --algorithm arc reports the documented error",
        result.combined_output.find(
            "--conflict-record-dir requires --algorithm arc") !=
            std::string::npos);
    fs::remove_all(dir);
    return ok;
}

bool testRejectsOrParallelWorkerProcesses() {
    const auto dir = makeScratchDir("rejects_or_parallel");
    const auto result = runApp(
        {"--scenario", "random_crossing", "--num-robots", "4", "--seed", "1",
         "--scenario-generation-seed", "1", "--algorithm", "arc",
         "--or-parallel-worker-processes", "2", "--conflict-record-dir",
         dir.string()},
        dir / "log.txt");
    bool ok = expectTrue("--or-parallel-worker-processes 2 exits non-zero",
                         result.exit_code != 0);
    ok &= expectTrue(
        "--or-parallel-worker-processes 2 reports the documented error",
        result.combined_output.find("not compatible with "
                                    "--or-parallel-worker-processes") !=
            std::string::npos);
    fs::remove_all(dir);
    return ok;
}

bool testValidUsageSucceeds() {
    // Sanity check that the two rejection tests above are really testing
    // the validation branch, not some other unrelated failure mode: the
    // same flags minus the offending one must succeed.
    const auto dir = makeScratchDir("valid_usage");
    const auto result = runApp(
        {"--scenario", "random_crossing", "--num-robots", "4", "--seed", "1",
         "--scenario-generation-seed", "1", "--algorithm", "arc",
         "--conflict-record-dir", dir.string()},
        dir / "log.txt");
    const bool ok = expectEq("valid --conflict-record-dir usage exits zero",
                             result.exit_code, 0);
    fs::remove_all(dir);
    return ok;
}

// ---------------------------------------------------------------------------
// 2. Known-conflict scenario: schema field-by-field + robots ordering, at
//    the full-pipeline level (real CLI args, real Vamp collision backend).
// ---------------------------------------------------------------------------
bool testKnownConflictScenarioSchemaAndOrdering() {
    const int failures_before = g_failures;
    const auto dir = makeScratchDir("known_conflict_schema");
    const auto metrics_path = dir / "metrics.json";
    const auto result = runApp(
        {"--scenario", "random_crossing", "--num-robots", "8", "--seed", "1",
         "--scenario-generation-seed", "1", "--algorithm", "arc",
         "--conflict-record-dir", dir.string(), "--metrics-json",
         metrics_path.string()},
        dir / "log.txt");
    if (!expectEq("known-conflict scenario run exits zero", result.exit_code, 0)) {
        fs::remove_all(dir);
        return false;
    }

    const auto files = sortedConflictFiles(dir);
    if (!expectTrue("known-conflict scenario produced at least one record",
                    !files.empty())) {
        fs::remove_all(dir);
        return false;
    }

    const json record = json::parse(readFile(files.front()));

    for (const std::string &top_key :
        {"schema_version", "provenance", "environment", "robots",
         "seed_conflict", "windows"})
        expectHasKey("top-level record", record, top_key);

    expectEq("schema_version is 1", record["schema_version"].get<int>(), 1);

    const json &prov = record["provenance"];
    expectHasKey("provenance", prov, "run_args");
    const json &run_args = prov["run_args"];
    expectEq("run_args.num_robots reflects the actual CLI flag",
            run_args["num_robots"].get<int>(), 8);
    expectEq("run_args.seed reflects the actual CLI flag",
            run_args["seed"].get<int>(), 1);
    expectEq("run_args.scenario_generation_seed reflects the actual CLI flag",
            run_args["scenario_generation_seed"].get<int>(), 1);
    expectTrue("provenance.source.app is mobile_robot_2d_crossing",
              prov["source"]["app"].get<std::string>() ==
                  "mobile_robot_2d_crossing");
    expectTrue("provenance.source.scenario is random_crossing",
              prov["source"]["scenario"].get<std::string>() ==
                  "random_crossing");
    expectTrue("git_commit looks like a real commit or the documented "
              "'unknown' fallback",
              prov["git_commit"].get<std::string>() == "unknown" ||
                  prov["git_commit"].get<std::string>().size() == 40);

    expectTrue("robots is a non-empty array", record["robots"].is_array() &&
                                                  !record["robots"].empty());
    std::vector<int> indices;
    for (const auto &robot : record["robots"]) {
        for (const std::string &key :
            {"global_robot_index", "model", "cspace_bounds",
             "start_config_raw", "goal_config_raw", "start_config_valid",
             "goal_config_valid"})
            expectHasKey("robot entry", robot, key);
        indices.push_back(robot["global_robot_index"].get<int>());
    }
    expectTrue("robots array ordered ascending by global_robot_index",
              std::is_sorted(indices.begin(), indices.end()));

    for (const std::string &key :
        {"seed_robot_i", "seed_robot_j", "conflict_timestep", "kind", "alpha",
         "config_i", "config_j", "robot_selection_trace"})
        expectHasKey("seed_conflict", record["seed_conflict"], key);

    const json &windows = record["windows"];
    for (const std::string &key : {"max_t", "raw_window", "valid_window", "validity_search"})
        expectHasKey("windows", windows, key);

    fs::remove_all(dir);
    return g_failures == failures_before;
}

// ---------------------------------------------------------------------------
// 3. Exactly one record per real conflict (including cascades), and
//    valid_window / validity_search.trace cross-checked against the SAME
//    run's real repair_attempt_events telemetry, for EVERY conflict in a
//    multi-conflict run.
// ---------------------------------------------------------------------------
bool testExactlyOneRecordAndValidWindowMatchesTelemetryAcrossManyConflicts() {
    const int failures_before = g_failures;
    // 12 robots / seed 1 / scenario-generation-seed 2: empirically produces
    // 22 conflicts in this scenario, a dozen of them cascade-merged
    // (robots.size() > 2), giving broad coverage in one run.
    const auto dir = makeScratchDir("many_conflicts");
    const auto metrics_path = dir / "metrics.json";
    const auto result = runApp(
        {"--scenario", "random_crossing", "--num-robots", "12", "--seed", "1",
         "--scenario-generation-seed", "2", "--algorithm", "arc",
         "--conflict-record-dir", dir.string(), "--metrics-json",
         metrics_path.string()},
        dir / "log.txt");
    if (!expectEq("many-conflict scenario run exits zero", result.exit_code, 0)) {
        fs::remove_all(dir);
        return false;
    }

    const json metrics = json::parse(readFile(metrics_path));
    const auto &planner_stats = metrics.at("planner_stats");
    const auto num_conflicts = planner_stats.at("num_conflicts").get<std::uint64_t>();
    const auto files = sortedConflictFiles(dir);

    if (!expectTrue("this scenario is expected to produce multiple conflicts "
                    "(otherwise the cross-check below is not meaningful)",
                    num_conflicts >= 5)) {
        fs::remove_all(dir);
        return false;
    }
    expectEq("one record file per ARC-reported conflict",
            static_cast<std::int64_t>(files.size()),
            static_cast<std::int64_t>(num_conflicts));

    // Group real repair_attempt_events by repair_id, preserving order.
    std::map<std::uint64_t, std::vector<json>> events_by_repair_id;
    for (const auto &event : planner_stats.at("repair_attempt_events"))
        events_by_repair_id[event.at("repair_id").get<std::uint64_t>()].push_back(event);

    std::set<std::uint64_t> seen_sequence_indices;
    bool saw_cascade = false;
    for (const auto &file : files) {
        const json record = json::parse(readFile(file));
        const auto sequence_index =
            record["provenance"]["conflict_sequence_index"].get<std::uint64_t>();
        if (!seen_sequence_indices.insert(sequence_index).second) {
            std::cerr << "conflict_record_cli_regression: duplicate "
                        "conflict_sequence_index "
                      << sequence_index << " (file " << file << ")\n";
            ++g_failures;
        }

        std::vector<int> indices;
        for (const auto &robot : record["robots"])
            indices.push_back(robot["global_robot_index"].get<int>());
        if (!std::is_sorted(indices.begin(), indices.end())) {
            std::cerr << "conflict_record_cli_regression: robots not "
                        "ascending in "
                      << file << "\n";
            ++g_failures;
        }
        if (!record["seed_conflict"]["robot_selection_trace"].empty() ||
            record["robots"].size() > 2)
            saw_cascade = true;

        const auto repair_id = record["provenance"]["repair_id"].get<std::uint64_t>();
        const auto events_it = events_by_repair_id.find(repair_id);
        if (!expectTrue("captured record's repair_id has matching real "
                        "repair_attempt_events (file " +
                            file.string() + ")",
                        events_it != events_by_repair_id.end()))
            continue;

        // Real telemetry prefix: InitialWindow/InitialValid phase attempts,
        // up to and including the first one where endpoints_valid flips
        // true (that is the exact point solveSubproblemOnPaths would first
        // invoke a solver -- everything after is out of this record's
        // scope).
        std::vector<std::pair<int, int>> real_prefix_windows;
        bool real_found_valid = false;
        int real_begin_t = 0, real_end_t = 0;
        for (const auto &event : events_it->second) {
            const std::string phase = event.at("phase").get<std::string>();
            if (phase != "initial_window" && phase != "initial_valid")
                break;
            real_prefix_windows.emplace_back(event.at("window_start_t").get<int>(),
                                             event.at("window_end_t").get<int>());
            if (event.at("endpoints_valid").get<bool>()) {
                real_found_valid = true;
                real_begin_t = event.at("window_start_t").get<int>();
                real_end_t = event.at("window_end_t").get<int>();
                break;
            }
        }

        const auto &vw = record["windows"]["valid_window"];
        const auto &trace = record["windows"]["validity_search"]["trace"];
        std::vector<std::pair<int, int>> captured_trace_windows;
        for (const auto &step : trace)
            captured_trace_windows.emplace_back(step["begin_t"].get<int>(),
                                                step["end_t"].get<int>());

        const std::string ctx = "repair_id " + std::to_string(repair_id) +
            " (file " + file.string() + ")";
        expectEq("real telemetry found-a-valid-window agrees with captured "
                "valid_window.found for " +
                    ctx,
                real_found_valid ? 1 : 0, vw["found"].get<bool>() ? 1 : 0);
        if (vw["found"].get<bool>()) {
            expectEq("captured valid_window.begin_t == real telemetry "
                    "window_start_t for " +
                        ctx,
                    vw["begin_t"].get<int>(), real_begin_t);
            expectEq("captured valid_window.end_t == real telemetry "
                    "window_end_t for " +
                        ctx,
                    vw["end_t"].get<int>(), real_end_t);
        }
        expectTrue("captured validity_search.trace matches the real "
                  "attempt-event window sequence step-by-step for " +
                      ctx,
                  captured_trace_windows == real_prefix_windows);
    }

    expectEq("no missing conflict_sequence_index values (1..num_conflicts)",
            static_cast<std::int64_t>(seen_sequence_indices.size()),
            static_cast<std::int64_t>(num_conflicts));
    expectTrue("this run includes at least one cascade-merged conflict "
              "(robots.size()>2 or non-empty robot_selection_trace)",
              saw_cascade);

    fs::remove_all(dir);
    return g_failures == failures_before;
}

// ---------------------------------------------------------------------------
// 4. Determinism: identical seeds -> byte-identical captured records across
//    two independent runs.
// ---------------------------------------------------------------------------
bool testDeterministicAcrossRuns() {
    const int failures_before = g_failures;
    const auto dir_a = makeScratchDir("determinism_a");
    const auto dir_b = makeScratchDir("determinism_b");
    const std::vector<std::string> args = {
        "--scenario", "random_crossing", "--num-robots", "8", "--seed", "3",
        "--scenario-generation-seed", "4", "--algorithm", "arc",
        "--conflict-record-dir", ""};

    auto run_into = [&](const fs::path &dir) {
        auto local_args = args;
        local_args.back() = dir.string();
        return runApp(local_args, dir / "log.txt");
    };

    const auto result_a = run_into(dir_a);
    const auto result_b = run_into(dir_b);
    bool ok = expectEq("determinism run A exits zero", result_a.exit_code, 0);
    ok &= expectEq("determinism run B exits zero", result_b.exit_code, 0);

    const auto files_a = sortedConflictFiles(dir_a);
    const auto files_b = sortedConflictFiles(dir_b);
    ok &= expectEq("same number of captured records across two runs",
                   static_cast<std::int64_t>(files_a.size()),
                   static_cast<std::int64_t>(files_b.size()));

    for (std::size_t i = 0; i < files_a.size() && i < files_b.size(); ++i) {
        const std::string content_a = readFile(files_a[i]);
        const std::string content_b = readFile(files_b[i]);
        if (content_a != content_b) {
            std::cerr << "conflict_record_cli_regression: byte-level "
                        "mismatch between "
                      << files_a[i] << " and " << files_b[i] << "\n";
            ++g_failures;
        }
    }

    fs::remove_all(dir_a);
    fs::remove_all(dir_b);
    return g_failures == failures_before && ok;
}

} // namespace

int main() {
    struct NamedTest {
        const char *name;
        bool (*fn)();
    };
    const std::vector<NamedTest> tests = {
        {"testRequiresAlgorithmArc", testRequiresAlgorithmArc},
        {"testRejectsOrParallelWorkerProcesses", testRejectsOrParallelWorkerProcesses},
        {"testValidUsageSucceeds", testValidUsageSucceeds},
        {"testKnownConflictScenarioSchemaAndOrdering",
         testKnownConflictScenarioSchemaAndOrdering},
        {"testExactlyOneRecordAndValidWindowMatchesTelemetryAcrossManyConflicts",
         testExactlyOneRecordAndValidWindowMatchesTelemetryAcrossManyConflicts},
        {"testDeterministicAcrossRuns", testDeterministicAcrossRuns},
    };

    bool all_ok = true;
    for (const auto &test : tests) {
        const int failures_before = g_failures;
        const bool ok = test.fn() && g_failures == failures_before;
        if (!ok) {
            std::cerr << "conflict_record_cli_regression: FAILED " << test.name
                      << "\n";
            all_ok = false;
        }
    }

    if (!all_ok)
        return 1;

    std::cout << "conflict_record_cli_regression: OK\n";
    return 0;
}
