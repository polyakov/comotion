// Coverage for ARC's SubproblemRecord capture
// (comotion-vadim/requirements/data-collection/subproblem-record-design.md,
// schema_version 1), added on feature/conflict-record-collection. Uses the
// same white-box "probe subclass" pattern as arc_exact_only_regression.cpp
// (ArcHistoryProbe) to reach ARC's protected capture/search internals
// directly, without depending on OMPL solver randomness wherever a
// deterministic hand-built scenario suffices.

#include "comotion/planning/ARC.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/PlannerStatus.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {

using json = nlohmann::json;

class SubproblemRecordProbe : public comotion::ARC {
public:
    using ValidWindowSearchResult = ARC::ValidWindowSearchResult;
    using HorizonAndRawWindow = ARC::SubproblemHorizonAndRawWindow;

    void captureRecord(const comotion::SubproblemConflict &conflict,
                       const std::vector<comotion::Path> &working_paths) {
        captureSubproblemRecord(conflict, working_paths);
    }

    ValidWindowSearchResult
    findValidWindowProbe(const std::vector<int> &involved_robots,
                         const std::vector<comotion::Path> &working_paths,
                         int raw_begin_t, int raw_end_t,
                         std::size_t max_t) const {
        return findValidWindow(involved_robots, working_paths, raw_begin_t,
                               raw_end_t, max_t);
    }

    HorizonAndRawWindow
    horizonAndRawWindow(const comotion::SubproblemConflict &conflict,
                        const std::vector<comotion::Path> &working_paths) const {
        return subproblemHorizonAndRawWindow(conflict, working_paths);
    }

    comotion::SubproblemConflict
    expandConflict(const comotion::Conflict &conflict) const {
        return expandConflictForSubproblem(conflict);
    }

    void recordHistory(const std::vector<int> &robots, int window_start_t,
                       int window_end_t) {
        recordAppliedRepairHistory(robots, window_start_t, window_end_t);
    }

    bool solveProbe(const comotion::SubproblemConflict &conflict,
                    double global_time_limit,
                    std::vector<comotion::Path> &working_paths,
                    int *window_start_t_out, int *window_end_t_out) {
        true_arrival_timesteps_.clear();
        true_arrival_timesteps_.reserve(working_paths.size());
        for (const auto &path : working_paths) {
            true_arrival_timesteps_.push_back(
                static_cast<std::uint64_t>(path.arrival_timestep()));
        }
        return solveSubproblemOnPaths(conflict, Clock::now(), global_time_limit,
                                      working_paths, window_start_t_out,
                                      window_end_t_out);
    }

    nlohmann::json repairAttemptEvents() const {
        return repairAttemptEventsJson();
    }

    std::uint64_t numConflictsSeen() const { return num_conflicts_; }
    void bumpNumConflicts() { ++num_conflicts_; }
};

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 1.0) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-20.0, -20.0, 0.0},
        std::vector<double>{20.0, 20.0, 1.5});
}

comotion::Path
makeDensePath(const std::vector<std::vector<double>> &configurations) {
    comotion::Path path;
    for (const auto &configuration : configurations)
        path.push_back(configuration);
    path.markDenseTimestepsImplicit();
    return path;
}

int g_failures = 0;

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "subproblem_record_capture_regression: " << label
                  << " expected true\n";
        ++g_failures;
    }
    return value;
}

bool expectEq(const std::string &label, std::int64_t actual,
              std::int64_t expected) {
    if (actual != expected) {
        std::cerr << "subproblem_record_capture_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        ++g_failures;
        return false;
    }
    return true;
}

bool expectHasKey(const std::string &label, const json &object,
                  const std::string &key) {
    if (!object.is_object() || !object.contains(key)) {
        std::cerr << "subproblem_record_capture_regression: " << label
                  << " missing key '" << key << "'\n";
        ++g_failures;
        return false;
    }
    return true;
}

bool expectNoKey(const std::string &label, const json &object,
                 const std::string &key) {
    if (object.is_object() && object.contains(key)) {
        std::cerr << "subproblem_record_capture_regression: " << label
                  << " unexpectedly has key '" << key << "'\n";
        ++g_failures;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Shared fixture: 3 robots, one obstacle sphere and one obstacle cylinder,
// a seed conflict between robots 0/1, and a cascade-selected robot 2. Robot
// paths are constructed so the raw window is already endpoint-valid (no
// expansion needed) -- used for schema/shape checks where the specific
// window-search dynamics do not matter.
// ---------------------------------------------------------------------------

struct Fixture {
    std::shared_ptr<comotion::MultiRobotProblem> problem;
    std::vector<comotion::Path> working_paths;
    comotion::SubproblemConflict conflict;
};

Fixture makeValidWindowFixture() {
    Fixture fx;
    fx.problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    fx.problem->setResolution(64);
    fx.problem->setVmax(2.0);
    fx.problem->setObstacles({comotion::ObstacleSphere{{15.0, 15.0, 0.75}, 1.0}});
    fx.problem->setCylinderObstacles(
        {comotion::ObstacleCylinder{{-15.0, -15.0, 0.75}, {0.0, 0.0, 1.0}, 1.0, 2.0}});

    // Three well-separated robots; robot 0 and 1 collide head-on at t in
    // [4,5], robot 2 is far away throughout (added only via an explicit
    // expansion_trace entry below, to exercise robot_selection_trace
    // rendering without depending on real cascade-merge timing).
    fx.problem->addRobot(makeSphereRobot(0.5), {-5.0, 0.0, 0.75}, {5.0, 0.0, 0.75});
    fx.problem->addRobot(makeSphereRobot(0.5), {5.0, 0.0, 0.75}, {-5.0, 0.0, 0.75});
    fx.problem->addRobot(makeSphereRobot(0.5), {0.0, 10.0, 0.75}, {0.0, -10.0, 0.75});

    std::vector<std::vector<double>> path0, path1, path2;
    for (int t = 0; t <= 10; ++t) {
        double x = -5.0 + static_cast<double>(t);
        path0.push_back({x, 0.0, 0.75});
        path1.push_back({-x, 0.0, 0.75});
        path2.push_back({0.0, 10.0 - 2.0 * t, 0.75});
    }
    fx.working_paths = {makeDensePath(path0), makeDensePath(path1),
                        makeDensePath(path2)};

    fx.conflict.seed_robot_i = 0;
    fx.conflict.seed_robot_j = 1;
    fx.conflict.conflict_timestep = 5;
    fx.conflict.window_begin_t = 3;
    fx.conflict.window_end_t = 7;
    fx.conflict.robots = {0, 1, 2};
    fx.conflict.kind = comotion::ConflictKind::Vertex;
    fx.conflict.alpha = 0.25;
    fx.conflict.config_i = {0.0, 0.0, 0.75};
    fx.conflict.config_j = {0.0, 0.0, 0.75};
    fx.conflict.expansion_trace = {
        {0, 2, 0, 1, 3, 7, {11, 12}},
    };
    return fx;
}

json renderOne(const SubproblemRecordProbe &probe) {
    const json source = {{"app", "mobile_robot_2d_crossing"},
                         {"scenario", "random_crossing"}};
    const json run_args = {{"num_robots", 3}, {"seed", 1}};
    const json records =
        probe.subproblemRecordsJson(1, source, run_args, "deadbeef");
    if (!expectEq("record count for renderOne", records.size(), 1))
        return json();
    return records.front();
}

// ---------------------------------------------------------------------------
// 1. Schema field-by-field shape check.
// ---------------------------------------------------------------------------
bool testSchemaShape() {
    auto fx = makeValidWindowFixture();
    SubproblemRecordProbe probe;
    probe.setProblem(fx.problem);
    probe.setCaptureSubproblemRecords(true);
    probe.captureRecord(fx.conflict, fx.working_paths);

    if (!expectEq("captured record count", probe.capturedSubproblemRecords().size(), 1))
        return false;

    const json record = renderOne(probe);
    if (record.is_null())
        return false;

    bool ok = true;
    ok &= expectHasKey("top-level", record, "schema_version");
    ok &= expectTrue("schema_version is int", record["schema_version"].is_number_integer());

    ok &= expectHasKey("top-level", record, "provenance");
    const json &prov = record["provenance"];
    ok &= expectHasKey("provenance", prov, "source");
    ok &= expectHasKey("provenance", prov, "run_args");
    ok &= expectHasKey("provenance", prov, "git_commit");
    ok &= expectTrue("git_commit is string", prov["git_commit"].is_string());
    ok &= expectHasKey("provenance", prov, "conflict_sequence_index");
    ok &= expectTrue("conflict_sequence_index numeric",
                     prov["conflict_sequence_index"].is_number());
    ok &= expectHasKey("provenance", prov, "repair_id");
    ok &= expectTrue("repair_id numeric", prov["repair_id"].is_number());
    ok &= expectHasKey("provenance.source", prov["source"], "app");
    ok &= expectHasKey("provenance.source", prov["source"], "scenario");

    ok &= expectHasKey("top-level", record, "environment");
    const json &env = record["environment"];
    for (const std::string &key :
        {"obstacles_spheres", "obstacles_cylinders", "vmax", "resolution",
         "collision_backend"})
        ok &= expectHasKey("environment", env, key);
    ok &= expectTrue("obstacles_spheres is array", env["obstacles_spheres"].is_array());
    ok &= expectEq("one sphere obstacle", env["obstacles_spheres"].size(), 1);
    ok &= expectHasKey("obstacle sphere", env["obstacles_spheres"][0], "center");
    ok &= expectHasKey("obstacle sphere", env["obstacles_spheres"][0], "radius");
    ok &= expectEq("sphere center dims", env["obstacles_spheres"][0]["center"].size(), 3);
    ok &= expectTrue("obstacles_cylinders is array", env["obstacles_cylinders"].is_array());
    ok &= expectEq("one cylinder obstacle", env["obstacles_cylinders"].size(), 1);
    for (const std::string &key : {"center", "axis", "radius", "half_height"})
        ok &= expectHasKey("obstacle cylinder", env["obstacles_cylinders"][0], key);
    ok &= expectTrue("collision_backend is string", env["collision_backend"].is_string());
    ok &= expectTrue("collision_backend value is Spheres",
                     env["collision_backend"].get<std::string>() == "Spheres");

    ok &= expectHasKey("top-level", record, "robots");
    ok &= expectTrue("robots is array", record["robots"].is_array());
    ok &= expectEq("robots count matches conflict.robots", record["robots"].size(), 3);
    for (const auto &robot : record["robots"]) {
        ok &= expectHasKey("robot entry", robot, "global_robot_index");
        ok &= expectHasKey("robot entry", robot, "model");
        ok &= expectHasKey("robot.model", robot["model"], "radius");
        ok &= expectHasKey("robot.model", robot["model"], "workspace_min");
        ok &= expectHasKey("robot.model", robot["model"], "workspace_max");
        ok &= expectEq("workspace_min dims", robot["model"]["workspace_min"].size(), 3);
        ok &= expectEq("workspace_max dims", robot["model"]["workspace_max"].size(), 3);
        ok &= expectHasKey("robot entry", robot, "cspace_bounds");
        ok &= expectHasKey("robot entry", robot, "start_config_raw");
        ok &= expectHasKey("robot entry", robot, "goal_config_raw");
        ok &= expectHasKey("robot entry", robot, "start_config_valid");
        ok &= expectHasKey("robot entry", robot, "goal_config_valid");
    }

    ok &= expectHasKey("top-level", record, "seed_conflict");
    const json &seed_conflict = record["seed_conflict"];
    for (const std::string &key :
        {"seed_robot_i", "seed_robot_j", "conflict_timestep", "kind", "alpha",
         "config_i", "config_j", "robot_selection_trace"})
        ok &= expectHasKey("seed_conflict", seed_conflict, key);
    ok &= expectTrue("kind is 'Vertex'",
                     seed_conflict["kind"].get<std::string>() == "Vertex");
    ok &= expectEq("robot_selection_trace has one step",
                   seed_conflict["robot_selection_trace"].size(), 1);
    if (seed_conflict["robot_selection_trace"].size() == 1) {
        const json &step = seed_conflict["robot_selection_trace"][0];
        for (const std::string &key :
            {"from_robot", "added_robot", "window_robot_a", "window_robot_b",
             "window_start_t", "window_end_t", "history_event_ids"})
            ok &= expectHasKey("robot_selection_trace step", step, key);
        ok &= expectEq("expansion trace added_robot", step["added_robot"].get<int>(), 2);
        ok &= expectEq("expansion trace history event count",
                       step["history_event_ids"].size(), 2);
    }

    ok &= expectHasKey("top-level", record, "windows");
    const json &windows = record["windows"];
    ok &= expectHasKey("windows", windows, "max_t");
    ok &= expectHasKey("windows", windows, "raw_window");
    ok &= expectHasKey("windows", windows, "valid_window");
    ok &= expectHasKey("windows", windows, "validity_search");
    for (const std::string &key :
        {"begin_t", "end_t", "start_valid", "goal_valid", "endpoints_valid"}) {
        ok &= expectHasKey("raw_window", windows["raw_window"], key);
        ok &= expectHasKey("valid_window", windows["valid_window"], key);
    }
    ok &= expectHasKey("valid_window", windows["valid_window"], "found");
    const json &vs = windows["validity_search"];
    ok &= expectHasKey("validity_search", vs, "expansion_count");
    ok &= expectHasKey("validity_search", vs, "trace");
    ok &= expectHasKey("validity_search", vs, "cost");
    ok &= expectTrue("validity_search.cost is null (not yet instrumented)",
                     vs["cost"].is_null());
    ok &= expectTrue("validity_search.trace is array", vs["trace"].is_array());
    for (const auto &step : vs["trace"]) {
        for (const std::string &key : {"begin_t", "end_t", "start_valid", "goal_valid"})
            ok &= expectHasKey("validity_search.trace step", step, key);
        // Per the schema, trace entries carry only begin_t/end_t/
        // start_valid/goal_valid -- NOT endpoints_valid (that field is only
        // on raw_window/valid_window themselves).
        ok &= expectNoKey("validity_search.trace step", step, "endpoints_valid");
    }

    // Raw window was already endpoint-valid by construction -> zero
    // expansions, valid_window == raw_window.
    ok &= expectTrue("raw window already valid",
                     windows["raw_window"]["endpoints_valid"].get<bool>());
    ok &= expectTrue("valid window found", windows["valid_window"]["found"].get<bool>());
    ok &= expectEq("zero expansions needed",
                   vs["expansion_count"].get<std::uint64_t>(), 0);
    ok &= expectEq("valid_window.begin_t == raw_window.begin_t",
                   windows["valid_window"]["begin_t"].get<int>(),
                   windows["raw_window"]["begin_t"].get<int>());
    ok &= expectEq("valid_window.end_t == raw_window.end_t",
                   windows["valid_window"]["end_t"].get<int>(),
                   windows["raw_window"]["end_t"].get<int>());

    return ok;
}

// ---------------------------------------------------------------------------
// 2. robots array ordering: ascending by global_robot_index, matching
//    subproblemRobotsForConflict's sorted-set draining (real cascade path).
// ---------------------------------------------------------------------------
bool testRobotsOrderedAscendingRealCascade() {
    SubproblemRecordProbe probe;
    // Seed pair (5, 2) with a cascade pulling in robot 8 out of numeric
    // insertion order -- final set must render ascending {2, 5, 8}.
    probe.recordHistory({2, 8}, 10, 20);
    const auto conflict = probe.expandConflict(comotion::Conflict{5, 2, 15});

    if (!expectTrue("cascade actually pulled in robot 8",
                    std::find(conflict.robots.begin(), conflict.robots.end(), 8) !=
                        conflict.robots.end()))
        return false;

    return expectTrue("conflict.robots ascending",
                      std::is_sorted(conflict.robots.begin(), conflict.robots.end()));
}

bool testRobotsOrderPreservedVerbatimByCapture() {
    // captureSubproblemRecord/subproblemRecordsJson must not silently
    // reorder whatever robot list they are handed -- correctness of the
    // "ascending" invariant is entirely the caller's (expandConflictForSubproblem's)
    // responsibility, not something capture re-derives. Feed it an
    // out-of-order list directly and confirm the rendered array preserves it
    // (a bug here would mask a caller-side ordering violation).
    auto fx = makeValidWindowFixture();
    fx.conflict.robots = {2, 0, 1}; // deliberately NOT ascending
    fx.conflict.expansion_trace.clear();

    SubproblemRecordProbe probe;
    probe.setProblem(fx.problem);
    probe.setCaptureSubproblemRecords(true);
    probe.captureRecord(fx.conflict, fx.working_paths);

    const json record = renderOne(probe);
    if (record.is_null())
        return false;
    std::vector<int> indices;
    for (const auto &robot : record["robots"])
        indices.push_back(robot["global_robot_index"].get<int>());
    return expectTrue("capture preserves given (here: non-ascending) robot order verbatim",
                      indices == std::vector<int>({2, 0, 1}));
}

// ---------------------------------------------------------------------------
// 3. cspace_bounds null exactly when temporal_full_window (or bounds
//    disabled), non-null and correctly shaped otherwise.
// ---------------------------------------------------------------------------
bool testCspaceBoundsNullIffFullWindowOrDisabled() {
    auto fx = makeValidWindowFixture();

    // (a) Narrow window (not full horizon), bounds enabled (default) ->
    // non-null, shaped [lo,hi] with 3 dims, lo <= hi.
    {
        SubproblemRecordProbe probe;
        probe.setProblem(fx.problem);
        probe.setCaptureSubproblemRecords(true);
        probe.captureRecord(fx.conflict, fx.working_paths); // window [3,7] of [0,10]
        const json record = renderOne(probe);
        if (record.is_null())
            return false;
        for (const auto &robot : record["robots"]) {
            if (!expectTrue("narrow window: cspace_bounds present",
                            !robot["cspace_bounds"].is_null()))
                return false;
            const json &bounds = robot["cspace_bounds"];
            if (!expectHasKey("cspace_bounds", bounds, "lo") ||
                !expectHasKey("cspace_bounds", bounds, "hi"))
                return false;
            if (!expectEq("cspace_bounds.lo dims", bounds["lo"].size(), 3) ||
                !expectEq("cspace_bounds.hi dims", bounds["hi"].size(), 3))
                return false;
            for (std::size_t d = 0; d < 3; ++d) {
                if (!expectTrue("cspace_bounds lo <= hi per dim",
                                bounds["lo"][d].get<double>() <=
                                    bounds["hi"][d].get<double>()))
                    return false;
            }
        }
    }

    // (b) Full-horizon window -> null regardless of use_cspace_bounds_.
    {
        Fixture full = fx;
        full.conflict.window_begin_t = 0;
        full.conflict.window_end_t = 10; // == max_t - 1 for an 11-point path
        SubproblemRecordProbe probe;
        probe.setProblem(full.problem);
        probe.setCaptureSubproblemRecords(true);
        probe.captureRecord(full.conflict, full.working_paths);
        const json record = renderOne(probe);
        if (record.is_null())
            return false;
        if (!expectTrue("full window: raw window is reported as full",
                        record["windows"]["raw_window"]["end_t"].get<int>() >= 10))
            return false;
        for (const auto &robot : record["robots"]) {
            if (!expectTrue("full window: cspace_bounds is null",
                            robot["cspace_bounds"].is_null()))
                return false;
        }
    }

    // (c) Narrow window, but use_cspace_bounds_ explicitly off -> null.
    {
        SubproblemRecordProbe probe;
        probe.setProblem(fx.problem);
        probe.setUseCspaceBounds(false);
        probe.setCaptureSubproblemRecords(true);
        probe.captureRecord(fx.conflict, fx.working_paths);
        const json record = renderOne(probe);
        if (record.is_null())
            return false;
        for (const auto &robot : record["robots"]) {
            if (!expectTrue("bounds disabled: cspace_bounds is null",
                            robot["cspace_bounds"].is_null()))
                return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// 4. windows.valid_window.found == false (global window failure): two
//    robots whose paths keep them colliding at every single timestep across
//    the whole horizon, so no window -- however wide -- is ever endpoint
//    valid. Verify the uniform-but-meaningless shape (all keys present,
//    defaulted, no crash) rather than a crash or missing keys.
// ---------------------------------------------------------------------------
bool testGlobalWindowFailureUniformShape() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(2.0);
    problem->addRobot(makeSphereRobot(0.5), {0.0, 0.0, 0.75}, {0.0, 0.0, 0.75});
    problem->addRobot(makeSphereRobot(0.5), {0.0, 0.0, 0.75}, {0.0, 0.0, 0.75});

    const std::vector<double> colliding{0.0, 0.0, 0.75};
    std::vector<comotion::Path> working_paths = {
        makeDensePath(std::vector<std::vector<double>>(9, colliding)),
        makeDensePath(std::vector<std::vector<double>>(9, colliding)),
    };

    comotion::SubproblemConflict conflict;
    conflict.seed_robot_i = 0;
    conflict.seed_robot_j = 1;
    conflict.conflict_timestep = 4;
    conflict.window_begin_t = 3;
    conflict.window_end_t = 5;
    conflict.robots = {0, 1};

    SubproblemRecordProbe probe;
    probe.setProblem(problem);
    probe.setCaptureSubproblemRecords(true);
    probe.captureRecord(conflict, working_paths);

    if (!expectEq("one record captured (negative example, still emitted)",
                  probe.capturedSubproblemRecords().size(), 1))
        return false;

    const json record = renderOne(probe);
    if (record.is_null())
        return false;

    const json &vw = record["windows"]["valid_window"];
    bool ok = true;
    ok &= expectTrue("valid_window.found is false",
                     !vw["found"].get<bool>());
    // Uniform shape: keys still present with well-formed (if meaningless)
    // values, not omitted / not a crash.
    for (const std::string &key :
        {"begin_t", "end_t", "start_valid", "goal_valid", "endpoints_valid"})
        ok &= expectHasKey("valid_window (found=false)", vw, key);
    ok &= expectTrue("start_valid default false", !vw["start_valid"].get<bool>());
    ok &= expectTrue("goal_valid default false", !vw["goal_valid"].get<bool>());
    ok &= expectTrue("endpoints_valid default false", !vw["endpoints_valid"].get<bool>());

    for (const auto &robot : record["robots"]) {
        ok &= expectTrue("no valid window -> start_config_valid null",
                         robot["start_config_valid"].is_null());
        ok &= expectTrue("no valid window -> goal_config_valid null",
                         robot["goal_config_valid"].is_null());
        ok &= expectTrue("no valid window -> cspace_bounds null",
                         robot["cspace_bounds"].is_null());
    }

    // The search must actually have walked out to the global window before
    // giving up (a genuine failure, not an early bail-out).
    const json &trace = record["windows"]["validity_search"]["trace"];
    ok &= expectTrue("validity search trace non-empty", !trace.empty());
    ok &= expectTrue("validity search reached the full horizon",
                     trace.back()["begin_t"].get<int>() == 0 &&
                         static_cast<std::size_t>(trace.back()["end_t"].get<int>()) >=
                             record["windows"]["max_t"].get<std::size_t>() - 1);

    return ok;
}

// ---------------------------------------------------------------------------
// 5. findValidWindow() vs. the REAL solveSubproblemOnPaths loop: same
//    conflict/paths fed to both independently; both must land on the same
//    terminal valid window, and the trace must match the corresponding
//    prefix of real RepairAttemptEvent telemetry.
// ---------------------------------------------------------------------------
bool testFindValidWindowMatchesRealResolutionTelemetry() {
    const int failures_before = g_failures;
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(100.0);
    problem->addRobot(makeSphereRobot(0.5), {0.0, 0.0, 0.75}, {0.0, 0.0, 0.75});
    problem->addRobot(makeSphereRobot(0.5), {0.0, 0.0, 0.75}, {0.0, 0.0, 0.75});

    const std::vector<double> colliding{0.0, 0.0, 0.75};
    const std::vector<double> separated{5.0, 0.0, 0.75};
    // Robot 0 sits still (colliding position) the whole time. Robot 1 is
    // separated at t=0,1,2, colliding at t=3..7 (a band centered on the
    // conflict timestep), separated again at t=8,9,10. The raw window
    // [4,6] sits entirely inside the colliding band (both endpoints
    // invalid), so a real search must grow symmetrically outward from
    // center t=5 until it clears the band on both sides -- exercising a
    // genuine multi-step InitialValid search that succeeds strictly
    // between the horizon's edges (not by degenerating to the global
    // window).
    std::vector<comotion::Path> working_paths = {
        makeDensePath(std::vector<std::vector<double>>(11, colliding)),
        makeDensePath({separated, separated, separated, colliding, colliding,
                      colliding, colliding, colliding, separated, separated,
                      separated}),
    };

    comotion::SubproblemConflict conflict;
    conflict.seed_robot_i = 0;
    conflict.seed_robot_j = 1;
    conflict.conflict_timestep = 5;
    conflict.window_begin_t = 4;
    conflict.window_end_t = 6; // both endpoints inside the colliding band
    conflict.robots = {0, 1};

    SubproblemRecordProbe search_probe;
    search_probe.setProblem(problem);
    search_probe.setUseCspaceBounds(false);
    search_probe.setInitialValidWindowExpansionStep(1);
    const auto horizon = search_probe.horizonAndRawWindow(conflict, working_paths);
    const auto search_result = search_probe.findValidWindowProbe(
        conflict.robots, working_paths, horizon.raw_begin_t, horizon.raw_end_t,
        horizon.max_t);

    if (!expectTrue("independent search found a valid window", search_result.found))
        return false;
    if (!expectTrue("independent search actually needed expansion (non-trivial case)",
                    search_result.trace.size() > 1))
        return false;
    if (!expectTrue("valid window found strictly inside the horizon, not by "
                    "degenerating to the global window",
                    !(search_result.begin_t == 0 &&
                      static_cast<std::size_t>(search_result.end_t) >=
                          horizon.max_t - 1)))
        return false;

    // Now run the REAL resolution loop on an identical fresh problem/paths
    // (solveSubproblemOnPaths mutates telemetry/state, so use a separate
    // probe instance rather than reusing search_probe).
    SubproblemRecordProbe live_probe;
    live_probe.setProblem(problem);
    live_probe.setUseCspaceBounds(false);
    live_probe.setInitialValidWindowExpansionStep(1);
    live_probe.setLocalSolverMode(comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    live_probe.setLocalCompositeRrtMaxSamples(50);
    std::vector<comotion::Path> live_paths = working_paths;
    int live_start_t = 0, live_end_t = 0;
    live_probe.solveProbe(conflict, 3.0, live_paths, &live_start_t, &live_end_t);

    const json events = live_probe.repairAttemptEvents();
    if (!expectTrue("real loop produced at least one attempt event", !events.empty()))
        return false;

    // Find the first attempt (in InitialWindow/InitialValid phase) where
    // endpoints_valid flips true -- this is exactly the window
    // findValidWindow()'s independent replay should converge to.
    bool found_valid_attempt = false;
    int real_begin_t = 0, real_end_t = 0;
    std::vector<std::pair<int, int>> real_prefix_windows;
    for (const auto &event : events) {
        const std::string phase = event["phase"].get<std::string>();
        if (phase != "initial_window" && phase != "initial_valid")
            break; // Main-phase attempts (post solver-invocation) are out of scope.
        real_prefix_windows.emplace_back(event["window_start_t"].get<int>(),
                                         event["window_end_t"].get<int>());
        if (event["endpoints_valid"].get<bool>()) {
            found_valid_attempt = true;
            real_begin_t = event["window_start_t"].get<int>();
            real_end_t = event["window_end_t"].get<int>();
            break;
        }
    }

    if (!expectTrue("real resolution loop also found an endpoint-valid window "
                    "before invoking a solver",
                    found_valid_attempt))
        return false;

    if (!expectEq("findValidWindow.begin_t == real telemetry window_start_t",
                  search_result.begin_t, real_begin_t))
        return false;
    if (!expectEq("findValidWindow.end_t == real telemetry window_end_t",
                  search_result.end_t, real_end_t))
        return false;

    // The independent replay's full intermediate trace must match the real
    // loop's prefix of attempted windows one-for-one (not just the final
    // answer) -- this is the actual claim being tested: findValidWindow
    // reproduces the identical window sequence, not merely an equivalent
    // endpoint.
    if (!expectEq("trace length matches real attempt-event prefix length",
                  search_result.trace.size(), real_prefix_windows.size()))
        return false;
    bool sequence_matches = true;
    for (std::size_t i = 0; i < search_result.trace.size() && i < real_prefix_windows.size();
        ++i) {
        sequence_matches &= (search_result.trace[i].begin_t == real_prefix_windows[i].first &&
                            search_result.trace[i].end_t == real_prefix_windows[i].second);
    }
    expectTrue("full intermediate window sequence matches real telemetry, step-by-step",
              sequence_matches);

    return g_failures == failures_before;
}

// ---------------------------------------------------------------------------
// 6. Exactly one record per real conflict across a full solve() run with
//    multiple conflicts, including at least one cascade-merged conflict
//    with a non-empty robot_selection_trace.
// ---------------------------------------------------------------------------
std::shared_ptr<comotion::MultiRobotProblem> makeCircleCrossingProblem() {
    // 8 robots on a circle, each crossing to the antipodal point through the
    // center -- guaranteed to produce several sequential conflicts as ARC
    // resolves them one at a time, and likely to cascade since repaired
    // windows cluster near the shared center crossing.
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(128);
    problem->setVmax(2.0);
    problem->setObstacles({comotion::ObstacleSphere{{0.0, 0.0, 0.75}, 1.5}});

    const std::vector<std::vector<double>> starts = {
        {10.0, 0.0, 0.75},   {7.07, 7.07, 0.75},   {0.0, 10.0, 0.75},
        {-7.07, 7.07, 0.75}, {-10.0, 0.0, 0.75},   {-7.07, -7.07, 0.75},
        {0.0, -10.0, 0.75},  {7.07, -7.07, 0.75},
    };
    const std::vector<std::vector<double>> goals = {
        {-10.0, 0.0, 0.75}, {-7.07, -7.07, 0.75}, {0.0, -10.0, 0.75},
        {7.07, -7.07, 0.75}, {10.0, 0.0, 0.75},   {7.07, 7.07, 0.75},
        {0.0, 10.0, 0.75},  {-7.07, 7.07, 0.75},
    };
    for (std::size_t i = 0; i < starts.size(); ++i)
        problem->addRobot(makeSphereRobot(1.0), starts[i], goals[i]);
    return problem;
}

bool testExactlyOneRecordPerConflictOverRealRun() {
    const int failures_before = g_failures;
    comotion::seedOmplGlobalFromUserPlanningSeed(6);
    auto problem = makeCircleCrossingProblem();
    comotion::ARC planner;
    planner.setPlanningSeed(6);
    planner.setProblem(problem);
    planner.setInitialWindow(30);
    planner.setExpansionStep(30);
    planner.setCaptureSubproblemRecords(true);

    const auto status = planner.solve(20.0);
    if (!expectTrue("circle-crossing scenario reaches a terminal status",
                    status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
                        status == ompl::base::PlannerStatus::TIMEOUT))
        return false;

    const auto &stats = planner.plannerStatsJson();
    const auto num_conflicts = stats["num_conflicts"].get<std::uint64_t>();
    const auto &captured = planner.capturedSubproblemRecords();

    if (!expectTrue("at least one conflict actually occurred in this scenario",
                    num_conflicts > 0))
        return false;
    if (!expectEq("captured record count == ARC's own num_conflicts counter",
                  static_cast<std::int64_t>(captured.size()),
                  static_cast<std::int64_t>(num_conflicts)))
        return false;

    std::set<std::uint64_t> sequence_indices;
    bool saw_cascade = false;
    for (const auto &record : captured) {
        if (!sequence_indices.insert(record.conflict_sequence_index).second) {
            std::cerr << "subproblem_record_capture_regression: duplicate "
                        "conflict_sequence_index "
                      << record.conflict_sequence_index << "\n";
            ++g_failures;
        }
        if (!record.robot_selection_trace.empty() || record.robots.size() > 2)
            saw_cascade = true;
    }
    if (!expectEq("no missing sequence indices (1..num_conflicts, no gaps)",
                  static_cast<std::int64_t>(sequence_indices.size()),
                  static_cast<std::int64_t>(num_conflicts)))
        return false;

    // Not a hard requirement of correctness (a run could legitimately have
    // zero cascades), but this scenario is specifically chosen to produce
    // one; flag it clearly if that ever stops being true so the scenario
    // can be revisited rather than silently losing cascade coverage.
    if (!saw_cascade) {
        std::cerr << "subproblem_record_capture_regression: WARNING -- "
                    "circle-crossing scenario produced no cascade-merged "
                    "conflict (robots.size()>2 or non-empty "
                    "robot_selection_trace) this run; cascade-path capture "
                    "is untested by this particular run\n";
    }

    return g_failures == failures_before;
}

} // namespace

int main() {
    struct NamedTest {
        const char *name;
        bool (*fn)();
    };
    const std::vector<NamedTest> tests = {
        {"testSchemaShape", testSchemaShape},
        {"testRobotsOrderedAscendingRealCascade", testRobotsOrderedAscendingRealCascade},
        {"testRobotsOrderPreservedVerbatimByCapture",
         testRobotsOrderPreservedVerbatimByCapture},
        {"testCspaceBoundsNullIffFullWindowOrDisabled",
         testCspaceBoundsNullIffFullWindowOrDisabled},
        {"testGlobalWindowFailureUniformShape", testGlobalWindowFailureUniformShape},
        {"testFindValidWindowMatchesRealResolutionTelemetry",
         testFindValidWindowMatchesRealResolutionTelemetry},
        {"testExactlyOneRecordPerConflictOverRealRun",
         testExactlyOneRecordPerConflictOverRealRun},
    };

    bool all_ok = true;
    for (const auto &test : tests) {
        const int failures_before = g_failures;
        const bool ok = test.fn() && g_failures == failures_before;
        if (!ok) {
            std::cerr << "subproblem_record_capture_regression: FAILED "
                      << test.name << "\n";
            all_ok = false;
        }
    }

    if (!all_ok)
        return 1;

    std::cout << "subproblem_record_capture_regression: OK\n";
    return 0;
}
