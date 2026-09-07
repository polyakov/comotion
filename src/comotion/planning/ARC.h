#pragma once

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/PathSimplification.h"
#include "comotion/planning/PrioritizedSTRRT.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace comotion {

/// Stable planner API.
///
/// Adaptive Robot Coordination (ARC): computes initial per-robot paths, then
/// iteratively finds and resolves conflicts via local subproblems.
class ARC : public MultiRobotPlanner {
public:
    enum class ExpansionPolicy {
        Linear,
        Logarithmic,
        Exponential,
        CustomMultiplied,
    };

    enum class LocalSolverMode {
        Both,
        PrioritizedStrrtOnly,
        CompositeRrtOnly,
    };
    using CancellationCallback = std::function<bool()>;

    struct VisualizationRepair {
        std::size_t conflict_index = 0;
        std::vector<int> robots;
        int window_start_t = 0;
        int window_end_t = 0;
        std::vector<Path> local_paths;
    };

    struct VisualizationIteration {
        std::vector<Path> paths;
        bool conflict_scan_completed = false;
        std::vector<SubproblemConflict> conflicts;
        std::vector<VisualizationRepair> repairs;
    };

    // --- SubproblemRecord capture (data-collection/subproblem-record-design.md) ---
    // Mirrors the schema's `windows.raw_window` / `windows.valid_window`: a
    // temporal window plus whether its composite start/goal configs are
    // collision-free.
    struct SubproblemRecordWindowState {
        int begin_t = 0;
        int end_t = 0;
        bool start_valid = false;
        bool goal_valid = false;
        bool endpoints_valid = false;
    };

    struct SubproblemRecordCspaceBounds {
        std::vector<double> lo;
        std::vector<double> hi;
    };

    // Mirrors the schema's `RobotEntry`.
    struct SubproblemRecordRobotEntry {
        int global_robot_index = -1;
        double model_radius = 0.0;
        std::vector<double> model_workspace_min;
        std::vector<double> model_workspace_max;
        // null when temporal_full_window or !use_cspace_bounds_, matching
        // ARC.cpp's own skip conditions for cspace-bound computation.
        std::optional<SubproblemRecordCspaceBounds> cspace_bounds;
        std::vector<double> start_config_raw;
        std::vector<double> goal_config_raw;
        // null when the valid-window search never found a valid window.
        std::optional<std::vector<double>> start_config_valid;
        std::optional<std::vector<double>> goal_config_valid;
    };

    // One captured raw, unrepaired conflict -- everything needed to render
    // the schema's SubproblemRecord except the run-level `provenance` block
    // (schema_version/source/run_args/git_commit), which the caller (the
    // application, not ARC) supplies via subproblemRecordsJson() below.
    struct CapturedSubproblemRecord {
        std::uint64_t conflict_sequence_index = 0;
        std::uint64_t repair_id = 0;
        int seed_robot_i = -1;
        int seed_robot_j = -1;
        int conflict_timestep = 0;
        ConflictKind kind = ConflictKind::Vertex;
        double alpha = 0.0;
        std::vector<double> config_i;
        std::vector<double> config_j;
        std::vector<SubproblemConflict::ExpansionTraceStep>
            robot_selection_trace;
        std::size_t max_t = 0;
        SubproblemRecordWindowState raw_window;
        bool valid_window_found = false;
        SubproblemRecordWindowState valid_window;
        std::uint64_t validity_search_expansion_count = 0;
        std::vector<SubproblemRecordWindowState> validity_search_trace;
        // Ordered ascending by global_robot_index (== conflict.robots order);
        // this IS the PrioritizedSTRRT priority order -- do not reorder.
        std::vector<SubproblemRecordRobotEntry> robots;
    };

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "ARC"; }

    /// Retain intermediate path sets, conflict batches, and applied repairs for
    /// ARC process artifacts. Disabled by default because histories can be
    /// large. Applications additionally gate capture and serialization on
    /// --output-paths together with --track-arc-history.
    void setVisualizationTraceEnabled(bool enabled) {
        visualization_trace_enabled_ = enabled;
        if (!enabled)
            visualization_trace_.clear();
    }
    bool visualizationTraceEnabled() const {
        return visualization_trace_enabled_;
    }
    const std::vector<VisualizationIteration> &visualizationTrace() const {
        return visualization_trace_;
    }

    /// Capture one SubproblemRecord per raw, unrepaired conflict ARC detects
    /// during solve(), at the moment each conflict is about to be handed to
    /// resolveConflictOnPaths -- i.e. before any repair attempt (a1/a3/a4)
    /// touches it. Disabled by default; applications gate this on their own
    /// flag (e.g. --conflict-record-dir).
    ///
    /// Unlike RepairAttemptEvent telemetry (always recorded), this defaults
    /// off because it does real added work per conflict beyond what
    /// resolution itself needs: an independent findValidWindow() replay
    /// (its own sequence of MultiRobotProblem rebuilds and composite
    /// validity checks) plus a per-robot cspace-bounds recomputation, none
    /// of which the real resolution path would otherwise pay for when
    /// nobody wants a corpus.
    void setCaptureSubproblemRecords(bool enabled) {
        capture_subproblem_records_ = enabled;
        if (!enabled)
            captured_subproblem_records_.clear();
    }
    bool captureSubproblemRecordsEnabled() const {
        return capture_subproblem_records_;
    }
    const std::vector<CapturedSubproblemRecord> &
    capturedSubproblemRecords() const {
        return captured_subproblem_records_;
    }
    /// Renders captured records to the SubproblemRecord JSON schema
    /// (subproblem-record-design.md), one array entry per capture. ARC
    /// supplies everything it knows (seed conflict, windows, robot
    /// geometry/configs, environment from the live `problem_`); the caller
    /// supplies the `provenance` fields ARC has no way to know (CLI args
    /// used to regenerate the run, git commit). Call before the
    /// planner/problem is torn down -- environment is read from `problem_`
    /// live, not cached at capture time.
    nlohmann::json subproblemRecordsJson(
        int schema_version, const nlohmann::json &provenance_source,
        const nlohmann::json &provenance_run_args,
        const std::string &git_commit) const;

    void setInitialWindow(int w) { initial_window_ = std::max(1, w); }
    void setExpansionStep(double e) {
        expansion_step_ = (std::isfinite(e) && e > 0.0) ? e : 1.0;
    }
    /// Temporal-window growth after local-repair failures. For zero-based
    /// expansion index k, the symmetric half-width is computed from the
    /// discovered valid base half-width. Linear targets
    /// base + expansion_step * (k + 1), Logarithmic targets
    /// base + expansion_step * log2(k + 2), and Exponential targets
    /// base + expansion_step * 2^k. CustomMultiplied targets
    /// discovered_valid_half_width * custom_expansion_multipliers[k].
    ///
    /// Formula-based policies jump to the global interval when their rounded,
    /// clipped target repeats the preceding window. Logarithmic also jumps
    /// globally when both remaining tails are within 10% of the full horizon.
    /// CustomMultiplied jumps globally after its ordered sequence is exhausted.
    void setExpansionPolicy(ExpansionPolicy policy) {
        expansion_policy_ = policy;
    }
    ExpansionPolicy expansionPolicy() const { return expansion_policy_; }
    void setCustomExpansionMultipliers(std::vector<double> multipliers);
    const std::vector<double> &customExpansionMultipliers() const {
        return custom_expansion_multipliers_;
    }

    /// Temporal-window growth used only until a subproblem first has valid
    /// composite start and goal configurations. Each setting inherits the
    /// corresponding main expansion setting until explicitly overridden.
    ///
    /// Once a valid endpoint window is found, ARC attempts that window before
    /// starting the main expansion schedule at index zero. The validity phase
    /// is not re-entered if a later main-expanded window has invalid endpoints.
    void setInitialValidWindowExpansionStep(double e) {
        initial_valid_window_expansion_step_ =
            (std::isfinite(e) && e > 0.0) ? e : 1.0;
    }
    void clearInitialValidWindowExpansionStep() {
        initial_valid_window_expansion_step_.reset();
    }
    double initialValidWindowExpansionStep() const {
        return initial_valid_window_expansion_step_.value_or(expansion_step_);
    }
    bool initialValidWindowExpansionStepInheritsMain() const {
        return !initial_valid_window_expansion_step_.has_value();
    }
    void setInitialValidWindowExpansionPolicy(ExpansionPolicy policy) {
        initial_valid_window_expansion_policy_ = policy;
    }
    void clearInitialValidWindowExpansionPolicy() {
        initial_valid_window_expansion_policy_.reset();
    }
    ExpansionPolicy initialValidWindowExpansionPolicy() const {
        return initial_valid_window_expansion_policy_.value_or(
            expansion_policy_);
    }
    bool initialValidWindowExpansionPolicyInheritsMain() const {
        return !initial_valid_window_expansion_policy_.has_value();
    }
    void setInitialValidWindowExpansionMultipliers(
        std::vector<double> multipliers);
    void clearInitialValidWindowExpansionMultipliers() {
        initial_valid_window_expansion_multipliers_.reset();
    }
    const std::vector<double> &
    initialValidWindowExpansionMultipliers() const {
        return initial_valid_window_expansion_multipliers_
            ? *initial_valid_window_expansion_multipliers_
            : custom_expansion_multipliers_;
    }
    bool initialValidWindowExpansionMultipliersInheritMain() const {
        return !initial_valid_window_expansion_multipliers_.has_value();
    }
    /// When true (default), endpoint-validity search expands both temporal
    /// sides together. When false, only an invalid start side and/or invalid
    /// goal side is expanded. After both endpoints are valid, all main-policy
    /// windows are symmetric about the midpoint of the discovered interval.
    void setInitialValidWindowExpansionSymmetric(bool symmetric) {
        initial_valid_window_expansion_symmetric_ = symmetric;
    }
    bool initialValidWindowExpansionSymmetric() const {
        return initial_valid_window_expansion_symmetric_;
    }

    /// Cap CompositeRRT (RRTConnect) outer-loop iterations per local solve call when
    /// the temporal window does not yet span the full global horizon. On the final
    /// full-window local call, the cap is ignored (time budget only).
    /// Zero disables the cap for all non-final calls as well.
    void setLocalCompositeRrtMaxSamples(unsigned n) { local_composite_rrt_max_samples_ = n; }
    /// Maximum RRTConnect extension length for local composite repairs. A
    /// non-positive value restores OMPL's automatic range selection.
    void setLocalCompositeRrtRange(double distance) {
        if (distance > 0.0)
            local_composite_rrt_range_ = distance;
        else
            local_composite_rrt_range_.reset();
    }
    std::optional<double> localCompositeRrtRange() const {
        return local_composite_rrt_range_;
    }
    void setLocalCompositeRrtUseMakespanMetric(bool v) {
        local_composite_rrt_use_makespan_metric_ = v;
    }
    void setLocalSolverMode(LocalSolverMode mode) { local_solver_mode_ = mode; }
    LocalSolverMode localSolverMode() const { return local_solver_mode_; }
    /// Per-robot STRRT* solve-loop iteration cap for ARC's local
    /// PrioritizedSTRRT attempt. Zero disables the cap.
    void setLocalPrioritizedStrrtMaxIterations(unsigned int iterations) {
        local_prioritized_strrt_max_iterations_ = iterations;
    }
    unsigned int localPrioritizedStrrtMaxIterations() const {
        return local_prioritized_strrt_max_iterations_;
    }
    void setLocalPrioritizedStrrtReturnFirstSolution(bool enabled) {
        local_prioritized_strrt_return_first_solution_ = enabled;
    }
    bool localPrioritizedStrrtReturnFirstSolution() const {
        return local_prioritized_strrt_return_first_solution_;
    }
    void setLocalPrioritizedStrrtRewiring(StrrtRewiring mode) {
        local_prioritized_strrt_rewiring_ = mode;
    }
    StrrtRewiring localPrioritizedStrrtRewiring() const {
        return local_prioritized_strrt_rewiring_;
    }
    void setLocalPrioritizedStrrtPersistAtGoal(bool enabled) {
        local_prioritized_strrt_persist_at_goal_ = enabled;
    }
    bool localPrioritizedStrrtPersistAtGoal() const {
        return local_prioritized_strrt_persist_at_goal_;
    }
    void setUseCspaceBounds(bool v) { use_cspace_bounds_ = v; }
    void setCspaceBoundMargin(float m) { cspace_bound_margin_ = m; }
    /// Minimum per-joint bound width for subproblem C-space boxes (stationary robots).
    void setMinCspaceBoundRange(double r) { min_cspace_bound_range_ = r; }
    void setPathSimplificationOptions(PathSimplificationOptions options) {
        simplification_options_ =
            detail::normalizePathSimplificationOptions(options);
    }
    PathSimplificationOptions getPathSimplificationOptions() const {
        return simplification_options_;
    }
    void setConflictPathSimplificationOptions(
        PathSimplificationOptions options) {
        conflict_simplification_options_ =
            detail::normalizePathSimplificationOptions(options);
    }
    void clearConflictPathSimplificationOptions() {
        conflict_simplification_options_.reset();
    }
    void setSimplificationMaxSteps(unsigned int max_steps) {
        simplification_options_.max_shortcut_steps = std::max(1u, max_steps);
    }
    unsigned int getSimplificationMaxSteps() const {
        return simplification_options_.max_shortcut_steps;
    }
    void setSimplifyInitialSolutions(bool simplify) {
        simplify_initial_solutions_ = simplify;
    }
    bool getSimplifyInitialSolutions() const {
        return simplify_initial_solutions_;
    }
    void setSimplifyConflictSolutions(bool simplify) {
        simplify_conflict_solutions_ = simplify;
    }
    bool getSimplifyConflictSolutions() const {
        return simplify_conflict_solutions_;
    }
    void setSimplifySolution(bool simplify) {
        simplify_initial_solutions_ = simplify;
        simplify_conflict_solutions_ = simplify;
    }

    /// Optional makespan bound in native CoMotion timestep units. When set, ARC uses
    /// bounded AO-RRTC for initial individual paths and local composite repairs.
    void setGlobalMakespanBoundTimesteps(std::uint64_t bound) {
        global_makespan_bound_timesteps_ = bound;
    }
    void clearGlobalMakespanBoundTimesteps() {
        global_makespan_bound_timesteps_.reset();
    }
    void setBoundedLocalRepairEpsilonTimesteps(std::uint64_t epsilon) {
        bounded_local_repair_epsilon_timesteps_ = epsilon;
    }
    std::uint64_t boundedLocalRepairEpsilonTimesteps() const {
        return bounded_local_repair_epsilon_timesteps_;
    }

    /// Multiplier for local STRRT* OMPL time upper bound: (end_t - start_t) timesteps /
    /// resolution (seconds) times this factor. Must be positive (default 4).
    void setStrrtSpaceTimeSpanFactor(double f) {
        strrt_space_time_span_factor_ = (f > 0.0) ? f : 4.0;
    }

    /// Deprecated: ARC no longer caps local PrioritizedSTRRT / CompositeRRT wall time
    /// below the remaining global budget. Local solvers receive the full remaining wall
    /// time (recomputed after each layer). Kept for API compatibility.
    [[deprecated("ARC local solvers use full remaining wall time; this setting is ignored.")]]
    void setLocalSolverMaxBudget(double /*max_seconds*/) {}

    /// Deprecated: unused; local wall budgets are always the remaining global time.
    [[deprecated("ARC local solvers use full remaining wall time; this setting is ignored.")]]
    void setLocalSolverBudgetExpansionIncrement(double /*seconds_per_expansion*/) {}

protected:
    struct ArcPlannerStatsSummary {
        LocalSolverMode local_solver_mode = LocalSolverMode::Both;
        unsigned int local_prioritized_strrt_max_iterations = 0;
        bool local_prioritized_strrt_return_first_solution = true;
        StrrtRewiring local_prioritized_strrt_rewiring =
            StrrtRewiring::KNearest;
        bool local_prioritized_strrt_persist_at_goal = false;
        bool local_composite_rrt_use_makespan_metric = false;
        std::uint64_t bounded_local_repair_epsilon_timesteps = 1;
        std::uint64_t num_conflicts = 0;
        std::uint64_t subproblem_attempts = 0;
        std::uint64_t temporal_expansions = 0;
        std::uint64_t initial_valid_temporal_expansions = 0;
        std::uint64_t main_temporal_expansions = 0;
        double initial_solution_times_seconds_wall_clock = 0.0;
        double initial_solution_times_seconds_cpu = 0.0;
        double initial_simplification_times_seconds_wall_clock = 0.0;
        double local_composite_simplification_times_seconds_wall_clock = 0.0;
        double conflict_detection_times_seconds_wall_clock = 0.0;
        double conflict_detection_times_seconds_cpu = 0.0;
        double conflict_resolution_times_seconds_wall_clock = 0.0;
        double conflict_resolution_times_seconds_total = 0.0;
        double conflict_resolution_times_seconds_cpu = 0.0;
        std::uint64_t subproblem_batches = 0;
    };

    struct ProcessTreeCpuUsageSnapshot {
        double self_seconds = 0.0;
        double children_seconds = 0.0;
    };

    using Clock = std::chrono::steady_clock;

    void startVisualizationIteration(const std::vector<Path> &paths);
    void setVisualizationConflicts(
        const std::vector<SubproblemConflict> &conflicts);
    void appendVisualizationRepair(std::size_t conflict_index,
                                   const std::vector<int> &robots,
                                   int window_start_t, int window_end_t,
                                   const std::vector<Path> &local_paths);
    void replaceVisualizationTrace(
        const std::vector<VisualizationIteration> &trace) {
        if (visualization_trace_enabled_)
            visualization_trace_ = trace;
    }

    struct RepairWindow {
        int window_start_t = 0;
        int window_end_t = 0;
        std::vector<int> history_event_ids;
    };

    struct AppliedRepairHistoryEvent {
        int event_id = -1;
        std::vector<int> robots;
        int window_start_t = 0;
        int window_end_t = 0;
    };

    struct RepairOutcome {
        bool resolved = false;
        int window_start_t = 0;
        int window_end_t = 0;
        std::vector<int> final_involved_robots;
        std::vector<Path> local_patch_paths;
    };

    struct ExpansionScheduleState {
        bool initial_valid_window_established = false;
        bool last_expansion_used_initial_valid_schedule = false;
        std::size_t initial_valid_expansion_index = 0;
        std::size_t main_expansion_index = 0;
        bool initial_search_geometry_initialized = false;
        std::int64_t initial_search_center_twice = 0;
        std::int64_t initial_search_half_width_twice = 0;
        std::int64_t main_window_center_twice = 0;
        std::int64_t main_base_half_width_twice = 0;
    };

    enum class RepairAttemptPhase {
        InitialWindow,
        InitialValid,
        Main,
    };

    struct RepairAttemptEvent {
        std::uint64_t repair_id = 0;
        std::uint64_t attempt_index = 0;
        int seed_robot_i = -1;
        int seed_robot_j = -1;
        int conflict_timestep = 0;
        std::vector<int> robots;
        RepairAttemptPhase phase = RepairAttemptPhase::InitialWindow;
        std::optional<std::size_t> expansion_index;
        int window_start_t = 0;
        int window_end_t = 0;
        std::size_t max_t = 0;
        // The expansion schedule's explicit global sentinel is [0, max_t].
        bool effective_global = false;
        // Solver behavior treats [0, max_t - 1] as spanning every waypoint.
        bool temporal_full_window = false;
        bool validity_checked = false;
        bool start_valid = false;
        bool goal_valid = false;
        bool endpoints_valid = false;
        bool bounded_epsilon_skipped = false;
        bool prioritized_invoked = false;
        bool composite_invoked = false;
        bool solver_invoked = false;
        bool resolved = false;
        std::uint32_t attempt_root_planning_seed = 0;
        std::optional<std::uint32_t> prioritized_planning_seed;
        std::optional<std::uint32_t> composite_planning_seed;
        std::optional<std::uint_fast32_t> composite_state_sampler_seed;
        std::optional<std::uint_fast32_t> composite_rrt_connect_seed;
        std::optional<std::uint_fast32_t> composite_path_simplifier_seed;
        std::optional<std::int64_t> main_window_center_twice;
        std::optional<std::int64_t> main_base_half_width_twice;
        std::string solved_by;
        std::string outcome = "pending";
    };

    struct IndividualPlanResult {
        bool success = false;
        int status_type = 0;
        std::string status_message;
        Path path;
        std::uint64_t arrival_timestep = 0;
        std::uint64_t solve_ns = 0;
        std::uint64_t simplify_ns = 0;
        double cpu_seconds = 0.0;
        std::string error_message;
    };

    void resetArcSolveState();

    IndividualPlanResult
    planIndividualPath(int robot_index, double solve_budget_seconds,
                       std::optional<std::uint32_t> local_seed = std::nullopt);
    void recordInitialIndividualPlanStats(const IndividualPlanResult &result);
    void finishInitialIndividualPaths(std::vector<Path> &working_paths);

    // Plan individual paths for each robot using RRTConnect; each solve uses remaining
    // wall time until the global ARC timeLimit.
    bool planIndividualPaths(const Clock::time_point &solve_start,
                             double timeLimit,
                             std::vector<Path> &working_paths);

    // Attempt to solve a subproblem with the solver hierarchy
    /// On success, if window_start_t_out is non-null, writes native path timestep
    /// index of the local window start (same coordinates as conflict.timestep).
    /// `global_time_limit` is the total wall budget for ARC::solve (seconds from
    /// `solve_start`); remaining time is recomputed from the clock each expansion.
    bool solveSubproblemOnPaths(const SubproblemConflict &conflict,
                                const Clock::time_point &solve_start,
                                double global_time_limit,
                                std::vector<Path> &working_paths,
                                 int *window_start_t_out = nullptr,
                                 int *window_end_t_out = nullptr,
                                 std::vector<Path> *local_paths_out = nullptr,
                                 bool apply_solution_to_paths = true,
                                 CancellationCallback cancel_requested = {});

    std::pair<int, int>
    nextExpansionWindow(int start_t, int end_t, std::size_t max_t,
                        std::size_t expansion_index) const;
    std::pair<int, int> nextExpansionWindowWithSettings(
        int start_t, int end_t, std::size_t max_t,
        std::size_t expansion_index, ExpansionPolicy policy,
        double expansion_step,
        const std::vector<double> &custom_multipliers) const;
    std::pair<int, int>
    nextInitialValidExpansionWindow(int start_t, int end_t,
                                    std::size_t max_t,
                                    std::size_t expansion_index) const;
    std::pair<int, int> nextInitialValidExpansionWindow(
        int start_t, int end_t, std::size_t max_t,
        std::size_t expansion_index, bool start_valid, bool goal_valid,
        const ExpansionScheduleState &state) const;
    std::pair<int, int> nextMainExpansionWindow(
        int start_t, int end_t, std::size_t max_t,
        std::size_t expansion_index,
        const ExpansionScheduleState &state) const;
    std::pair<int, int> nextExpansionWindowAfterAttempt(
        int start_t, int end_t, std::size_t max_t,
        bool start_valid, bool goal_valid,
        ExpansionScheduleState &state) const;
    std::pair<int, int> nextExpansionWindowAfterAttempt(
        int start_t, int end_t, std::size_t max_t,
        bool local_endpoints_valid, ExpansionScheduleState &state) const {
        return nextExpansionWindowAfterAttempt(
            start_t, end_t, max_t, local_endpoints_valid,
            local_endpoints_valid, state);
    }
    void establishMainWindowGeometry(int start_t, int end_t,
                                     ExpansionScheduleState &state) const;
    std::pair<int, int> symmetricWindowFromGeometry(
        std::int64_t center_twice, std::int64_t half_width_twice,
        std::size_t max_t) const;
    std::pair<int, int> absoluteExpansionWindow(
        int start_t, int end_t, std::size_t max_t,
        std::size_t expansion_index, ExpansionPolicy policy,
        double expansion_step,
        const std::vector<double> &custom_multipliers,
        std::int64_t center_twice,
        std::int64_t base_half_width_twice) const;

    // Splice subproblem solution into the global paths
    void spliceSolutionIntoPaths(const std::vector<int> &involved_robots,
                                 int start_t, int end_t,
                                 const std::vector<Path> &local_paths,
                                 std::vector<Path> &working_paths);
    void spliceSolutionIntoPaths(const std::vector<int> &involved_robots,
                                 int start_t, int end_t,
                                 const std::vector<const Path *> &local_paths,
                                 std::vector<Path> &working_paths);

    RepairOutcome resolveConflictOnPaths(
        const SubproblemConflict &conflict,
        const Clock::time_point &solve_start, double global_time_limit,
        std::vector<Path> &working_paths,
        bool apply_solution_to_paths = true,
        CancellationCallback cancel_requested = {});

    ArcPlannerStatsSummary currentArcPlannerStatsSummary() const;
    static nlohmann::json
    plannerStatsJsonFromSummary(
        const ArcPlannerStatsSummary &summary,
        const std::vector<double> *conflict_resolution_times_seconds = nullptr,
        const std::vector<double> *conflict_detection_times_seconds = nullptr);
    static ProcessTreeCpuUsageSnapshot processTreeCpuUsageSnapshot();
    static double elapsedProcessTreeCpuSeconds(
        const ProcessTreeCpuUsageSnapshot &start,
        const ProcessTreeCpuUsageSnapshot &finish);
    static nlohmann::json conflictFindTimingJson(
        const std::vector<double> &main_process_wall_seconds,
        const std::vector<double> &process_tree_cpu_seconds,
        const std::vector<double> &build_worker_wall_seconds,
        const std::vector<double> &build_worker_cpu_seconds,
        const std::vector<double> &collision_worker_wall_seconds,
        const std::vector<double> &collision_worker_cpu_seconds,
        const std::vector<int> &critical_worker_index,
        const std::vector<double> &critical_worker_build_wall_seconds,
        const std::vector<double> &critical_worker_collision_wall_seconds,
        const std::vector<double> &critical_worker_total_wall_seconds);
    nlohmann::json repairAttemptEventsJson() const;
    nlohmann::json conflictResolutionEventsJson() const;
    nlohmann::json conflictSolveCountsByExpansionStageJson() const;

    void initializeConflictScanStarts(std::size_t robot_count);
    CompositePathValidationOptions conflictScanOptions() const;
    void applyConflictScanProgress(
        const std::vector<std::size_t> &next_t_begin_by_pair);
    void resetConflictScanStartsForRobots(const std::vector<int> &robots,
                                          int start_t);
    void updateDerivedConflictScanStart();

    void recordAppliedRepairHistory(const std::vector<int> &robots,
                                    int window_start_t, int window_end_t);
    const std::vector<AppliedRepairHistoryEvent> &
    appliedRepairHistoryEvents() const {
        return applied_repair_history_events_;
    }

    virtual SubproblemConflict
    expandConflictForSubproblem(const Conflict &conflict) const;

    // --- Shared window-search primitives ---
    // Factored out of solveSubproblemOnPaths so its endpoint-validity-check
    // and window-expansion-search logic exists in exactly one place: the
    // real resolution loop below and findValidWindow() (used only by
    // SubproblemRecord capture, see captureSubproblemRecord()) are both
    // built from these same pieces plus the pre-existing
    // nextExpansionWindowAfterAttempt(), so they cannot drift apart.

    struct SubproblemEndpointValidity {
        bool start_valid = false;
        bool goal_valid = false;
    };

    struct SubproblemHorizonAndRawWindow {
        std::size_t max_t = 0;
        int raw_begin_t = 0;
        int raw_end_t = 0;
    };

    struct ValidWindowSearchResult {
        // false only if even the full horizon [0, max_t] has invalid
        // endpoints.
        bool found = false;
        int begin_t = 0;
        int end_t = 0;
        bool start_valid = false;
        bool goal_valid = false;
        // Every window tried, in order, including the raw window (first
        // entry) and the terminal window (valid or global-and-failed).
        std::vector<SubproblemRecordWindowState> trace;
    };

    // max_t (path horizon over involved robots) and the raw temporal
    // window for a conflict, matching solveSubproblemOnPaths's own
    // window-arithmetic (ARC.cpp:860-876): max_t = max involved-robot path
    // length; raw window = conflict.window_begin_t/window_end_t clamped
    // into [0, max_t].
    SubproblemHorizonAndRawWindow subproblemHorizonAndRawWindow(
        const SubproblemConflict &conflict,
        const std::vector<Path> &working_paths) const;

    // Standalone MultiRobotProblem containing exactly `involved_robots`,
    // with obstacles/vmax/resolution copied from problem_ and each robot's
    // start/goal pinned to its existing global-path config at
    // start_t/end_t (ARC.cpp:928-950). Pure: reads problem_ and
    // working_paths only, mutates neither.
    std::shared_ptr<MultiRobotProblem> buildSubproblemForWindow(
        const std::vector<int> &involved_robots,
        const std::vector<Path> &working_paths, int start_t,
        int end_t) const;

    // Composite start/goal collision validity for a subproblem already
    // built (by buildSubproblemForWindow or equivalently) for the window
    // it was built against (ARC.cpp:952-972).
    SubproblemEndpointValidity checkSubproblemEndpointValidity(
        const MultiRobotProblem &sub_problem) const;

    // Per-robot C-space sampling bounds for a window, indexed the same as
    // `involved_robots` (ARC.cpp:990-1069's margin/clamp geometry).
    // nullopt entries mean "no bound" -- matches ARC's own skip conditions
    // (use_cspace_bounds_ off, robot path empty, or temporal_full_window).
    std::vector<std::optional<SubproblemRecordCspaceBounds>>
    computeSubproblemCspaceBounds(const std::vector<int> &involved_robots,
                                  const std::vector<Path> &working_paths,
                                  int start_t, int end_t,
                                  bool temporal_full_window) const;

    static bool isTemporalFullWindow(int start_t, int end_t,
                                     std::size_t max_t) {
        return start_t == 0 && max_t > 0 &&
               static_cast<std::size_t>(end_t) >= max_t - 1;
    }

    // True when nextExpansionWindowAfterAttempt() returned the same window
    // it was given (`next` == `prev`) as an INTENTIONAL re-attempt at the
    // same bounds -- CustomMultiplied policy re-emitting an unfinished
    // multiplier entry -- rather than genuine expansion stagnation. Shared
    // by the real resolution loop (which can be in either phase) and
    // findValidWindow() (which is always in the InitialValid phase, so its
    // main_index_before is unused/irrelevant whenever it calls this).
    bool isRepeatedCustomWindow(int prev_start_t, std::size_t prev_end_t,
                                int next_start_t, int next_end_t,
                                bool used_initial_valid_schedule,
                                std::size_t initial_valid_index_before,
                                std::size_t main_index_before) const {
        if (!(next_start_t == prev_start_t &&
              static_cast<std::size_t>(next_end_t) == prev_end_t)) {
            return false;
        }
        if (used_initial_valid_schedule) {
            return initialValidWindowExpansionPolicy() ==
                       ExpansionPolicy::CustomMultiplied &&
                   initial_valid_index_before <
                       initialValidWindowExpansionMultipliers().size();
        }
        return expansion_policy_ == ExpansionPolicy::CustomMultiplied &&
               main_index_before < custom_expansion_multipliers_.size();
    }

    // Independently replays ARC's endpoint-validity-check +
    // window-expansion search (solveSubproblemOnPaths steps 2 and 5,
    // InitialValid phase only) to find the first temporal window whose
    // composite start and goal configs are collision-free -- WITHOUT
    // invoking any a1/a3 solver and WITHOUT mutating any ARC state
    // (repair_window_schedule_, solution_paths_, counters, RNG, etc). Uses
    // a fresh, local ExpansionScheduleState, so it never touches the
    // schedule state a live solveSubproblemOnPaths call owns.
    ValidWindowSearchResult
    findValidWindow(const std::vector<int> &involved_robots,
                    const std::vector<Path> &working_paths, int raw_begin_t,
                    int raw_end_t, std::size_t max_t) const;

    // Builds and appends one CapturedSubproblemRecord for `conflict` as
    // detected against `working_paths` -- the raw, unrepaired conflict,
    // captured before resolveConflictOnPaths is ever called on it. No-op
    // unless capture_subproblem_records_ is set.
    void captureSubproblemRecord(const SubproblemConflict &conflict,
                                 const std::vector<Path> &working_paths);

    // Cascade merge via recursive repair-window closure: start from the
    // conflicting pair and repeatedly union robots with prior pair repair
    // windows that intersect the proposed conflict patch window.
    std::vector<int> subproblemRobotsForConflict(int robot_i, int robot_j,
                                                 int window_start_t,
                                                 int window_end_t,
                                                 std::vector<
                                                     SubproblemConflict::
                                                         ExpansionTraceStep> *
                                                     trace_out = nullptr) const;
    const std::vector<RepairWindow> *
    repairWindowsForRobots(int robot_i, int robot_j) const;
    int conflictWindowStart(const Conflict &conflict) const {
        return std::max(0, conflict.timestep - initial_window_);
    }
    std::vector<Path> solution_paths_;
    std::map<int, std::map<int, std::vector<RepairWindow>>>
        repair_window_schedule_;
    std::vector<AppliedRepairHistoryEvent> applied_repair_history_events_;
    std::size_t conflict_scan_robot_count_ = 0;
    std::vector<int> pair_conflict_scan_start_t_;
    std::vector<std::uint64_t> true_arrival_timesteps_;
    /// Native timestep index where last successful local replan began; next conflict
    /// scan can start there (prefix unchanged). Maintained as the minimum of
    /// pair_conflict_scan_start_t_ for coarse tracing/stat summaries.
    int last_subproblem_window_start_ = -1;
    int initial_window_ = 20;
    double expansion_step_ = 20.0;
    ExpansionPolicy expansion_policy_ = ExpansionPolicy::Linear;
    std::vector<double> custom_expansion_multipliers_{
        1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 4.0, 8.0};
    std::optional<double> initial_valid_window_expansion_step_;
    std::optional<ExpansionPolicy> initial_valid_window_expansion_policy_;
    std::optional<std::vector<double>>
        initial_valid_window_expansion_multipliers_;
    bool initial_valid_window_expansion_symmetric_ = true;
    unsigned local_composite_rrt_max_samples_{0};
    std::optional<double> local_composite_rrt_range_;
    bool local_composite_rrt_use_makespan_metric_{false};
    LocalSolverMode local_solver_mode_ = LocalSolverMode::Both;
    unsigned int local_prioritized_strrt_max_iterations_ = 5;
    bool local_prioritized_strrt_return_first_solution_ = true;
    StrrtRewiring local_prioritized_strrt_rewiring_ =
        StrrtRewiring::KNearest;
    bool local_prioritized_strrt_persist_at_goal_ = false;
    bool use_cspace_bounds_ = true;
    float cspace_bound_margin_ = 0.5f;
    double min_cspace_bound_range_ = 0.1;
    bool simplify_initial_solutions_ = true;
    bool simplify_conflict_solutions_ = false;
    PathSimplificationOptions simplification_options_{};
    std::optional<PathSimplificationOptions> conflict_simplification_options_;
    std::optional<std::uint64_t> global_makespan_bound_timesteps_;
    std::uint64_t bounded_local_repair_epsilon_timesteps_ = 1;
    bool warned_bounded_prioritized_disabled_ = false;
    /// Scales local window duration (seconds) -> SpaceTimeStateSpace upper bound.
    double strrt_space_time_span_factor_ = 4.0;
    std::uint64_t num_conflicts_ = 0;
    std::uint64_t num_subproblem_attempts_ = 0;
    std::uint64_t num_temporal_expansions_ = 0;
    std::uint64_t num_initial_valid_temporal_expansions_ = 0;
    std::uint64_t num_main_temporal_expansions_ = 0;
    std::uint64_t next_repair_attempt_id_ = 0;
    std::vector<RepairAttemptEvent> repair_attempt_events_;
    double initial_solution_times_seconds_wall_clock_ = 0.0;
    double initial_solution_times_seconds_cpu_ = 0.0;
    double initial_simplification_times_seconds_wall_clock_ = 0.0;
    double local_composite_simplification_times_seconds_wall_clock_ = 0.0;
    std::vector<double> conflict_detection_times_seconds_;
    std::vector<double> conflict_detection_times_cpu_seconds_;
    // Per-round conflict-search timing reported by plannerStatsJson().
    std::vector<double> conflict_find_main_process_wall_seconds_;
    std::vector<double> conflict_find_process_tree_cpu_seconds_;
    std::vector<double> conflict_find_build_worker_wall_seconds_;
    std::vector<double> conflict_find_build_worker_cpu_seconds_;
    std::vector<double> conflict_find_collision_worker_wall_seconds_;
    std::vector<double> conflict_find_collision_worker_cpu_seconds_;
    std::vector<int> conflict_find_critical_worker_index_;
    std::vector<double> conflict_find_critical_worker_build_wall_seconds_;
    std::vector<double>
        conflict_find_critical_worker_collision_wall_seconds_;
    std::vector<double> conflict_find_critical_worker_total_wall_seconds_;
    std::vector<double> conflict_resolution_times_seconds_;
    std::vector<double> conflict_resolution_times_cpu_seconds_;
    bool visualization_trace_enabled_ = false;
    std::vector<VisualizationIteration> visualization_trace_;
    bool capture_subproblem_records_ = false;
    std::vector<CapturedSubproblemRecord> captured_subproblem_records_;
};

using ArcLocalSolverMode = ARC::LocalSolverMode;
using ArcExpansionPolicy = ARC::ExpansionPolicy;

} // namespace comotion
