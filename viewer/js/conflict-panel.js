/**
 * Pure helpers for the conflict-list panel: deriving where a loaded result's
 * SubproblemRecord conflict files live, validating/summarizing one record,
 * and building its detail readout. Kept free of fetch()/DOM so it can be
 * unit-tested directly, mirroring the js/roadmap-lines.js split.
 *
 * SubproblemRecord schema:
 * comotion-vadim/requirements/data-collection/subproblem-record-design.md
 */

/**
 * Server-root-relative directory a loaded result's raw conflict pool lives
 * under, derived from `benchmark.context.num_robots`/`.seed` — the same
 * naming convention random_crossing's own output filenames already use
 * (`outputBasename()` in mobile_robot_2d_crossing.cpp: `..._n<N>_seed<S>_...`)
 * and that the conflict-collection sweep's `full-pool/n<N>_seed<S>/`
 * directories were named to match. Requires the `comotion/vadim` symlink
 * (-> ../comotion-vadim) to be present and the viewer server to be started
 * from the `comotion/` repo root (per viewer/README.md) so `/vadim/...`
 * resolves. Returns null when the loaded result isn't a recognizable
 * mobile_robot_2d_crossing run (missing num_robots/seed) — the panel stays
 * hidden in that case, not an error.
 */
function conflictFolderPath(resultData) {
  const context = resultData?.benchmark?.context;
  const numRobots = context?.num_robots;
  const seed = context?.seed;
  if (!Number.isInteger(numRobots) || !Number.isInteger(seed)) return null;
  return `/vadim/data/conflict-records/full-pool/n${numRobots}_seed${seed}/`;
}

/**
 * Minimal shape check before trusting a fetched conflict-record JSON file —
 * defensive against a partial/malformed file the same way
 * roadmap-lines.js's sanitizeRoadmapEntry guards against malformed roadmap
 * entries. Also doubles as the "was this actually a conflict-record JSON
 * and not e.g. an HTML error page" check when probing sequential filenames.
 */
function isWellFormedConflictRecord(record) {
  return !!(
    record &&
    record.provenance &&
    Number.isInteger(record.provenance.conflict_sequence_index) &&
    record.seed_conflict &&
    Number.isInteger(record.seed_conflict.conflict_timestep) &&
    Number.isInteger(record.seed_conflict.seed_robot_i) &&
    Number.isInteger(record.seed_conflict.seed_robot_j) &&
    Array.isArray(record.robots) &&
    record.windows
  );
}

/** Global robot indices involved in a conflict, in the record's own order. */
function conflictRobotIndices(record) {
  return record.robots.map((robot) => robot.global_robot_index);
}

/**
 * One-line list-row label, e.g. "#3 — robots 1,3 @ t=940" for a plain
 * two-robot conflict, or "#65 — 17 robots (seed 14,26) @ t=7377" once
 * cascade-merging pulled in more than the colliding pair.
 */
function conflictSummaryLine(record) {
  const seq = record.provenance.conflict_sequence_index;
  const t = record.seed_conflict.conflict_timestep;
  const indices = conflictRobotIndices(record);
  const { seed_robot_i: seedI, seed_robot_j: seedJ } = record.seed_conflict;
  const robotsText =
    indices.length <= 2
      ? `robots ${indices.join(",")}`
      : `${indices.length} robots (seed ${seedI},${seedJ})`;
  return `#${seq} — ${robotsText} @ t=${t}`;
}

/**
 * Detail readout lines for the panel's expanded view, shown after clicking
 * a row. Plain strings (one per line) rather than markup, kept in this pure
 * module so the exact wording is unit-testable.
 */
function conflictDetailLines(record) {
  const p = record.provenance;
  const w = record.windows;
  const sc = record.seed_conflict;
  const validWindow = w.valid_window.found
    ? `[${w.valid_window.begin_t}, ${w.valid_window.end_t}]`
    : "not found";
  return [
    `Conflict #${p.conflict_sequence_index} (repair_id ${p.repair_id})`,
    `Robots involved (${record.robots.length}): ${conflictRobotIndices(record).join(", ")}`,
    `Seed pair: ${sc.seed_robot_i}, ${sc.seed_robot_j} — kind=${sc.kind}, alpha=${sc.alpha}`,
    `Conflict timestep: ${sc.conflict_timestep} (path horizon max_t=${w.max_t})`,
    `Raw window: [${w.raw_window.begin_t}, ${w.raw_window.end_t}]` +
      ` (endpoints_valid=${w.raw_window.endpoints_valid})`,
    `Valid window: ${validWindow} (expansions=${w.validity_search.expansion_count})`,
    `Run: N=${p.run_args.num_robots} seed=${p.run_args.seed}` +
      ` scenario_generation_seed=${p.run_args.scenario_generation_seed}`,
    `git_commit: ${(p.git_commit || "unknown").slice(0, 10)}`,
  ];
}

export {
  conflictFolderPath,
  isWellFormedConflictRecord,
  conflictRobotIndices,
  conflictSummaryLine,
  conflictDetailLines,
};
