import assert from "node:assert/strict";

import {
  conflictFolderPath,
  isWellFormedConflictRecord,
  conflictRobotIndices,
  conflictSummaryLine,
  conflictDetailLines,
} from "../js/conflict-panel.js";

// conflictFolderPath: derived from benchmark.context.num_robots/.seed, the
// same values the app's own outputBasename() embeds in result filenames.
assert.equal(
  conflictFolderPath({ benchmark: { context: { num_robots: 8, seed: 5 } } }),
  "/vadim/data/conflict-records/full-pool/n8_seed5/"
);
assert.equal(
  conflictFolderPath({ benchmark: { context: { num_robots: 64, seed: 1 } } }),
  "/vadim/data/conflict-records/full-pool/n64_seed1/"
);
assert.equal(conflictFolderPath({}), null, "missing benchmark.context");
assert.equal(
  conflictFolderPath({ benchmark: { context: { num_robots: 8 } } }),
  null,
  "missing seed"
);
assert.equal(
  conflictFolderPath({ benchmark: { context: { seed: 1 } } }),
  null,
  "missing num_robots"
);
assert.equal(
  conflictFolderPath({ benchmark: { context: { num_robots: "8", seed: 1 } } }),
  null,
  "non-integer num_robots must not silently coerce"
);
assert.equal(conflictFolderPath(null), null);
assert.equal(conflictFolderPath(undefined), null);

// A minimal, well-formed two-robot record (only the fields the panel reads).
function makeRecord(overrides = {}) {
  return {
    provenance: {
      conflict_sequence_index: 3,
      repair_id: 2,
      git_commit: "abcdef1234567890",
      run_args: { num_robots: 8, seed: 5, scenario_generation_seed: 5 },
      ...overrides.provenance,
    },
    seed_conflict: {
      seed_robot_i: 1,
      seed_robot_j: 3,
      conflict_timestep: 940,
      kind: "Vertex",
      alpha: 0,
      ...overrides.seed_conflict,
    },
    robots: overrides.robots ?? [
      { global_robot_index: 1 },
      { global_robot_index: 3 },
    ],
    windows: {
      max_t: 5000,
      raw_window: { begin_t: 740, end_t: 1140, endpoints_valid: true },
      valid_window: { found: true, begin_t: 740, end_t: 1140 },
      validity_search: { expansion_count: 0 },
      ...overrides.windows,
    },
  };
}

// isWellFormedConflictRecord: the sequential-fetch loop's stopping
// condition, so it must reject anything that isn't a real conflict record
// (e.g. a 200-OK HTML error page) rather than crash or loop forever.
assert.equal(isWellFormedConflictRecord(makeRecord()), true);
assert.equal(isWellFormedConflictRecord(null), false);
assert.equal(isWellFormedConflictRecord(undefined), false);
assert.equal(isWellFormedConflictRecord({}), false);
assert.equal(isWellFormedConflictRecord("<html>not json-shaped</html>"), false);
assert.equal(
  isWellFormedConflictRecord({ ...makeRecord(), provenance: null }),
  false
);
assert.equal(
  isWellFormedConflictRecord({
    ...makeRecord(),
    seed_conflict: { seed_robot_i: 1, seed_robot_j: 3 }, // missing conflict_timestep
  }),
  false
);
assert.equal(
  isWellFormedConflictRecord({ ...makeRecord(), robots: "not-an-array" }),
  false
);

// conflictRobotIndices: preserves the record's own robot ordering (which is
// ascending global_robot_index per the schema — the panel must not re-sort).
assert.deepEqual(
  conflictRobotIndices(
    makeRecord({
      robots: [
        { global_robot_index: 1 },
        { global_robot_index: 5 },
        { global_robot_index: 9 },
      ],
    })
  ),
  [1, 5, 9]
);

// conflictSummaryLine: plain "robots i,j" for a two-robot conflict, ...
assert.equal(conflictSummaryLine(makeRecord()), "#3 — robots 1,3 @ t=940");

// ...and "<n> robots (seed i,j)" once cascade-merging pulled in more.
assert.equal(
  conflictSummaryLine(
    makeRecord({
      provenance: { conflict_sequence_index: 65 },
      seed_conflict: { seed_robot_i: 14, seed_robot_j: 26, conflict_timestep: 7377 },
      robots: Array.from({ length: 17 }, (_, i) => ({ global_robot_index: i + 1 })),
    })
  ),
  "#65 — 17 robots (seed 14,26) @ t=7377"
);

// conflictDetailLines: exact wording matters here (it's what a person reads
// in the panel), so pin the whole array rather than spot-checking fields.
assert.deepEqual(conflictDetailLines(makeRecord()), [
  "Conflict #3 (repair_id 2)",
  "Robots involved (2): 1, 3",
  "Seed pair: 1, 3 — kind=Vertex, alpha=0",
  "Conflict timestep: 940 (path horizon max_t=5000)",
  "Raw window: [740, 1140] (endpoints_valid=true)",
  "Valid window: [740, 1140] (expansions=0)",
  "Run: N=8 seed=5 scenario_generation_seed=5",
  "git_commit: abcdef1234",
]);

// conflictDetailLines: "not found" wording when ARC's window search never
// converged (windows.valid_window.found === false) — the schema's negative-
// example case, must not crash on the absent begin_t/end_t.
assert.deepEqual(
  conflictDetailLines(
    makeRecord({
      windows: {
        max_t: 5000,
        raw_window: { begin_t: 0, end_t: 5000, endpoints_valid: false },
        valid_window: { found: false, begin_t: 0, end_t: 0 },
        validity_search: { expansion_count: 4 },
      },
    })
  )[5],
  "Valid window: not found (expansions=4)"
);

console.log("conflict-panel.test.mjs: all assertions passed");
