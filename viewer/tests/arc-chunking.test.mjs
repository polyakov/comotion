import assert from "node:assert/strict";

import {
  isChunkedArcVisualization,
  resolveChunkUrl,
  chunkIndexForIteration,
  chunksInWindow,
  buildArcTimelineFromSummaries,
} from "../js/arc-chunking.js";
import { buildArcTimeline } from "../js/arc-playback.js";

// isChunkedArcVisualization: the backward-compatibility switch. Must be
// false for every shape an existing (unchunked) result can have.
assert.equal(
  isChunkedArcVisualization({
    arc_visualization: { iteration_chunks: [{ file: "a.json" }] },
  }),
  true
);
assert.equal(
  isChunkedArcVisualization({ arc_visualization: { iteration_chunks: [] } }),
  false,
  "empty array must not count as chunked"
);
assert.equal(
  isChunkedArcVisualization({ arc_visualization: { iterations: [{}] } }),
  false,
  "today's inline-iterations shape must not be mistaken for chunked"
);
assert.equal(isChunkedArcVisualization({ arc_visualization: null }), false);
assert.equal(isChunkedArcVisualization({}), false);
assert.equal(isChunkedArcVisualization(null), false);
assert.equal(isChunkedArcVisualization(undefined), false);
assert.equal(
  isChunkedArcVisualization({ arc_visualization: { iteration_chunks: "nope" } }),
  false
);

// resolveChunkUrl: chunks are always resolved as siblings of the base file.
assert.equal(
  resolveChunkUrl(
    "http://localhost:8000/vadim/data/x/result.chunked.json",
    "result.chunk0.json"
  ),
  "http://localhost:8000/vadim/data/x/result.chunk0.json"
);
assert.equal(
  resolveChunkUrl(
    "http://localhost:8000/vadim/data/x/result.chunked.json?foo=bar",
    "result.chunk1.json"
  ),
  "http://localhost:8000/vadim/data/x/result.chunk1.json",
  "query string on the base URL must not leak into the resolved chunk URL"
);

// chunkIndexForIteration: manifest ranges are contiguous and inclusive.
const manifest = [
  { file: "c0.json", iteration_start: 0, iteration_end: 4 },
  { file: "c1.json", iteration_start: 5, iteration_end: 9 },
  { file: "c2.json", iteration_start: 10, iteration_end: 10 },
];
assert.equal(chunkIndexForIteration(manifest, 0), 0);
assert.equal(chunkIndexForIteration(manifest, 4), 0);
assert.equal(chunkIndexForIteration(manifest, 5), 1);
assert.equal(chunkIndexForIteration(manifest, 9), 1);
assert.equal(chunkIndexForIteration(manifest, 10), 2);
assert.equal(chunkIndexForIteration(manifest, 11), -1, "past the end of the manifest");
assert.equal(chunkIndexForIteration(manifest, -1), -1);
assert.equal(chunkIndexForIteration([], 0), -1);

// chunksInWindow: center chunk plus one neighbor each side, clipped to
// the manifest's valid range (no out-of-bounds indices, no phantom
// negative/overflow entries at the ends).
assert.deepEqual(chunksInWindow(manifest, 1), [0, 1, 2]);
assert.deepEqual(chunksInWindow(manifest, 0), [0, 1], "no chunk -1 to include");
assert.deepEqual(chunksInWindow(manifest, 2), [1, 2], "no chunk 3 to include");
assert.deepEqual(chunksInWindow([manifest[0]], 0), [0], "single-chunk manifest");

// buildArcTimelineFromSummaries: must produce byte-identical frame
// sequences to buildArcTimeline() given equivalent data — this is the
// property the whole on-demand-loading tier depends on, since the
// timeline is what sizes the slider and drives every frame lookup before
// any chunk is ever fetched.
function iterationAndSummary({
  timesteps,
  conflicts = [],
  conflictScanCompleted = true,
  repairPathLengths = null, // null = no repairs; [] = repairs with no paths; [n,...] = paths of those lengths
}) {
  const paths = Array.from({ length: 2 }, () => new Array(timesteps).fill([0, 0, 0]));
  const repairs =
    repairPathLengths === null
      ? []
      : [{
          robots: [0, 1],
          paths: repairPathLengths.map((len) => new Array(len).fill([0, 0, 0])),
        }];
  const iteration = {
    paths,
    timesteps,
    conflict_scan_completed: conflictScanCompleted,
    conflicts,
    repairs,
  };
  const summary = {
    timesteps,
    conflict_scan_completed: conflictScanCompleted,
    conflicts,
    repair_count: repairs.length,
    repair_frame_count:
      repairPathLengths === null || repairPathLengths.length === 0
        ? 0
        : Math.max(...repairPathLengths, 1) - 1,
  };
  return { iteration, summary };
}

function assertTimelinesMatch(cases) {
  const iterations = cases.map((c) => c.iteration);
  const summaries = cases.map((c) => c.summary);
  const fromFull = buildArcTimeline({ arc_visualization: { iterations } });
  const fromSummaries = buildArcTimelineFromSummaries(summaries);
  assert.deepEqual(fromSummaries, fromFull);
}

// Solved iteration, no conflicts.
assertTimelinesMatch([iterationAndSummary({ timesteps: 5 })]);

// Conflict detected partway through, no repair (matches a global-window-
// failed / not-yet-resolved case).
assertTimelinesMatch([
  iterationAndSummary({ timesteps: 20, conflicts: [{ robot_i: 0, robot_j: 1, timestep: 7 }] }),
]);

// Conflict + a real repair phase.
assertTimelinesMatch([
  iterationAndSummary({
    timesteps: 20,
    conflicts: [{ robot_i: 0, robot_j: 1, timestep: 7 }],
    repairPathLengths: [12, 15],
  }),
]);

// Multiple iterations back to back (the realistic multi-round case).
assertTimelinesMatch([
  iterationAndSummary({ timesteps: 30, conflicts: [{ robot_i: 0, robot_j: 1, timestep: 10 }], repairPathLengths: [8] }),
  iterationAndSummary({ timesteps: 40, conflicts: [{ robot_i: 1, robot_j: 2, timestep: 22 }] }),
  iterationAndSummary({ timesteps: 50 }), // final solved iteration
]);

// Empty/absent summaries -> empty timeline, matching buildArcTimeline's
// own empty-input behavior.
assert.deepEqual(buildArcTimelineFromSummaries([]), []);
assert.deepEqual(buildArcTimelineFromSummaries(undefined), []);
assert.deepEqual(buildArcTimelineFromSummaries(null), []);

console.log("arc-chunking.test.mjs: all assertions passed");
