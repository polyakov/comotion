import assert from "node:assert/strict";

import { normalizeArcVisualization, parseResult } from "../js/schema.js";

// Chunked manifests (comotion-vadim/requirements/viewer_result_chunking.md)
// must survive normalizeArcVisualization untouched -- there's no inline
// "iterations" yet at this point, and the pre-existing behavior for
// "no iterations array" is to null the whole thing out, which would
// destroy the chunk manifest before app.js ever gets to read it.
{
  const data = {
    robots: [{}],
    arc_visualization: {
      planner: "ARC",
      iteration_chunks: [{ file: "a.chunk0.json", iteration_start: 0, iteration_end: 4 }],
    },
  };
  normalizeArcVisualization(data);
  assert.equal(data.arc_visualization.iteration_chunks.length, 1);
  assert.equal(data.arc_visualization.planner, "ARC");
  assert.notEqual(data.arc_visualization, null, "chunk manifest must not be nulled out");
}

// An empty iteration_chunks array is not a valid chunked manifest -- must
// fall through to the ordinary "no iterations" -> null path, same as an
// unchunked result with no ARC history at all.
{
  const data = { robots: [{}], arc_visualization: { iteration_chunks: [] } };
  normalizeArcVisualization(data);
  assert.equal(data.arc_visualization, null);
}

// Regression guard: ordinary unchunked results (the overwhelming majority
// of files, including every non-mobile_robot_2d_crossing app) must
// normalize exactly as before -- no iteration_chunks key at all.
{
  const data = {
    robots: [{}, {}],
    arc_visualization: {
      planner: "ARC",
      workers: 0,
      iterations: [
        {
          paths: [[[0, 0, 0]], [[1, 1, 0]]],
          conflict_scan_completed: true,
          conflicts: [{ robot_i: 0, robot_j: 1, timestep: 0 }],
          repairs: [],
        },
      ],
    },
  };
  normalizeArcVisualization(data);
  assert.equal(data.arc_visualization.iterations.length, 1);
  assert.equal(data.arc_visualization.workers, 1, "workers still defaults to >=1");
}

// No arc_visualization at all -> stays null, same as always.
{
  const data = { robots: [{}] };
  normalizeArcVisualization(data);
  assert.equal(data.arc_visualization, null);
}

// End-to-end via parseResult(): a chunked result's manifest must still be
// present in the object parseResult() hands back to app.js, since that's
// exactly what loadFromUrl() inspects to decide whether to fetch chunks.
{
  const text = JSON.stringify({
    schema_version: "1.0",
    robots: [{ path: [[0, 0, 0]] }],
    arc_visualization: {
      planner: "ARC",
      iteration_chunks: [{ file: "x.chunk0.json", iteration_start: 0, iteration_end: 0 }],
    },
  });
  const data = parseResult(text);
  assert.notEqual(data, null);
  assert.equal(data.arc_visualization.iteration_chunks.length, 1);
}

console.log("schema.test.mjs: all assertions passed");
