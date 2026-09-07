import assert from "node:assert/strict";

import {
  isChunkedArcVisualization,
  resolveChunkUrl,
  mergeChunkedIterations,
} from "../js/arc-chunking.js";

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

// mergeChunkedIterations: concatenates in manifest/array order, regardless
// of internal shape quirks in any one chunk body.
assert.deepEqual(
  mergeChunkedIterations([
    { iterations: [{ id: 0 }, { id: 1 }] },
    { iterations: [{ id: 2 }] },
  ]),
  [{ id: 0 }, { id: 1 }, { id: 2 }]
);
assert.deepEqual(mergeChunkedIterations([]), []);
assert.deepEqual(
  mergeChunkedIterations([{ iterations: [] }, { iterations: [{ id: 0 }] }]),
  [{ id: 0 }]
);
assert.deepEqual(
  mergeChunkedIterations([{ notIterations: [] }]),
  [],
  "a malformed chunk body must not crash the merge, just contribute nothing"
);

console.log("arc-chunking.test.mjs: all assertions passed");
