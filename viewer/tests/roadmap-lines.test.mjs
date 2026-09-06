import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import {
  hasRoadmapData,
  robotNameForEntry,
  roadmapEntries,
  sanitizeRoadmapEdges,
  sanitizeRoadmapEntry,
  sanitizeRoadmapVertices,
} from "../js/roadmap-lines.js";

// hasRoadmapData: panel-presence check driven by data, not by playback state.
assert.equal(hasRoadmapData({ roadmaps: [{ robot_index: 0 }] }), true);
assert.equal(hasRoadmapData({ roadmaps: [] }), false, "empty array must not count as present");
assert.equal(hasRoadmapData({}), false);
assert.equal(hasRoadmapData(undefined), false);
assert.equal(hasRoadmapData(null), false);
assert.equal(hasRoadmapData({ roadmaps: "not-an-array" }), false);

// sanitizeRoadmapVertices: keeps well-formed [x, y, z] points at their
// original index (nulling malformed ones in place) so edge indices into the
// array stay valid.
assert.deepEqual(
  sanitizeRoadmapVertices([
    [0, 0, 0],
    [1, 2], // too short
    [1, 2, 3],
    "not-a-point",
    [1, 2, NaN], // non-finite
    null,
    [4, 5, 6, 7], // extra dims still fine, first three are used
  ]),
  [[0, 0, 0], null, [1, 2, 3], null, null, null, [4, 5, 6, 7]]
);
assert.deepEqual(sanitizeRoadmapVertices(undefined), []);
assert.deepEqual(sanitizeRoadmapVertices(null), []);
assert.deepEqual(sanitizeRoadmapVertices([]), []);

// sanitizeRoadmapEdges: keeps only in-bounds integer pairs whose endpoints
// both survived vertex sanitization.
const sanitizedVertices = [[0, 0, 0], null, [1, 1, 1], [2, 2, 2]];
assert.deepEqual(
  sanitizeRoadmapEdges(
    [
      [0, 2], // valid
      [2, 3], // valid
      [0, 1], // endpoint 1 is null (malformed vertex)
      [0, 9], // out of bounds
      [-1, 0], // negative index
      "not-a-pair",
      [0], // wrong length
      [0.5, 2], // non-integer
    ],
    sanitizedVertices
  ),
  [[0, 2], [2, 3]]
);
assert.deepEqual(sanitizeRoadmapEdges(undefined, sanitizedVertices), []);
assert.deepEqual(sanitizeRoadmapEdges(null, sanitizedVertices), []);
assert.deepEqual(sanitizeRoadmapEdges([[0, 1]], []), []);

// sanitizeRoadmapEntry: combines both, keyed by robot_index.
assert.deepEqual(
  sanitizeRoadmapEntry({
    robot_index: 2,
    vertices: [[0, 0, 0], [1, 1, 1], [2, 2, 2]],
    edges: [[0, 1], [1, 2], [0, 2]],
    start_vertex: 0,
    goal_vertex: 1,
  }),
  {
    robot_index: 2,
    vertices: [[0, 0, 0], [1, 1, 1], [2, 2, 2]],
    edges: [[0, 1], [1, 2], [0, 2]],
  }
);
// Malformed entry (missing/non-integer robot_index) is dropped, not thrown on.
assert.equal(sanitizeRoadmapEntry({ vertices: [], edges: [] }), null);
assert.equal(sanitizeRoadmapEntry(null), null);
assert.equal(sanitizeRoadmapEntry(undefined), null);
assert.equal(sanitizeRoadmapEntry({ robot_index: "0" }), null, "robot_index must be an integer, not a numeric string");

// A malformed vertex drops any edge touching it, without disturbing other
// edges' indices into the (still same-length) vertices array.
assert.deepEqual(
  sanitizeRoadmapEntry({
    robot_index: 0,
    vertices: [[0, 0, 0], "bad", [2, 2, 2]],
    edges: [[0, 1], [0, 2], [1, 2]],
  }),
  {
    robot_index: 0,
    vertices: [[0, 0, 0], null, [2, 2, 2]],
    edges: [[0, 2]],
  }
);

// roadmapEntries: full pipeline over resultData.roadmaps, dropping
// unusable entries and leaving usable ones sanitized.
const resultData = {
  robots: [
    { robot_type: "sphere", name: "sphere_0" },
    { robot_type: "sphere" }, // no name -> falls back to "Robot <index>"
  ],
  roadmaps: [
    {
      robot_index: 0,
      vertices: [[0, 0, 0], [1, 0, 0]],
      edges: [[0, 1], [1, 0]], // reverse duplicate; not this module's job to dedupe (server already did)
      start_vertex: 0,
      goal_vertex: 1,
    },
    { robot_index: "bad" }, // dropped
    {
      robot_index: 1,
      vertices: [[5, 5, 5], [6, 6, 6]],
      edges: [[0, 1]],
      start_vertex: 0,
      goal_vertex: 1,
    },
  ],
};
const entries = roadmapEntries(resultData);
assert.equal(entries.length, 2);
assert.deepEqual(entries.map((e) => e.robot_index), [0, 1]);
assert.deepEqual(entries[0].vertices, [[0, 0, 0], [1, 0, 0]]);
assert.deepEqual(entries[0].edges, [[0, 1], [1, 0]]);

assert.deepEqual(roadmapEntries({ roadmaps: [] }), []);
assert.deepEqual(roadmapEntries(undefined), []);
assert.deepEqual(roadmapEntries(null), []);

// robotNameForEntry: robots[robot_index].name, falling back to a synthesized label.
assert.equal(robotNameForEntry(resultData, entries[0]), "sphere_0");
assert.equal(robotNameForEntry(resultData, entries[1]), "Robot 1");
assert.equal(robotNameForEntry({ robots: [] }, { robot_index: 0 }), "Robot 0");
assert.equal(robotNameForEntry(undefined, { robot_index: 3 }), "Robot 3");

// index.html wiring: roadmap panel is present, starts hidden, and its rows
// container is separate from the (empty, JS-populated) heading.
const viewerHtml = readFileSync(new URL("../index.html", import.meta.url), "utf8");
assert.match(viewerHtml, /<div id="roadmap-panel" hidden>/);
assert.match(viewerHtml, /<div id="roadmap-rows"><\/div>/);

console.log("roadmap-lines.test: OK");
