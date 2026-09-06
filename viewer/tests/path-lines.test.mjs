import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import {
  arcPathLinesNeedRebuild,
  arcPathPointsForRobot,
  hasNoTraceableRobots,
  isTraceableRobot,
  sanitizePathPoints,
  solutionPathPointsForRobot,
  traceableRobotIndices,
} from "../js/path-lines.js";

// isTraceableRobot / traceableRobotIndices / hasNoTraceableRobots
assert.equal(isTraceableRobot({ robot_type: "sphere" }), true);
assert.equal(isTraceableRobot({ robot_type: "panda" }), false);
assert.equal(isTraceableRobot(null), false);

const mixedRobots = [
  { robot_type: "sphere" },
  { robot_type: "panda" },
  { robot_type: "sphere" },
  { robot_type: "planar3" },
];
assert.deepEqual(traceableRobotIndices(mixedRobots), [0, 2]);
assert.deepEqual(traceableRobotIndices([]), []);
assert.deepEqual(traceableRobotIndices(undefined), []);

assert.equal(hasNoTraceableRobots(mixedRobots), false);
assert.equal(hasNoTraceableRobots([{ robot_type: "panda" }]), true);
assert.equal(hasNoTraceableRobots([]), true);
assert.equal(hasNoTraceableRobots(undefined), true);
assert.equal(hasNoTraceableRobots(null), true);
assert.equal(
  hasNoTraceableRobots([{ robot_type: "sphere" }]),
  false,
  "single all-sphere result must leave the checkbox enabled"
);

// Malformed entries in `robots` (missing robot_type, null holes) must not
// break index bookkeeping or be mistaken for traceable robots.
assert.deepEqual(
  traceableRobotIndices([{ robot_type: "sphere" }, null, {}, undefined, { robot_type: "sphere" }]),
  [0, 4]
);

// sanitizePathPoints: keeps well-formed [x, y, z] entries, drops malformed ones.
assert.deepEqual(
  sanitizePathPoints([
    [0, 0, 0],
    [1, 2, 3],
    [1, 2], // too short
    "not-a-point",
    [1, 2, NaN], // non-finite
    null,
    [4, 5, 6, 7], // extra dims still fine, first three are used
  ]),
  [
    [0, 0, 0],
    [1, 2, 3],
    [4, 5, 6, 7],
  ]
);
assert.deepEqual(sanitizePathPoints(undefined), []);
assert.deepEqual(sanitizePathPoints(null), []);

// solutionPathPointsForRobot
assert.deepEqual(
  solutionPathPointsForRobot({ robot_type: "sphere", path: [[0, 0, 0], [1, 1, 1]] }),
  [[0, 0, 0], [1, 1, 1]]
);
assert.deepEqual(solutionPathPointsForRobot({ robot_type: "sphere" }), []);
assert.deepEqual(solutionPathPointsForRobot(undefined), []);
assert.deepEqual(solutionPathPointsForRobot(null), []);
// Malformed entries within an otherwise-valid path are dropped, not thrown on.
assert.deepEqual(
  solutionPathPointsForRobot({
    robot_type: "sphere",
    path: [[0, 0, 0], [1, "x", 1], [2, 2, 2]],
  }),
  [[0, 0, 0], [2, 2, 2]]
);

// arcPathPointsForRobot: reads iteration.paths[robotIndex], valid for both
// "paths" and "repairs" phases since both read the same iteration object.
const iteration = {
  paths: [
    [[0, 0, 0], [1, 0, 0]],
    [[2, 0, 0], [3, 0, 0]],
  ],
};
assert.deepEqual(arcPathPointsForRobot(iteration, 0), [[0, 0, 0], [1, 0, 0]]);
assert.deepEqual(arcPathPointsForRobot(iteration, 1), [[2, 0, 0], [3, 0, 0]]);
assert.deepEqual(arcPathPointsForRobot(iteration, 5), []);
assert.deepEqual(arcPathPointsForRobot(null, 0), []);
assert.deepEqual(arcPathPointsForRobot(undefined, 0), []);
assert.deepEqual(arcPathPointsForRobot({}, 0), []);
// Same iteration object backs both the "paths" and "repairs" phases of a
// frame, so repeated lookups for the same robot must be stable.
assert.deepEqual(
  arcPathPointsForRobot(iteration, 0),
  arcPathPointsForRobot(iteration, 0)
);
// Malformed points inside an iteration's per-robot path are sanitized too.
const iterationWithJunk = {
  paths: [[[0, 0, 0], [1, 2], [1, 1, 1]]],
};
assert.deepEqual(arcPathPointsForRobot(iterationWithJunk, 0), [[0, 0, 0], [1, 1, 1]]);

// arcPathLinesNeedRebuild: only true when the drawn iteration differs from
// the currently displayed one (redraw on iteration change, not every tick).
assert.equal(arcPathLinesNeedRebuild(null, 0), true);
assert.equal(arcPathLinesNeedRebuild(0, 0), false);
assert.equal(arcPathLinesNeedRebuild(0, 1), true);
assert.equal(arcPathLinesNeedRebuild(2, 2), false);
// Undefined "nothing drawn yet" state (e.g. before the first rebuild) must
// still trigger a rebuild rather than being coerced into matching frame 0.
assert.equal(arcPathLinesNeedRebuild(undefined, 0), true);
assert.equal(arcPathLinesNeedRebuild(undefined, undefined), false);

// isTraceableRobot: additional shapes seen from real result JSON.
assert.equal(isTraceableRobot({}), false, "missing robot_type is not traceable");
assert.equal(isTraceableRobot(undefined), false);
assert.equal(isTraceableRobot({ robot_type: "Sphere" }), false, "robot_type match is exact/case-sensitive, per schema");

// index.html wiring: checkbox present, toolbar-field convention, default unchecked.
const viewerHtml = readFileSync(new URL("../index.html", import.meta.url), "utf8");
assert.match(viewerHtml, /<input type="checkbox" id="show-paths-checkbox"\s*\/?>/);
assert.doesNotMatch(
  viewerHtml,
  /<input type="checkbox" id="show-paths-checkbox"[^>]*checked/
);

console.log("path-lines.test: OK");
