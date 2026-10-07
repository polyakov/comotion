import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import {
  ARC_CONFLICT_HOLD_TIMESTEPS,
  arcFrameDurationTimesteps,
  buildArcTimeline,
  configAtPath,
  findArcTimelineIndexForConflict,
  firstReachedConflict,
} from "../js/arc-playback.js";

const trace = {
  arc_visualization: {
    iterations: [
      {
        paths: [
          [[0], [1], [2], [3]],
          [[3], [2], [1], [0]],
        ],
        timesteps: 4,
        conflict_scan_completed: true,
        conflicts: [{ robot_i: 0, robot_j: 1, robots: [0, 1], timestep: 1 }],
        repairs: [{
          conflict_index: 0,
          robots: [0, 1],
          paths: [
            [[1], [1.5], [2]],
            [[2], [2.5], [1]],
          ],
        }],
      },
      {
        paths: [
          [[0], [1], [2], [3]],
          [[3], [2.5], [1.5], [0]],
        ],
        timesteps: 4,
        conflict_scan_completed: true,
        conflicts: [],
        repairs: [],
      },
    ],
  },
};

const timeline = buildArcTimeline(trace);
assert.equal(timeline.length, 9);
assert.deepEqual(
  timeline.map((frame) => frame.phase),
  ["paths", "paths", "repairs", "repairs", "repairs", "paths", "paths", "paths", "paths"]
);
assert.equal(firstReachedConflict(trace.arc_visualization.iterations[0], 0, 0), null);
assert.equal(firstReachedConflict(trace.arc_visualization.iterations[0], 0, 1)?.timestep, 1);
assert.deepEqual(configAtPath([[0], [1]], 20), [1]);
assert.equal(timeline.at(-1).solution, true);
assert.equal(ARC_CONFLICT_HOLD_TIMESTEPS, 10);
assert.equal(arcFrameDurationTimesteps(trace, timeline[0]), 1);
assert.equal(arcFrameDurationTimesteps(trace, timeline[1]), 10);
assert.equal(arcFrameDurationTimesteps(trace, timeline[1], 0.25), 2.5);
assert.equal(arcFrameDurationTimesteps(trace, timeline[1], 20), 200);
assert.equal(arcFrameDurationTimesteps(trace, timeline[2]), 1);

// findArcTimelineIndexForConflict: global timeline layout here is
// [it0/paths/t0, it0/paths/t1, it0/repairs/t0, it0/repairs/t1,
//  it0/repairs/t2, it1/paths/t0, it1/paths/t1, it1/paths/t2, it1/paths/t3]
// -- the conflict in iteration 0 fires at timestep 1, which is index 1
// (the raw path timestep) NOT the fix's own answer -- this is exactly the
// bug this function fixes (see conflict-panel click handler in app.js):
// the record's conflict_timestep must never be handed to setTimestep()
// directly once more than one iteration/phase is in play.
assert.equal(findArcTimelineIndexForConflict(timeline, 0, 0), 0);
assert.equal(findArcTimelineIndexForConflict(timeline, 0, 1), 1);
assert.equal(findArcTimelineIndexForConflict(timeline, 1, 0), 5);
assert.equal(findArcTimelineIndexForConflict(timeline, 1, 2), 7);
assert.equal(findArcTimelineIndexForConflict(timeline, 1, 3), 8);
assert.equal(
  findArcTimelineIndexForConflict(timeline, 0, 2),
  -1,
  "must not match the repairs-phase frame at global index 4 that happens to share timestep 2"
);
assert.equal(findArcTimelineIndexForConflict(timeline, 0, 99), -1, "timestep past this iteration's paths phase");
assert.equal(findArcTimelineIndexForConflict(timeline, 5, 0), -1, "iteration index that doesn't exist");
assert.equal(findArcTimelineIndexForConflict([], 0, 0), -1);

const viewerHtml = readFileSync(new URL("../index.html", import.meta.url), "utf8");
assert.match(viewerHtml, /<option value="50">50×<\/option>/);
assert.match(viewerHtml, /<option value="100">100×<\/option>/);

console.log("arc-playback.test: OK");
