function configAtPath(path, timestep) {
  if (!Array.isArray(path) || path.length === 0) return [];
  return path[Math.max(0, Math.min(timestep, path.length - 1))];
}

function hasArcVisualization(data) {
  return Array.isArray(data?.arc_visualization?.iterations) &&
    data.arc_visualization.iterations.length > 0;
}

const ARC_CONFLICT_HOLD_TIMESTEPS = 10;

function buildArcTimeline(data) {
  if (!hasArcVisualization(data)) return [];
  const frames = [];
  data.arc_visualization.iterations.forEach((iteration, iterationIndex) => {
    const pathEnd = Math.max(
      0,
      (Number(iteration.timesteps) ||
        Math.max(...iteration.paths.map((path) => path.length), 0)) - 1
    );
    const conflicts = iteration.conflicts || [];
    const conflictEnd = conflicts.length > 0
      ? Math.max(...conflicts.map((conflict) => Number(conflict.timestep) || 0))
      : pathEnd;
    const detectionEnd = Math.max(0, Math.min(pathEnd, conflictEnd));
    for (let timestep = 0; timestep <= detectionEnd; ++timestep) {
      frames.push({
        phase: "paths",
        iterationIndex,
        timestep,
        phaseEnd: detectionEnd,
        solution:
          iteration.conflict_scan_completed &&
          conflicts.length === 0 &&
          timestep === detectionEnd,
      });
    }

    if (conflicts.length > 0 && iteration.repairs.length > 0) {
      const repairEnd = Math.max(
        0,
        Math.max(
          ...iteration.repairs.flatMap((repair) =>
            repair.paths.map((path) => path.length)
          ),
          1
        ) - 1
      );
      for (let timestep = 0; timestep <= repairEnd; ++timestep) {
        frames.push({
          phase: "repairs",
          iterationIndex,
          timestep,
          phaseEnd: repairEnd,
        });
      }
    }
  });
  return frames;
}

function arcFrameDurationTimesteps(data, frame, playbackSpeed = 1) {
  if (!frame || frame.phase !== "paths") return 1;
  const iteration =
    data?.arc_visualization?.iterations?.[frame.iterationIndex];
  const isConflictTimestep = (iteration?.conflicts || []).some(
    (conflict) => Number(conflict.timestep) === frame.timestep
  );
  const speed =
    Number.isFinite(playbackSpeed) && playbackSpeed > 0 ? playbackSpeed : 1;
  return isConflictTimestep ? ARC_CONFLICT_HOLD_TIMESTEPS * speed : 1;
}

function conflictRobots(conflict) {
  if (Array.isArray(conflict.robots) && conflict.robots.length > 0)
    return conflict.robots;
  return [conflict.robot_i, conflict.robot_j];
}

function firstReachedConflict(iteration, robotIndex, timestep) {
  return (iteration.conflicts || []).reduce((earliest, conflict) => {
    if (
      !conflictRobots(conflict).includes(robotIndex) ||
      Number(conflict.timestep) > timestep
    ) {
      return earliest;
    }
    return !earliest ||
      Number(conflict.timestep) < Number(earliest.timestep)
      ? conflict
      : earliest;
  }, null);
}

/**
 * Global arcTimeline index of the "paths"-phase frame showing a raw
 * conflict-record's exact colliding configuration, or -1 if none matches.
 *
 * `iterationIndex`/`timestep` here are NOT interchangeable with a
 * conflict-record's own `conflict_sequence_index`/`conflict_timestep`
 * without translation first: `iterationIndex` is
 * `conflict_sequence_index - 1` (each ARC-history iteration corresponds
 * to exactly one detected-and-resolved conflict, in order — verified
 * against real data: iteration N's "paths" phase at its recorded
 * conflict's timestep reproduces that conflict's config_i/config_j
 * exactly), and `timestep` is the record's `conflict_timestep` as-is
 * (a "paths"-phase frame's local `timestep` field already IS the raw
 * per-robot path timestep — see buildArcTimeline()). The bug this fixes:
 * arcTimeline's *global* array index is a different, non-interchangeable
 * number from either of those — every iteration's "paths" phase restarts
 * its local `timestep` count at 0, so naively passing a raw
 * `conflict_timestep` straight to setTimestep() only ever lands correctly
 * by coincidence, for a conflict inside iteration 0.
 */
function findArcTimelineIndexForConflict(arcTimeline, iterationIndex, timestep) {
  for (let i = 0; i < arcTimeline.length; i++) {
    const frame = arcTimeline[i];
    if (
      frame.iterationIndex === iterationIndex &&
      frame.phase === "paths" &&
      frame.timestep === timestep
    ) {
      return i;
    }
  }
  return -1;
}

export {
  ARC_CONFLICT_HOLD_TIMESTEPS,
  arcFrameDurationTimesteps,
  buildArcTimeline,
  configAtPath,
  conflictRobots,
  findArcTimelineIndexForConflict,
  firstReachedConflict,
  hasArcVisualization,
};
