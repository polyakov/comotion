/**
 * Pure helpers for the "Show planned paths" feature: deciding which robots
 * can be traced as a static polyline, and picking the point set to draw for
 * each playback mode. Kept free of Three.js/DOM so it can be unit-tested
 * directly, mirroring the js/arc-playback.js split.
 */

/** Only sphere robots have Cartesian [x, y, z] path points; other robot
 * types (URDF-based arms) store joint configs, which need forward
 * kinematics to trace and are out of scope. */
function isTraceableRobot(robot) {
  return !!robot && robot.robot_type === "sphere";
}

/** Indices into `robots` whose path can be drawn as a static line. */
function traceableRobotIndices(robots) {
  return (robots || []).reduce((indices, robot, i) => {
    if (isTraceableRobot(robot)) indices.push(i);
    return indices;
  }, []);
}

/** True when no robot in the result can be traced (checkbox should be disabled). */
function hasNoTraceableRobots(robots) {
  return traceableRobotIndices(robots).length === 0;
}

/**
 * Filter a raw path (array of per-timestep configs) down to well-formed
 * [x, y, z] points, defensive against malformed/short entries the same way
 * schema.js guards `configAt`.
 */
function sanitizePathPoints(path) {
  if (!Array.isArray(path)) return [];
  return path.filter(
    (p) =>
      Array.isArray(p) &&
      p.length >= 3 &&
      Number.isFinite(p[0]) &&
      Number.isFinite(p[1]) &&
      Number.isFinite(p[2])
  );
}

/** Path points to draw for one sphere robot in "solution" playback mode. */
function solutionPathPointsForRobot(robot) {
  return sanitizePathPoints(robot?.path);
}

/**
 * Path points to draw for one sphere robot in "arc" playback mode, for the
 * given iteration. Valid for both the "paths" and "repairs" phases of an
 * ARC frame, since both phases belong to the same iteration object and
 * repairs don't change the candidate path set being traced.
 */
function arcPathPointsForRobot(iteration, robotIndex) {
  return sanitizePathPoints(iteration?.paths?.[robotIndex]);
}

/**
 * True when the currently-drawn ARC path lines belong to a different
 * iteration than the one now displayed, and so must be rebuilt. Used to
 * redraw only on iteration change, not on every timestep tick within it.
 */
function arcPathLinesNeedRebuild(drawnIterationIndex, frameIterationIndex) {
  return drawnIterationIndex !== frameIterationIndex;
}

export {
  isTraceableRobot,
  traceableRobotIndices,
  hasNoTraceableRobots,
  sanitizePathPoints,
  solutionPathPointsForRobot,
  arcPathPointsForRobot,
  arcPathLinesNeedRebuild,
};
