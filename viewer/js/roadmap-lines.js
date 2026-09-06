/**
 * Pure helpers for the per-robot roadmap visualization feature: deciding
 * whether roadmap data is present, and sanitizing/filtering the roadmap
 * entries to render. Kept free of Three.js/DOM so it can be unit-tested
 * directly, mirroring the js/path-lines.js split.
 */

/** True when `resultData.roadmaps` is a non-empty array (panel-presence check). */
function hasRoadmapData(resultData) {
  return Array.isArray(resultData?.roadmaps) && resultData.roadmaps.length > 0;
}

/**
 * A well-formed [x, y, z] vertex, or null for a malformed one. Vertices are
 * kept at their original index (not filtered out positionally) so edge
 * indices `[i, j]` from the raw data keep pointing at the right slot; a
 * malformed vertex instead makes any edge touching it get dropped in
 * `sanitizeRoadmapEntry`, the same "drop the bad point, don't shift the
 * rest" defensiveness as path-lines.js's `sanitizePathPoints`.
 */
function sanitizeRoadmapVertex(vertex) {
  return Array.isArray(vertex) &&
    vertex.length >= 3 &&
    Number.isFinite(vertex[0]) &&
    Number.isFinite(vertex[1]) &&
    Number.isFinite(vertex[2])
    ? vertex
    : null;
}

/** Raw `vertices` array to a same-length array of sanitized vertices/nulls. */
function sanitizeRoadmapVertices(vertices) {
  if (!Array.isArray(vertices)) return [];
  return vertices.map(sanitizeRoadmapVertex);
}

/**
 * Raw `edges` filtered to well-formed [i, j] integer pairs that index inside
 * `vertexCount` and whose endpoints both survived sanitization.
 */
function sanitizeRoadmapEdges(edges, sanitizedVertices) {
  if (!Array.isArray(edges)) return [];
  const vertexCount = sanitizedVertices.length;
  return edges.filter(
    (edge) =>
      Array.isArray(edge) &&
      edge.length === 2 &&
      Number.isInteger(edge[0]) &&
      Number.isInteger(edge[1]) &&
      edge[0] >= 0 &&
      edge[0] < vertexCount &&
      edge[1] >= 0 &&
      edge[1] < vertexCount &&
      sanitizedVertices[edge[0]] !== null &&
      sanitizedVertices[edge[1]] !== null
  );
}

/**
 * One `resultData.roadmaps` entry, sanitized for rendering, or null if the
 * entry itself is too malformed to use (missing/non-integer robot_index).
 */
function sanitizeRoadmapEntry(entry) {
  if (!entry || !Number.isInteger(entry.robot_index)) return null;
  const vertices = sanitizeRoadmapVertices(entry.vertices);
  const edges = sanitizeRoadmapEdges(entry.edges, vertices);
  return { robot_index: entry.robot_index, vertices, edges };
}

/**
 * Usable roadmap entries to render, from `resultData.roadmaps`. Each entry
 * has `robot_index`, sanitized `vertices` (malformed points become `null`
 * but keep their index), and sanitized `edges` (only pairs whose endpoints
 * are valid, in-bounds vertices).
 */
function roadmapEntries(resultData) {
  if (!hasRoadmapData(resultData)) return [];
  return resultData.roadmaps
    .map(sanitizeRoadmapEntry)
    .filter((entry) => entry !== null);
}

/** Display name for an entry's robot, e.g. for a panel row label. */
function robotNameForEntry(resultData, entry) {
  const robot = resultData?.robots?.[entry?.robot_index];
  return robot?.name || `Robot ${entry?.robot_index}`;
}

export {
  hasRoadmapData,
  sanitizeRoadmapVertices,
  sanitizeRoadmapEdges,
  sanitizeRoadmapEntry,
  roadmapEntries,
  robotNameForEntry,
};
