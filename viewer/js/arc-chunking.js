/**
 * Pure helpers for loading chunked arc_visualization.iterations (Tier 1 of
 * comotion-vadim/requirements/viewer_result_chunking.md). Kept free of
 * fetch()/DOM so it's unit-testable, mirroring the js/roadmap-lines.js and
 * js/conflict-panel.js split.
 *
 * A chunked result's arc_visualization has no inline "iterations" array;
 * instead it carries an "iteration_chunks" manifest (chunk filename +
 * iteration range + byte size) and an "iteration_summaries" array (per-
 * iteration lightweight metadata, unused by this Tier-1 loading path but
 * kept for a future on-demand-loading tier). See the design doc for why.
 */

/** True when `data.arc_visualization.iteration_chunks` is a non-empty
 * array — i.e. iterations must be fetched from chunk files rather than
 * read inline. False for every existing (unchunked) result, which is the
 * required backward-compatible default. */
function isChunkedArcVisualization(data) {
  return Array.isArray(data?.arc_visualization?.iteration_chunks) &&
    data.arc_visualization.iteration_chunks.length > 0;
}

/** Resolve one manifest entry's chunk filename against the base result
 * file's own URL — chunks are always siblings of the base file. */
function resolveChunkUrl(baseUrl, chunkFilename) {
  return new URL(chunkFilename, baseUrl).href;
}

/**
 * Concatenate already-fetched-and-parsed chunk bodies (each `{ iterations:
 * [...] }`, in the same order as `iteration_chunks`) into one flat
 * iterations array, in manifest order — regardless of the order the
 * fetches themselves settled in, since chunks are fetched in parallel.
 */
function mergeChunkedIterations(chunkBodies) {
  const merged = [];
  for (const body of chunkBodies) {
    const iterations = body?.iterations;
    if (Array.isArray(iterations)) merged.push(...iterations);
  }
  return merged;
}

export { isChunkedArcVisualization, resolveChunkUrl, mergeChunkedIterations };
