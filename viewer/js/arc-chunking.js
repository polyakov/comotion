/**
 * Pure helpers for on-demand loading of chunked arc_visualization.iterations
 * (Tier 2 of comotion-vadim/requirements/viewer_result_chunking.md). Kept
 * free of fetch()/DOM so it's unit-testable, mirroring the
 * js/roadmap-lines.js and js/conflict-panel.js split.
 *
 * A chunked result's arc_visualization has no inline "iterations" array;
 * instead it carries an "iteration_chunks" manifest (chunk filename +
 * iteration range + byte size) and an "iteration_summaries" array
 * (lightweight per-iteration metadata). js/app.js uses
 * buildArcTimelineFromSummaries() to know the complete timeline up front,
 * then fetches only a sliding 3-chunk window (current ± 1) as playback/
 * scrubbing moves, evicting chunks that fall outside it.
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
 * Which chunk (index into `manifest`) covers a given global iteration
 * index, or -1 if none does (out-of-range iterationIndex). Manifest
 * entries are contiguous and sorted by construction (the post-processing
 * script writes them in order), so a linear scan is fine — chunk counts
 * are small (tens, not thousands).
 */
function chunkIndexForIteration(manifest, iterationIndex) {
  for (let i = 0; i < manifest.length; i++) {
    if (
      iterationIndex >= manifest[i].iteration_start &&
      iterationIndex <= manifest[i].iteration_end
    ) {
      return i;
    }
  }
  return -1;
}

/**
 * Chunk indices to keep resident for a given "center" chunk: itself plus
 * one neighbor on each side, clipped to the manifest's valid range. This
 * is the sliding window on-demand loading uses both for what to fetch and
 * (checked against currently-loaded chunks) what to evict.
 */
function chunksInWindow(manifest, centerChunkIndex) {
  const window = [];
  for (let i = centerChunkIndex - 1; i <= centerChunkIndex + 1; i++) {
    if (i >= 0 && i < manifest.length) window.push(i);
  }
  return window;
}

/**
 * Same frame sequence buildArcTimeline() (js/arc-playback.js) would
 * produce, but computed from `iteration_summaries` — the lightweight,
 * always-present-up-front per-iteration metadata a chunked base file
 * carries — rather than from the heavy `iterations` array. This is what
 * lets the timeline (and therefore the slider's range and every frame's
 * phase/iterationIndex/timestep) be known immediately, before any chunk
 * has been fetched: chunk data is only needed once a specific frame is
 * actually about to be rendered.
 *
 * Must stay in exact lockstep with buildArcTimeline()'s logic — see
 * comotion-vadim/scripts/chunk_viewer_result.py's iteration_summary()
 * for how each summary field is derived from the same source data
 * buildArcTimeline() would otherwise read directly.
 */
function buildArcTimelineFromSummaries(summaries) {
  if (!Array.isArray(summaries) || summaries.length === 0) return [];
  const frames = [];
  summaries.forEach((summary, iterationIndex) => {
    const pathEnd = Math.max(0, (Number(summary.timesteps) || 0) - 1);
    const conflicts = summary.conflicts || [];
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
          summary.conflict_scan_completed === true &&
          conflicts.length === 0 &&
          timestep === detectionEnd,
      });
    }

    if (conflicts.length > 0 && Number(summary.repair_count) > 0) {
      const repairEnd = Math.max(0, Number(summary.repair_frame_count) || 0);
      for (let timestep = 0; timestep <= repairEnd; ++timestep) {
        frames.push({ phase: "repairs", iterationIndex, timestep, phaseEnd: repairEnd });
      }
    }
  });
  return frames;
}

export {
  isChunkedArcVisualization,
  resolveChunkUrl,
  chunkIndexForIteration,
  chunksInWindow,
  buildArcTimelineFromSummaries,
};
