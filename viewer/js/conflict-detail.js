/**
 * Standalone conflict-detail page. Opened as a separate tab from a row's
 * link in the main viewer's Conflicts panel (js/app.js's
 * addConflictPanelRow); takes the conflict-record JSON file's URL via a
 * `?file=` query parameter and shows its content.
 *
 * Starting point is deliberately minimal: fetch the file and show the raw
 * JSON, pretty-printed, plus the one-line summary the main panel already
 * shows. Structured/formatted rendering of specific fields can build on
 * top of this later without changing how the page is reached.
 */

import { conflictSummaryLine, isWellFormedConflictRecord } from "./conflict-panel.js";

function showError(message) {
  const error = document.getElementById("error");
  error.textContent = message;
  error.hidden = false;
}

async function main() {
  const titleEl = document.getElementById("title");
  const summaryEl = document.getElementById("summary");
  const dumpEl = document.getElementById("json-dump");

  const file = new URLSearchParams(window.location.search).get("file");
  if (!file) {
    titleEl.textContent = "No conflict record specified";
    showError('Open this page with a "?file=<path>" query parameter, e.g. via a link from the Conflicts panel in the main viewer.');
    return;
  }

  titleEl.textContent = file;
  document.title = `Conflict Detail — ${file}`;

  let response;
  try {
    response = await fetch(file);
  } catch (err) {
    showError(`Failed to fetch ${file}: ${err.message}`);
    return;
  }
  if (!response.ok) {
    showError(`Failed to fetch ${file}: HTTP ${response.status}`);
    return;
  }

  const text = await response.text();
  let record;
  try {
    record = JSON.parse(text);
  } catch (err) {
    showError(`Response from ${file} was not valid JSON: ${err.message}`);
    dumpEl.textContent = text;
    return;
  }

  if (isWellFormedConflictRecord(record)) {
    summaryEl.textContent = conflictSummaryLine(record);
  }
  dumpEl.textContent = JSON.stringify(record, null, 2);
}

main();
