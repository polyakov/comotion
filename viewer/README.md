# CoMotion Path Viewer

The browser viewer displays path-result JSON produced by CoMotion applications.
It supports final-path playback for FCL, sphere, and VAMP runs, plus ARC process
playback for ARC-family planners.

## Create a Result

Build CoMotion using the main [installation instructions](../README.md#installation),
then write a result from any supported application:

```bash
./build/apps/panda_cage \
  --num-robots 8 \
  --output-endpoint-paths \
  --output-dir benchmarks/results/viewer_demo
```

This creates a start/goal-only result without running the planner. Use
`--output-paths` instead to run the default ARC planner and save its solution.

## Run the Viewer

Serve the repository root so robot models and meshes resolve correctly:

```bash
python3 -m http.server 8000
```

Open `http://localhost:8000/viewer/`.

Load a `*_result.json` file with either:

- **Load JSON** in the viewer; or
- `?file=<path>`, where the path is relative to the repository root.

For the example above:

```text
http://localhost:8000/viewer/?file=benchmarks/results/viewer_demo/panda_cage_n8_task0_seed0_EndpointPath_result.json
```

## ARC Process Playback

ARC process playback supports `arc`, `ao_arc`, and `parallel_arc`. Enable
both history flags when creating the result:

```bash
./build/apps/mobile_robot_2d_crossing \
  --scenario parallel \
  --num-robots 4 \
  --algorithm parallel_arc \
  --parallel-arc-worker-processes 2 \
  --output-paths \
  --track-arc-history \
  --output-dir benchmarks/results/viewer_demo
```

Loading the result selects ARC process mode automatically. Use the **Mode**
selector to switch between the ARC process and final path.

## Planned Paths

Check **Show planned paths** in the toolbar to draw each sphere robot's
full path as a static line, in that robot's palette color. Only
`robot_type: "sphere"` robots can be traced this way (URDF-based arms store
joint configs, not Cartesian points); if a loaded result has no sphere
robots, the checkbox is disabled with an explanatory tooltip. If a result
mixes sphere and non-sphere robots, the checkbox stays enabled and traces
only the sphere robots. The checkbox is available in both playback modes:
in **Solution path** mode it draws the full solved path, and in **ARC
process** mode it draws the currently displayed iteration's candidate
paths, redrawing only when scrubbing moves into a different iteration —
always in the robot's own palette color. The moving marker itself is also
in its own palette color by default in ARC mode, momentarily switching to a
shared state color when something ARC-specific is happening to that robot:
red while it's at a just-reached conflict, green for every robot once the
iteration's full path set is conflict-free, or a shared per-conflict-group
color while it's involved in a local repair.

The full path is always shown at once; scrubbing the timestep only moves
the marker along it and never truncates or redraws the line. Changing
**Palette** re-colors any path lines already on screen. Path lines are
unaffected by the lighting and geometry-mode toggles, and stay visible in
2D cross-section mode. There's no per-robot show/hide — one checkbox
controls all traced robots — and loading a new result file clears existing
path lines and resets the checkbox to unchecked.

## Roadmaps

`drrt`, `drrt_star`, and `ao_drrt` build a per-robot PRM* roadmap during
planning. Pass `--output-roadmaps` alongside `--output-paths` to embed each
sphere robot's roadmap (vertices + deduplicated undirected edges) in the
result:

```bash
./build/apps/mobile_robot_2d_crossing \
  --scenario parallel \
  --num-robots 4 \
  --algorithm drrt \
  --output-paths \
  --output-roadmaps \
  --output-dir benchmarks/results/viewer_demo
```

Loading such a result shows a **Roadmaps** panel (top-right) with one row
per sphere robot that has roadmap data, each with a color swatch matching
that robot's palette color and a checkbox. Checkboxes start unchecked —
there's no "show all" default — and each independently shows/hides that
one robot's roadmap as a graph of thin, subdued lines (opacity ~0.25, no
glow, no vertex markers even at the start/goal vertices) in the robot's
palette color, kept visually secondary to any planned-path line shown at
the same time. Only `robot_type: "sphere"` robots get roadmap data, for the
same reason as Planned Paths above; the panel is absent entirely (not just
empty) when the result has no roadmap data, e.g. a non-dRRT algorithm, a
dRRT run without `--output-roadmaps`, `--output-roadmaps` without
`--output-paths` (a silent no-op, matching `--track-arc-history`'s
behavior), or a dRRT run where no robot is a sphere.

Each robot's roadmap density is controlled by `--drrt-roadmap-size <n>`
(default varies by app/scenario) — the number of `vertices` per roadmap
entry equals that setting, so a larger value produces a denser graph and a
larger result file.

Roadmap visibility is independent of the planned-path checkbox and of
playback mode — roadmap data is static per robot (the planning structure,
not a per-timestep or per-ARC-iteration artifact), so it doesn't change
between **Solution path** and **ARC process** modes. Changing **Palette**
re-colors any roadmaps already on screen. Loading a new result file rebuilds
the panel from scratch and resets every checkbox to unchecked.

## Controls

| Key | Action |
|---|---|
| Left | Step backward |
| Right | Step forward |
| Shift+Left | Step backward 10 steps |
| Shift+Right | Step forward 10 steps |
| Home | First timestep |
| End | Last timestep |
| Space | Play or pause |

Run the viewer's unit tests with:

```bash
cd viewer && node --test tests/*.test.mjs
```
