# Why every right-hand turn cusped (upstream nav2 bug)

## Symptom

Right turns produced forward-reverse-forward manoeuvres; the mirrored left turns
did not. Measured over 18 mirrored (radius, angle) pairs against
`ComputePathToPose`, planner-only, no controller involved:

| | left | right |
|---|---|---|
| cusps in the returned path | 0 / 18 | 15 / 18 (30 cusps total) |
| mean path length ÷ ideal arc | 0.960 | **1.280** (worst 1.96) |

The same asymmetry showed up while driving: `corner_right` took 36.2 s over
6.59 m with **105 direction reversals** and 36.4 % of the distance in reverse,
against `corner_left` at 9.5 s over 3.03 m with none.

## Cause

`AnalyticExpansion::tryAnalyticExpansion` (`nav2_smac_planner`) re-runs the
Reeds-Shepp expansion at turning radii from 1x to 4x the configured minimum and
keeps the best-scoring candidate. The scoring function measures **one** sample
gap and extrapolates it across the whole expansion:

```cpp
// Analytic expansions are consistently spaced
const float distance = hypotf(
  expansion[1].proposed_coords.x - expansion[0].proposed_coords.x,
  expansion[1].proposed_coords.y - expansion[0].proposed_coords.y);
for (auto iter = expansion.begin(); iter != expansion.end(); ++iter) {
  score += distance * (1.0 + weight * iter->node->getCost() / 252.0f);
}
```

The comment is half right. The samples are evenly spaced in **arc length**, not
in Euclidean distance — across a cusp the chord between two consecutive samples
collapses towards zero. So a candidate that *begins with a reversal* is scored at
a fraction of its true length, and the refinement loop prefers it.

Measured for the 0.60 m / 30 deg pair:

| | left | right |
|---|---|---|
| candidates surviving discretisation | 55 | 37 |
| ...that start with a cusp (`gap01` < 1 cell) | **0** | **7** |
| candidate the old scorer picks | 4 samples, true length 4.70 cells | 9 samples, true length **10.67** cells |
| its score | 6.27 | **3.80** |

The 10.67-cell cusped path scored `9 x 0.422 = 3.80` and beat the 4.70-cell
clean arc's `6.27`: **2.3x longer, scored 40 % cheaper.**

Left turns are unaffected only because no cusp-at-start candidate survives on
that side, so there is nothing for the broken score to prefer. (Reeds-Shepp
itself is symmetric — an isolated OMPL test gives bit-identical distances for
all 15 mirrored pairs. The left/right split in which candidates survive comes
from `static_cast<unsigned int>` truncation of the interpolated poses, harmless
on its own.)

## Fix

Accumulate each sample's own spacing instead of extrapolating the first gap.
The change lives on our fork:

**[vis81/navigation2, branch `picar2/1.3.12`](https://github.com/vis81/navigation2/tree/picar2/1.3.12)**

Two commits on upstream tag 1.3.12, deliberately kept apart:

| commit | what |
|---|---|
| `0411c11` | the fix — one file, `analytic_expansion.cpp`. Upstreamable as is (`ahead_by=1, files=1` against the tag) |
| `4ff413c` | `COLCON_IGNORE` in all 43 other packages, so a plain checkout builds only `nav2_smac_planner` |

Because the ignores are committed, no patching or post-processing is needed —
a bare clone is already correct, and the working tree stays clean:

```bash
make deps            # vcs import picks it up from .repos
make build
```

On the Pi, `sync2pi` skips `src/navigation2`, so the robot fetches it itself:

```bash
make nav2-overlay    # plain git; needs no ROS and no container
make build           # EXEC_ENV=docker on the Pi
```

**One branch serves both machines** even though the PC runs nav2 1.3.12 and the
Pi image 1.3.11. Between those tags `nav2_smac_planner` differs by exactly one
line — the `<version>` string in `package.xml` — and the only ABI-relevant
header it includes, `nav2_costmap_2d/inflation_layer.hpp`, gained a single
statement inside an existing inline body, with no change to class layout. Since
only `nav2_smac_planner` is built, every other header comes from
`/opt/ros/jazzy` on whichever machine is compiling. If a future nav2 changes
smac's own sources, rebase the branch onto the matching tag.

colcon's overlay puts the workspace plugin ahead of `/opt/ros/jazzy` at load
time. Verified in `/proc/<planner_server>/maps` on the robot:

```
/ws/build-docker/nav2_smac_planner/libnav2_smac_planner.so   <- workspace
/opt/ros/jazzy/lib/libnav2_costmap_2d_core.so                <- system
```

`src/` is gitignored, so the checkout is never committed.

## Verification

Planner sweep, 18 mirrored pairs, patched overlay in the real workspace:

| | before | after |
|---|---|---|
| right cusps | 30 | **0** |
| mean path ÷ arc, left | 0.960 | 0.960 |
| mean path ÷ arc, right | 1.280 | **0.963** |

Driven trials, ground_truth mode, n=2 each:

| scenario | before | after |
|---|---|---|
| `corner_right` | 36.2 s / 6.59 m / 105 reversals; 10.7 s / 3.07 m / 0 | 9.7 s / 3.02 m / 0; 9.8 s / 3.04 m / 0 |
| `turn_open_right` | 2 plan cusps, 2 reversals (both) | 0 cusps, 0 reversals (both) |
| `corner_left`, `turn_open_left` | unchanged | unchanged |

Regression check — scenarios where reversing is genuinely required keep it:

| scenario | before | after |
|---|---|---|
| `dead_end_reverse` | 5 plan cusps, succeeds | 5 plan cusps, succeeds |
| `doorway`, `open_straight` | — | identical to 3 decimals |

## Upstream

Present unchanged on `jazzy` (1.3.12) and `kilted`. Introduced by
[#3962](https://github.com/ros-navigation/navigation2/pull/3962) ("Prevent
analytic expansions from shortcutting Smac Planner feasible paths", 2024-01-23),
which added the refinement loop and this scoring function.

Not yet reported upstream. The fork branch is PR-ready as-is — it is one clean
commit on an upstream tag, touching one file.
