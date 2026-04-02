"""
gen_demo_vis.py — realistic breadcrumb density visualisation for presentation.

Simulates 5 drones exploring a maze-like indoor arena (inspired by the SAFMC
2026 Cat Swarm Challenge booklet layout).  Key realism features:

  - Inner walls create corridors — drones navigate around them, not through.
  - Biased random-walk with heading persistence (VFH-like, not lawnmower).
  - Cumulative odometry drift (positions smear progressively from truth).
  - Gaussian blur on the density grid (smooths cell edges, realistic look).
  - Most cells end up visited (drift fills gaps) but corridors are brighter.

Run from laptop/:  python3 gen_demo_vis.py
"""

import sys
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from crumb_store import CrumbStore

# ─────────────────────────────────────────────────────────────────────────────
# Constants (match visualise.py / protocol.py)
# ─────────────────────────────────────────────────────────────────────────────

VFH_BINS    = 32
_BIN_WIDTH  = 2.0 * math.pi / VFH_BINS
_BIN_ANGLES = np.array([i * _BIN_WIDTH for i in range(VFH_BINS)])

DRONE_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3",
                "#ff7f00", "#a65628", "#f781bf", "#999999"]

OUTPUT_FILE = "demo_vis.png"

# ─────────────────────────────────────────────────────────────────────────────
# Arena wall layout — approximate, inspired by the SAFMC 2026 play-field
# diagram (Section 3.2 of the challenge booklet).
#
# Competition arena is 20 × 20 m; our setup.yaml arena is 14 × 17 m
# (x ∈ [0,14] North, y ∈ [−3,14] East).  Walls are scaled / adapted to
# fit our arena while preserving the flavour of the competition layout:
#   • Start area along the south edge (low x).
#   • Corridors between inner walls in the Known Search Area.
#   • A large enclosed Unknown Search Area deeper in the arena.
#
# Each wall is an axis-aligned rectangle (x_lo, x_hi, y_lo, y_hi).
# ─────────────────────────────────────────────────────────────────────────────

WALL_RECTS: list[tuple[float, float, float, float]] = [
    # ── South inner walls (corridor dividers near start area) ───────────
    (2.8, 3.3,  -2.5,  4.5),     # E-W wall, west side
    (2.8, 3.3,   8.5, 13.0),     # E-W wall, east side
    # gap at y ∈ [4.5, 8.5] is the main corridor entrance

    # ── Mid-field vertical stubs (pillar-like obstacles) ────────────────
    (5.0, 5.5,  -2.5,  1.5),     # west stub
    (5.0, 5.5,  11.5, 13.5),     # east stub

    # ── Unknown Search Area boundary box (approx 5 × 6 m) ──────────────
    # West wall  (N-S line at y ≈ 3.5)
    (6.5, 11.5,  3.0,  3.5),
    # East wall  (N-S line at y ≈ 10.5)
    (6.5, 11.5, 10.0, 10.5),
    # South wall (E-W line at x ≈ 6.5) — with gap at y ∈ [5.5, 8.0]
    (6.2, 6.7,   3.5,  5.5),
    (6.2, 6.7,   8.0, 10.0),
    # North wall (E-W line at x ≈ 11.5) — with small gap y ∈ [6, 7.5]
    (11.2, 11.7,  3.5,  6.0),
    (11.2, 11.7,  7.5, 10.5),

    # ── Far-north wall stubs ────────────────────────────────────────────
    (12.5, 13.0, -2.0,  2.0),
    (12.5, 13.0, 11.5, 13.5),
]

# Inflate for collision detection (drone radius ~ 0.25 m)
_WALL_MARGIN = 0.30


def _point_in_wall(x: float, y: float) -> bool:
    """True if (x, y) is inside any inflated wall rectangle."""
    for (x0, x1, y0, y1) in WALL_RECTS:
        if (x0 - _WALL_MARGIN <= x <= x1 + _WALL_MARGIN and
                y0 - _WALL_MARGIN <= y <= y1 + _WALL_MARGIN):
            return True
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Trail generation — biased random walk with wall avoidance and drift
# ─────────────────────────────────────────────────────────────────────────────

def gen_trail_with_drift(
    start: tuple[float, float],
    arena: dict,
    zone_y: tuple[float, float],
    rng: np.random.Generator,
    n_steps: int = 4000,
) -> list[tuple[float, float]]:
    """
    Simulate a single drone exploring the arena.

    The drone navigates with heading persistence toward randomly sampled
    goals within its assigned y-zone, bouncing off arena boundaries and
    walls.  Recorded positions include cumulative odometry drift (a 2-D
    Brownian offset that grows with distance travelled).

    Returns a list of *recorded* (drifted) (x, y) positions.
    """
    ax_min = float(arena["min_x"])
    ax_max = float(arena["max_x"])
    ay_min = float(arena["min_y"])
    ay_max = float(arena["max_y"])
    zone_lo, zone_hi = float(zone_y[0]), float(zone_y[1])

    STEP        = 0.07    # m per tick (≈ 0.5 m/s at ~7 Hz sampling)
    MAX_TURN    = 0.30    # rad per tick — heading persistence
    HEAD_NOISE  = 0.10    # rad per tick — steering jitter
    DRIFT_SIGMA = 0.015   # m per tick — drift accumulation rate
    MARGIN      = 0.30    # arena-boundary keep-out (m)

    x, y = float(start[0]), float(start[1])
    heading = rng.uniform(-0.3, 0.3)       # roughly northward (+x)

    drift_x, drift_y = 0.0, 0.0
    recorded: list[tuple[float, float]] = []

    # First goal: fly forward into the arena
    goal_x = float(rng.uniform(3.0, 6.0))
    goal_y = float(rng.uniform(
        max(ay_min + 1, zone_lo + 0.5),
        min(ay_max - 1, zone_hi - 0.5),
    ))
    goal_timer = 200

    for _ in range(n_steps):
        # ── Pick a new exploration goal when needed ──────────────────────
        if goal_timer <= 0:
            for _try in range(20):
                gx = float(rng.uniform(ax_min + 0.5, ax_max - 0.5))
                gy = float(rng.uniform(
                    np.clip(zone_lo + 0.3, ay_min + 0.5, ay_max - 0.5),
                    np.clip(zone_hi - 0.3, ay_min + 0.5, ay_max - 0.5),
                ))
                if not _point_in_wall(gx, gy):
                    goal_x, goal_y = gx, gy
                    break
            goal_timer = int(rng.uniform(80, 300))

        goal_timer -= 1
        if math.hypot(goal_x - x, goal_y - y) < 0.5:
            goal_timer = 0                        # reached → pick next

        # ── Steer toward goal (limited turn rate) ────────────────────────
        desired = math.atan2(goal_y - y, goal_x - x)
        diff = (desired - heading + math.pi) % (2 * math.pi) - math.pi
        heading += (float(np.clip(diff, -MAX_TURN, MAX_TURN))
                    + float(rng.normal(0, HEAD_NOISE)))

        # ── Propose next position ────────────────────────────────────────
        nx = x + STEP * math.cos(heading)
        ny = y + STEP * math.sin(heading)

        # Arena-boundary bounce
        if nx < ax_min + MARGIN or nx > ax_max - MARGIN:
            heading = math.pi - heading + float(rng.normal(0, 0.25))
            nx = float(np.clip(nx, ax_min + MARGIN, ax_max - MARGIN))
            goal_timer = 0
        if ny < ay_min + MARGIN or ny > ay_max - MARGIN:
            heading = -heading + float(rng.normal(0, 0.25))
            ny = float(np.clip(ny, ay_min + MARGIN, ay_max - MARGIN))
            goal_timer = 0

        # Wall collision → stay put and turn away
        if _point_in_wall(nx, ny):
            heading += float(rng.uniform(0.8, 2.5)) * (1.0 if rng.random() > 0.5 else -1.0)
            goal_timer = 0
            # Don't update x, y — stay at current position
        else:
            x, y = nx, ny

        # ── Accumulate odometry drift (Brownian) ─────────────────────────
        drift_x += float(rng.normal(0, DRIFT_SIGMA))
        drift_y += float(rng.normal(0, DRIFT_SIGMA))

        recorded.append((x + drift_x, y + drift_y))

    return recorded


# ─────────────────────────────────────────────────────────────────────────────
# Gaussian blur (with scipy fallback)
# ─────────────────────────────────────────────────────────────────────────────

def _blur(Z: np.ndarray, sigma: float) -> np.ndarray:
    try:
        from scipy.ndimage import gaussian_filter
        return gaussian_filter(Z, sigma=sigma)
    except ImportError:
        # Simple separable 1-D convolution fallback
        k = int(math.ceil(3 * sigma))
        kern = np.exp(-0.5 * (np.arange(-k, k + 1) / sigma) ** 2)
        kern /= kern.sum()
        out = np.apply_along_axis(
            lambda row: np.convolve(row, kern, mode="same"), 0, Z)
        out = np.apply_along_axis(
            lambda row: np.convolve(row, kern, mode="same"), 1, out)
        return out


# ─────────────────────────────────────────────────────────────────────────────
# VFH fake data
# ─────────────────────────────────────────────────────────────────────────────

def _fake_vfh(idx: int, heading_rad: float) -> list[bool]:
    rng = np.random.default_rng(idx * 17 + 3)
    blocked: list[bool] = []
    for b in range(VFH_BINS):
        angle = b * _BIN_WIDTH
        diff = abs((angle - heading_rad + math.pi) % (2 * math.pi) - math.pi)
        p = 0.04 + 0.55 * (diff / math.pi) ** 2
        blocked.append(bool(rng.random() < p))
    return blocked


# ─────────────────────────────────────────────────────────────────────────────
# Figure construction
# ─────────────────────────────────────────────────────────────────────────────

def build_figure(
    Z_norm: np.ndarray,
    x_edges: np.ndarray,
    y_edges: np.ndarray,
    drone_ids: list[int],
    last_pos: dict[int, tuple[float, float]],
    headings: dict[int, float],
    vfh_data: dict[int, list[bool]],
    nav_tags: dict,
    arena: dict,
) -> plt.Figure:
    x_min, x_max = arena["min_x"], arena["max_x"]
    y_min, y_max = arena["min_y"], arena["max_y"]
    n_drones = len(drone_ids)

    fig_w = max(12, n_drones * 2.8)
    fig = plt.figure(figsize=(fig_w, fig_w + 2.5), facecolor="#111111")
    gs = gridspec.GridSpec(
        2, n_drones,
        height_ratios=[fig_w, 2.5],
        hspace=0.38, wspace=0.45,
    )

    # ── Map ──────────────────────────────────────────────────────────────
    ax = fig.add_subplot(gs[0, :], facecolor="#1a1a1a")
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)", color="white")
    ax.set_ylabel("North (m)", color="white")
    ax.set_title("Fleet Exploration — Breadcrumb Density Grid",
                 color="white", fontsize=14)
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_edgecolor("#555555")
    margin = 0.6
    ax.set_xlim(y_min - margin, y_max + margin)
    ax.set_ylim(x_min - margin, x_max + margin)

    # Density heatmap
    mesh = ax.pcolormesh(
        y_edges, x_edges, Z_norm,
        cmap="hot", vmin=0.0, vmax=1.0, shading="flat", zorder=1,
    )
    cb = fig.colorbar(mesh, ax=ax, label="Relative visit density")
    cb.ax.yaxis.label.set_color("white")
    cb.ax.tick_params(colors="white")

    # Arena boundary
    ax.add_patch(patches.Rectangle(
        (y_min, x_min), y_max - y_min, x_max - x_min,
        linewidth=2, edgecolor="cyan", facecolor="none",
        linestyle="--", zorder=3,
    ))

    # Wall outlines (semi-transparent so the density shows through)
    for (wx0, wx1, wy0, wy1) in WALL_RECTS:
        ax.add_patch(patches.Rectangle(
            (wy0, wx0), wy1 - wy0, wx1 - wx0,
            linewidth=1.2, edgecolor="#66cccc", facecolor="#222222",
            alpha=0.55, zorder=2,
        ))

    # Nav tags
    for tag_id, pos in nav_tags.items():
        ax.plot(pos["map_y"], pos["map_x"], "c^", markersize=10, zorder=4)
        ax.annotate(
            f"T{tag_id}", (pos["map_y"], pos["map_x"]),
            textcoords="offset points", xytext=(5, 5),
            fontsize=8, color="cyan", zorder=4,
        )

    # Drone markers
    for i, did in enumerate(drone_ids):
        color = DRONE_COLORS[i % len(DRONE_COLORS)]
        lx, ly = last_pos[did]
        ax.plot(ly, lx, "o", color=color, markersize=12,
                markeredgecolor="white", markeredgewidth=1.5,
                zorder=5, label=f"Drone {did}")

    ax.legend(loc="upper right", fontsize=9,
              facecolor="#222222", labelcolor="white", framealpha=0.8)

    # ── VFH polar subplots ───────────────────────────────────────────────
    for i, did in enumerate(drone_ids):
        color = DRONE_COLORS[i % len(DRONE_COLORS)]
        pax = fig.add_subplot(gs[1, i], projection="polar",
                              facecolor="#1a1a1a")
        pax.set_theta_zero_location("N")
        pax.set_theta_direction(-1)
        pax.set_ylim(0, 1)
        pax.set_yticks([])
        pax.set_title(f"D{did} VFH", fontsize=8, pad=3, color="white")
        pax.tick_params(colors="#888888", labelsize=6)
        pax.spines["polar"].set_color("#444444")

        blocked = vfh_data.get(did, [False] * VFH_BINS)
        bar_colors = ["#e41a1c" if b else "#4daf4a" for b in blocked]
        pax.bar(_BIN_ANGLES, np.ones(VFH_BINS), width=_BIN_WIDTH,
                align="edge", color=bar_colors, alpha=0.8, linewidth=0)

        h = headings.get(did, 0.0)
        pax.annotate("", xy=(h, 0.9), xytext=(h, 0.0),
                     arrowprops=dict(arrowstyle="->", color=color, lw=2))

    return fig


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    cfg_path = Path(__file__).parent / "setup.yaml"
    cfg      = yaml.safe_load(cfg_path.read_text())
    arena    = cfg["arena"]
    nav_tags = cfg.get("nav_tags", {})
    drone_ids = sorted(int(d) for d in cfg["drones"])
    starts = {
        int(did): (info["start_x"], info["start_y"])
        for did, info in cfg["drones"].items()
    }

    # Zone assignment — divide the East-axis (y) into overlapping bands
    ay_min, ay_max = arena["min_y"], arena["max_y"]
    n = len(drone_ids)
    band = (ay_max - ay_min) / n
    zones: dict[int, tuple[float, float]] = {}
    for i, did in enumerate(drone_ids):
        lo = ay_min + i * band - band * 0.45     # generous overlap
        hi = ay_min + (i + 1) * band + band * 0.45
        zones[did] = (max(ay_min, lo), min(ay_max, hi))

    # Generate trails with cumulative drift
    all_recorded: dict[int, list[tuple[float, float]]] = {}
    for did in drone_ids:
        drone_rng = np.random.default_rng(did * 997 + 13)
        all_recorded[did] = gen_trail_with_drift(
            starts[did], arena, zones[did], drone_rng, n_steps=4000,
        )

    # Populate CrumbStore
    store = CrumbStore(
        cell_size=0.15,
        arena_bounds=(arena["min_x"], arena["max_x"],
                      arena["min_y"], arena["max_y"]),
    )
    for did, positions in all_recorded.items():
        for rx, ry in positions:
            store.add_position(did, rx, ry)

    # ── Build density grid, blur, normalise ──────────────────────────────
    cell = store._cell_size
    x_min, x_max = arena["min_x"], arena["max_x"]
    y_min, y_max = arena["min_y"], arena["max_y"]
    ix0, ix1 = math.floor(x_min / cell), math.ceil(x_max / cell)
    iy0, iy1 = math.floor(y_min / cell), math.ceil(y_max / cell)
    n_x, n_y = ix1 - ix0, iy1 - iy0
    x_edges = np.linspace(ix0 * cell, ix1 * cell, n_x + 1)
    y_edges = np.linspace(iy0 * cell, iy1 * cell, n_y + 1)

    Z = np.zeros((n_x, n_y), dtype=np.float32)
    for (gx, gy), count in store.grid_snapshot().items():
        xi, yi = gx - ix0, gy - iy0
        if 0 <= xi < n_x and 0 <= yi < n_y:
            Z[xi, yi] = float(count)

    Z = _blur(Z, sigma=2.5)
    mx = Z.max()
    Z_norm = Z / mx if mx > 0 else Z

    # ── Drone headings (from last few recorded positions) ────────────────
    headings: dict[int, float] = {}
    last_pos: dict[int, tuple[float, float]] = {}
    for did in drone_ids:
        rec = all_recorded[did]
        last_pos[did] = rec[-1]
        if len(rec) > 20:
            dx = rec[-1][0] - rec[-20][0]
            dy = rec[-1][1] - rec[-20][1]
            headings[did] = math.atan2(dy, dx)
        else:
            headings[did] = 0.0

    vfh_data = {did: _fake_vfh(i, headings[did])
                for i, did in enumerate(drone_ids)}

    # ── Render and save ──────────────────────────────────────────────────
    fig = build_figure(
        Z_norm, x_edges, y_edges,
        drone_ids, last_pos, headings, vfh_data,
        nav_tags, arena,
    )

    out = Path(__file__).parent / OUTPUT_FILE
    fig.savefig(out, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
