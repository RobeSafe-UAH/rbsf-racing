#!/usr/bin/env python3
"""
Write the track catalog: docs/MAP_CATALOG.md plus one preview image per track.

Reads every track in src/base_system/rbsf_worlds/tracks/ (the source tree, so new tracks need
no rebuild) and measures it from its own files: the map footprint, the reference lines (length,
corridor width, driving direction, speed range), the cones and bridges its yaml declares and the
per-track controller tunings in rbsf_path_tracking. The previews show the map the simulators
load (cones drawn in), the bridges, the reference lines and the start/finish point with the
driving direction.

Where each track comes from is not in its files: add a SOURCES entry for every new track (the
script refuses tracks without one). Rerun it after adding or regenerating a track. Run it in the
dev container with the workspace sourced (rbsf_worlds, numpy, opencv, matplotlib):
    python3 scripts/tracks/catalog.py
"""

import argparse
import pathlib
import sys
import textwrap

import cv2
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from rbsf_worlds.centerline import signed_area
from rbsf_worlds.cones import cone_positions, draw_cones
from rbsf_worlds.paths import arc_lengths, load_waypoints
import yaml

plt.switch_backend('Agg')  # headless: figures are only saved

ROOT = pathlib.Path(__file__).resolve().parents[2]
TRACKS = ROOT / 'src/base_system/rbsf_worlds/tracks'
TUNINGS = ROOT / 'src/controller/rbsf_path_tracking/config/tracks'
OUT = ROOT / 'docs'
IMAGES = 'map_catalog'  # preview directory, relative to the catalog
PAD = 1.0  # m of map shown around the free space
COLORS = {'centerline': '#2a9d55', 'raceline': '#2e6fbd', 'start': '#d1495b',
          'cone': '#ff6a13', 'bridge': '#8a5cc2'}

INTRO = (
    'Every track in `rbsf_worlds`. Both simulators and the raceline behaviour take any of them as '
    '`track:=<name>`. File formats, provenance and the commands that generated each track: '
    '[`tracks/README.md`](../src/base_system/rbsf_worlds/tracks/README.md). Other tracks mapped '
    'by the car stay in their session (`maps/<stamp>/track/`, `track:=<yaml path>`) and are not '
    'listed here.')
LEGEND = (
    'Lengths are of the closed reference lines. The width is the corridor across the centerline '
    '(wall to wall), its narrowest and widest point. The start is the first point of the '
    'reference line, the start/finish line.')

# (source, notes) per track, in catalog order. tracks/README.md has the full provenance and the
# commands that generated each track.
SOURCES = {
    'icra2025': (
        'ICRA 2025 RoboRacer competition, Atlanta',
        'From [Noce99/AuRoRus](https://github.com/Noce99/AuRoRus) `maps/Atlanta_2025` (UniBo, '
        'MIT/Apache-2.0), whose car won that race. The start/finish line and the driving '
        'direction are theirs.'),
    'icra2026_classic': (
        'ICRA 2026 RoboRacer competition, Vienna: Classic Cup',
        "The organisers' map. Dashed walls are open borders (gaps under 0.4 m); the left side "
        'splits into two lanes and the line takes the inner one.'),
    'icra2026_master': (
        'ICRA 2026 RoboRacer competition, Vienna: Master Cup',
        "The organisers' map, with a 0.7 m bridge over a five-way crossing (3D in MVSim, a flat "
        'crossing in f1tenth_gym). The line passes the crossing twice below the deck and once '
        'over it.'),
    'iros2026': (
        'IROS 2026 RoboRacer competition',
        "Generated from the organisers' drawing (thin wall lines)."),
    'iv2026': (
        'IEEE IV 2026 simulation map',
        "Generated from the organisers' drawing (1 px wall lines). The drawing gives no driving "
        'direction: counter-clockwise by choice.'),
    'eps_circuit': (
        'Mapped by our car',
        'Session `20261021_122444`, Cartographer.'),
    'eps_oval_single_wall': (
        'Built from one of our mapping sessions',
        'Session `20261007_122948`, Cartographer. Bays open off the loop, so centring '
        'controllers drive into them.'),
    'levine': (
        'f1tenth_gym_ros `maps/levine`',
        'Building corridors. Four cone rows close the corridors that lead off the loop.'),
    'spielberg': (
        'f1tenth_gym_ros `maps/Spielberg`',
        'The only track with a raceline (with a speed profile). Its centerline widths are a '
        'nominal 1.1 m per side, not measured from the map.'),
}


class Track:
    """One track directory: its map, reference lines, cones, bridges and tunings."""

    def __init__(self, directory):
        """Load the track in `directory`."""
        self.name = directory.name
        self.yaml = directory / f'{self.name}.yaml'
        self.spec = yaml.safe_load(self.yaml.read_text()) or {}
        self.resolution = float(self.spec['resolution'])
        self.origin = [float(v) for v in self.spec['origin'][:2]]
        self.cones = cone_positions(self.spec.get('cones'))
        self.bridges = self.spec.get('bridges') or []
        image = cv2.imread(str(directory / self.spec['image']), cv2.IMREAD_GRAYSCALE)
        self.image = draw_cones(image, self.cones, self.resolution, self.origin,
                                self.spec.get('negate', 0))
        self.lines = {}
        for kind in ('centerline', 'raceline'):
            path = directory / (self.spec.get(kind) or f'{self.name}_{kind}.csv')
            if path.exists():
                xy, speeds = load_waypoints(path)
                widths = None
                if kind == 'centerline':
                    data = np.loadtxt(path, delimiter=',', comments='#', ndmin=2)
                    widths = data[:, 2] + data[:, 3]
                self.lines[kind] = {'xy': xy, 'speeds': speeds, 'length': arc_lengths(xy)[1],
                                    'widths': widths}
        self.tunings = sorted(p.name for p in TUNINGS.glob(f'{self.name}_*.yaml'))

    def extent(self):
        """(x0, x1, y0, y1) of the whole image, in map coordinates."""
        height, width = self.image.shape
        return (self.origin[0], self.origin[0] + width * self.resolution,
                self.origin[1], self.origin[1] + height * self.resolution)

    def free_box(self):
        """
        Return (x0, x1, y0, y1) of the drivable space, in map coordinates.

        That is the free region around the start (4-connected, cones included): a map may be
        free outside its walls too (levine's unknown area, spielberg's background).
        """
        free = (255 - self.image.astype(float)) / 255.0 < float(self.spec['free_thresh'])
        _, labels = cv2.connectedComponents(free.astype(np.uint8), connectivity=4)
        height = self.image.shape[0]
        col, row = ((self.reference()['xy'][0] - self.origin) / self.resolution).astype(int)
        rows, cols = np.nonzero(labels == labels[height - 1 - row, col])
        return (self.origin[0] + cols.min() * self.resolution,
                self.origin[0] + (cols.max() + 1) * self.resolution,
                self.origin[1] + (height - rows.max() - 1) * self.resolution,
                self.origin[1] + (height - rows.min()) * self.resolution)

    def reference(self):
        """Return the line the stack drives by default: the raceline if there is one."""
        return self.lines.get('raceline') or self.lines.get('centerline')


def bridge_patches(bridge):
    """Ramps and deck of a `bridges:` entry as rectangles along its axis."""
    start, end = np.array(bridge['start'], float), np.array(bridge['end'], float)
    axis = (end - start) / np.linalg.norm(end - start)
    normal = np.array([-axis[1], axis[0]])
    half = float(bridge['width']) / 2.0
    s0, s1 = bridge['deck']
    patches = []
    for a, b, alpha in ((0.0, s0, 0.25), (s0, s1, 0.55), (s1, np.linalg.norm(end - start), 0.25)):
        corners = [start + axis * s + normal * n for s, n in
                   ((a, -half), (b, -half), (b, half), (a, half))]
        patches.append(mpatches.Polygon(corners, closed=True, fc=COLORS['bridge'], ec='none',
                                        alpha=alpha))
    return patches


def preview(track, path):
    """Save the track preview: map, bridges, cones, lines, start and direction."""
    x0, x1, y0, y1 = track.free_box()
    x0, x1, y0, y1 = x0 - PAD, x1 + PAD, y0 - PAD, y1 + PAD
    longest = 7.0  # in
    scale = longest / max(x1 - x0, y1 - y0)
    fig, ax = plt.subplots(figsize=(max(3.5, (x1 - x0) * scale) + 0.8,
                                    max(3.5, (y1 - y0) * scale) + 0.6))
    ax.imshow(track.image, cmap='gray', vmin=0, vmax=255, extent=track.extent(),
              interpolation='nearest')
    for bridge in track.bridges:
        for patch in bridge_patches(bridge):
            ax.add_patch(patch)
        for row in bridge.get('under_cones') or []:
            ax.plot([row[0], row[2]], [row[1], row[3]], c=COLORS['cone'], lw=1.5, ls=':')
    if track.cones:
        cones = np.array(track.cones)
        ax.scatter(cones[:, 0], cones[:, 1], s=4, c=COLORS['cone'], lw=0, zorder=3)
    for kind, line in track.lines.items():
        loop = np.vstack([line['xy'], line['xy'][:1]])
        ax.plot(loop[:, 0], loop[:, 1], c=COLORS[kind], lw=1.4, label=kind)
    xy = track.reference()['xy']
    s, length = arc_lengths(xy)
    ahead = np.searchsorted(s, min(0.06 * max(x1 - x0, y1 - y0), 0.1 * length))  # arrow tip
    ax.annotate('', xy=xy[ahead], xytext=xy[0], zorder=4,
                arrowprops={'arrowstyle': '-|>', 'color': COLORS['start'], 'lw': 2.2,
                            'mutation_scale': 16})
    ax.plot(*xy[0], 'o', c=COLORS['start'], ms=6, zorder=4, label='start, driving direction')
    handles, _ = ax.get_legend_handles_labels()
    if track.cones:
        handles.append(plt.Line2D([], [], ls='none', marker='o', ms=4, c=COLORS['cone'],
                                  label='cones'))
    if track.bridges:
        handles.append(mpatches.Patch(fc=COLORS['bridge'], alpha=0.55, label='bridge deck'))
    ax.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, -0.12),
              ncol=len(handles), fontsize=8, frameon=False)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(track.name)
    ax.tick_params(labelsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=90, bbox_inches='tight')
    plt.close(fig)


def facts(track):
    """Summary values of a track, as strings."""
    x0, x1, y0, y1 = track.free_box()
    reference = track.reference()
    center = track.lines.get('centerline')
    out = {
        'footprint': f'{x1 - x0:.1f} x {y1 - y0:.1f} m',
        'resolution': f'{track.resolution * 100:g} cm/px',
        'direction': 'counter-clockwise' if signed_area(reference['xy']) > 0 else 'clockwise',
        'start': f'({reference["xy"][0, 0]:.2f}, {reference["xy"][0, 1]:.2f})',
        'lines': ', '.join(f'{kind} {line["length"]:.1f} m'
                           for kind, line in track.lines.items()),
        'width': (f'{center["widths"].min():.2f}-{center["widths"].max():.2f} m'
                  if center and center['widths'] is not None else '-'),
    }
    raceline = track.lines.get('raceline')
    if raceline and raceline['speeds'] is not None:
        out['lines'] += (f' ({raceline["speeds"].min():.1f}-{raceline["speeds"].max():.1f} '
                         'm/s)')
    extras = []
    if track.cones:
        extras.append(f'{len(track.cones)} cones')
    if track.bridges:
        extras.append(f'{len(track.bridges)} bridge' + ('s' if len(track.bridges) > 1 else ''))
    out['extras'] = ', '.join(extras) or '-'
    out['tunings'] = ', '.join(f'`{name}`' for name in track.tunings) or '-'
    return out


def paragraph(text):
    """Wrap a Markdown paragraph like the hand-written docs."""
    return textwrap.fill(text, 99, break_long_words=False, break_on_hyphens=False)


def catalog(tracks, values):
    """Markdown of the catalog."""
    lines = [
        '# Map catalog',
        '',
        '<!-- Generated by scripts/tracks/catalog.py from src/base_system/rbsf_worlds/tracks/.\n'
        '     Do not edit by hand: rerun it. -->',
        '',
        paragraph(INTRO),
        '',
        paragraph(LEGEND),
        '',
        '| Track | Source | Lap | Width | Direction | Extras |',
        '|---|---|---|---|---|---|',
    ]
    for track in tracks:
        v = values[track.name]
        lines.append(f'| [`{track.name}`](#{track.name}) | {SOURCES[track.name][0]} | '
                     f'{track.reference()["length"]:.1f} m | {v["width"]} | {v["direction"]} | '
                     f'{v["extras"]} |')
    for track in tracks:
        v = values[track.name]
        source, notes = SOURCES[track.name]
        lines += [
            '',
            f'## {track.name}',
            '',
            f'![{track.name}]({IMAGES}/{track.name}.png)',
            '',
            paragraph(f'{source}. {notes}'),
            '',
            '| | |',
            '|---|---|',
            f'| Reference lines | {v["lines"]} |',
            f'| Corridor width | {v["width"]} |',
            f'| Driving direction | {v["direction"]}, from {v["start"]} |',
            f'| Footprint | {v["footprint"]} (drivable space), {v["resolution"]} |',
            f'| Extras | {v["extras"]} |',
            f'| Path-tracking tunings | {v["tunings"]} |',
        ]
    return '\n'.join(lines) + '\n'


def main():
    """Write the catalog and its previews."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--out', type=pathlib.Path, default=OUT,
                        help='Directory of MAP_CATALOG.md (default: docs/).')
    args = parser.parse_args()

    names = sorted(p.name for p in TRACKS.iterdir() if (p / f'{p.name}.yaml').exists())
    missing = [name for name in names if name not in SOURCES]
    if missing:
        sys.exit(f'No SOURCES entry for {", ".join(missing)}: add one to {__file__}')
    tracks = [Track(TRACKS / name) for name in SOURCES if name in names]
    images = args.out / IMAGES
    images.mkdir(parents=True, exist_ok=True)
    for stale in {p.stem for p in images.glob('*.png')} - set(names):
        (images / f'{stale}.png').unlink()
    values = {}
    for track in tracks:
        preview(track, images / f'{track.name}.png')
        values[track.name] = facts(track)
        print(f'{track.name}: {values[track.name]["lines"]}, {values[track.name]["direction"]}')
    (args.out / 'MAP_CATALOG.md').write_text(catalog(tracks, values))
    print(f'Wrote {args.out / "MAP_CATALOG.md"} and {len(tracks)} previews in {images}')


if __name__ == '__main__':
    main()
