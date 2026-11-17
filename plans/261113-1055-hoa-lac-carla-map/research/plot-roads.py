"""Draw OSM roads with names and tag coverage; outline the chosen core frame.

    python3 plot-roads.py map.osm out.png [south,west,north,east]

The optional bounding box fixes the drawing area when the file has no <bounds> element.
"""
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

src, out = sys.argv[1], sys.argv[2]
root = ET.parse(src).getroot()
nodes = {n.get("id"): (float(n.get("lon")), float(n.get("lat"))) for n in root.findall("node")}
b = root.find("bounds")
if len(sys.argv) > 3:
    S, W_, N, E = (float(v) for v in sys.argv[3].split(","))
elif b is not None:
    S, W_, N, E = (float(b.get(k)) for k in ("minlat", "minlon", "maxlat", "maxlon"))
else:
    lons = [v[0] for v in nodes.values()]
    lats = [v[1] for v in nodes.values()]
    S, W_, N, E = min(lats), min(lons), max(lats), max(lons)
FRAME = (21.0030, 105.5270, 21.0190, 105.5450)  # south, west, north, east of the chosen core grid
kx = 111320 * math.cos(math.radians((S + N) / 2))
ky = 110574


def conv(lon, lat):
    return (lon - W_) * kx, (lat - S) * ky


def xy(nid):
    return conv(*nodes[nid])


def pts(way):
    return [xy(r.get("ref")) for r in way.findall("nd") if r.get("ref") in nodes]


def tags(way):
    return {t.get("k"): t.get("v") for t in way.findall("tag")}


W, H = conv(E, N)
fig, ax = plt.subplots(figsize=(18, 17))
for w in root.findall("way"):
    if "building" in tags(w):
        p = pts(w)
        if len(p) > 2:
            ax.fill(*zip(*p), color="#e4e4e4", lw=0.3, ec="#bdbdbd", zorder=1)

style = {"motorway": ("#8250df", 3.5), "motorway_link": ("#8250df", 2.0), "trunk": ("#d1242f", 3.5),
         "trunk_link": ("#d1242f", 2.0), "primary": ("#e16f24", 3.0), "primary_link": ("#e16f24", 2.0),
         "secondary": ("#e16f24", 2.5), "tertiary": ("#1f6feb", 3.0), "tertiary_link": ("#1f6feb", 2.0),
         "residential": ("#2da44e", 1.6), "unclassified": ("#2da44e", 1.6), "service": ("#8c8c8c", 1.0),
         "track": ("#c9a227", 0.9), "path": ("#c9a227", 0.7), "footway": ("#d6a5d6", 0.7),
         "pedestrian": ("#d6a5d6", 0.7), "cycleway": ("#d6a5d6", 0.7), "construction": ("#999", 1.5)}
MAIN = ("tertiary", "tertiary_link")
longest = {}
for w in root.findall("way"):
    t = tags(w)
    if "highway" not in t:
        continue
    p = pts(w)
    if len(p) < 2:
        continue
    color, lw = style.get(t["highway"], ("#666", 1))
    if t["highway"] in MAIN and ("lanes" not in t or "oneway" not in t):
        color = "#e5484d"
    ax.plot(*zip(*p), color=color, lw=lw, solid_capstyle="round", zorder=3 if lw > 1.5 else 2)
    name = t.get("name")
    if name and re.search(r"N\d\d|Quốc lộ|Thăng Long", name) and "Vòng" not in name and "song hành" not in name:
        inside = [q for q in p if 0 <= q[0] <= W and 0 <= q[1] <= H]
        seg = sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))
        if inside and seg > longest.get(name, (0, None))[0]:
            longest[name] = (seg, inside[len(inside) // 2])

for name, (_, (x, y)) in longest.items():
    label = name.replace("Đường ", "").replace(" (Láng - Hoà Lạc)", "").replace(" - Đường Hồ Chí Minh", "")
    ax.annotate(label, (x, y), fontsize=11, fontweight="bold", ha="center", va="center", zorder=6,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#555", lw=0.8))

x0, y0 = conv(FRAME[1], FRAME[0])
x1, y1 = conv(FRAME[3], FRAME[2])
ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="black", lw=2, ls="--", zorder=7))
ax.annotate("core frame 1.9 x 1.8 km", (x0, y1), xytext=(4, 6), textcoords="offset points", fontsize=11, zorder=7)

ax.set_aspect("equal")
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.set_xlabel("m (east)")
ax.set_ylabel("m (north)")
ax.set_title("Hoa Lac (OSM, ODbL). Purple = motorway, dark red = trunk (QL21), orange = primary, blue = tertiary with lanes+oneway,\n"
             "light red = tertiary missing lanes or oneway, green = residential, grey = service, yellow = track/path, pink = footway",
             fontsize=10)
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(out, dpi=100)
