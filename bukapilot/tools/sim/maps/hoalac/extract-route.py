"""Cut the hand-sketched road frame out of the raw OSM data and locate the route-mask checkpoints.

    python3 extract-route.py RAW.osm SKETCH.osm MASK.png OUT_DIR

RAW.osm     untouched OSM download (roads, buildings, landuse ...). Elements marked action='delete' are ignored.
SKETCH.osm  JOSM file whose new (negative id) ways trace the wanted road frame by hand.
MASK.png    white-on-black drawing of the same frame; small hollow circles are checkpoints. Roads drawn in the
            mask but missing from the sketch are extracted too.

Writes to OUT_DIR:
  hoalac-route.osm          real road pieces that follow the sketch or the mask (original tags and geometry) plus the
                            roadside objects (buildings, landuse, barriers, sidewalks ...) near them
  hoalac-checkpoints.json   checkpoint positions (lat/lon, local metres, nearest road and its distance)
  hoalac-route-preview.png  sketch, extracted roads, roadside and numbered checkpoints
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import cv2
import matplotlib
import numpy as np
from scipy.spatial import cKDTree

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROAD_TYPES = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
              "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
SIDE_HIGHWAYS = {"footway", "path", "cycleway", "pedestrian"}
ROADSIDE_KEYS = ("building", "landuse", "natural", "leisure", "amenity", "barrier", "waterway", "area", "man_made")

ROAD_BUFFER_M = 80.0     # sketch is drawn by hand with straight strokes: a curving road may bow this far away
ROAD_ANGLE_DEG = 35.0    # a road segment must run roughly along the sketch, not across it
GUIDE_SLACK_M = 20.0     # a road may follow any guide stroke at most this much farther than the nearest one
MASK_EXTRA_M = 25.0      # mask strokes farther than this from the sketch count as extra roads
MIN_PIECE_M = 40.0       # drop tiny road fragments left at junctions
SIDEWALK_BUFFER_M = 30.0
ROADSIDE_M = 80.0        # roadside objects with any node this close to an extracted road are kept
ROADSIDE_MAX_EXTENT_M = 1000.0  # skip campus outlines and other areas far larger than the corridor
SAMPLE_M = 4.0
ID_BASE = 9_000_000_000  # ids for road pieces split out of longer OSM ways


def tags_of(el):
    return {t.get("k"): t.get("v") for t in el.findall("tag")}


def live(el):
    return el.get("action") != "delete"


class Frame:
    """Local equirectangular projection in metres."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.kx = 111320 * math.cos(math.radians(lat0))
        self.ky = 110574

    def xy(self, lat, lon):
        return np.array([(lon - self.lon0) * self.kx, (lat - self.lat0) * self.ky])

    def latlon(self, x, y):
        return self.lat0 + y / self.ky, self.lon0 + x / self.kx


def densify(pts, step=SAMPLE_M):
    """Sample a polyline every `step` metres; returns points and the unit direction at each sample."""
    out, dirs = [], []
    for a, b in zip(pts[:-1], pts[1:]):
        seg = b - a
        n = max(1, int(np.linalg.norm(seg) // step))
        d = seg / (np.linalg.norm(seg) + 1e-9)
        for i in range(n):
            out.append(a + seg * i / n)
            dirs.append(d)
    out.append(pts[-1])
    dirs.append(dirs[-1] if dirs else np.array([1.0, 0.0]))
    return np.array(out), np.array(dirs)


def polyline_len(pts):
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0


def main(raw_path, sketch_path, mask_path, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    raw = ET.parse(raw_path).getroot()
    sketch = ET.parse(sketch_path).getroot()

    nodes = {n.get("id"): n for n in raw.findall("node") if live(n)}
    ways = [w for w in raw.findall("way") if live(w)]
    lats = [float(n.get("lat")) for n in nodes.values()]
    lons = [float(n.get("lon")) for n in nodes.values()]
    fr = Frame(min(lats), min(lons))
    nxy = {i: fr.xy(float(n.get("lat")), float(n.get("lon"))) for i, n in nodes.items()}

    # Sketch lines: new ways in the JOSM file.
    snodes = {n.get("id"): fr.xy(float(n.get("lat")), float(n.get("lon"))) for n in sketch.findall("node")}
    sketch_lines = []
    for w in sketch.findall("way"):
        if w.get("id").startswith("-") and live(w):
            sketch_lines.append(np.array([snodes[r.get("ref")] for r in w.findall("nd")]))
    if not sketch_lines:
        sys.exit("no sketch ways (negative ids) found in " + sketch_path)
    sk_pts, sk_dir = zip(*(densify(p) for p in sketch_lines))
    sk_pts, sk_dir = np.vstack(sk_pts), np.vstack(sk_dir)

    # Guide = sketch + mask strokes the sketch lacks.
    mask = read_mask(mask_path, sketch_lines)
    extra = cKDTree(sk_pts).query(mask["centerline"])[0] > MASK_EXTRA_M
    guide_pts = np.vstack([sk_pts, mask["centerline"][extra]])
    guide_dir = np.vstack([sk_dir, mask["centerline_dir"][extra]])
    guide_tree = cKDTree(guide_pts)
    min_cos = math.cos(math.radians(ROAD_ANGLE_DEG))

    def follows_sketch(a, b, buffer_m):
        """True when segment a-b lies within buffer of a guide line that runs the same way."""
        mids, d = densify(np.array([a, b]))
        nearest = guide_tree.query(mids)[0]
        if np.max(nearest) > buffer_m:
            return False
        # Only guide lines about as close as the nearest one count, so a side road at a junction
        # cannot borrow the direction of a distant parallel stroke.
        aligned = [np.max(np.abs(guide_dir[near] @ d[0])) >= min_cos
                   for near in guide_tree.query_ball_point(mids, np.minimum(nearest + GUIDE_SLACK_M, buffer_m))]
        return np.mean(aligned) >= 0.5

    def pieces(way, types, buffer_m):
        t = tags_of(way)
        if t.get("highway") not in types:
            return []
        refs = [r.get("ref") for r in way.findall("nd") if r.get("ref") in nodes]
        keep = [follows_sketch(nxy[a], nxy[b], buffer_m) for a, b in zip(refs[:-1], refs[1:])]
        out, cur = [], []
        for i, k in enumerate(keep):
            if k:
                cur = cur or [refs[i]]
                cur.append(refs[i + 1])
            elif cur:
                out.append(cur)
                cur = []
        if cur:
            out.append(cur)
        return [(way, p, len(p) == len(refs)) for p in out if polyline_len(np.array([nxy[r] for r in p])) >= MIN_PIECE_M]

    roads = [p for w in ways for p in pieces(w, ROAD_TYPES, ROAD_BUFFER_M)]
    if not roads:
        sys.exit("no OSM road follows the sketch")
    road_pts = np.vstack([densify(np.array([nxy[r] for r in refs]))[0] for _, refs, _ in roads])
    road_tree = cKDTree(road_pts)

    # Sidewalks and paths running along the extracted roads.
    def near_road(refs):
        return road_tree.query(np.array([nxy[r] for r in refs]))[0]

    sidewalks = []
    for w, refs, whole in (p for w in ways for p in pieces(w, SIDE_HIGHWAYS, ROAD_BUFFER_M + SIDEWALK_BUFFER_M)):
        if np.median(near_road(refs)) <= SIDEWALK_BUFFER_M:
            sidewalks.append((w, refs, whole))

    # Roadside objects: any node within ROADSIDE_M of an extracted road; the whole object is kept.
    roadside = []
    for w in ways:
        t = tags_of(w)
        if "highway" in t or not any(k in t for k in ROADSIDE_KEYS):
            continue
        refs = [r.get("ref") for r in w.findall("nd") if r.get("ref") in nodes]
        if not refs:
            continue
        p = np.array([nxy[r] for r in refs])
        if np.linalg.norm(p.max(0) - p.min(0)) <= ROADSIDE_MAX_EXTENT_M and np.min(near_road(refs)) <= ROADSIDE_M:
            roadside.append(w)

    # Multipolygon relations touching the corridor bring all their member ways.
    way_by_id = {w.get("id"): w for w in ways}
    kept_ids = {w.get("id") for w in roadside}
    relations = []
    for rel in raw.findall("relation"):
        if not live(rel) or tags_of(rel).get("type") != "multipolygon":
            continue
        members = [m.get("ref") for m in rel.findall("member") if m.get("type") == "way"]
        if not all(m in way_by_id for m in members):
            continue
        member_pts = [nxy[r.get("ref")] for m in members for r in way_by_id[m].findall("nd") if r.get("ref") in nxy]
        member_pts = np.array(member_pts)
        if (len(member_pts) and np.linalg.norm(member_pts.max(0) - member_pts.min(0)) <= ROADSIDE_MAX_EXTENT_M
                and np.min(road_tree.query(member_pts)[0]) <= ROADSIDE_M):
            relations.append(rel)
            for m in members:
                if m not in kept_ids:
                    roadside.append(way_by_id[m])
                    kept_ids.add(m)

    # Coverage: every guide sample should have a real road nearby.
    cover_d = road_tree.query(guide_pts)[0]
    gaps = guide_pts[cover_d > ROAD_BUFFER_M]

    drivable = [(w, [r.get("ref") for r in w.findall("nd") if r.get("ref") in nodes], True) for w in ways
                if tags_of(w).get("highway") in ROAD_TYPES]
    checkpoints = snap_checkpoints(mask["circles"], fr, roads, drivable, nxy)
    mask_only = mask["centerline"][extra]

    write_osm(os.path.join(out_dir, "hoalac-route.osm"), raw, nodes, roads + sidewalks, roadside, relations, fr)
    with open(os.path.join(out_dir, "hoalac-checkpoints.json"), "w", encoding="utf-8") as f:
        json.dump({"source_mask": os.path.basename(mask_path),
                   "origin_latlon": [fr.lat0, fr.lon0],
                   "checkpoints": checkpoints}, f, ensure_ascii=False, indent=2)
    plot(os.path.join(out_dir, "hoalac-route-preview.png"), sketch_lines, roads, sidewalks, roadside, nxy, checkpoints, gaps,
         mask_only)

    names = sorted({tags_of(w).get("name", "(no name)") for w, _, _ in roads})
    print(f"roads: {len(roads)} pieces ({sum(1 for *_, whole in roads if not whole)} split from longer ways), "
          f"{sum(polyline_len(np.array([nxy[r] for r in refs])) for _, refs, _ in roads) / 1000:.1f} km")
    print("road names:", ", ".join(names))
    print(f"sidewalks/paths: {len(sidewalks)}, roadside objects: {len(roadside)}, multipolygons: {len(relations)}")
    print(f"mask-only roads added to the guide: {extra.sum() * MASK_STEP_PX * mask['m_per_px'] / 1000:.1f} km of stroke")
    print(f"guide coverage: {100 * np.mean(cover_d <= ROAD_BUFFER_M):.1f}% of sketch + mask within {ROAD_BUFFER_M:.0f} m "
          f"of an extracted road (max gap distance {cover_d.max():.0f} m)")
    print(f"checkpoints: {len(checkpoints)}")
    for c in checkpoints:
        flag = "" if c["on_extracted_route"] else "  NOT ON EXTRACTED ROUTE"
        print(f"  #{c['id']:2d} {c['lat']:.6f},{c['lon']:.6f}  {c['road']} (snapped {c['snap_distance_m']:.0f} m){flag}")
    if len(gaps):
        print(f"guide without a road within {ROAD_BUFFER_M:.0f} m near:",
              ", ".join(f"{a:.5f},{b:.5f}" for a, b in (fr.latlon(*g) for g in gaps[:: max(1, len(gaps) // 8)])))


def fit_similarity(src, dst):
    """Least-squares rotation + uniform scale + translation mapping src onto dst."""
    ms, md = src.mean(0), dst.mean(0)
    a, b = src - ms, dst - md
    u, sv, vt = np.linalg.svd(a.T @ b)
    d = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])
    rot = (u @ d @ vt).T
    scale = np.sum(sv * np.diag(d)) / np.sum(a ** 2)
    return scale * rot, md - scale * rot @ ms


def road_index(pieces, nxy):
    pts, names, wids = [], [], []
    for wy, refs, _ in pieces:
        if len(refs) < 2:
            continue
        p = densify(np.array([nxy[r] for r in refs]), 1.0)[0]
        t = tags_of(wy)
        pts.append(p)
        names += [t.get("name") or t.get("ref") or f"{t.get('highway')} way {wy.get('id')}"] * len(p)
        wids += [wy.get("id")] * len(p)
    return cKDTree(np.vstack(pts)), np.vstack(pts), names, wids


MASK_STEP_PX = 1.0


def read_mask(mask_path, sketch_lines):
    """Find checkpoint circles and the stroke centreline in the mask, georeferenced onto the sketch (metres)."""
    img = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    white = (img > 127).astype(np.uint8)
    # Drop specks: keep only large white components (the road frame).
    n, lab, stats, _ = cv2.connectedComponentsWithStats(white, connectivity=8)
    frame = np.isin(lab, [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] > 500]).astype(np.uint8)

    # Holes: black components that do not touch the border. Small, round, equally sized ones are checkpoints;
    # larger holes are blocks enclosed by roads.
    n, lab, stats, cent = cv2.connectedComponentsWithStats(1 - frame, connectivity=4)
    h, w = frame.shape
    holes = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if x == 0 or y == 0 or x + bw >= w or y + bh >= h:
            continue
        if 0.75 <= bw / bh <= 1.33 and area / (bw * bh) >= 0.75:
            holes.append((i, area, cent[i]))
    typical = np.median([a for _, a, _ in holes]) if holes else 0
    circle_ids = [i for i, a, _ in holes if 0.5 * typical <= a <= 2 * typical]
    circles = np.array([cent[i] for i in circle_ids]).reshape(-1, 2)

    # Centreline of the strokes with the checkpoint circles filled, so rings do not become fake roads.
    solid = (frame | np.isin(lab, circle_ids)).astype(np.uint8) * 255
    ys, xs = np.nonzero(cv2.ximgproc.thinning(solid))
    line_px = np.column_stack([xs, -ys]).astype(float)  # image y grows downward

    # Georeference: similarity transform pixel -> metres, fitted by two-way ICP against the sketch.
    # Two-way matching keeps roads drawn only in the mask from dragging the fit.
    ys, xs = np.nonzero(frame)
    px = np.column_stack([xs, -ys]).astype(float)
    sk = np.vstack([densify(p, 2.0)[0] for p in sketch_lines])
    sk_tree = cKDTree(sk)
    scale = np.mean((sk.max(0) - sk.min(0)) / (px.max(0) - px.min(0)))
    M, t = scale * np.eye(2), sk.mean(0) - scale * px.mean(0)
    for _ in range(100):
        q = px @ M.T + t
        d1, i1 = sk_tree.query(q)
        d2, i2 = cKDTree(q).query(sk)
        thr = np.percentile(np.r_[d1, d2], 80)
        M, t = fit_similarity(np.r_[px[d1 < thr], px[i2[d2 < thr]]], np.r_[sk[i1[d1 < thr]], sk[d2 < thr]])
    m_per_px = float(np.sqrt(abs(np.linalg.det(M))))
    d_sk = cKDTree(px @ M.T + t).query(sk)[0]
    print(f"mask georeference: sketch-to-mask distance median {np.median(d_sk):.1f} m, 90th percentile "
          f"{np.percentile(d_sk, 90):.1f} m, scale {m_per_px:.2f} m/px, "
          f"rotation {math.degrees(math.atan2(M[1, 0], M[0, 0])):.1f} deg")

    # Local stroke direction from the principal axis of nearby centreline pixels.
    line_m = line_px @ M.T + t
    tree = cKDTree(line_m)
    dirs = np.zeros_like(line_m)
    for k, near in enumerate(tree.query_ball_point(line_m, 4 * m_per_px)):
        p = line_m[near] - line_m[near].mean(0)
        dirs[k] = np.linalg.svd(p, full_matrices=False)[2][0] if len(near) > 2 else (1.0, 0.0)
    circles_m = np.column_stack([circles[:, 0], -circles[:, 1]]) @ M.T + t
    return {"circles": [(c_px, c_m) for c_px, c_m in zip(circles, circles_m)], "centerline": line_m,
            "centerline_dir": dirs, "m_per_px": m_per_px}


def snap_checkpoints(circles, fr, roads, drivable, nxy):
    """Snap each mask circle to the nearest drivable road and say whether that road is in the extracted route."""
    route_tree = road_index(roads, nxy)[0]
    all_tree, all_pts, all_names, all_wids = road_index(drivable, nxy)
    out = []
    # Number checkpoints north to south, then west to east.
    for k, (c_px, c_m) in enumerate(sorted(circles, key=lambda c: (-round(c[1][1] / 50), c[1][0])), 1):
        d, j = all_tree.query(c_m)
        sx_, sy_ = all_pts[j]
        lat, lon = fr.latlon(sx_, sy_)
        out.append({"id": k, "lat": round(lat, 7), "lon": round(lon, 7),
                    "x_m": round(float(sx_), 1), "y_m": round(float(sy_), 1),
                    "mask_px": [round(float(c_px[0]), 1), round(float(c_px[1]), 1)],
                    "road": all_names[j], "osm_way": all_wids[j], "snap_distance_m": round(float(d), 1),
                    "on_extracted_route": bool(route_tree.query(all_pts[j])[0] < 1.5)})
    return out


def write_osm(path, raw, nodes, road_pieces, roadside, relations, fr):
    root = ET.Element("osm", version="0.6", generator="extract-route.py")
    used = set()
    out_ways = []
    next_id = ID_BASE
    for w, refs, whole in road_pieces:
        if whole:
            wid = w.get("id")
        else:
            next_id += 1
            wid = str(next_id)
        out_ways.append((wid, refs, tags_of(w), w))
    for w in roadside:
        refs = [r.get("ref") for r in w.findall("nd") if r.get("ref") in nodes]
        out_ways.append((w.get("id"), refs, tags_of(w), w))
    for _, refs, _, _ in out_ways:
        used.update(refs)

    pts = np.array([[float(nodes[i].get("lat")), float(nodes[i].get("lon"))] for i in used])
    ET.SubElement(root, "bounds", minlat=f"{pts[:, 0].min():.7f}", minlon=f"{pts[:, 1].min():.7f}",
                  maxlat=f"{pts[:, 0].max():.7f}", maxlon=f"{pts[:, 1].max():.7f}")
    for i in sorted(used, key=int):
        n = nodes[i]
        el = ET.SubElement(root, "node", id=i, visible="true", version=n.get("version", "1"),
                           lat=n.get("lat"), lon=n.get("lon"))
        for k, v in tags_of(n).items():
            ET.SubElement(el, "tag", k=k, v=v)
    seen = set()
    for wid, refs, t, src in out_ways:
        if wid in seen:
            continue
        seen.add(wid)
        el = ET.SubElement(root, "way", id=wid, visible="true", version=src.get("version", "1"))
        for r in refs:
            ET.SubElement(el, "nd", ref=r)
        for k, v in t.items():
            ET.SubElement(el, "tag", k=k, v=v)
    for rel in relations:
        el = ET.SubElement(root, "relation", id=rel.get("id"), visible="true", version=rel.get("version", "1"))
        for m in rel.findall("member"):
            if m.get("type") == "way" and m.get("ref") in seen:
                ET.SubElement(el, "member", type="way", ref=m.get("ref"), role=m.get("role", ""))
        for k, v in tags_of(rel).items():
            ET.SubElement(el, "tag", k=k, v=v)
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="UTF-8", xml_declaration=True)


def plot(path, sketch_lines, roads, sidewalks, roadside, nxy, checkpoints, gaps, mask_only):
    fig, ax = plt.subplots(figsize=(16, 17))
    for w in roadside:
        p = np.array([nxy[r.get("ref")] for r in w.findall("nd") if r.get("ref") in nxy])
        if len(p) < 2:
            continue
        t = tags_of(w)
        if "building" in t and len(p) > 2:
            ax.fill(p[:, 0], p[:, 1], color="#d9d9d9", ec="#9e9e9e", lw=0.4, zorder=1)
        elif "landuse" in t or "natural" in t or "leisure" in t:
            ax.fill(p[:, 0], p[:, 1], color="#cfe8c4", alpha=0.5, lw=0, zorder=0)
        else:
            ax.plot(p[:, 0], p[:, 1], color="#a07850", lw=0.6, zorder=1)
    if len(mask_only):
        ax.scatter(mask_only[:, 0], mask_only[:, 1], s=2, color="#555", zorder=2, label="mask-only road (added to guide)")
    for p in sketch_lines:
        ax.plot(p[:, 0], p[:, 1], color="#ff9f1c", lw=10, alpha=0.25, solid_capstyle="round", zorder=2)
    for _, refs, _ in sidewalks:
        p = np.array([nxy[r] for r in refs])
        ax.plot(p[:, 0], p[:, 1], color="#c27bc2", lw=0.8, zorder=3)
    for _, refs, _ in roads:
        p = np.array([nxy[r] for r in refs])
        ax.plot(p[:, 0], p[:, 1], color="#1f6feb", lw=2.2, zorder=4)
    if len(gaps):
        ax.scatter(gaps[:, 0], gaps[:, 1], s=6, color="red", zorder=5, label="sketch without a matching road")
    for c in checkpoints:
        ax.scatter(c["x_m"], c["y_m"], s=260, facecolor="white", lw=2.5, zorder=6,
                   edgecolor="#d1242f" if c["on_extracted_route"] else "#8250df")
        ax.annotate(str(c["id"]), (c["x_m"], c["y_m"]), ha="center", va="center", fontsize=9, fontweight="bold", zorder=7)
    ax.plot([], [], color="#ff9f1c", lw=8, alpha=0.3, label="hand sketch")
    ax.plot([], [], color="#1f6feb", lw=2.2, label="extracted OSM road")
    ax.plot([], [], color="#c27bc2", lw=0.8, label="sidewalk / path")
    ax.fill([], [], color="#d9d9d9", label="roadside building")
    ax.scatter([], [], s=120, facecolor="white", edgecolor="#d1242f", lw=2, label="checkpoint on extracted route")
    ax.scatter([], [], s=120, facecolor="white", edgecolor="#8250df", lw=2, label="checkpoint off extracted route")
    ax.legend(loc="lower right")
    ax.set_aspect("equal")
    ax.set_xlabel("m (east)")
    ax.set_ylabel("m (north)")
    ax.set_title("Hoa Lac route extracted from OSM (ODbL) along the hand sketch")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


if __name__ == "__main__":
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    main(*sys.argv[1:])
