import sys, time, re, collections
import carla
src, dst = sys.argv[1], sys.argv[2]
data = open(src, encoding='utf-8').read()
s = carla.Osm2OdrSettings()
print('defaults: lane_width', s.default_lane_width, 'use_offsets', s.use_offsets, 'center_map', s.center_map, 'proj', s.proj_string)
s.set_osm_way_types(["motorway","motorway_link","trunk","trunk_link","primary","primary_link","secondary","secondary_link","tertiary","tertiary_link","unclassified","residential"])
t = time.time()
xodr = carla.Osm2Odr.convert(data, s)
print('convert seconds', round(time.time() - t, 1), 'xodr bytes', len(xodr))
open(dst, 'w', encoding='utf-8').write(xodr)
print('roads', len(re.findall(r'<road ', xodr)), 'junctions', len(re.findall(r'<junction ', xodr)), 'signals', len(re.findall(r'<signal ', xodr)))
print('roadMark types', collections.Counter(re.findall(r'<roadMark [^>]*type="([^"]+)"', xodr)).most_common(6))
print('lane types', collections.Counter(re.findall(r'<lane [^>]*type="([^"]+)"', xodr)).most_common(8))
print('elevation nonzero', sum(1 for m in re.findall(r'<elevation [^>]*a="([^"]+)"', xodr) if abs(float(m)) > 1e-6))
