import sys
import xml.etree.ElementTree as ET
src, dst, w, s, e, n = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:7])
root = ET.parse(src).getroot()
nodes = {x.get('id'): x for x in root.iter('node')}
inside = {i for i, x in nodes.items() if s <= float(x.get('lat')) <= n and w <= float(x.get('lon')) <= e}
out = ET.Element('osm', root.attrib)
keep_nodes, ways = set(), []
for way in root.iter('way'):
    if not any(t.get('k') == 'highway' for t in way.findall('tag')):
        continue
    refs = [nd.get('ref') for nd in way.findall('nd')]
    if any(r in inside for r in refs):
        ways.append(way); keep_nodes.update(refs)
for i in keep_nodes:
    if i in nodes: out.append(nodes[i])
for way in ways: out.append(way)
ET.ElementTree(out).write(dst, encoding='utf-8', xml_declaration=True)
print('ways', len(ways), 'nodes', len(keep_nodes))
