#!/bin/bash
# Download the raw OpenStreetMap data (roads and buildings) for the Hoa Lac core grid.
# Data: (c) OpenStreetMap contributors, ODbL. https://www.openstreetmap.org/copyright
#
#   ./fetch-osm.sh            # writes hoalac-raw.osm and hoalac-raw.osm.meta next to this script
#
# hoalac-raw.osm is the untouched download. hoalac-core.osm is the hand-edited copy used to generate the map; do not overwrite it.

set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
OUT="$DIR/hoalac-raw.osm"

# south,west,north,east
BBOX="20.98981,105.51733,21.02499,105.55707"
SERVERS=(
  "https://overpass-api.de/api/interpreter"
  "https://overpass.kumi.systems/api/interpreter"
)
ATTEMPTS="${ATTEMPTS:-4}"

QUERY="[out:xml][timeout:120];
(
  way[\"highway\"]($BBOX);
  way[\"building\"]($BBOX);
);
(._;>;);
out meta;"

tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT

for attempt in $(seq 1 "$ATTEMPTS"); do
  for server in "${SERVERS[@]}"; do
    echo "Attempt $attempt: $server"
    if curl -fsS --max-time 180 -A "hoalac-osm-fetch" --data-urlencode "data=$QUERY" "$server" -o "$tmp" \
       && grep -q "<osm" "$tmp" && ! grep -q "<remark>" "$tmp"; then
      mv "$tmp" "$OUT"
      {
        echo "source: $server"
        echo "bbox (south,west,north,east): $BBOX"
        echo "downloaded: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "license: ODbL, (c) OpenStreetMap contributors"
      } > "$OUT.meta"
      echo "Saved $OUT ($(du -h "$OUT" | cut -f1))"
      exit 0
    fi
  done
  sleep $((attempt * 10))
done

echo "Overpass did not return usable data after $ATTEMPTS attempts." >&2
exit 1
