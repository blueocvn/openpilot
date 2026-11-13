#!/usr/bin/env bash
# Obstacle benchmark: the same scenarios, the same numbers, for every candidate fix. Engaged runs, headless.
#
#   bash bench.sh short                      # about 5 min: stationary motorcycle + clear road, Town05, 30 km/h, 3 repeats
#   bash bench.sh full                       # the whole matrix, 3 repeats per cell (BENCH_JOBS=2 runs two stacks side by side)
#   bash bench.sh list                       # the cells `full` would run, after the filters below
#   SCENARIOS=container,barrier TOWNS=Town04_Opt SPEEDS=60 bash bench.sh full    # only those cells
#   bash bench.sh run <scenario> <town> <set_kph> [repeats]
#   bash bench.sh summary [dir]              # table + CSV of a finished benchmark directory
#   bash bench.sh stop                       # stop the benchmark stack
#
# Scenarios: stationary-motorcycle slow-motorcycle parked-car barrier container follow-motorcycle clear-road overtake
# Towns: Town04_Opt (spawn 170, follow-motorcycle 171) or Town05_Opt (spawn 126, clear-road 1).
#
# The benchmark has its own CARLA servers (:2100, :2110, ... off-screen) and its own bridge containers, so a simulation
# you are driving at the same time keeps running; they only share the GPU. The simulation runs in real time (openpilot's
# processes use the wall clock), so the way to finish sooner is more runs side by side: BENCH_JOBS=<n>. Every result
# records realtime_factor; a value below 0.95 means the machine was overloaded and the run is discarded by the summary.
# Measured here (16 GB GPU, the user's own sim also running): 2 stacks keep 1.0 or more, 3 stacks drop to 0.8. Results: $BENCH_OUT (default
# <bukapilot>/.carla/bench/<timestamp>): one JSON per run, one bridge log per run, results.csv and results.md.
#
# Filters for `full` and `list` (comma-separated, all optional; a cell must match every filter that is set):
#   SCENARIOS=stationary-motorcycle,container   TOWNS=Town04_Opt   SPEEDS=30
# Environment: REPEATS=<n> (repeats for `run`/`full`, default 3: runs repeat to within about 1 m), BENCH_JOBS=<n>
#   (stacks run side by side, default 1), BENCH_OUT=<dir>, BENCH_CLEAR_M=<metres> (clear-road length, default 1000), BENCH_PORT=2100 (stack n uses BENCH_PORT + 10 n), BENCH_QUALITY=High|Low, RUN_CAP_S=<seconds> (per-run limit, default: from the obstacle distance and set speed),
#   OBSTACLE_M=<metres> (distance to the obstacle; default: 400 in Town04, 250 in Town05),
#   BRIDGE_ARGS="..." (extra bridge arguments).
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
BENCH_PORT="${BENCH_PORT:-2100}"
JOBS="${BENCH_JOBS:-1}"

# stack_env <n>: names and ports of stack n, so that stacks side by side never share a container or a CARLA server
stack_env() {
  STACK_ID="$1"
  STACK_ENV=(CARLA_PORT=$((BENCH_PORT + 10 * $1)) CARLA_NAME="carla_bench_$1" NAME="bukapilot_bench_$1" CARLA_HEADLESS=1
             CARLA_QUALITY="${BENCH_QUALITY:-High}" OP_UI=0 CARLA_KEYS=0 BLOCK=ui)
}
stack_env 0

# spawn_point <scenario> <town>. Spawn points were picked on the map for the longest straight road ahead:
# Town04 170 is straight for 450 m, 171 for 715 m (the follow scenario); Town05's best is 126 with 360 m, of which
# 130 m is junction. The clear-road scenario keeps its original spawn points (Town04 170, Town05 1: with bends).
spawn_point() {
  case "$2:$1" in
    Town04_Opt:clear-road) echo 170 ;;
    Town04_Opt:follow-motorcycle) echo 171 ;;
    Town04_Opt:*) echo 170 ;;
    Town05_Opt:clear-road) echo 1 ;;
    Town05_Opt:*) echo 126 ;;
    *) echo "unknown town '$2' (Town04_Opt or Town05_Opt)" >&2; return 1 ;;
  esac
}

# Distance from the spawn point to a static obstacle. It stands on the straight, where the car cannot take another
# branch, and not inside or right after a junction: the car must be able to stop on marked road. Town05 has no
# straight longer than 360 m and a junction every 90 m (25 m long, at 25, 80, 170, 260 m from spawn 126). Past the
# junction at 170 m the car leaves the straight (measured: it never meets an obstacle at 250 m), so the obstacle
# stands at 150 m, between the junctions at 105 and 170 m. Town04 170: junction at 245 to 300 m, clear to 450 m.
straight_m() {
  case "$1" in
    Town04_Opt) echo 400 ;;
    Town05_Opt) echo 150 ;;
  esac
}

# End of the follow run (m driven). Town04 spawn 171 is straight for 715 m, then bends gently; the motorcycle keeps
# to the road. Town05 has no road long enough: the follow scenario is not part of the Town05 matrix.
follow_m() {
  case "$1" in
    Town04_Opt) echo 1300 ;;
    Town05_Opt) echo 150 ;;
  esac
}

# End of the slow-motorcycle run (m driven): the car has met the motorcycle and settled behind it, and has not yet
# reached the next junction (Town05 at 170 m), where a car crawling at 2 m/s loses its lane lines.
slow_m() {
  case "$1" in
    Town04_Opt) echo 230 ;;
    Town05_Opt) echo 160 ;;
  esac
}

# run_cap <scenario> <town> <kph>: the --duration limit (wall seconds, which includes about 30 s of start-up and
# engaging): twice the time to cover the distance at the set speed, plus 30 s. A run that has not ended by then is
# lost (the car never met the obstacle), and it is cut short instead of lasting minutes.
run_cap() {
  local metres
  case "$1" in
    clear-road) metres="${BENCH_CLEAR_M:-1000}" ;;
    follow-motorcycle) metres="$(follow_m "$2")" ;;
    slow-motorcycle) metres="$(slow_m "$2")" ;;
    *) metres="${OBSTACLE_M:-$(straight_m "$2")}" ;;
  esac
  echo $((30 + 30 + 2 * metres * 36 / (10 * $3)))
}

# obstacle_for <scenario> <town> -> obstacle spec ("" for none)
obstacle_for() {
  local OBSTACLE_M="${OBSTACLE_M:-$(straight_m "$2")}"
  case "$1" in
    stationary-motorcycle) echo "motorcycle:$OBSTACLE_M" ;;
    slow-motorcycle) echo "motorcycle:100:7.2" ;;
    parked-car) echo "car:$OBSTACLE_M" ;;
    barrier) echo "barrier:$OBSTACLE_M" ;;
    container) echo "container:$OBSTACLE_M" ;;
    follow-motorcycle) echo "motorcycle:60:40" ;;
    clear-road) echo "" ;;
    overtake) echo "barrier:$OBSTACLE_M" ;;
    *) echo "unknown scenario '$1'" >&2; return 1 ;;
  esac
}

stop_stack() {
  docker ps -aq --filter "name=^(bukapilot|carla)_bench" | xargs -r docker rm -f >/dev/null 2>&1 || true
  echo "benchmark stacks stopped"
}

ensure_carla() {
  env "${STACK_ENV[@]}" bash "$DIR/run.sh" server --detach
}

# run_one <scenario> <town> <set_kph> <index>
run_one() {
  local scenario="$1" town="$2" kph="$3" index="$4" spawn obstacle name
  spawn="$(spawn_point "$scenario" "$town")" || return 1
  obstacle="$(obstacle_for "$scenario" "$town")" || return 1
  name="${scenario}_${town}_${kph}_$(printf '%02d' "$index")"
  local result="$OUT/$name.json" log="$OUT/$name.log" cap="${RUN_CAP_S:-$(run_cap "$scenario" "$town" "$kph")}"
  local run_env=(LEAD_RESULT_FILE="$result" LEAD_SCENARIO="$scenario/$town/$kph" OPENPILOT_MANAGER_LOG="$OUT/$name.bukapilot.log")
  case "$scenario" in
    clear-road) run_env+=(LEAD_RUN_DISTANCE_M="${BENCH_CLEAR_M:-1000}") ;;
    slow-motorcycle) run_env+=(LEAD_RUN_DISTANCE_M="$(slow_m "$town")") ;;
    follow-motorcycle) run_env+=(LEAD_RUN_DISTANCE_M="$(follow_m "$town")") ;;
    overtake) run_env+=(LEAD_OVERTAKE=1) ;;
  esac
  local args=(--carla-town "$town" --carla-spawn-point "$spawn" --cruise-kph "$kph" --duration "$cap" --no-carla-keys)
  [ -n "$obstacle" ] && args+=(--obstacle-spec "$obstacle")
  # shellcheck disable=SC2206
  args+=(${BRIDGE_ARGS:-})

  echo "== $name  (${obstacle:-no obstacle}, cap ${cap}s)"
  rm -f "$result"
  env "${STACK_ENV[@]}" "${run_env[@]}" bash "$DIR/run.sh" bridge "${args[@]}" </dev/null >"$log" 2>&1
  docker rm -f "bukapilot_bench_$STACK_ID" >/dev/null 2>&1 || true
  if [ ! -s "$result" ]; then
    echo "   no result written (see $log)"
    printf '{"scenario": "%s/%s/%s", "engaged": false, "end_reason": "no result"}\n' "$scenario" "$town" "$kph" >"$result"
  else
    python3 -c "
import json,sys
r=json.load(open('$result'))
print('   ', r.get('end_reason'), 'min_gap', r.get('min_gap_m'), 'collision', r.get('collision'), 'peak_decel', r.get('peak_decel_mps2'), 'false_brake', r.get('false_brake_events'))"
  fi
}

# run_set <repeats> then "scenario:town:kph" items. Runs are dealt out to BENCH_JOBS stacks, each in its own
# background subshell with its own CARLA and bridge container; the runs of one stack go in order, so each stack
# loads a map at most once or twice.
run_set() {
  local repeats="$1" item i task n=0 id
  shift
  local tasks=()
  for item in "$@"; do
    for ((i = 1; i <= repeats; i++)); do tasks+=("$item:$i"); done
  done
  for ((id = 0; id < JOBS; id++)); do
    (
      stack_env "$id"
      ensure_carla || exit 1
      n=0
      for task in "${tasks[@]}"; do
        if [ $((n % JOBS)) -eq "$id" ]; then
          IFS=: read -r scenario town kph index <<<"$task"
          run_one "$scenario" "$town" "$kph" "$index"
        fi
        n=$((n + 1))
      done
    ) > >(sed -u "s/^/[stack $id] /") 2>&1 &
  done
  wait
}

summarize() {
  python3 "$ROOT/tools/sim/bridge/bench_summary.py" "$1" --csv "$1/results.csv" --markdown "$1/results.md"
}

# keep <scenario> <town> <kph> succeeds when the cell passes the SCENARIOS, TOWNS and SPEEDS filters
keep() {
  local f
  for f in "$1:${SCENARIOS:-}" "$2:${TOWNS:-}" "$3:${SPEEDS:-}"; do
    [ -z "${f#*:}" ] || [[ ",${f#*:}," == *",${f%%:*},"* ]] || return 1
  done
}

all_items() {
  local town kph scenario
  for town in Town05_Opt Town04_Opt; do
    for kph in 30 60; do
      # Town05 at 60 km/h: the car leaves the straight in the first intersections (no lane lines) in every run
      [ "$town" = Town05_Opt ] && [ "$kph" = 60 ] && continue
      for scenario in stationary-motorcycle slow-motorcycle parked-car barrier container clear-road; do
        echo "$scenario:$town:$kph"
      done
    done
    # follow needs a long road (Town04 only); overtake starts from a standstill, so its set speed does not matter
    [ "$town" = Town04_Opt ] && echo "follow-motorcycle:$town:60"
    echo "overtake:$town:30"
  done
}

full_items() {
  local item scenario town kph
  for item in $(all_items); do
    IFS=: read -r scenario town kph <<<"$item"
    keep "$scenario" "$town" "$kph" && echo "$item"
  done
}

new_out() { OUT="${BENCH_OUT:-$ROOT/.carla/bench/$(date +%Y%m%d-%H%M%S)}"; mkdir -p "$OUT"; echo "results: $OUT"; }

case "${1:-}" in
  short)
    new_out
    BENCH_CLEAR_M="${BENCH_CLEAR_M:-600}"
    export BENCH_CLEAR_M
    run_set "${REPEATS:-3}" stationary-motorcycle:Town05_Opt:30 clear-road:Town05_Opt:30
    summarize "$OUT"
    ;;
  full)
    new_out
    mapfile -t items < <(full_items)
    [ "${#items[@]}" -gt 0 ] || { echo "no cell matches the filters; see: bash bench.sh list" >&2; exit 2; }
    run_set "${REPEATS:-3}" "${items[@]}"
    summarize "$OUT"
    ;;
  run)
    [ $# -ge 4 ] || { echo "usage: $0 run <scenario> <town> <set_kph> [repeats]" >&2; exit 2; }
    new_out
    run_set "${5:-${REPEATS:-1}}" "$2:$3:$4"
    summarize "$OUT"
    ;;
  list)
    mapfile -t items < <(full_items)
    [ "${#items[@]}" -gt 0 ] && printf '%s\n' "${items[@]}"
    echo "${#items[@]} cells x ${REPEATS:-3} repeats = $((${#items[@]} * ${REPEATS:-3})) runs"
    ;;
  summary) summarize "${2:?usage: $0 summary <dir>}" ;;
  stop) stop_stack ;;
  *)
    sed -n '2,/^set -uo/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//' >&2
    exit 2
    ;;
esac
