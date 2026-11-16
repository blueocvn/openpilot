#!/usr/bin/env bash
# Obstacle benchmark: the same cells, the same numbers, for every candidate fix. Engaged runs, headless.
#
# A cell is three parameters: the obstacle, the obstacle's speed and the ego car's set speed.
#
#   bash bench.sh standard                   # the preset cells (list shows them) at BENCH_EGO (60 km/h), about 8 min
#   bash bench.sh short                      # about 5 min: the two motorcycle cells that fail most (standing, 2 m/s)
#   bash bench.sh run <obstacle> <obstacle_kph> <ego_kph> [repeats]     # one cell, e.g. run motorcycle 20 60
#   bash bench.sh list [grid]                # the cells `standard` (or `grid`) would run after the filters below
#   OBSTACLE=car,none bash bench.sh standard                            # only the preset cells that match
#   OBSTACLE=motorcycle OBSTACLE_KPH=0,10,20 bash bench.sh grid         # every combination, not only the presets
# Every cell runs once (REPEATS=1); a failed run gets exactly one more run and never a third (RETRY_FAILED=1).
#   bash bench.sh summary [dir]              # table + CSV of a finished benchmark directory
#   bash bench.sh stop                       # stop the benchmark stacks
#
# obstacle:      none | motorcycle | car | barrier | container
# obstacle_kph:  0 = stands still. Only a motorcycle can move; it drives straight ahead at that constant speed.
# ego_kph:       the cruise set speed; the preset cells use BENCH_EGO (default 60). A moving obstacle must be slower than the ego car, or the car never
#                catches up: such a cell is skipped.
# OVERTAKE=1     adds the scripted driver: after the car stops behind the obstacle it steers out and passes.
#
# The cell decides what is measured and the pass rule (see bench.md):
#   no obstacle                        clear road: no false braking, speed near the set speed
#   stands still, or at most 3 m/s     stop in front of it: no contact, at least 4 m left
#   faster than 3 m/s                  follow it: steady gap close to the one the planner aims for
# Map: Town04_Opt only (spawn 170; a motorcycle faster than 3 m/s uses spawn 171, which is straight for longer).
# Town05 was dropped: its straights are short and its intersections (no lane lines) throw the car off the road.
#
# The benchmark has its own CARLA servers (:2100, :2110, ... off-screen) and its own bridge containers, so a simulation
# you are driving at the same time keeps running; they only share the GPU. The simulation runs in real time (openpilot's
# processes use the wall clock), so the way to finish sooner is more runs side by side: BENCH_JOBS=<n>. Every result
# records realtime_factor; a value below 0.95 means the machine was overloaded and the run is discarded by the summary.
# Measured here (16 GB GPU, the user's own sim also running): 2 stacks keep 1.0 or more, 3 stacks drop to 0.8.
# Results: $BENCH_OUT (default <bukapilot>/.carla/bench/<timestamp>): one JSON per run, one bridge log per run,
# results.csv and results.md.
#
# Filters for `standard`, `list` and `grid` (comma-separated; for standard and list both optional, a cell must match
# every filter that is set; grid needs OBSTACLE and OBSTACLE_KPH, and takes its set speeds from EGO, default BENCH_EGO):
#   OBSTACLE=motorcycle,car   OBSTACLE_KPH=0,20   EGO=50,60
# PARAMS=Key=1,Other=0 sets those boolean Params in every run (feature flags under test).
# TRACE=1 writes <run>.trace/truth.jsonl for every run (TRACE_IMAGES=1 adds the camera frames).
# Environment: CARLA_VERSION=<release> (default 0.9.15), BENCH_EGO=<kph> (set speed of the preset cells, default 60),
#   REPEATS=<n> (default 1), RETRY_FAILED=0|1 (default 1), BENCH_JOBS=<n> (stacks side by side, default 1), BENCH_OUT=<dir>,
#   BENCH_CLEAR_M=<metres> (clear-road length, default 300), BENCH_FOLLOW_M=<metres> (follow length, default 500),
#   BENCH_FOLLOW_START_M=<metres> (where the followed motorcycle starts, default 30; it waits until the car engages),
#   BENCH_PORT=2100 (stack n uses BENCH_PORT + 10 n),
#   BENCH_QUALITY=High|Low, RUN_CAP_S=<seconds> (per-run limit, default: from the distance and the set speed),
#   OBSTACLE_M=<metres> (distance to a standing obstacle; default 400), BRIDGE_ARGS="..." (extra bridge arguments).
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
BENCH_PORT="${BENCH_PORT:-2100}"
EGO_KPH="${BENCH_EGO:-60}"   # set speed of the preset cells
JOBS="${BENCH_JOBS:-1}"
TOWN=Town04_Opt
CARLA_VERSION="${CARLA_VERSION:-0.9.15}"
# the bukapilot image carries the carla client, so it is per CARLA version (0.9.12 keeps its original name)
if [ "$CARLA_VERSION" == "0.9.12" ]; then IMAGE="${IMAGE:-bukapilot-sim:local}"; else IMAGE="${IMAGE:-bukapilot-sim:carla-$CARLA_VERSION}"; fi
export CARLA_VERSION IMAGE
OBSTACLES="none motorcycle car barrier container"

# stack_env <n>: names and ports of stack n, so that stacks side by side never share a container or a CARLA server
stack_env() {
  STACK_ID="$1"
  STACK_ENV=(CARLA_PORT=$((BENCH_PORT + 10 * $1)) CARLA_NAME="carla_bench_$1" NAME="bukapilot_bench_$1" CARLA_HEADLESS=1
             CARLA_QUALITY="${BENCH_QUALITY:-High}" OP_UI=0 CARLA_KEYS=0 BLOCK=ui SKIP_BUILD=1 CARLA_CLEAN_START=1)
}
stack_env 0

# compile_once: scons runs here, once, in a throwaway container, and the stacks start with SKIP_BUILD=1. Two stacks
# running scons at the same time in the same tree corrupt its database (seen: dblite os.replace failure).
compile_once() {
  docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONPATH="$ROOT" -v "$ROOT:$ROOT" -w "$ROOT" \
    "$IMAGE" bash -c 'scons -j"$(nproc)"' >/dev/null 2>&1 \
    || { echo "scons failed: run it by hand in the $IMAGE container" >&2; return 1; }
}

# less <a> <b>: a < b for decimal numbers
less() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a < b) }'; }

FOLLOW_KPH=10.8   # 3 m/s: a moving obstacle slower than this is "slow" (stop behind it), faster is "follow"

# regime <obstacle> <obstacle_kph> <ego_kph> -> clear | static | slow | follow, or "invalid: <reason>" (exit 1)
regime() {
  local kind="$1" okph="$2" ego="$3"
  case " $OBSTACLES " in *" $kind "*) ;; *) echo "invalid: unknown obstacle '$kind' (one of: $OBSTACLES)"; return 1 ;; esac
  if [ "$kind" = none ]; then echo clear; return 0; fi
  if ! less 0 "$okph"; then echo static; return 0; fi
  if [ "$kind" != motorcycle ]; then echo "invalid: only a motorcycle can move"; return 1; fi
  if ! less "$okph" "$ego"; then echo "invalid: obstacle ${okph} km/h is not slower than ego ${ego} km/h (the car never catches up)"; return 1; fi
  if less "$okph" "$FOLLOW_KPH"; then echo slow; else echo follow; fi
}

# Town04 spawn 170 is straight for 450 m (junction at 245 to 300 m), spawn 171 for 715 m: a follow run needs the longer road.
spawn_point() { [ "$1" = follow ] && echo 171 || echo 170; }

# Distances (m): a standing obstacle on the straight, where the car cannot take another branch; where a moving
# motorcycle starts; how far the car drives before a slow / follow run ends (the car has met the motorcycle and
# settled behind it, before the junction for slow; spawn 171 is straight for 715 m and then bends gently for follow).
STATIC_M="${OBSTACLE_M:-400}"
SLOW_START_M=100;   SLOW_END_M=230
FOLLOW_START_M="${BENCH_FOLLOW_START_M:-30}";  FOLLOW_END_M="${BENCH_FOLLOW_M:-500}"

# run_metres <regime> -> how far the run is meant to last (m), for the time limit
run_metres() {
  case "$1" in
    clear) echo "${BENCH_CLEAR_M:-300}" ;;
    slow) echo "$SLOW_END_M" ;;
    follow) echo "$FOLLOW_END_M" ;;
    *) echo "$STATIC_M" ;;
  esac
}

# run_cap <regime> <ego_kph>: the --duration limit (wall seconds, which includes about 30 s of start-up and
# engaging): twice the time to cover the distance at the set speed, plus 30 s. A run that has not ended by then is
# lost (the car never met the obstacle), and it is cut short instead of lasting minutes.
run_cap() { awk -v m="$(run_metres "$1")" -v k="$2" 'BEGIN { printf "%d", 60 + 2 * m * 3.6 / k }'; }

stop_stack() {
  docker ps -aq --filter "name=^(bukapilot|carla)_bench" | xargs -r docker rm -f >/dev/null 2>&1 || true
  echo "benchmark stacks stopped"
}

ensure_carla() {
  env "${STACK_ENV[@]}" bash "$DIR/run.sh" server --detach
}

# run_one <obstacle> <obstacle_kph> <ego_kph> <overtake|-> <index>
run_one() {
  local kind="$1" okph="$2" ego="$3" overtake="$4" index="$5" mode spawn name label
  mode="$(regime "$kind" "$okph" "$ego")" || { echo "skipped ${kind}:${okph}:${ego}: ${mode#invalid: }"; return 0; }
  spawn="$(spawn_point "$mode")"
  label="$kind/$okph/$ego"
  name="${kind}-${okph}kph_ego${ego}"
  [ "$overtake" = overtake ] && { label="$label/overtake"; name="${name}_overtake"; }
  name="${name}_$(printf '%02d' "$index")"
  local result="$OUT/$name.json" log="$OUT/$name.log" cap="${RUN_CAP_S:-$(run_cap "$mode" "$ego")}"
  local spec=""
  case "$mode" in
    static) spec="$kind:$STATIC_M" ;;
    slow) spec="motorcycle:$SLOW_START_M:$okph" ;;
    follow) spec="motorcycle:$FOLLOW_START_M:$okph" ;;
  esac
  # TRACE=1: also write <run>.trace/truth.jsonl, the model's lead output next to the truth, one line per camera frame
  local trace=()
  [ -n "${TRACE:-}" ] && trace=(LEAD_RECORD_DIR="$OUT/$name.trace" LEAD_RECORD_IMAGES="${TRACE_IMAGES:-0}")
  # PARAMS="VisionLeadHold=1,VisionLeadDistanceMargin=1": boolean Params for every run (the flags under test)
  local run_env=("${trace[@]}" SIM_PARAMS="${PARAMS:-}" LEAD_RESULT_FILE="$result" LEAD_SCENARIO="$label"
                 OPENPILOT_MANAGER_LOG="$OUT/$name.bukapilot.log")
  case "$mode" in
    clear | slow | follow) run_env+=(LEAD_RUN_DISTANCE_M="$(run_metres "$mode")") ;;
  esac
  [ "$overtake" = overtake ] && run_env+=(LEAD_OVERTAKE=1)
  local args=(--carla-town "$TOWN" --carla-spawn-point "$spawn" --cruise-kph "$ego" --duration "$cap" --no-carla-keys)
  [ -n "$spec" ] && args+=(--obstacle-spec "$spec")
  # shellcheck disable=SC2206
  args+=(${BRIDGE_ARGS:-})

  echo "== $name  (${spec:-no obstacle}, cap ${cap}s)"
  rm -f "$result"
  env "${STACK_ENV[@]}" "${run_env[@]}" bash "$DIR/run.sh" bridge "${args[@]}" </dev/null >"$log" 2>&1
  docker rm -f "bukapilot_bench_$STACK_ID" >/dev/null 2>&1 || true
  if [ ! -s "$result" ]; then
    echo "   no result written (see $log)"
    printf '{"scenario": "%s", "engaged": false, "end_reason": "no result"}\n' "$label" >"$result"
  else
    python3 -c "
import json,sys
r=json.load(open('$result'))
print('   ', r.get('end_reason'), 'min_gap', r.get('min_gap_m'), 'collision', r.get('collision'), 'peak_decel', r.get('peak_decel_mps2'), 'false_brake', r.get('false_brake_events'))"
  fi
  # exit status: the run's verdict under the same pass rule as the summary
  python3 -c "
import json, sys
sys.path.insert(0, '$ROOT/tools/sim/bridge')
import bench_summary
passed, reason = bench_summary.judge(json.load(open('$result')))
print('    passed' if passed else '    FAILED: ' + reason)
sys.exit(0 if passed else 1)"
}

# run_set <repeats> then cells "obstacle:obstacle_kph:ego_kph[:overtake]". Runs are dealt out to BENCH_JOBS stacks,
# each in its own background subshell with its own CARLA and bridge container.
run_set() {
  local repeats="$1" cell i task n=0 id
  shift
  local tasks=()
  for cell in "$@"; do
    for ((i = 1; i <= repeats; i++)); do tasks+=("$cell|$i"); done
  done
  compile_once || return 1
  for ((id = 0; id < JOBS; id++)); do
    (
      stack_env "$id"
      ensure_carla || exit 1
      n=0
      for task in "${tasks[@]}"; do
        if [ $((n % JOBS)) -eq "$id" ]; then
          # task is "kind:okph:ego[:overtake]|index"
          IFS=: read -r kind okph ego flag <<<"${task%|*}"
          # RETRY_FAILED=1: a failed run gets exactly one more run, never a third; its index follows the last repeat
          run_one "$kind" "$okph" "$ego" "${flag:--}" "${task#*|}" \
            || { [ "${RETRY_FAILED:-1}" = 1 ] && run_one "$kind" "$okph" "$ego" "${flag:--}" "$((${task#*|} + repeats))"; } \
            || true
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

# in_list <value> <comma list>: succeeds when the list is empty or contains the value
in_list() { [ -z "$2" ] || [[ ",$2," == *",$1,"* ]]; }

# keep <obstacle> <obstacle_kph> succeeds when the cell passes the OBSTACLE and OBSTACLE_KPH filters
keep() { in_list "$1" "${OBSTACLE:-}" && in_list "$2" "${OBSTACLE_KPH:-}"; }

# The preset cells, all at EGO_KPH. Static objects on the road (barrier, container) are left to automatic emergency
# braking, not the driving model, so they are not presets; `run barrier 0 60` still runs one.
preset_cells() {
  echo "motorcycle:0:$EGO_KPH"      # stationary motorcycle
  echo "motorcycle:7.2:$EGO_KPH"    # motorcycle at 2 m/s
  echo "car:0:$EGO_KPH"             # parked car
  echo "none:0:$EGO_KPH"            # clear road
  less 40 "$EGO_KPH" && echo "motorcycle:40:$EGO_KPH"   # follow a motorcycle at 40 km/h
}

# with_overtake <cell>: adds ":overtake" under OVERTAKE=1 (the preset overtake cell has it already)
with_overtake() { if [ -n "${OVERTAKE:-}" ] && [[ "$1" != *:overtake ]]; then echo "$1:overtake"; else echo "$1"; fi; }

# cells_for <standard|grid> -> the cells to run, one per line
cells_for() {
  local cell kind okph ego
  if [ "$1" = grid ]; then
    [ -n "${OBSTACLE:-}" ] && [ -n "${OBSTACLE_KPH:-}" ] || { echo "grid needs OBSTACLE and OBSTACLE_KPH" >&2; return 2; }
    local egos="${EGO:-$EGO_KPH}"
    for kind in ${OBSTACLE//,/ }; do
      for okph in ${OBSTACLE_KPH//,/ }; do
        for ego in ${egos//,/ }; do with_overtake "$kind:$okph:$ego"; done
      done
    done
    return 0
  fi
  while IFS= read -r cell; do
    IFS=: read -r kind okph ego _ <<<"$cell"
    keep "$kind" "$okph" && echo "$cell"
  done < <(preset_cells)
  return 0   # the last cell not matching must not look like a failure
}

# runnable <cells...> -> prints the ones the regime allows; the others go to stderr with the reason
runnable() {
  local cell kind okph ego out
  for cell in "$@"; do
    IFS=: read -r kind okph ego _ <<<"$cell"
    if out="$(regime "$kind" "$okph" "$ego")"; then echo "$cell"; else echo "skipped $cell: ${out#invalid: }" >&2; fi
  done
}

new_out() { OUT="${BENCH_OUT:-$ROOT/.carla/bench/$(date +%Y%m%d-%H%M%S)}"; mkdir -p "$OUT"; echo "results: $OUT"; }

# plan <standard|grid>: sets `cells` to the runnable cells, or exits when there are none
plan() {
  local all
  all="$(cells_for "$1")" || exit 2
  [ -n "$all" ] || { echo "no cell matches the filters; see: bash bench.sh list" >&2; exit 2; }
  mapfile -t cells < <(runnable $all)
  [ "${#cells[@]}" -gt 0 ] || { echo "no runnable cell" >&2; exit 2; }
}

case "${1:-}" in
  short)
    # the two motorcycle cells that fail most often (measured 2026-11-16, CARLA 0.9.12 and 0.9.15)
    new_out
    run_set "${REPEATS:-1}" "motorcycle:0:$EGO_KPH" "motorcycle:7.2:$EGO_KPH"
    summarize "$OUT"
    ;;
  standard | grid)
    plan "$1"
    new_out
    run_set "${REPEATS:-1}" "${cells[@]}"
    summarize "$OUT"
    ;;
  run)
    [ $# -ge 4 ] || { echo "usage: $0 run <obstacle> <obstacle_kph> <ego_kph> [repeats]" >&2; exit 2; }
    reason="$(regime "$2" "$3" "$4")" || { echo "${reason#invalid: }" >&2; exit 2; }
    new_out
    run_set "${5:-${REPEATS:-1}}" "$(with_overtake "$2:$3:$4")"
    summarize "$OUT"
    ;;
  list)
    plan "${2:-standard}"   # `list` shows the preset cells, `list grid` the combinations of OBSTACLE, OBSTACLE_KPH and EGO
    printf '%s\n' "${cells[@]}"
    echo "${#cells[@]} cells x ${REPEATS:-1} repeats = $((${#cells[@]} * ${REPEATS:-1})) runs (a failed run gets one more)"
    ;;
  summary) summarize "${2:?usage: $0 summary <dir>}" ;;
  stop) stop_stack ;;
  *)
    sed -n '2,/^set -uo/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//' >&2
    exit 2
    ;;
esac
