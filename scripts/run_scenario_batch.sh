#!/bin/bash
# ---------------------------------------------------------------------------
# run_scenario_batch.sh [--dry-run] ROWS_FILE
#
# Run a list of recorded scenarios and laps one after another, unattended.
# ROWS_FILE has one run per line ('#' starts a comment):
#   aeb ROW SPEED [depth|lidar]            -> run_aeb_scenario.sh (AEB_SENSOR)
#   acc ROW [SET_SPEED]                    -> run_acc_scenario.sh
#   lap MODE LABEL DUR WORLD [STACK_ARGS]  -> run_headless_lap.sh (rest of the line is STACK_ARGS)
# Each run gets its own log; a run whose stack never came up healthy is repeated
# once. A run with an INVALID RUN line is marked INVALID and never counted as a
# result. The summary table (run, status, session, log) is printed at the end
# and written to the batch directory, with the stamp to pass to
# compare_aeb_sessions.py --since.
# Logs: log/scenario_batch/<stamp>/NN_<name>.log and summary.md
# Exit status: 0 when every run is OK, 1 otherwise.
# ---------------------------------------------------------------------------
DRY=0
[ "$1" = --dry-run ] && { DRY=1; shift; }
ROWS=${1:?usage: run_scenario_batch.sh [--dry-run] ROWS_FILE}
[ -r "$ROWS" ] || { echo "cannot read $ROWS"; exit 2; }
WS="${VLM_AV_WS:-$HOME/vlm_av_ws}"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$WS/log/scenario_batch/$STAMP"
cd "$WS" || exit 1

# one line of ROWS_FILE -> NAME, DATA_DIR and the command in CMD (an env prefix is part of it)
parse() {
    local kind=$1; shift
    case "$kind" in
        aeb)
            [ $# -ge 2 ] || return 1
            local sensor=${3:-depth}
            case "$sensor" in depth|lidar) ;; *) return 1 ;; esac
            NAME="aeb_${1}_${2}_${sensor}"; DATA_DIR=data/aeb_scenarios
            CMD=(env "AEB_SENSOR=$sensor" ./scripts/run_aeb_scenario.sh "$1" "$2") ;;
        acc)
            [ $# -ge 1 ] || return 1
            NAME="acc_${1}_${2:-3.0}"; DATA_DIR=data/aeb_scenarios
            CMD=(./scripts/run_acc_scenario.sh "$1" ${2:+"$2"}) ;;
        lap)
            [ $# -ge 4 ] || return 1
            local mode=$1 label=$2 dur=$3 world=$4; shift 4
            NAME="lap_${label}"; DATA_DIR=data/av_laps
            CMD=(env "STACK_ARGS=$*" ./scripts/run_headless_lap.sh "$mode" "$label" "$dur" "$world") ;;
        *) return 1 ;;
    esac
}

mapfile -t LINES < <(sed 's/#.*//' "$ROWS" | awk 'NF')
[ ${#LINES[@]} -gt 0 ] || { echo "no runs in $ROWS"; exit 2; }
for line in "${LINES[@]}"; do
    # shellcheck disable=SC2086  # the line is split into words on purpose
    parse $line || { echo "bad line in $ROWS: $line"; exit 2; }
done
if [ "$DRY" = 1 ]; then
    for line in "${LINES[@]}"; do
        # shellcheck disable=SC2086
        parse $line; echo "${CMD[*]}"
    done
    exit 0
fi

mkdir -p "$OUT"
SUMMARY="$OUT/summary.md"
{ echo "| # | Run | Status | Session | Log |"; echo "| --- | --- | --- | --- | --- |"; } > "$SUMMARY"
n=0; bad=0
for line in "${LINES[@]}"; do
    n=$((n + 1))
    # shellcheck disable=SC2086
    parse $line
    log="$OUT/$(printf '%02d' "$n")_$NAME.log"
    status=OK
    for attempt in 1 2; do
        touch "$OUT/.start"
        echo "[$(date +%H:%M:%S)] run $n/${#LINES[@]}: ${CMD[*]} (attempt $attempt)"
        "${CMD[@]}" > "$log" 2>&1
        rc=$?
        grep -q "stack never came up healthy" "$log" || break
        [ "$attempt" = 1 ] && cp "$log" "${log%.log}_attempt1.log"
    done
    session=$(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d -newer "$OUT/.start" 2>/dev/null | sort | tail -1)
    if grep -q "stack never came up healthy" "$log"; then
        status="FAILED (stack never came up, twice)"
    elif grep -q "INVALID RUN" "$log"; then
        status="INVALID: $(grep -m1 'INVALID RUN' "$log" | cut -c1-90)"
    elif [ "$rc" != 0 ]; then
        status="FAILED (exit $rc)"
    elif [ -z "$session" ]; then
        status="FAILED (no session directory)"
    fi
    [ "$status" = OK ] || bad=$((bad + 1))
    echo "| $n | $line | $status | ${session:--} | ${log#"$WS"/} |" >> "$SUMMARY"
    echo "    -> $status ${session:+($session)}"
done
rm -f "$OUT/.start"
"$WS/scripts/stop_av_stack.sh" > /dev/null 2>&1
echo
cat "$SUMMARY"
echo
echo "batch stamp: $STAMP   (python3 tools/avkit/compare_aeb_sessions.py --since $STAMP)"
[ "$bad" = 0 ]
