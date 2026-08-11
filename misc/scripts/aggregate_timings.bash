#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: aggregate_timings.bash [--top FRACTION] [TIMINGS_JSON]

Aggregate leaf timer samples by name, sorted by total time.

Arguments:
  TIMINGS_JSON   Timing file to read. Defaults to timings.json. Use - for stdin.

Options:
  --top FRACTION Select the slowest fraction of top-level frames before
                 aggregating. For example, --top 0.1 selects the slowest 10%.
  -h, --help     Show this help.
EOF
}

timings_path="timings.json"
top="null"

while (($# > 0)); do
  case "$1" in
    --top)
      top="${2:?--top requires a fraction}"
      shift 2
      ;;
    -h | --help)
      usage
      exit
      ;;
    *)
      timings_path="$1"
      shift
      ;;
  esac
done

jq --argjson top "$top" '
  def samples:
    .. | objects
    | select(has("name") and has("duration_ms"))
    | select(.children | length == 0)
    | {name, total: .duration_ms};

  (if $top == null then
     .
   else
     sort_by(.duration_ms)
     | .[(length * (1 - $top) | floor):]
   end)
  | [samples]
  | group_by(.name)
  | map({
      name: .[0].name,
      count: length,
      total_ms: (map(.total) | add),
      avg_ms: ((map(.total) | add) / length)
    })
  | sort_by(.total_ms)
  | reverse
' "$timings_path"
