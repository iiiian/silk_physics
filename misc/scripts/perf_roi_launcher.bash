#!/usr/bin/env bash

# Launcher for perf region of interest (ROI) profiling.
# Setup control and ack file descriptor, inject perf control flag, and set control fd env var.

set -euo pipefail

if (( $# == 0 )); then
  echo "usage: $0 'perf stat/record ... -- workload ...'" >&2
  echo "   or: $0 perf stat/record ... -- workload ..." >&2
  exit 2
fi

# Tmp dir for control/ack pipe. XXXXXX is placeholder for rand hash.
tmpdir=$(mktemp -d /tmp/perf-roi.XXXXXX)

# close fd and cleanup tmp dir.
cleanup() {
  if [[ -n ${ack_fd:-} ]]; then
    exec {ack_fd}>&- || true
  fi
  if [[ -n ${ctl_fd:-} ]]; then
    exec {ctl_fd}>&- || true
  fi
  rm -rf "$tmpdir"
}
trap cleanup EXIT

# create named control/act pipe and open them
mkfifo "$tmpdir/ctl" "$tmpdir/ack"
exec {ctl_fd}<>"$tmpdir/ctl"
exec {ack_fd}<>"$tmpdir/ack"

# program should read these env var to get fd.
export PERF_CTL_FD="$ctl_fd"
export PERF_ACK_FD="$ack_fd"

# Add the control options after `perf stat`/`perf record` and before the
# caller's remaining options and workload command.
perf() {
  if (( $# == 0 )) || [[ "$1" != stat && "$1" != record ]]; then
    echo "perf_roi_launcher: expected 'perf stat' or 'perf record'" >&2
    return 2
  fi

  local mode="$1"
  shift
  command perf "$mode" \
    --delay=-1 \
    --control "fd:${ctl_fd},${ack_fd}" \
    "$@"
}

if (( $# == 1 )); then
  # This form intentionally accepts a shell command string, including quoted
  # workload arguments, e.g. 'perf stat ... -- ./program --arg "value"'.
  eval "$1"
elif [[ "$1" == perf ]]; then
  shift
  perf "$@"
else
  echo "perf_roi_launcher: command must start with perf" >&2
  exit 2
fi
