#!/usr/bin/env bash
set -euo pipefail

BIN="${1:-./test}"

if ! command -v perf >/dev/null 2>&1; then
  echo "perf is not installed. Try: sudo apt install linux-tools-common linux-tools-generic"
  exit 1
fi

if [[ ! -x "$BIN" ]]; then
  echo "Binary not found or not executable: $BIN"
  echo "Usage: ./profile_perf.sh ./test"
  exit 1
fi

echo "== perf stat =="
if ! perf stat -e cycles,instructions,branches,branch-misses,cache-misses "$BIN"; then
  echo
  echo "Hardware perf events are not available for this user/session."
  echo "Try one of these on the VM:"
  echo "  sudo sysctl kernel.perf_event_paranoid=1"
  echo "  sudo ./profile_perf.sh $BIN"
  echo
  echo "Falling back to software events only."
  perf stat -e task-clock,context-switches,cpu-migrations,page-faults "$BIN"
fi

echo
echo "== perf record =="
if ! perf record -F 999 -g -- "$BIN"; then
  echo
  echo "Hardware sampling failed. Trying software clock sampling."
  perf record -e cpu-clock -F 999 -g -- "$BIN"
fi

echo
echo "== top hotspots =="
perf report --stdio --no-children | head -120
