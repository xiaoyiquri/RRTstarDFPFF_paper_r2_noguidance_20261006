#!/usr/bin/env bash
set -euo pipefail

BIN="${1:-./test}"
if [[ $# -gt 0 ]]; then
  shift
fi

if ! command -v perf >/dev/null 2>&1; then
  echo "perf is not installed. Try: sudo apt install linux-tools-common linux-tools-generic"
  exit 1
fi

if [[ ! -x "$BIN" ]]; then
  echo "Binary not found or not executable: $BIN"
  echo "Usage: ./profile_benchmark.sh ./test [extra benchmark options]"
  exit 1
fi

export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export OPENCV_FOR_THREADS_NUM=1

DEFAULT_ARGS=(--benchmark --warmup 5 --runs 50 --samples 4000 --seed 40 --timeout 30)
if [[ $# -gt 0 ]]; then
  BENCHMARK_ARGS=(--benchmark "$@")
else
  BENCHMARK_ARGS=("${DEFAULT_ARGS[@]}")
fi

echo "== benchmark command =="
printf '%q ' "$BIN" "${BENCHMARK_ARGS[@]}"
echo

echo
echo "== perf stat =="
if perf stat -e cycles -- true >/dev/null 2>&1; then
  perf stat -r 5 \
    -e cycles,instructions,branches,branch-misses,cache-references,cache-misses \
    -- "$BIN" "${BENCHMARK_ARGS[@]}"
else
  echo "Hardware PMU events are unavailable. Using software counters."
  perf stat -r 5 \
    -e task-clock,context-switches,cpu-migrations,page-faults \
    -- "$BIN" "${BENCHMARK_ARGS[@]}"
fi

echo
echo "== perf record =="
if perf stat -e cycles -- true >/dev/null 2>&1; then
  perf record -e cycles -F 999 -g -- "$BIN" "${BENCHMARK_ARGS[@]}"
else
  echo "Hardware sampling is unavailable. Using software CPU-clock sampling."
  perf record -e cpu-clock -F 999 -g -- "$BIN" "${BENCHMARK_ARGS[@]}"
fi

echo
echo "== planner hotspots =="
perf report --stdio --no-children | head -160
