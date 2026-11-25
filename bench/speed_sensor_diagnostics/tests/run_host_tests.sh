#!/usr/bin/env sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=$(mktemp -d "speed-diag-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$project_dir/main" \
  -I"$project_dir/components/speed_sensor" \
  "$project_dir/tests/test_diag_logic.c" \
  "$project_dir/main/diag_logic.c" \
  "$project_dir/components/speed_sensor/speed_measurement.c" \
  -lm -o "$build_dir/test_diag_logic"
"$build_dir/test_diag_logic"
