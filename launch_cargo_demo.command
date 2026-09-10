#!/bin/zsh
set -eu
cd -- "$(dirname -- "$0")"
if (( $# == 0 )); then
  set -- warehouse_curve_demo --camera side --speedup 1
fi
exec .venv/bin/mjpython -m simulator view "$@"
