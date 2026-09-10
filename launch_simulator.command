#!/bin/zsh
set -eu
cd -- "$(dirname -- "$0")"
if (( $# == 0 )); then
  set -- drone_demo
fi
exec .venv/bin/mjpython -m simulator view "$@"
