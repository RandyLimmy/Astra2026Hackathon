#!/bin/zsh
set -eu
cd -- "$(dirname -- "$0")"
exec .venv/bin/mjpython -m simulator view "${1:-wheel_loss}"
