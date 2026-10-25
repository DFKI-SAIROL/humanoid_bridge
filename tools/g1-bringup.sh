#!/usr/bin/env bash
# The G1's bring-up: tools/robot-bringup.sh --robot g1 (its --help has every option).
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/robot-bringup.sh" --robot g1 "$@"
