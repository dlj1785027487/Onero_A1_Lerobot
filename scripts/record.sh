#!/usr/bin/env bash
# Record an A1 CAN leader/follower session; all arguments go to the CLI.
# Usage: bash scripts/record.sh --session-config CONFIG [options]

set -Eeuo pipefail
exec onero-a1-lerobot-record "$@"
