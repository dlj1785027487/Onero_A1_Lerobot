#!/usr/bin/env bash
# Replay an A1 dataset to the configured follower; all arguments go to the CLI.
# Usage: bash scripts/replay.sh --session-config CONFIG [options]

set -Eeuo pipefail
exec onero-a1-replay-episode "$@"
