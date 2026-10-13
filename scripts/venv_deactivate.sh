#!/usr/bin/env bash
# Source this file so deactivation changes the current shell.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf 'Use deactivate in the activated shell, or: source "%s"\n' "$0" >&2
    exit 1
fi

if declare -F deactivate >/dev/null 2>&1; then
    deactivate
elif [[ -n "${VIRTUAL_ENV:-}" ]]; then
    printf 'No deactivate function in this shell. Source the environment\047s bin/activate first.\n' >&2
    return 1
fi

return 0
