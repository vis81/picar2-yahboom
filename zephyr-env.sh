#!/usr/bin/env bash
# Register this project as a Zephyr extra module.
#
# Source this after the workspace activate.sh:
#   source ws-v3.7.0/activate.sh
#   source yahboom/zephyr-env.sh
#
# Or add to your shell's direnv .envrc for automatic activation.

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${ZEPHYR_EXTRA_MODULES:-}" ]]; then
    export ZEPHYR_EXTRA_MODULES="${ZEPHYR_EXTRA_MODULES};${THIS_DIR}"
else
    export ZEPHYR_EXTRA_MODULES="${THIS_DIR}"
fi

echo "App module: ${THIS_DIR}"
