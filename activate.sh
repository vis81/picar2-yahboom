#!/usr/bin/env bash
# Activate the Zephyr build environment for this workspace.
#
# Usage:
#   source activate.sh        — activate in current shell (use this daily)
#   bash   activate.sh        — run first-time setup then exit
#
# On first run: creates .venv, installs west, runs west init + west update,
#               installs Zephyr Python deps, and downloads the SDK.
# On subsequent runs: fast — just sources the venv and sets env vars.
#
# After switching git branches, run:
#   source activate.sh && west update

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$REPO/west.yml"
SDK_PLATFORM="${SDK_PLATFORM:-linux-x86_64}"

# ── Read SDK version and toolchain from west.yml extras ──────────────────────
_read_manifest() {
    python3 -c "
import yaml, pathlib, sys
m = yaml.safe_load(pathlib.Path('$MANIFEST').read_text())
extras = m['manifest'].get('extras', {})
print(extras.get('zephyr-sdk-version', ''))
print(extras.get('zephyr-sdk-toolchain', 'arm-zephyr-eabi'))
"
}

if ! command -v python3 &>/dev/null; then
    echo "ERROR: python3 not found." >&2
    return 1 2>/dev/null || exit 1
fi

# yaml may not be available yet outside the venv — bootstrap read via venv if needed
if python3 -c "import yaml" &>/dev/null 2>&1; then
    read -r SDK_VERSION SDK_TOOLCHAIN <<< "$(_read_manifest)"
fi

SDK_DIR="$REPO/sdks/zephyr-sdk-${SDK_VERSION}"

# ── venv ─────────────────────────────────────────────────────────────────────
if [[ ! -d "$REPO/.venv" ]]; then
    echo "  Creating Python venv ..."
    python3 -m venv "$REPO/.venv"
fi
# shellcheck source=/dev/null
source "$REPO/.venv/bin/activate"

# Now yaml is available; re-read if we hadn't yet
if [[ -z "${SDK_VERSION:-}" ]]; then
    read -r SDK_VERSION SDK_TOOLCHAIN <<< "$(_read_manifest)"
    SDK_DIR="$REPO/sdks/zephyr-sdk-${SDK_VERSION}"
fi

# ── west ─────────────────────────────────────────────────────────────────────
if ! pip show west &>/dev/null 2>&1; then
    echo "  Installing west ..."
    pip install -q west
fi

# ── west init ────────────────────────────────────────────────────────────────
if [[ ! -d "$REPO/.west" ]]; then
    echo "  Initialising west workspace ..."
    (cd "$REPO" && west init -l .)
fi

# ── west update ──────────────────────────────────────────────────────────────
if [[ ! -d "$REPO/zephyr" ]]; then
    echo "  Running west update (first time — this may take a few minutes) ..."
    (cd "$REPO" && west update)
    pip install -q -r "$REPO/zephyr/scripts/requirements.txt"
fi

# ── SDK ──────────────────────────────────────────────────────────────────────
if [[ -n "$SDK_VERSION" ]] && [[ ! -d "$SDK_DIR" ]]; then
    echo "  Downloading Zephyr SDK ${SDK_VERSION} ..."
    mkdir -p "$REPO/sdks"
    ARCHIVE="zephyr-sdk-${SDK_VERSION}_${SDK_PLATFORM}_minimal.tar.xz"
    URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}/${ARCHIVE}"
    wget -q --show-progress -O "/tmp/${ARCHIVE}" "$URL"
    tar -xf "/tmp/${ARCHIVE}" -C "$REPO/sdks"
    "$SDK_DIR/setup.sh" -t "$SDK_TOOLCHAIN" -c
    echo "  SDK installed at $SDK_DIR"
fi

# ── export ───────────────────────────────────────────────────────────────────
export ZEPHYR_SDK_INSTALL_DIR="$SDK_DIR"
export ZEPHYR_BASE="$REPO/zephyr"

# ── status ───────────────────────────────────────────────────────────────────
_branch=$(git -C "$REPO" branch --show-current 2>/dev/null || echo "(detached)")
_zver=$(git -C "$REPO/zephyr" describe --tags 2>/dev/null || echo "?")
echo "Branch : $_branch"
echo "Zephyr : $_zver"
echo "SDK    : ${SDK_VERSION:-?}  ($SDK_DIR)"
