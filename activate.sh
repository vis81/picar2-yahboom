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
SDK_PLATFORM="${SDK_PLATFORM:-linux-x86_64}"
SDK_TOOLCHAIN="${SDK_TOOLCHAIN:-arm-zephyr-eabi}"

# ── SDK version from sdk-version.txt ────────────────────────────────────────
SDK_VERSION=$(tr -d '[:space:]' < "$REPO/sdk-version.txt")
SDK_DIR="$REPO/zephyr_os/sdks/zephyr-sdk-${SDK_VERSION}"

# ── venv ─────────────────────────────────────────────────────────────────────
if [[ ! -d "$REPO/.venv" ]]; then
    echo "  Creating Python venv ..."
    python3 -m venv "$REPO/.venv"
fi
# shellcheck source=/dev/null
source "$REPO/.venv/bin/activate"

# ── west ─────────────────────────────────────────────────────────────────────
if ! pip show west &>/dev/null 2>&1; then
    echo "  Installing west ..."
    pip install -q west
fi

# ── west init ────────────────────────────────────────────────────────────────
# west init -l creates .west/ in the PARENT directory by design (it treats the
# parent as the workspace root).  We instead write .west/config manually so
# the workspace root is the repo itself and all paths in west.yml resolve
# inside this directory.
if [[ ! -f "$REPO/.west/config" ]]; then
    echo "  Initialising west workspace ..."
    mkdir -p "$REPO/.west"
    cat > "$REPO/.west/config" << 'EOF'
[manifest]
path = .
file = west.yml
EOF
fi

# ── west update ──────────────────────────────────────────────────────────────
if [[ ! -d "$REPO/zephyr_os/zephyr" ]]; then
    echo "  Running west update (first time — this may take a few minutes) ..."
    (cd "$REPO" && west update)
fi

# ── Zephyr Python requirements ───────────────────────────────────────────────
_req="$REPO/zephyr_os/zephyr/scripts/requirements.txt"
_marker="$REPO/.venv/.zephyr_req_installed"
if [[ -f "$_req" ]] && [[ ! -f "$_marker" ]]; then
    echo "  Installing Zephyr Python requirements ..."
    pip install -q -r "$_req"
    touch "$_marker"
fi

# ── SDK ──────────────────────────────────────────────────────────────────────
if [[ ! -d "$SDK_DIR" ]]; then
    echo "  Downloading Zephyr SDK ${SDK_VERSION} ..."
    mkdir -p "$REPO/zephyr_os/sdks"
    ARCHIVE="zephyr-sdk-${SDK_VERSION}_${SDK_PLATFORM}_minimal.tar.xz"
    URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}/${ARCHIVE}"
    wget -q --show-progress -O "/tmp/${ARCHIVE}" "$URL"
    tar -xf "/tmp/${ARCHIVE}" -C "$REPO/zephyr_os/sdks"
    "$SDK_DIR/setup.sh" -t "$SDK_TOOLCHAIN" -c
    echo "  SDK installed at $SDK_DIR"
fi

# ── export ───────────────────────────────────────────────────────────────────
export ZEPHYR_SDK_INSTALL_DIR="$SDK_DIR"
export ZEPHYR_BASE="$REPO/zephyr_os/zephyr"

# ── status ───────────────────────────────────────────────────────────────────
_branch=$(git -C "$REPO" branch --show-current 2>/dev/null || echo "(detached)")
_zver=$(git -C "$REPO/zephyr_os/zephyr" describe --tags 2>/dev/null || echo "?")
echo "Branch : $_branch"
echo "Zephyr : $_zver"
echo "SDK    : ${SDK_VERSION}  ($SDK_DIR)"
