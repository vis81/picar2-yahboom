#!/usr/bin/env bash
# Universal Zephyr workspace manager.
#
# LAYOUT (all inside this directory):
#
#   yahboom/                   ← you are here (app source + manager root)
#   ├── setup-workspace.sh
#   ├── Makefile
#   ├── sdks/                  ← shared SDKs (one copy, referenced by all workspaces)
#   │   └── zephyr-sdk-0.17.3/
#   └── ws/                    ← versioned workspaces
#       └── v3.7.0/
#           ├── manifests/west.yml
#           ├── .west/
#           ├── zephyr/
#           ├── modules/
#           ├── .venv/
#           └── activate.sh
#
# COMMANDS:
#   new <zephyr-ver> <sdk-ver> [workspace-name]
#         Create a new versioned workspace under ws/.
#         zephyr-ver : e.g. v3.7.0
#         sdk-ver    : e.g. 0.17.0
#         name       : defaults to <zephyr-ver>
#
#   sdk download <sdk-ver>
#         Download and install an SDK into sdks/.
#
#   sdk list
#         List locally available SDKs.
#
#   list  List all workspaces under ws/.
#
#   help  Show this message.
#
# ENVIRONMENT:
#   SDK_PLATFORM  Platform string  (default: linux-x86_64)
#   TOOLCHAIN     Toolchain target (default: arm-zephyr-eabi)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_BASE="$SCRIPT_DIR/sdks"
WS_BASE="$SCRIPT_DIR/ws"
SDK_PLATFORM="${SDK_PLATFORM:-linux-x86_64}"
TOOLCHAIN="${TOOLCHAIN:-arm-zephyr-eabi}"
MODULES="${MODULES:-cmsis hal_stm32 mcuboot}"

SDK_RELEASES_URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download"

# ── helpers ─────────────────────────────────────────────────────────────────

die()  { printf "ERROR: %b\n" "$*" >&2; exit 1; }
info() { echo "  $*"; }
hr()   { echo "──────────────────────────────────────────────"; }

sdk_dir() { echo "$SDK_BASE/zephyr-sdk-${1}"; }
ws_dir()  { echo "$WS_BASE/${1}"; }

# ── subcommands ──────────────────────────────────────────────────────────────

cmd_help() {
    sed -n '/^# LAYOUT/,/^[^#]/p' "$0" | grep '^#' | sed 's/^# \{0,3\}//'
}

cmd_sdk_list() {
    echo "Available SDKs in $SDK_BASE:"
    if [[ ! -d "$SDK_BASE" ]] || [[ -z "$(ls "$SDK_BASE"/zephyr-sdk-* 2>/dev/null)" ]]; then
        echo "  (none)"
        return
    fi
    for d in "$SDK_BASE"/zephyr-sdk-*/; do
        [[ -d "$d" ]] || continue
        local ver
        ver="$(basename "$d" | sed 's/zephyr-sdk-//')"
        echo "  $ver  →  $d"
    done
}

cmd_sdk_download() {
    local ver="${1:?sdk version required (e.g. 0.17.0)}"
    local dest
    dest="$(sdk_dir "$ver")"

    if [[ -d "$dest" ]]; then
        if [[ -d "$dest/$TOOLCHAIN" ]]; then
            info "SDK $ver already installed at $dest"
            return 0
        fi
        info "SDK $ver directory exists but toolchain $TOOLCHAIN is missing — running setup ..."
        "$dest/setup.sh" -t "$TOOLCHAIN" -c
        info "SDK $ver toolchain installed."
        return 0
    fi

    local pkg="zephyr-sdk-${ver}_${SDK_PLATFORM}_minimal.tar.xz"
    local url="${SDK_RELEASES_URL}/v${ver}/${pkg}"
    local download_dir="$SDK_BASE/downloads"
    local archive="$download_dir/$pkg"

    mkdir -p "$download_dir"

    if [[ ! -f "$archive" ]]; then
        info "Downloading $pkg ..."
        wget --show-progress -q -O "$archive" "$url" || {
            rm -f "$archive"
            die "Download failed: $url"
        }
    else
        info "Using cached archive: $archive"
    fi

    info "Extracting ..."
    mkdir -p "$SDK_BASE"
    tar -xf "$archive" -C "$SDK_BASE"

    local extracted_dir
    # || true absorbs SIGPIPE (141) that tar receives when head closes the pipe early
    extracted_dir=$( { tar -tf "$archive" || true; } | head -1 | cut -d/ -f1 )
    local expected="zephyr-sdk-${ver}"
    if [[ "$extracted_dir" != "$expected" ]] && [[ -d "$SDK_BASE/$extracted_dir" ]]; then
        mv "$SDK_BASE/$extracted_dir" "$dest"
    fi

    info "Running SDK setup (toolchain: $TOOLCHAIN) ..."
    "$dest/setup.sh" -t "$TOOLCHAIN" -c

    info "SDK $ver installed at $dest"
}

cmd_list() {
    echo "Workspaces in $WS_BASE:"
    local found=0
    for d in "$WS_BASE"/*/; do
        [[ -d "$d/.west" ]] || continue
        local name
        name="$(basename "$d")"
        local zephyr_ver="(unknown)"
        if [[ -d "$d/zephyr" ]]; then
            zephyr_ver=$(git -C "$d/zephyr" describe --tags 2>/dev/null \
                      || git -C "$d/zephyr" rev-parse --short HEAD 2>/dev/null \
                      || echo "(unknown)")
        fi
        local sdk_info="(none)"
        if [[ -f "$d/activate.sh" ]]; then
            sdk_info=$(grep -o 'ZEPHYR_SDK_INSTALL_DIR=.*' "$d/activate.sh" 2>/dev/null \
                     | sed 's/.*zephyr-sdk-//;s/".*//' | tr -d '\n')
        fi
        echo "  $name  OS=$zephyr_ver  SDK=$sdk_info  →  $d"
        found=1
    done
    [[ $found -eq 1 ]] || echo "  (none)"
}

cmd_new() {
    local zephyr_ver="${1:?zephyr version required (e.g. v3.7.0)}"
    local sdk_ver="${2:?sdk version required (e.g. 0.17.0)}"
    local ws_name="${3:-${zephyr_ver}}"
    local ws_path
    ws_path="$(ws_dir "$ws_name")"

    local manifest_file="west.yml"

    local sdk_path
    sdk_path="$(sdk_dir "$sdk_ver")"
    if [[ ! -d "$sdk_path" ]]; then
        echo "SDK $sdk_ver not found at $sdk_path"
        read -r -p "Download it now? [y/N] " answer
        [[ "$answer" =~ ^[Yy]$ ]] || \
            die "Aborting. Run:  $0 sdk download $sdk_ver"
        cmd_sdk_download "$sdk_ver"
    fi

    if [[ -d "$ws_path" ]]; then
        die "Workspace already exists: $ws_path\nDelete it first to recreate."
    fi

    hr
    echo "  Zephyr OS : $zephyr_ver"
    echo "  SDK       : $sdk_ver  ($sdk_path)"
    echo "  Workspace : $ws_path"
    hr

    mkdir -p "$ws_path"
    cd "$ws_path"

    mkdir -p manifests
    local allowlist=""
    for mod in $MODULES; do
        allowlist+="          - $mod"$'\n'
    done
    cat > "manifests/$manifest_file" <<MANIFEST
manifest:
  remotes:
    - name: zephyrproject-rtos
      url-base: https://github.com/zephyrproject-rtos
  projects:
    - name: zephyr
      remote: zephyrproject-rtos
      revision: ${zephyr_ver}
      import:
        name-allowlist:
${allowlist}
MANIFEST

    python3 -m venv .venv
    # shellcheck source=/dev/null
    source .venv/bin/activate
    pip install -q west

    west init -l manifests --mf "$manifest_file"

    west update

    pip install -q -r zephyr/scripts/requirements.txt

    local sdk_dirname
    sdk_dirname="$(basename "$sdk_path")"
    cat > activate.sh <<EOF
#!/usr/bin/env bash
# Activate the ${zephyr_ver} workspace environment.
# Usage: source activate.sh  (from anywhere)

WORKSPACE="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"

source "\$WORKSPACE/.venv/bin/activate"

export ZEPHYR_SDK_INSTALL_DIR="\$WORKSPACE/../../sdks/${sdk_dirname}"
export ZEPHYR_BASE="\$WORKSPACE/zephyr"

echo "Workspace : \$WORKSPACE"
echo "Zephyr OS : \$(cd "\$ZEPHYR_BASE" && git describe --tags 2>/dev/null || git rev-parse --short HEAD)"
echo "SDK       : \$ZEPHYR_SDK_INSTALL_DIR"
EOF
    chmod +x activate.sh

    hr
    echo "  Done. To use this workspace:"
    echo ""
    echo "    source ws/${ws_name}/activate.sh"
    hr
}

# ── dispatch ─────────────────────────────────────────────────────────────────

CMD="${1:-help}"
shift || true

case "$CMD" in
    new)         cmd_new "$@" ;;
    sdk)
        SUBCMD="${1:?sdk subcommand required: download | list}"
        shift
        case "$SUBCMD" in
            download) cmd_sdk_download "$@" ;;
            list)     cmd_sdk_list ;;
            *) die "Unknown sdk subcommand: $SUBCMD  (download | list)" ;;
        esac
        ;;
    list)        cmd_list ;;
    help|--help) cmd_help ;;
    *)           die "Unknown command: $CMD  (run '$0 help')" ;;
esac
