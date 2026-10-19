#!/usr/bin/env bash
# Fetches what the robot_console's 3D view needs, once per machine:
#   three.js + urdf-loader -> $DATA/vendor          (pinned versions below)
#   the Unitree G1 URDF + meshes -> $DATA/g1_description
# Without them the page falls back to its list view.
#   bridge/robot_console/scripts/fetch_assets.sh [--force]
#
# Env:
#   ROBOT_CONSOLE_DATA  destination root (default ~/.local/share/robot_console)
#   G1_DESCRIPTION_REPO upstream remote
#   G1_DESCRIPTION_REF  branch or tag to fetch

set -euo pipefail

DATA="${ROBOT_CONSOLE_DATA:-$HOME/.local/share/robot_console}"
VENDOR="$DATA/vendor"
DEST="$DATA/g1_description"
THREE=0.180.0
URDF_LOADER=0.13.1
CDN="https://cdn.jsdelivr.net/npm"
UPSTREAM="${G1_DESCRIPTION_REPO:-https://github.com/unitreerobotics/unitree_ros.git}"
UPSTREAM_REF="${G1_DESCRIPTION_REF:-master}"
SUBTREE="robots/g1_description"

# Most-wanted first: the tool commands the Dex3 hands, so prefer a with-hand 29-DoF model.
URDF_PREFERENCE=(
    "g1_29dof_with_hand*.urdf"
    "g1_29dof*.urdf"
    "g1_*dof_with_hand*.urdf"
    "g1_*.urdf"
    "*.urdf"
)

FORCE=0
for arg in "$@"; do
    case "$arg" in
        --force) FORCE=1 ;;
        -h|--help)
            sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "unknown argument: $arg -- expected --force or --help" >&2
            exit 2
            ;;
    esac
done

command -v git >/dev/null 2>&1 || {
    echo "git not found on PATH -- install git, then re-run $0" >&2
    exit 1
}

# Pick the best URDF present under $DEST, echoing its absolute path. Empty if none.
pick_urdf() {
    local pattern found
    for pattern in "${URDF_PREFERENCE[@]}"; do
        found="$(find "$DEST" -type f -name "$pattern" -print 2>/dev/null | sort | head -n 1)"
        if [ -n "$found" ]; then
            echo "$found"
            return 0
        fi
    done
    return 0
}

# True when $DEST already holds a usable URDF plus at least one mesh.
assets_present() {
    local urdf
    [ -d "$DEST" ] || return 1
    urdf="$(pick_urdf)"
    [ -n "$urdf" ] && [ -s "$urdf" ] || return 1
    [ -n "$(find "$DEST" -type d -name meshes -print -quit 2>/dev/null)" ] || return 1
    [ -n "$(find "$DEST" -type f \( -name '*.STL' -o -name '*.stl' -o -name '*.dae' -o -name '*.obj' \) -print -quit 2>/dev/null)" ]
}

# --- three.js + urdf-loader ------------------------------------------------
# path under $VENDOR -> URL. three.module.js imports three.core.js; URDFLoader
# imports STLLoader and ColladaLoader, which imports TGALoader.
VENDOR_FILES=(
    "three/three.module.js $CDN/three@$THREE/build/three.module.js"
    "three/three.core.js $CDN/three@$THREE/build/three.core.js"
    "three/examples/jsm/loaders/STLLoader.js $CDN/three@$THREE/examples/jsm/loaders/STLLoader.js"
    "three/examples/jsm/loaders/ColladaLoader.js $CDN/three@$THREE/examples/jsm/loaders/ColladaLoader.js"
    "three/examples/jsm/loaders/TGALoader.js $CDN/three@$THREE/examples/jsm/loaders/TGALoader.js"
    "three/examples/jsm/controls/OrbitControls.js $CDN/three@$THREE/examples/jsm/controls/OrbitControls.js"
    "urdf-loader/URDFLoader.js $CDN/urdf-loader@$URDF_LOADER/src/URDFLoader.js"
    "urdf-loader/URDFClasses.js $CDN/urdf-loader@$URDF_LOADER/src/URDFClasses.js"
)
command -v curl >/dev/null 2>&1 || { echo "curl not found on PATH" >&2; exit 1; }
for entry in "${VENDOR_FILES[@]}"; do
    rel="${entry%% *}"
    url="${entry#* }"
    out="$VENDOR/$rel"
    if [ "$FORCE" -eq 0 ] && [ -s "$out" ]; then
        continue
    fi
    mkdir -p "$(dirname "$out")"
    curl -fsSL -o "$out.part" "$url" || { echo "download failed: $url" >&2; rm -f "$out.part"; exit 1; }
    [ -s "$out.part" ] || { echo "empty download: $url" >&2; exit 1; }
    mv "$out.part" "$out"
done
echo "vendor:         $VENDOR (three $THREE, urdf-loader $URDF_LOADER)"

# --- G1 description ----------------------------------------------------------
if [ "$FORCE" -eq 0 ] && assets_present; then
    echo "g1_description already present -- skipping fetch (re-run with --force to refetch)"
    echo "config line:    urdf: $(pick_urdf)"
    exit 0
fi

TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/g1_description.XXXXXX")"
cleanup() { rm -rf "$TMP_DIR"; }
trap cleanup EXIT

echo "fetching $SUBTREE from $UPSTREAM ($UPSTREAM_REF) -- sparse checkout, no other robots"
if ! git clone \
    --depth 1 \
    --filter=blob:none \
    --sparse \
    --branch "$UPSTREAM_REF" \
    "$UPSTREAM" "$TMP_DIR/unitree_ros" >/dev/null 2>"$TMP_DIR/git.err"; then
    echo "git clone failed -- no network, a proxy in the way, or a bad ref '$UPSTREAM_REF'" >&2
    echo "  upstream: $UPSTREAM" >&2
    sed 's/^/  git: /' "$TMP_DIR/git.err" >&2
    exit 1
fi

if ! git -C "$TMP_DIR/unitree_ros" sparse-checkout set "$SUBTREE" >/dev/null 2>"$TMP_DIR/git.err"; then
    echo "sparse-checkout of '$SUBTREE' failed -- upstream layout may have changed" >&2
    sed 's/^/  git: /' "$TMP_DIR/git.err" >&2
    exit 1
fi

SRC="$TMP_DIR/unitree_ros/$SUBTREE"
if [ ! -d "$SRC" ]; then
    echo "'$SUBTREE' is not in $UPSTREAM at ref '$UPSTREAM_REF' -- upstream moved it" >&2
    echo "top-level robots/ entries found:" >&2
    find "$TMP_DIR/unitree_ros/robots" -mindepth 1 -maxdepth 1 2>/dev/null | sort | sed 's/^/  /' >&2
    exit 1
fi

if [ "$FORCE" -eq 1 ] && [ -d "$DEST" ]; then
    echo "--force: clearing $DEST"
    rm -rf "${DEST:?}"
fi

mkdir -p "$DEST"
cp -R "$SRC/." "$DEST/"

# --- verify concretely, do not trust the copy ------------------------------
URDF="$(pick_urdf)"
if [ -z "$URDF" ]; then
    echo "fetch completed but no .urdf landed in $DEST -- upstream layout changed" >&2
    echo "what was installed:" >&2
    find "$DEST" -maxdepth 2 -mindepth 1 | sort | sed 's/^/  /' >&2
    exit 1
fi
if [ ! -s "$URDF" ]; then
    echo "URDF $URDF is empty -- refetch with --force" >&2
    exit 1
fi

MESH_DIR="$(find "$DEST" -type d -name meshes -print -quit 2>/dev/null || true)"
if [ -z "$MESH_DIR" ]; then
    echo "no meshes/ directory under $DEST -- the model will render as empty links" >&2
    echo "directories installed:" >&2
    find "$DEST" -maxdepth 2 -type d -mindepth 1 | sort | sed 's/^/  /' >&2
    exit 1
fi
MESH_COUNT="$(find "$MESH_DIR" -type f | wc -l | tr -d ' ')"
if [ "$MESH_COUNT" -eq 0 ]; then
    echo "$MESH_DIR is empty -- LFS pointers or a partial checkout; retry with --force" >&2
    exit 1
fi

# The frontend needs the exact filename, and upstream's naming is not guaranteed,
# so always show every URDF that shipped alongside the one we picked.
echo
echo "installed into: $DEST"
echo "meshes:         $MESH_DIR ($MESH_COUNT files)"
echo "URDF:           $URDF"
echo
echo "config line:    urdf: $URDF"
echo "all URDFs available (console.urdf may name any of these):"
find "$DEST" -type f -name '*.urdf' | sort | sed 's/^/  /'
