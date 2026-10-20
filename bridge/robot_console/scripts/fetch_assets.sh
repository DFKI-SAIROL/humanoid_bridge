#!/usr/bin/env bash
# Fetches what the robot_console's 3D view needs, once per machine:
#   three.js + urdf-loader -> $DATA/vendor                  (pinned versions below)
#   the G1 model           -> $DATA/g1_mjlab or $DATA/g1_description, plus a SOURCE file
# Without them the page falls back to its list view.
#   bridge/robot_console/scripts/fetch_assets.sh [--model mjlab|unitree] [--force]
#
#   mjlab    mjlab's g1.xml (29 DoF rev 1.0, rubber hands), converted to URDF (default)
#   unitree  unitree_ros robots/g1_description (with Dex3 hands)
#
# Env:
#   ROBOT_CONSOLE_DATA  destination root (default ~/.local/share/robot_console)

set -euo pipefail

DATA="${ROBOT_CONSOLE_DATA:-$HOME/.local/share/robot_console}"
VENDOR="$DATA/vendor"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THREE=0.180.0
URDF_LOADER=0.13.1
CDN="https://cdn.jsdelivr.net/npm"

MJLAB_REPO=https://github.com/mujocolab/mjlab.git
MJLAB_SHA=6fc2a60264383732d926923a9fff30e0a3899157   # last g1.xml change, 2026-05-24
MJLAB_DIR=src/mjlab/asset_zoo/robots/unitree_g1/xmls
UNITREE_REPO=https://github.com/unitreerobotics/unitree_ros.git
UNITREE_REF=master
UNITREE_DIR=robots/g1_description

MODEL=mjlab
FORCE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --force) FORCE=1 ;;
        --model) MODEL="${2:-}"; shift ;;
        -h|--help) sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1 -- expected --model, --force or --help" >&2; exit 2 ;;
    esac
    shift
done
case "$MODEL" in
    mjlab) DEST="$DATA/g1_mjlab" ;;
    unitree) DEST="$DATA/g1_description" ;;
    *) echo "--model must be mjlab or unitree, not '$MODEL'" >&2; exit 2 ;;
esac

for tool in git curl python3; do
    command -v "$tool" >/dev/null 2>&1 || { echo "$tool not found on PATH" >&2; exit 1; }
done

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

# --- G1 model ----------------------------------------------------------------
if [ "$FORCE" -eq 0 ] && [ -s "$DEST/SOURCE" ]; then
    echo "$DEST already present -- skipping fetch (re-run with --force to refetch)"
    echo "model:          $(cat "$DEST/SOURCE")"
    exit 0
fi

TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/robot_console_model.XXXXXX")"
trap 'rm -rf "$TMP_DIR"' EXIT

# sparse_fetch <repo> <ref> <subdir>: checks out only <subdir> of <ref> into $TMP_DIR/repo.
sparse_fetch() {
    local repo="$1" ref="$2" subdir="$3" clone="$TMP_DIR/repo"
    echo "fetching $subdir from $repo @ $ref -- sparse, depth 1"
    git init -q "$clone"
    git -C "$clone" remote add origin "$repo"
    git -C "$clone" sparse-checkout set "$subdir"
    if ! git -C "$clone" fetch -q --depth 1 --filter=blob:none origin "$ref" 2>"$TMP_DIR/git.err" \
        || ! git -C "$clone" checkout -q FETCH_HEAD 2>>"$TMP_DIR/git.err"; then
        echo "git fetch failed -- no network, a proxy in the way, or a bad ref '$ref'" >&2
        sed 's/^/  git: /' "$TMP_DIR/git.err" >&2
        exit 1
    fi
    [ -d "$clone/$subdir" ] || { echo "'$subdir' is not in $repo at '$ref'" >&2; exit 1; }
}

STAGE="$TMP_DIR/stage"
mkdir -p "$STAGE"
if [ "$MODEL" = mjlab ]; then
    sparse_fetch "$MJLAB_REPO" "$MJLAB_SHA" "$MJLAB_DIR"
    cp "$TMP_DIR/repo/$MJLAB_DIR/g1.xml" "$STAGE/g1.xml"
    cp -R "$TMP_DIR/repo/$MJLAB_DIR/assets" "$STAGE/meshes"
    python3 "$HERE/mjcf_to_urdf.py" "$STAGE/g1.xml" "$STAGE/g1.urdf" --mesh-dir meshes
    URDF="$DEST/g1.urdf"
    echo "mjlab @ ${MJLAB_SHA:0:7} $MJLAB_DIR/g1.xml" >"$STAGE/SOURCE"
else
    sparse_fetch "$UNITREE_REPO" "$UNITREE_REF" "$UNITREE_DIR"
    cp -R "$TMP_DIR/repo/$UNITREE_DIR/." "$STAGE/"
    URDF="$DEST/g1_29dof_with_hand.urdf"
    echo "unitree_ros @ $(git -C "$TMP_DIR/repo" rev-parse --short FETCH_HEAD) $UNITREE_DIR" >"$STAGE/SOURCE"
fi

# --- verify, then install --------------------------------------------------------
[ -s "$STAGE/$(basename "$URDF")" ] || { echo "no $(basename "$URDF") after the fetch -- upstream layout changed" >&2; exit 1; }
MESH_COUNT="$(find "$STAGE/meshes" -type f 2>/dev/null | wc -l | tr -d ' ')"
[ "$MESH_COUNT" -gt 0 ] || { echo "no meshes after the fetch -- LFS pointers or a partial checkout" >&2; exit 1; }

rm -rf "${DEST:?}"
mkdir -p "$(dirname "$DEST")"
mv "$STAGE" "$DEST"

echo
echo "installed into: $DEST ($MESH_COUNT meshes)"
echo "model:          $(cat "$DEST/SOURCE")"
echo "config line:    urdf: $URDF"
