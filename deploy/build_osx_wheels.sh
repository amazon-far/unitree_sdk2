#!/bin/bash
set -e

# macOS wheel builder for unitree_sdk2
# This script clones the official unitree_sdk2 repo and builds wheels using pip

# Configuration
REPO_URL=${REPO_URL:-"https://github.com/unitreerobotics/unitree_sdk2_python.git"}
BRANCH=${BRANCH:-"main"}
PYTHON_VERSIONS=${PYTHON_VERSIONS:-"3.8 3.9 3.10 3.11"}

# Detect architecture and macOS version
ARCH=$(uname -m)
MACOS_VERSION=$(sw_vers -productVersion | cut -d '.' -f 1)

if [ "$ARCH" = "x86_64" ]; then
    PLATFORM="macosx_${MACOS_VERSION}_0_x86_64"
elif [ "$ARCH" = "arm64" ]; then
    # macOS 11 (Big Sur) was the first to support arm64
    # Use minimum version of 11 for compatibility
    MIN_VERSION=11
    if [ "$MACOS_VERSION" -lt "$MIN_VERSION" ]; then
        MACOS_VERSION=$MIN_VERSION
    fi
    PLATFORM="macosx_${MACOS_VERSION}_0_arm64"
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi

echo "=========================================="
echo "macOS Wheel Builder for unitree_sdk2"
echo "=========================================="
echo "Repository: $REPO_URL"
echo "Branch: $BRANCH"
echo "Python versions: $PYTHON_VERSIONS"
echo "Architecture: $ARCH"
echo "Platform tag: $PLATFORM"
echo ""

# Create output directory in original location (before changing to temp dir)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
OUTPUT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/dist"
mkdir -p "$OUTPUT_DIR"
echo "Output directory: $OUTPUT_DIR"
echo ""

# Create temporary directory
TEMP_DIR=$(mktemp -d)
echo "Working directory: $TEMP_DIR"

# Cleanup function
cleanup() {
    echo "Cleaning up temporary directory..."
    rm -rf "$TEMP_DIR"
}
trap cleanup EXIT

# Clone repository
echo "Cloning unitree_sdk2 repository..."
git clone "$REPO_URL" "$TEMP_DIR/unitree_sdk2"
cd "$TEMP_DIR/unitree_sdk2"

# Get version from this repo's pyproject.toml (not the cloned repo)
UNITREE_TAG=$(grep '^version = ' "$SCRIPT_DIR/../pyproject.toml" | sed 's/version = "\(.*\)"/\1/')
echo "Version: $UNITREE_TAG"
echo ""

# Build wheels for each Python version
for PYTHON_VER in $PYTHON_VERSIONS; do
    echo "=========================================="
    echo "Building wheel for Python $PYTHON_VER"
    echo "=========================================="
    
    # Find Python executable
    if command -v python${PYTHON_VER} &> /dev/null; then
        PYTHON_EXE="python${PYTHON_VER}"
    elif command -v python3 &> /dev/null && python3 --version | grep -q "$PYTHON_VER"; then
        PYTHON_EXE="python3"
    else
        echo "Warning: Python $PYTHON_VER not found, skipping..."
        continue
    fi
    
    echo "Using Python: $PYTHON_EXE ($(${PYTHON_EXE} --version))"
    
    # Install build dependencies
    echo "Installing build dependencies..."
    ${PYTHON_EXE} -m pip install --upgrade pip setuptools wheel build
    
    # Get Python tag
    PY_TAG=$(${PYTHON_EXE} -c "import sys; print(f'cp{sys.version_info.major}{sys.version_info.minor}')")
    echo "Setting py tag as: ${PY_TAG}"
    
    # Build wheel directly into output directory and rename
    echo "Building wheel with pip..."
    ${PYTHON_EXE} -m pip wheel . --no-deps --no-cache-dir -w "$OUTPUT_DIR" && \
    cd "$OUTPUT_DIR" && \
    LATEST_WHEEL=$(ls -t unitree_sdk2*.whl 2>/dev/null | head -1) && \
    if [ -n "$LATEST_WHEEL" ] && [ -f "$LATEST_WHEEL" ] && [[ ! "$LATEST_WHEEL" =~ ${PY_TAG}-${PY_TAG}-${PLATFORM}\.whl$ ]]; then
        new_name="unitree_sdk2-${UNITREE_TAG}-${PY_TAG}-${PY_TAG}-${PLATFORM}.whl"
        mv "$LATEST_WHEEL" "$new_name" && echo "Created: $new_name"
    else
        echo "Wheel: ${LATEST_WHEEL:-not found}"
    fi
    cd "$TEMP_DIR/unitree_sdk2"
    echo "✓ Wheel for Python $PYTHON_VER completed"
    echo ""
done

echo "=========================================="
echo "All wheels built successfully!"
echo "Output directory: $OUTPUT_DIR"
ls -lh "$OUTPUT_DIR"/*.whl 2>/dev/null || echo "No wheels found"
echo "=========================================="
