#!/bin/bash
# cibuildwheel before-build hook for far-unitree-sdk.
#
# Runs inside the manylinux container in the per-wheel Python environment
# (so `python` / `pip` here are the interpreter the wheel is being built for).
# Compiles the pybind11 extension against that Python, then stages the compiled
# module + vendored FastDDS libraries into the unitree_interface/ package so
# setuptools packages them and auditwheel can repair the result.
set -euo pipefail

PROJECT="${1:?usage: cibw_before_build.sh <project_dir>}"
cd "$PROJECT"

ARCH="$(uname -m)"
echo "[before-build] arch=$ARCH python=$(python -c 'import sys; print(sys.version)')"

# pybind11 provides the cmake config used by python_binding/CMakeLists.txt.
pip install "pybind11>=2.10" pybind11-stubgen

PYBIND11_DIR="$(python -c 'import pybind11; print(pybind11.get_cmake_dir())')"
PY_EXE="$(command -v python)"

# Clean any stale extension modules so we package exactly this build.
rm -f unitree_interface/*.so unitree_interface/*.so.*

BUILD_DIR="build_cibw"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cmake -S . -B "$BUILD_DIR" \
    -DBUILD_PYTHON_BINDING=ON \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON \
    -DCMAKE_INSTALL_RPATH='$ORIGIN' \
    -DPYTHON_EXECUTABLE="$PY_EXE" \
    -DPython3_EXECUTABLE="$PY_EXE" \
    -Dpybind11_DIR="$PYBIND11_DIR" \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF
cmake --build "$BUILD_DIR" -j"$(nproc)"

# Stage the freshly compiled extension module.
find "$BUILD_DIR" -name "unitree_interface.cpython-*.so" -exec cp -v {} unitree_interface/ \;

# Stage vendored DDS runtime libraries for this architecture. auditwheel will
# pull these into the wheel and rewrite the RPATH; we also keep them adjacent
# so the local build tree stays importable.
cp -v "thirdparty/lib/$ARCH/"*.so* unitree_interface/ 2>/dev/null || true

# Recreate the versioned FastRTPS/FastCDR symlinks the original build.sh made,
# matching the SONAMEs the extension links against.
( cd unitree_interface
  [ -f libfastrtps.so ] && ln -sf libfastrtps.so libfastrtps.so.2.13 || true
  [ -f libfastcdr.so ]  && ln -sf libfastcdr.so  libfastcdr.so.2     || true
)

# Stage the type stub. The authoritative, hand-maintained stub lives at
# python_binding/unitree_interface.pyi and is the source of truth for the public
# API — prefer it so the wheel is reproducible and does not depend on
# pybind11-stubgen succeeding inside the manylinux container (importing the freshly
# linked extension there is fragile because of the vendored DDS .so deps).
# pybind11-stubgen is only a fallback for a checkout that somehow lacks the committed
# stub, and it must actually produce the file — no silent skip.
if [ -f python_binding/unitree_interface.pyi ]; then
    cp -v python_binding/unitree_interface.pyi unitree_interface/unitree_interface.pyi
    echo "[before-build] staged committed stub python_binding/unitree_interface.pyi"
elif PYTHONPATH="unitree_interface:${PYTHONPATH:-}" \
     LD_LIBRARY_PATH="$PWD/unitree_interface:${LD_LIBRARY_PATH:-}" \
     pybind11-stubgen -o "$BUILD_DIR/stubs" unitree_interface \
     && [ -f "$BUILD_DIR/stubs/unitree_interface.pyi" ]; then
    cp -v "$BUILD_DIR/stubs/unitree_interface.pyi" unitree_interface/unitree_interface.pyi
    echo "[before-build] staged stubgen-generated stub"
else
    echo "[before-build] ERROR: no unitree_interface.pyi available (committed stub missing and stubgen failed)" >&2
    exit 1
fi

# Ship py.typed ONLY alongside a real .pyi. A py.typed marker without a stub makes
# mypy treat the compiled extension as a typed-but-empty module and report
# `attr-defined` on every symbol downstream (exactly the holosoma CI break this
# fixes) — worse than shipping no type info at all. Fail loudly rather than
# regress that.
if [ ! -f unitree_interface/unitree_interface.pyi ]; then
    echo "[before-build] ERROR: refusing to write py.typed without a .pyi stub" >&2
    exit 1
fi
touch unitree_interface/py.typed
echo "[before-build] staged files:"
ls -la unitree_interface/
