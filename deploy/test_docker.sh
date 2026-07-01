#!/bin/bash
set -e

ARCH=$1
PY_VER=$2

if [ -z "$ARCH" ] || [ -z "$PY_VER" ]; then
    echo "Usage: $0 <x86_64|aarch64> <3.10|3.11>"
    exit 1
fi

CP_TAG="cp${PY_VER//./}"
# Match the cibuildwheel/auditwheel output name, e.g.
# far_unitree_sdk-0.1.4-cp311-cp311-manylinux_2_31_x86_64.whl
WHEEL_GLOB="far_unitree_sdk-*-${CP_TAG}-${CP_TAG}-manylinux*_${ARCH}.whl"

echo "Testing wheel matching '$WHEEL_GLOB' on $ARCH with Python $PY_VER"

docker run --rm --platform linux/$ARCH \
    -v "$(pwd)/dist:/wheels:ro" \
    -v "$(pwd)/test_wheels.py:/test.py:ro" \
    python:${PY_VER}-slim \
    bash -c "set -e; whl=\$(ls /wheels/$WHEEL_GLOB | head -1); echo \"Installing \$whl\"; pip install -q \"\$whl\" && python /test.py"
