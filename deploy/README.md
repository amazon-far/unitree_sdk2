# Wheel Build & Test Scripts

## Build Wheels

Build Python wheels for specific versions and architectures:

```bash
# Build for Python 3.11, both architectures
PYTHON_VERSIONS="3.11" ./build_wheels.sh

# Build for specific architecture
PYTHON_VERSIONS="3.11" ARCHITECTURES="x86_64" ./build_wheels.sh

# Build multiple versions
PYTHON_VERSIONS="3.10 3.11" ./build_wheels.sh
```

Wheels are output to `dist/` directory.

## Test Wheels

Test a specific wheel:
```bash
./test_docker.sh x86_64 3.11
./test_docker.sh aarch64 3.10
```

Test all wheels:
```bash
./run_tests.sh
```

## Requirements

- Docker with buildx support
- Wheels must be in `../dist/` directory

### Setup Docker with buildx and QEMU (Ubuntu 22.04)

How to set up QEMU for Docker multi-architecture builds on your Linux system:


#### 1. Install QEMU and binfmt support

```bash
sudo apt-get update
sudo apt-get install -y qemu-user-static binfmt-support
```

This installs:
- `qemu-user-static`: Provides user-mode emulation for different architectures
- `binfmt-support`: Enables the kernel to execute foreign architecture binaries

#### 2. Register QEMU with Docker buildx

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

This command:
- Runs a privileged container that registers QEMU interpreters with the kernel
- The `--reset` flag clears any existing registrations
- The `-p yes` flag makes the registration persistent

#### 3. Create/update Docker buildx builder

```bash
docker buildx create --name multiarch --driver docker-container --use
docker buildx inspect --bootstrap
```

This creates a new builder instance that supports multiple platforms.

#### 4. Verify the setup

```bash
docker buildx ls
```

You should see your builder with multiple platforms listed, including `linux/amd64` and `linux/arm64`.

### Notes

- The QEMU registration persists across reboots on most systems
- Building for ARM64 on x86_64 will be slower due to emulation overhead (expect 2-5x longer build times)
- If you only need x86_64 wheels, you can skip this setup and use: `PYTHON_VERSIONS="3.11" ARCHITECTURES="x86_64" ./build_wheels.sh`
