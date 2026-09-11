# unitree_sdk2
Unitree robot sdk version 2.

### Python package (PyPI)

The Python bindings are published to PyPI as **`far-unitree-sdk`** (the `far-`
prefix avoids colliding with upstream Unitree packages). The import name is
unchanged:

```bash
pip install far-unitree-sdk
```

```python
import unitree_interface  # import name is unchanged
```

Wheels are prebuilt `manylinux` binaries (Python 3.8/3.9/3.10/3.11, x86_64 and
aarch64) with the FastDDS runtime libraries bundled in — no compiler or system
DDS install required.

### Prebuild environment
* OS  (Ubuntu 20.04 LTS)  
* CPU  (aarch64 and x86_64)   
* Compiler  (gcc version 9.4.0) 

### Build examples

To build the examples inside this repository:

```bash
mkdir build
cd build
cmake ..
make
```

### Installation

To build your own application with the SDK, you can install the unitree_sdk2 to your system directory:

```bash
mkdir build
cd build
cmake ..
sudo make install
```

Or install unitree_sdk2 to a specified directory:

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

You can refer to `example/cmake_sample` on how to import the unitree_sdk2 into your CMake project. 

Note that if you install the library to other places other than `/opt/unitree_robotics`, you need to make sure the path is added to "${CMAKE_PREFIX_PATH}" so that cmake can find it with "find_package()".

### Python DDS configuration

The optional pre-construction `dds_config` API, capability marker, singleton/error
semantics and network-isolated native test recipe are documented in
[python_binding/DDS_CONFIG.md](python_binding/DDS_CONFIG.md). Native regression
tests live separately in [`python_binding_tests/`](python_binding_tests/).

### Notice
For more reference information, please go to [Unitree Document Center](https://support.unitree.com/home/zh/developer).
