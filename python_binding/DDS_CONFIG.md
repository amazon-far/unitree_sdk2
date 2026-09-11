# Explicit DDS configuration (candidate API version 1)

`import unitree_interface as sdk` exports `DDS_CONFIG_API_VERSION = 1`, also
available from `unitree_interface.unitree_interface`. The API is prepared for
`far-unitree-sdk` 0.1.8; the capability marker is independent of the distribution
version. Consumer dependency pins are unchanged. Build and publish the release
from the merged commit via the existing `v0.1.8` tag workflow.

Every constructor and factory accepts a trailing **keyword-only** Python argument
`dds_config: Optional[str] = None`:

```python
sdk.UnitreeInterface(nic, robot_type, message_type, *, dds_config=None)
sdk.UnitreeInterface(nic, config, *, dds_config=None)  # config is RobotConfig
sdk.UnitreeInterface(nic, robot_type, message_type, num_motors, *, dds_config=None)
sdk.create_robot(nic, robot_type, message_type=sdk.MessageType.HG, *, dds_config=None)
sdk.create_robot_with_config(nic, config, *, dds_config=None)
# UnitreeInterface.create_g1/create_h1/create_h1_2/create_go2/create_custom
# retain their existing positional arguments/defaults and add the same keyword.
```

C++ counterparts take a trailing `const std::optional<std::string>&` defaulted to
`std::nullopt`. Existing source callers remain compatible; rebuilding C++ callers
is necessary because the binding class's native symbols change.

## Consumer contract

- Omitted/`None` calls the original `ChannelFactory::Init(0, nic)` path, without
  requiring a capability check in consumers. Existing 0.1.6/0.1.7 Python call
  shapes remain valid. No environment variable is read or written by this API.
- Explicit XML is **authoritative**: the NIC argument is unused. Supply all desired
  interfaces, peers and multicast settings in the document. There is no ROS URI
  alias, NIC merge, peer removal or fallback to NIC-only discovery.
- Check `getattr(sdk, "DDS_CONFIG_API_VERSION", None) == 1` **before constructing**
  any SDK object when opting in. Reject old wheels rather than retrying without
  the keyword. A marker check is not a network readiness/freshness check.
- Input is one complete inline `CycloneDDS` XML document with exactly one direct
  `Domain` element and explicit `Id="0"` or `Id="any"`. SDK domain is always 0,
  regardless of `ROS_DOMAIN_ID`. Profiles for other domains, multiple domains,
  legacy `<Id>` elements, filenames/URIs, BOMs, NULs and empty strings are rejected.
- Leading XML whitespace (space, tab, CR, LF only) is removed **only at native
  dispatch** because the archive interprets strings not starting with `<` as
  filenames. XML content is otherwise passed intact; no file is loaded by this
  opt-in API. XML declarations/default namespaces are allowed.
- XML syntax/envelope errors raise `ValueError` before native initialization and
  allow a corrected retry. Remaining Cyclone schema/interface errors propagate
  as native `RuntimeError`, never fallback. Once native initialization throws,
  subsequent attempts fail with `DDS initialization previously failed`; start a
  fresh isolated SDK process. The archive has no transactional rollback guarantee.
- A mutex serializes binding initialization. Successful initialization is retained
  for the process lifetime, even after all robot objects are destroyed or later
  channel construction fails. Repeated **byte-identical original** XML reuses it,
  including with a different unused NIC argument. Different XML (even equivalent
  formatting), legacy-to-explicit, explicit-to-legacy, or changed legacy NIC raises
  `RuntimeError` explaining that a fresh process is required. Same legacy NIC is
  reusable. Binding configuration must own the first SDK initialization: mixing
  direct external C++ `ChannelFactory::Init/Release` calls in the same process is
  unsupported; the archive exposes no initialization/config introspection.

Keep SDK CycloneDDS and ROS CycloneDDS in separate processes/libraries. The
sim-only MotionSwitcher responder remains opt-in and shares this configured
participant. Configure any separate Python SDK motion-release client before its
own first participant as well. Never enable the sim responder on a physical robot.

## Native path evidence

The x86_64 repository archive at v0.1.7 has SHA256
`f13bdc0a0b971766aef86260ba9ad12e51f89f1020194b00790d681b8957956f`.
Archive relocations establish `ChannelFactory::Init(JsonMap)` ->
`DdsFactoryModel::Init(JsonMap)` -> `DdsParameter::Init/GetParticipant` ->
`DdsParticipantParameter::GetConfig/GetDomainId` -> `DdsParticipant`.
A compiled probe confirms top-level `{"DomainId": uint32_t(0), "Config": xml}`
is copied to the participant parameter intact; this is the native call shape
used here. Do not call the filename overload with XML. `DdsParameter::GetConfig`
and `GetDomainId` are declared in the header but absent in this archive; the
probe uses `GetParticipant()` instead.

Native extension tests inspect Cyclone's actual effective-config trace for
interface `lo`, multicast false, both loopback peers and domain 0 with hostile
inherited ROS domain-42 configuration. They exercise native schema and interface
failures, all constructor/factory entry points, identity conflicts, retry policy,
capability exports and responder enable/idempotency. They do not perform a
MotionSwitcher RPC exchange or establish actual simulator state/commands.

The tests also found a pre-existing v0.1.7 command-writer destruction race. The
binding now calls the recurrent thread's virtual `Wait()` (requests quit/waits
for its callback) before resetting its pointer, matching the SDK reader shutdown
pattern. A baseline native no-config backtrace showed a freed bound callback in
`command_writer`; immediate construction/destruction is covered by regression tests.

## Local build and isolated tests

Use one supported Python (3.8–3.12), the repository's CMake build and original
vendored libraries; no release matrix or library replacement is required:

```sh
uv venv --python 3.11 /tmp/sdk-build-venv
uv pip install --python /tmp/sdk-build-venv/bin/python pybind11
cmake -S . -B /tmp/sdk-build \
  -DBUILD_PYTHON_BINDING=ON -DBUILD_EXAMPLES=OFF -DGENERATE_STUBS=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
  -DPYTHON_EXECUTABLE=/tmp/sdk-build-venv/bin/python \
  -Dpybind11_DIR=$(/tmp/sdk-build-venv/bin/python -m pybind11 --cmakedir)
cmake --build /tmp/sdk-build -j2
```

Stage the produced extension beside `unitree_interface/__init__.py`, the committed
`python_binding/unitree_interface.pyi`, and `thirdparty/lib/<arch>/*.so*` in a local
artifact directory. Set `PYTHONPATH` to that package's parent and `LD_LIBRARY_PATH`
to the staged library directory. Use a compatible Python/glibc runtime in an
owned container with **`--network none`**, read-only source mounts and no credentials:

```sh
# INSIDE that network-isolated container only:
python3.11 /sdk/python_binding_tests/test_dds_config.py -v
```

The test subprocesses inherit the container's network namespace. All peers are
loopback; these tests must not be run on host robot networks. Optionally set
`DDS_TEST_EVIDENCE_DIR` to retain the effective configuration trace.

Passing local tests is not A3 acceptance: exact installed GMP/Holosoma SDK artifacts,
separate auxiliary RPC, actual simulator low-state/odom/commands, source freshness,
peer loss and recovery still need independent runtime evidence. No publication,
physical-robot access, dependency upgrades or cluster deployment is implied.
