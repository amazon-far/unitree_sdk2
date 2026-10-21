"""Native binding tests for the G1 low-state fields; run only in network isolation.

Written from g1-health/docs/BINDING_LOW_STATE_SPEC.md (revised 2026-10-16) alone, without reading the
change under test. Test names carry the spec's "Required behaviour" number (test_reqN_...).

Each case runs in a fresh process because ChannelFactory owns process-wide DDS.
No mocks: one interface on loopback publishes a LowState with publish_low_state()
and reads it back with read_low_state(). Any "CRC Error" in a child's output fails the test.

Req 7 has no test here: it is checked by running both files in one run from the repo root,
  python -m unittest python_binding_tests/test_dds_config.py python_binding_tests/test_low_state_fields.py
plus `git diff --exit-code dev -- python_binding_tests/test_dds_config.py` at review time.

Questions on the spec, and the interpretation these tests take (the least demanding one):
  Q1. Req 2 gives no ordering guarantee (timing is out of scope), so reads are not required to be
      monotonic. Each read's motor+message-level group and its IMU group must each be exactly one
      published message, no newer than the latest publish. Messages are identified by seed-encoded
      fields (motor 0's motorstate, tick, imu.temperature) and then compared whole against that seed.
  Q2. Req 2's alternating test publishes hundreds of increasing seeds; DDS may overwrite intermediate
      ones before a read, so it requires at least 3 distinct seeds observed and the final seed to
      arrive, not every seed.
  Q3. Req 2 excludes mode_machine. It is also left out of the req 2, 5 and 8 checks; only req 6 checks it.
  Q4. Req 3 "before any message arrives" is taken as: read_low_state() straight after construction,
      with no publisher anywhere. A stray rt/lowstate publisher on the host would break that test.
  Q5. The spec's type table says List[List[int]] for sensor/reserve, so req 3 requires raw lists
      (rows included) for the new attributes. Value comparisons elsewhere normalise tuples.
  Q6. Req 6 "behave as before" has no dev build to compare against, so it is tested as a round trip
      through publish_low_state(). That assumes dev already round-tripped q, dq, tau_est, temperature,
      voltage, imu.quat/rpy/omega/accel and mode_machine.
  Q7. The spec does not say whether the existing motor.temperature (temperature[0]) must cover the full
      int16 range, so req 4 checks only the attributes it lists.
  Q8. Req 8: if assigning a wrong-length per-motor list is itself rejected (TypeError/ValueError), no
      such LowState can reach publish_low_state(); that is accepted. Wrong row lengths inside
      sensor/reserve are not a motor-count mismatch and are not tested.
  Q9. Req 9 lists examples ("e.g."); only the listed attributes are tested, and only at assignment.
      sensor, reserve, version and tick are not tested for out-of-range rejection.
  Q10. Req 5's single-publish test first republishes a different message until it arrives, so DDS
      matching cannot be what loses the single copy; the single copy itself is never republished.
  Q11. Elsewhere a message is published again on every poll, so the reader sees identical copies of
      one message; that is taken to still be "a single message published" for req 2.
  Q12. Tests assign whole lists and assign motor/imu back onto the state, because pybind's
      def_readwrite returns vectors and nested objects as copies.
"""
import os
import subprocess
import sys
import unittest

PRELUDE = '''
import math
import time
import unitree_interface as u
G1, HG = u.RobotType.G1, u.MessageType.HG

MOTOR_NEW = ["motor." + a for a in ("temperature_winding", "motorstate", "ddq", "mode", "sensor", "reserve")]
IMU_NEW = ["imu.temperature"]
TOP_NEW = ["tick", "version", "mode_pr", "reserve"]
NEW = MOTOR_NEW + IMU_NEW + TOP_NEW
MOTOR_EXISTING = ["motor." + a for a in ("q", "dq", "tau_est", "temperature", "voltage")]
IMU_EXISTING = ["imu." + a for a in ("quat", "rpy", "omega", "accel")]
TOP_EXISTING = ["mode_machine"]
EXISTING = MOTOR_EXISTING + IMU_EXISTING + TOP_EXISTING
ALL = NEW + EXISTING
MOTOR_ALL = MOTOR_NEW + MOTOR_EXISTING
IMU_ALL = IMU_NEW + IMU_EXISTING
# Req 2's groups: per-motor values with the message-level values, and the IMU on its own.
MSG = MOTOR_ALL + TOP_NEW
NO_MODE_MACHINE = MSG + IMU_ALL
MAX_SEED = 2000
AXIS = [1 / math.sqrt(14), 2 / math.sqrt(14), 3 / math.sqrt(14)]


def need(cond, msg=""):
    if not cond:
        raise AssertionError(msg)


def rejected(call, error, what):
    try:
        call()
    except error:
        return
    except Exception as exc:
        raise AssertionError(f"{what}: expected {error.__name__}, got {type(exc).__name__}: {exc}")
    raise AssertionError(f"{what}: expected {error.__name__}, nothing raised")


def values(n, k=0):
    """Distinctive values per motor and per field; every field differs between seeds k and k + 1.

    For 0 <= k < MAX_SEED every value stays in its field's range and the seed can be read back from
    motor 0's motorstate, tick and imu.temperature. No value is zero for k >= 1.
    """
    r = range(n)
    half = (0.5 + 0.002 * k) / 2
    return {
        "motor.q": [k + i / 8 for i in r],
        "motor.dq": [-(k + 1 + i / 16) for i in r],
        "motor.ddq": [2 * k + 0.5 + i / 32 for i in r],
        "motor.tau_est": [k + 0.25 + i / 4 for i in r],
        "motor.voltage": [20 + k + i / 2 for i in r],
        "motor.temperature": [30 + k % 64 + i for i in r],
        "motor.temperature_winding": [40 + 2 * k + i for i in r],
        "motor.motorstate": [0x01000000 + (k << 8) + i for i in r],
        "motor.mode": [10 + k % 200 + i for i in r],
        "motor.sensor": [[0x02000000 + (k << 8) + 2 * i, 0x02800000 + (k << 8) + 2 * i + 1] for i in r],
        "motor.reserve": [[0x03000000 + (j << 20) + (k << 8) + i for j in range(4)] for i in r],
        "imu.quat": [math.cos(half)] + [a * math.sin(half) for a in AXIS],
        "imu.rpy": [0.125 + k, -0.25 - k, 0.375],
        "imu.omega": [1.5, -2.5 + k, 3.5],
        "imu.accel": [0.25, -9.75, 1.0 + k],
        "imu.temperature": 25 + k,
        "tick": 0x10000 + k,
        "version": [0x04000000 + k, 0x05000000 + k],
        "mode_pr": 3 + k % 250,
        "reserve": [0x06000000 + (j << 16) + k for j in range(4)],
        "mode_machine": 5 + k % 200,
    }


def motor_seed(read):
    return (read["motor.motorstate"][0] - 0x01000000) >> 8


def top_seed(read):
    return read["tick"] - 0x10000


def imu_seed(read):
    return read["imu.temperature"] - 25


def zeros(n):
    return {
        "motor.temperature_winding": [0] * n,
        "motor.motorstate": [0] * n,
        "motor.ddq": [0.0] * n,
        "motor.mode": [0] * n,
        "motor.sensor": [[0, 0] for _ in range(n)],
        "motor.reserve": [[0, 0, 0, 0] for _ in range(n)],
        "imu.temperature": 0,
        "tick": 0,
        "version": [0, 0],
        "mode_pr": 0,
        "reserve": [0, 0, 0, 0],
    }


def make_state(n, v):
    state = u.LowState(n)
    motor, imu = state.motor, state.imu
    for key, val in v.items():
        if key.startswith("motor."):
            setattr(motor, key[len("motor."):], val)
        elif key.startswith("imu."):
            setattr(imu, key[len("imu."):], val)
    state.motor, state.imu = motor, imu
    for key, val in v.items():
        if "." not in key:
            setattr(state, key, val)
    return state


def norm(val):
    return [norm(x) for x in val] if isinstance(val, (list, tuple)) else val


def raw(state, key):
    obj = state
    for part in key.split("."):
        obj = getattr(obj, part)
    return obj


def get(state, key):
    return norm(raw(state, key))


def snapshot(state, keys=ALL):
    return {key: get(state, key) for key in keys}


def same(actual, expected):
    if isinstance(expected, list):
        return (isinstance(actual, list) and len(actual) == len(expected)
                and all(same(a, e) for a, e in zip(actual, expected)))
    if isinstance(actual, bool):
        return False
    if isinstance(expected, float):
        return isinstance(actual, (int, float)) and abs(actual - expected) <= 1e-5 * max(1.0, abs(expected))
    return isinstance(actual, int) and actual == expected


def mismatches(actual, expected, keys):
    return [f"{k}: got {actual[k]!r}, want {expected[k]!r}" for k in keys if not same(actual[k], expected[k])]


def check(actual, expected, keys, what):
    bad = mismatches(actual, expected, keys)
    need(not bad, what + ":\\n" + "\\n".join(bad))


def robot_and_n():
    robot = u.create_robot("lo", G1, HG)
    return robot, robot.get_num_motors()


def read_until(robot, v, keys, timeout, publish=None):
    """Poll read_low_state() until one read matches v on every key in `keys`; return that read."""
    deadline = time.monotonic() + timeout
    while True:
        if publish is not None:
            robot.publish_low_state(publish)
        time.sleep(0.01)
        read = snapshot(robot.read_low_state(), keys)
        bad = mismatches(read, v, keys)
        if not bad:
            return read
        if time.monotonic() > deadline:
            raise AssertionError(f"published low state not read back within {timeout}s "
                                 "(not received, or discarded as corrupted):\\n" + "\\n".join(bad))


def publish_until(robot, n, v, keys, timeout=3.0):
    """Publish v repeatedly until one read matches it on every key the caller asserts."""
    return read_until(robot, v, keys, timeout, publish=make_state(n, v))
'''


class LowStateFieldsNativeTest(unittest.TestCase):
    def run_native(self, body, env=None, timeout=25):
        result = subprocess.run([sys.executable, '-c', PRELUDE + body],
                                capture_output=True, text=True, timeout=timeout,
                                env=dict(os.environ, **(env or {})))
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, 0, output)
        self.assertNotIn('crc error', output.lower(), output)
        return result

    # 1. Every new attribute equals the received message, for motor slots 0 .. n-1.
    def test_req1_received_message_fills_every_new_attribute(self):
        self.run_native('''
robot, n = robot_and_n()
need(n == u.G1_NUM_MOTOR == 29, n)
publish_until(robot, n, values(n, 0), NEW)
''')

    def test_req1_second_seed_catches_constant_or_stale_values(self):
        self.run_native('''
robot, n = robot_and_n()
publish_until(robot, n, values(n, 3), NEW)
''')

    # 2. One read never mixes messages: per-motor and message-level values together, IMU on its own.
    def test_req2_single_message_supplies_every_value(self):
        # Before arrival each group must be wholly the pre-arrival state; after, wholly the message.
        self.run_native('''
robot, n = robot_and_n()
groups = (("per-motor + message-level", MSG), ("imu", IMU_ALL))
base = snapshot(robot.read_low_state(), NO_MODE_MACHINE)
v = values(n, 1)
state = make_state(n, v)
deadline = time.monotonic() + 3.0
arrived = False
while not arrived:
    robot.publish_low_state(state)
    for _ in range(5):
        read = snapshot(robot.read_low_state(), NO_MODE_MACHINE)
        for name, keys in groups:
            vs_msg = mismatches(read, v, keys)
            if vs_msg and mismatches(read, base, keys):
                raise AssertionError(f"{name} values mix the message with the pre-arrival state:\\n"
                                     + "\\n".join(vs_msg[:8]))
        if not mismatches(read, v, NO_MODE_MACHINE):
            arrived = True
            break
    if not arrived and time.monotonic() > deadline:
        raise AssertionError("published low state not read back within 3s:\\n"
                             + "\\n".join(mismatches(read, v, NO_MODE_MACHINE)))
    time.sleep(0.005)
for i in range(50):
    check(snapshot(robot.read_low_state(), NO_MODE_MACHINE), v, NO_MODE_MACHINE,
          f"read {i} after arrival, nothing else in flight")
    time.sleep(0.002)
''')

    def test_req2_reads_never_mix_values_across_increasing_messages(self):
        self.run_native('''
robot, n = robot_and_n()
cache = {}
def expect(k):
    if k not in cache:
        cache[k] = values(n, k)
    return cache[k]
publish_until(robot, n, expect(0), NO_MODE_MACHINE)
seen_msg, seen_imu = set(), set()
reads = latest = 0
deadline = time.monotonic() + 2.0
while time.monotonic() < deadline and latest < MAX_SEED - 1:
    latest += 1
    robot.publish_low_state(make_state(n, expect(latest)))
    for _ in range(5):
        read = snapshot(robot.read_low_state(), NO_MODE_MACHINE)
        reads += 1
        km, kt, ki = motor_seed(read), top_seed(read), imu_seed(read)
        need(km == kt, f"read {reads}: motors from seed {km} but tick from seed {kt}")
        need(0 <= km <= latest, f"read {reads}: motor seed {km} never published (latest {latest})")
        check(read, expect(km), MSG, f"read {reads}: per-motor and message-level values vs seed {km}")
        need(0 <= ki <= latest, f"read {reads}: IMU seed {ki} never published (latest {latest})")
        check(read, expect(ki), IMU_ALL, f"read {reads}: IMU values vs seed {ki}")
        seen_msg.add(km)
        seen_imu.add(ki)
    time.sleep(0.001)
publish_until(robot, n, expect(latest), NO_MODE_MACHINE)
need(len(seen_msg) >= 3, f"only {len(seen_msg)} distinct messages observed in {reads} reads: {sorted(seen_msg)}")
print(f"{latest} seeds published, {reads} reads, {len(seen_msg)} motor / {len(seen_imu)} IMU seeds observed")
''')

    # 3. Before any message arrives, every new attribute reads zero with the right shape.
    def test_req3_new_attributes_zero_before_any_message(self):
        self.run_native('''
robot, n = robot_and_n()
state = robot.read_low_state()
for key in NEW:
    val = raw(state, key)
    if key in ("imu.temperature", "tick", "mode_pr"):
        need(type(val) is int, f"{key}: {type(val).__name__}, want int")
    else:
        need(isinstance(val, list), f"{key}: {type(val).__name__}, want list")
for key, size in (("version", 2), ("reserve", 4)):
    need(len(raw(state, key)) == size, f"{key}: length {len(raw(state, key))}, want {size}")
for key in MOTOR_NEW:
    need(len(raw(state, key)) == n, f"{key}: length {len(raw(state, key))}, want {n}")
for key, size in (("motor.sensor", 2), ("motor.reserve", 4)):
    for i, row in enumerate(raw(state, key)):
        need(isinstance(row, list) and len(row) == size, f"{key}[{i}]: {row!r}, want a list of {size}")
check(snapshot(state, NEW), zeros(n), NEW, "new attributes before any message")
''')

    # 4. Full ranges preserved exactly.
    def test_req4_unsigned_maxima_and_negative_minimum(self):
        self.run_native('''
robot, n = robot_and_n()
v = values(n, 0)
v["motor.motorstate"] = [0xFFFFFFFF - i for i in range(n)]
v["motor.sensor"] = [[0xFFFFFFFF - 2 * i, 0x80000000 + i] for i in range(n)]
v["motor.reserve"] = [[0xFFFFFFFF - i, 0x80000000 + i, 0x7FFFFFFF - i, 0xFFFFFFFE - i] for i in range(n)]
v["motor.mode"] = [255 - i for i in range(n)]
v["motor.temperature_winding"] = [-32768 + i if i % 2 == 0 else 32767 - i for i in range(n)]
v["imu.temperature"] = -32768
v["tick"] = 0xFFFFFFFF
v["version"] = [0xFFFFFFFF, 0x80000000]
v["mode_pr"] = 255
v["reserve"] = [0xFFFFFFFF, 0xFFFFFFFE, 0x80000000, 0x7FFFFFFF]
publish_until(robot, n, v, NEW)
''')

    def test_req4_boundary_values_and_int16_maximum(self):
        self.run_native('''
robot, n = robot_and_n()
v = values(n, 0)
u32 = [0, 1, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFE, 0xFFFFFFFF]
u8 = [0, 1, 127, 128, 254, 255]
i16 = [-32768, -32767, -1, 0, 1, 32766, 32767]
v["motor.motorstate"] = [u32[i % 6] for i in range(n)]
v["motor.sensor"] = [[u32[(i + 1) % 6], u32[(i + 2) % 6]] for i in range(n)]
v["motor.reserve"] = [[u32[(i + j) % 6] for j in range(4)] for i in range(n)]
v["motor.mode"] = [u8[i % 6] for i in range(n)]
v["motor.temperature_winding"] = [i16[i % 7] for i in range(n)]
v["imu.temperature"] = 32767
v["tick"] = 0x80000000
v["version"] = [0x7FFFFFFF, 0xFFFFFFFF]
v["mode_pr"] = 128
v["reserve"] = [0x80000000, 0xFFFFFFFF, 0, 0x7FFFFFFF]
publish_until(robot, n, v, NEW)
''')

    def test_req4_small_negative_temperatures(self):
        self.run_native('''
robot, n = robot_and_n()
v = values(n, 0)
v["motor.temperature_winding"] = [-(i + 1) for i in range(n)]
v["imu.temperature"] = -1
v["tick"] = 0x7FFFFFFF
publish_until(robot, n, v, NEW)
''')

    # 5. publish_low_state() writes every new field; the message round-trips and is not discarded.
    def test_req5_single_publish_arrives_not_discarded(self):
        self.run_native('''
robot, n = robot_and_n()
publish_until(robot, n, values(n, 0), NEW)
v = values(n, 1)
robot.publish_low_state(make_state(n, v))
read_until(robot, v, NEW, timeout=3.0)
''')

    def test_req5_successive_publishes_each_round_trip(self):
        self.run_native('''
robot, n = robot_and_n()
for k in (0, 1, 2):
    publish_until(robot, n, values(n, k), NEW)
''')

    def test_req5_change_in_new_fields_alone_is_delivered(self):
        # Existing fields identical across both messages: only the new fields can show arrival.
        self.run_native('''
robot, n = robot_and_n()
first = values(n, 0)
publish_until(robot, n, first, NO_MODE_MACHINE)
second = {**first, **{key: val for key, val in values(n, 1).items() if key in NEW}}
publish_until(robot, n, second, NO_MODE_MACHINE)
''')

    # 6. Existing attributes unchanged; temperature is temperature[0], temperature_winding temperature[1].
    def test_req6_existing_attributes_round_trip_unchanged(self):
        self.run_native('''
robot, n = robot_and_n()
publish_until(robot, n, values(n, 2), EXISTING)
''')

    def test_req6_temperature_and_winding_differ_as_in_message(self):
        self.run_native('''
robot, n = robot_and_n()
mid = n // 2
v = values(n, 0)
v["motor.temperature"] = [30 + i for i in range(n)]
v["motor.temperature_winding"] = [30 + i + 3 * (i - mid) for i in range(n)]
diffs = [w - t for t, w in zip(v["motor.temperature"], v["motor.temperature_winding"])]
need(min(diffs) < 0 < max(diffs) and 0 in diffs, diffs)
read = publish_until(robot, n, v, ["motor.temperature", "motor.temperature_winding"])
got = [w - t for t, w in zip(read["motor.temperature"], read["motor.temperature_winding"])]
need(got == diffs, (got, diffs))
''')

    def test_req6_existing_api_shapes_unchanged(self):
        self.run_native('''
robot, n = robot_and_n()
for label, state in (("read", robot.read_low_state()), ("constructed", make_state(n, values(n, 0)))):
    for key in MOTOR_EXISTING:
        val = raw(state, key)
        need(isinstance(val, list), f"{label} {key}: {type(val).__name__}, want list")
        need(len(val) == n, f"{label} {key}: length {len(val)}, want {n}")
    for key, size in (("imu.quat", 4), ("imu.rpy", 3), ("imu.omega", 3), ("imu.accel", 3)):
        val = raw(state, key)
        need(isinstance(val, list), f"{label} {key}: {type(val).__name__}, want list")
        need(len(val) == size, f"{label} {key}: length {len(val)}, want {size}")
    need(type(state.mode_machine) is int, f"{label} mode_machine: {type(state.mode_machine).__name__}")
''')

    # 8. A per-motor list of the wrong length makes publish_low_state() raise ValueError and send nothing.
    def test_req8_wrong_motor_count_raises_and_publishes_nothing(self):
        self.run_native('''
robot, n = robot_and_n()
a, b = values(n, 0), values(n, 1)
publish_until(robot, n, a, NO_MODE_MACHINE)
attempts = 0
for key in MOTOR_ALL:
    for size in (0, n - 1, n + 1):
        bad = dict(b)
        bad[key] = (b[key] + [b[key][-1]]) if size > n else b[key][:size]
        try:
            state = make_state(n, bad)
        except (TypeError, ValueError):
            continue  # Q8: rejected already at assignment
        attempts += 1
        rejected(lambda: robot.publish_low_state(state), ValueError, f"{key} with {size} entries")
for size in (n - 1, n + 1):
    attempts += 1
    whole = make_state(size, values(size, 1))
    rejected(lambda: robot.publish_low_state(whole), ValueError, f"LowState({size})")
print(f"{attempts} wrong-length publishes rejected")
deadline = time.monotonic() + 0.5
while time.monotonic() < deadline:
    check(snapshot(robot.read_low_state(), NO_MODE_MACHINE), a, NO_MODE_MACHINE,
          "state changed after only rejected publishes")
    time.sleep(0.01)
# The channel is still live: a correct publish arrives.
publish_until(robot, n, b, NO_MODE_MACHINE)
''')

    # 9. Out-of-range values for the new integer attributes raise TypeError on assignment.
    def test_req9_out_of_range_assignment_raises_type_error(self):
        self.run_native('''
robot, n = robot_and_n()
state = u.LowState(n)
motor, imu = state.motor, state.imu
for attr, lo, hi in (("mode", 0, 255), ("temperature_winding", -32768, 32767), ("motorstate", 0, 0xFFFFFFFF)):
    for pos in (0, n - 1):
        for ok in (lo, hi):
            vals = [lo + 1] * n
            vals[pos] = ok
            setattr(motor, attr, vals)
        for out in (lo - 1, hi + 1):
            vals = [lo + 1] * n
            vals[pos] = out
            rejected(lambda: setattr(motor, attr, vals), TypeError, f"motor.{attr}[{pos}] = {out}")
for obj, label, attr, lo, hi in ((imu, "imu", "temperature", -32768, 32767), (state, "state", "mode_pr", 0, 255)):
    for ok in (lo, hi):
        setattr(obj, attr, ok)
    for out in (lo - 1, hi + 1):
        rejected(lambda: setattr(obj, attr, out), TypeError, f"{label}.{attr} = {out}")
''')


if __name__ == '__main__':
    unittest.main()
