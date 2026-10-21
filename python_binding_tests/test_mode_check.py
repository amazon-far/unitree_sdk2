"""Native binding tests for the read-only G1 mode check; run only in network isolation.

Written from g1-health/docs/BINDING_MODE_CHECK_SPEC.md (2026-10-16) alone, without reading the
change under test. Test names carry the spec's "Required behaviour" number (test_reqN_...).

Each case runs in a fresh process because ChannelFactory owns process-wide DDS.
No mocks: the real extension on loopback ("lo", default DDS config, as in test_low_state_fields.py).
Two cases also start helper processes on the same DDS domain: a second Python process running the
binding's own MotionSwitcher responder (req 1), and a small C++ rt/lowcmd subscriber compiled once
per run from the repository SDK (req 4). The binding has no documented Python rt/lowcmd subscriber.

Req 7 has no test here: it is checked by running this file together with the two existing files,
  python -m unittest python_binding_tests/test_dds_config.py python_binding_tests/test_low_state_fields.py \
      python_binding_tests/test_mode_check.py
plus `git diff --exit-code dev -- python_binding_tests/test_dds_config.py` at review time.

Questions on the spec, and the interpretation these tests take (the least demanding one):
  Q1. Req 1 "with the responder running": DDS discovery needs a moment, so the first calls may fail.
      Each method is retried until code 0 for up to 5 s; from then on every call (5 more) must return
      exactly (0, "", "") / (0, False). The responder is tested on the same interface, on a second
      interface in the same process, and in another process.
  Q2. The dev stub documents the responder as answering CheckMode/ReleaseMode only; the spec says
      get_silent() returns (0, False) with it. The spec wins: (0, False) is required, False as a bool.
  Q3. Req 1 says nothing about get_loco_fsm_id() with the responder (the responder is a MotionSwitcher,
      not a loco service). It is only checked for the general contract: no exception, (int, None) on a
      nonzero code, (0, int) on success.
  Q4. Return values must be tuples (namedtuples pass) of the stated length; code must be an int (not
      bool); form/name str; silent bool; fsm_id int. On a nonzero code: "" / None exactly.
  Q5. Req 2/3 give only an upper bound. Returning a nonzero code early (before timeout_s) is accepted;
      no lower bound is tested. Elapsed time is measured around the Python call.
  Q6. Req 3 "timeout_s <= 0 raises ValueError" is tested with 0, 0.0, -0.2 and -1, passed by keyword.
      NaN/inf and non-number types are unspecified and not tested.
  Q7. Req 4 "no new way to change the mode": the public UnitreeInterface names may only be the dev
      build's names (taken from dir() of the compiled dev module, which has more methods than the dev
      stub, e.g. publish_low_state, read_incoming_command) plus the three new methods. Every public
      name in the module, and on every class in it, is also checked against mode/loco-changing words.
  Q8. Req 4 rt/lowcmd: the dev binding publishes nothing on rt/lowcmd until write_low_command() is
      called, and publishes continuously after it. So the test never calls write_low_command() before
      the mode-check calls, waits 1 s after construction for discovery (loopback), and requires zero
      messages from construction through the calls plus 0.5 s. A final write_low_command() must then be
      seen, proving the subscriber was matched; matching is assumed to have completed within that 1 s.
  Q9. Req 5 is tested sequentially (publish, call, read back) for 20 calls, with and without the
      responder. Calling from a second thread is not required by the spec and not tested.
  Q10. Req 6: "created for the G1 (MessageType.HG)" is taken as every G1/HG constructor in the dev
      stub (create_robot, UnitreeInterface with type or config, create_g1, create_robot_with_config).
"""
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

PRELUDE = '''
import os
import subprocess
import sys
import threading
import time
import unitree_interface as u
G1, HG = u.RobotType.G1, u.MessageType.HG
NEW = ("check_motion_mode", "get_silent", "get_loco_fsm_id")
EMPTY = {"check_motion_mode": ("", ""), "get_silent": (None,), "get_loco_fsm_id": (None,)}
VALUE_TYPES = {"check_motion_mode": (str, str), "get_silent": (bool,), "get_loco_fsm_id": (int,)}
RESPONDER = (
    "import time, unitree_interface as u\\n"
    "r = u.create_robot('lo', u.RobotType.G1, u.MessageType.HG)\\n"
    "r.enable_motion_switcher_responder()\\n"
    "print('ready', flush=True)\\n"
    "time.sleep(120)\\n")


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


def robot():
    return u.create_robot("lo", G1, HG)


def call(r, name, **kw):
    """Call a mode-check method; check the result's shape (Q4); return (result, elapsed seconds)."""
    method = getattr(r, name, None)
    need(method is not None, f"UnitreeInterface has no {name}()")
    start = time.monotonic()
    try:
        res = method(**kw)
    except Exception as exc:
        raise AssertionError(f"{name}({kw}) raised {type(exc).__name__}: {exc}")
    elapsed = time.monotonic() - start
    need(isinstance(res, tuple) and len(res) == 1 + len(EMPTY[name]),
         f"{name}: {res!r}, want a tuple of {1 + len(EMPTY[name])}")
    need(type(res[0]) is int, f"{name}: code {res[0]!r} is {type(res[0]).__name__}, want int")
    if res[0] != 0:
        need(res[1:] == EMPTY[name] and all(type(a) is type(b) for a, b in zip(res[1:], EMPTY[name])),
             f"{name}: nonzero code {res[0]} with values {res[1:]!r}, want {EMPTY[name]!r}")
    else:
        need(all(type(v) is t for v, t in zip(res[1:], VALUE_TYPES[name])),
             f"{name}: code 0 with values {res[1:]!r}, want types {VALUE_TYPES[name]}")
    return res, elapsed


def exactly(res, want):
    return res == want and all(type(a) is type(b) for a, b in zip(res, want))


def until_ok(r, name, timeout=5.0, **kw):
    """Q1: retry while discovery completes; return the first code-0 result."""
    deadline = time.monotonic() + timeout
    while True:
        res, _ = call(r, name, **kw)
        if res[0] == 0:
            return res
        need(time.monotonic() < deadline, f"{name}: no code-0 answer within {timeout}s, last {res!r}")
        time.sleep(0.05)


def expect_responder_answers(r):
    for name, want in (("check_motion_mode", (0, "", "")), ("get_silent", (0, False))):
        res = until_ok(r, name)
        for i in range(6):
            need(exactly(res, want), f"{name} with responder, call {i}: {res!r}, want {want!r}")
            res, _ = call(r, name)
    call(r, "get_loco_fsm_id", timeout_s=0.2)  # Q3: shape only


def start_responder_process():
    p = subprocess.Popen([sys.executable, "-c", RESPONDER], stdout=subprocess.PIPE, text=True)
    wait_ready(p, "responder process")
    return p


def wait_ready(p, what):
    """Skip the SDK's own start-up lines until the helper prints "ready"."""
    lines = []
    for line in p.stdout:
        if line.strip() == "ready":
            return
        lines.append(line)
    p.kill()
    raise AssertionError(f"{what} did not start, rc={p.wait()}:\\n" + "".join(lines))


def start_lowcmd_counter():
    """C++ rt/lowcmd subscriber; returns (process, count()), count = messages seen so far."""
    p = subprocess.Popen([os.environ["LOWCMD_SUB"]], stdout=subprocess.PIPE, text=True)
    wait_ready(p, "rt/lowcmd subscriber")
    seen = {"n": 0}
    def pump():
        for out in p.stdout:
            if out.strip().isdigit():
                seen["n"] = int(out)
    threading.Thread(target=pump, daemon=True).start()
    return p, lambda: seen["n"]


def make_state(n, k):
    state = u.LowState(n)
    motor = state.motor
    motor.q = [k + i / 8 for i in range(n)]
    motor.tau_est = [-(k + 0.25 + i / 4) for i in range(n)]
    state.motor = motor
    return state


def read_back(r, n, k, timeout=3.0):
    want_q = [k + i / 8 for i in range(n)]
    want_tau = [-(k + 0.25 + i / 4) for i in range(n)]
    deadline = time.monotonic() + timeout
    while True:
        motor = r.read_low_state().motor
        if (all(abs(a - b) < 1e-5 for a, b in zip(motor.q, want_q))
                and all(abs(a - b) < 1e-5 for a, b in zip(motor.tau_est, want_tau))):
            return
        need(time.monotonic() < deadline,
             f"low state {k} not read back within {timeout}s: q[:3]={list(motor.q)[:3]}")
        time.sleep(0.01)
'''

# Public UnitreeInterface names of the compiled dev module (Q7).
DEV_METHODS = [
    'create_custom', 'create_g1', 'create_go2', 'create_h1', 'create_h1_2', 'create_zero_command',
    'enable_motion_switcher_responder', 'get_config', 'get_control_mode', 'get_default_kd',
    'get_default_kp', 'get_num_motors', 'get_robot_name', 'publish_low_state', 'publish_odom_state',
    'publish_wireless_controller', 'read_incoming_command', 'read_low_state', 'read_odom_state',
    'read_wireless_controller', 'set_control_mode', 'write_low_command',
]

LOWCMD_SUBSCRIBER = r'''
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
static std::atomic<long> count{0};
int main() {
  unitree::robot::ChannelFactory::Instance()->Init(0, "lo");
  unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_> sub("rt/lowcmd");
  sub.InitChannel([](const void*) { ++count; }, 1);
  std::printf("ready\n");
  std::fflush(stdout);
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::printf("%ld\n", count.load());
    std::fflush(stdout);
  }
}
'''


class ModeCheckNativeTest(unittest.TestCase):
    lowcmd_sub = None

    @classmethod
    def tearDownClass(cls):
        if cls.lowcmd_sub:
            shutil.rmtree(Path(cls.lowcmd_sub).parent, ignore_errors=True)

    @classmethod
    def build_lowcmd_subscriber(cls):
        if cls.lowcmd_sub:
            return cls.lowcmd_sub
        arch = platform.machine()
        directory = Path(tempfile.mkdtemp(prefix='lowcmd_sub_'))
        source, binary = directory / 'sub.cpp', directory / 'sub'
        source.write_text(LOWCMD_SUBSCRIBER)
        libs = ROOT / 'thirdparty' / 'lib' / arch
        compiler = shutil.which('g++') or shutil.which('c++')
        if compiler is None:
            raise AssertionError('no C++ compiler for the rt/lowcmd subscriber (req 4)')
        result = subprocess.run(
            [compiler, '-std=c++17', '-O1', f'-I{ROOT / "include"}', f'-I{ROOT / "thirdparty/include"}',
             f'-I{ROOT / "thirdparty/include/ddscxx"}', str(source), str(ROOT / 'lib' / arch / 'libunitree_sdk2.a'),
             f'-L{libs}', '-lddscxx', '-lddsc', '-lpthread', f'-Wl,-rpath,{libs}', '-o', str(binary)],
            capture_output=True, text=True, timeout=300)
        if result.returncode != 0:
            raise AssertionError('rt/lowcmd subscriber did not compile:\n' + result.stdout + result.stderr)
        cls.lowcmd_sub = str(binary)
        return cls.lowcmd_sub

    def run_native(self, body, env=None, timeout=60):
        result = subprocess.run([sys.executable, '-c', PRELUDE + body],
                                capture_output=True, text=True, timeout=timeout,
                                env=dict(os.environ, **(env or {})))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    # 1. With the binding's own responder: (0, "", "") and (0, False).
    def test_req1_responder_on_same_interface(self):
        self.run_native('''
r = robot()
r.enable_motion_switcher_responder()
expect_responder_answers(r)
''')

    def test_req1_responder_on_other_interface_same_process(self):
        self.run_native('''
a, b = robot(), robot()
b.enable_motion_switcher_responder()
expect_responder_answers(a)
''')

    def test_req1_responder_in_other_process(self):
        self.run_native('''
p = start_responder_process()
try:
    expect_responder_answers(robot())
finally:
    p.kill()
    p.wait()
''')

    # 2. No responder: nonzero code, empty value, within timeout_s + 1 s, no exception.
    def test_req2_no_responder_fails_cleanly_within_bound(self):
        self.run_native('''
r = robot()
for name in NEW:
    for kw in ({}, {"timeout_s": 1.0}, {"timeout_s": 0.5}):
        bound = kw.get("timeout_s", 1.0) + 1.0
        res, elapsed = call(r, name, **kw)
        need(res[0] != 0, f"{name}({kw}) with no responder returned code 0: {res!r}")
        need(elapsed < bound, f"{name}({kw}) took {elapsed:.2f}s, bound {bound}s")
''')

    # 3. timeout_s is honoured; timeout_s <= 0 raises ValueError.
    def test_req3_short_timeout_returns_quickly(self):
        self.run_native('''
r = robot()
for name in NEW:
    for i in range(3):
        res, elapsed = call(r, name, timeout_s=0.2)
        need(res[0] != 0, f"{name}(timeout_s=0.2) with no responder returned code 0: {res!r}")
        need(elapsed < 1.2, f"{name}(timeout_s=0.2) call {i} took {elapsed:.2f}s, want < 1.2s")
''')

    def test_req3_nonpositive_timeout_raises_value_error(self):
        self.run_native('''
r = robot()
for name in NEW:
    need(hasattr(r, name), f"UnitreeInterface has no {name}()")
    for bad in (0, 0.0, -0.2, -1):
        rejected(lambda: getattr(r, name)(timeout_s=bad), ValueError, f"{name}(timeout_s={bad})")
    call(r, name, timeout_s=0.2)  # still usable afterwards
''')

    # 4. Read-only: no mode-changing API, and nothing sent on rt/lowcmd.
    def test_req4_no_mode_changing_methods(self):
        self.run_native(f'''
from unitree_interface import unitree_interface as native
dev = set({DEV_METHODS!r})
public = {{n for n in dir(u.UnitreeInterface) if not n.startswith("_")}}
need(set(NEW) <= public, f"missing: {{sorted(set(NEW) - public)}}")
extra = public - dev - set(NEW)
need(not extra, f"UnitreeInterface gained methods beyond the three read-only ones: {{sorted(extra)}}")
WORDS = ("select", "release", "silent", "set_mode", "setmode", "switch", "fsm", "loco", "damp",
         "zero_torque", "stand", "squat", "sit", "balance", "velocity", "move", "wave", "shake")
ALLOWED = set(NEW) | {{"enable_motion_switcher_responder", "velocity", "position"}}
names = []
for mod in (u, native):
    for n in dir(mod):
        names.append((mod.__name__, n))
        obj = getattr(mod, n)
        if isinstance(obj, type):
            names += [(f"{{mod.__name__}}.{{n}}", m) for m in dir(obj)]
bad = sorted({{f"{{where}}.{{n}}" for where, n in names
              if not n.startswith("__") and n not in ALLOWED and any(w in n.lower() for w in WORDS)}})
need(not bad, f"mode-changing names exposed: {{bad}}")
''')

    LOWCMD_BODY = '''
p, count = start_lowcmd_counter()
try:
    r = robot()
    n = r.get_num_motors()
    if RESPONDER_ON:
        r.enable_motion_switcher_responder()
    time.sleep(1.0)  # Q8: discovery
    need(count() == 0, f"{count()} rt/lowcmd messages before any call; cannot attribute messages")
    for _ in range(3):
        for name in NEW:
            call(r, name, timeout_s=0.2)
    time.sleep(0.5)
    need(count() == 0, f"{count()} rt/lowcmd messages after the mode-check calls, want 0")
    # Positive control: the subscriber is matched and does see the binding's rt/lowcmd.
    r.write_low_command(r.create_zero_command())
    deadline = time.monotonic() + 3.0
    while count() == 0:
        need(time.monotonic() < deadline, "subscriber never saw write_low_command(); test inconclusive")
        time.sleep(0.02)
finally:
    p.kill()
    p.wait()
'''

    def test_req4_calls_send_nothing_on_lowcmd_without_responder(self):
        self.run_native('RESPONDER_ON = False\n' + self.LOWCMD_BODY,
                        env={'LOWCMD_SUB': self.build_lowcmd_subscriber()})

    def test_req4_calls_send_nothing_on_lowcmd_with_responder(self):
        self.run_native('RESPONDER_ON = True\n' + self.LOWCMD_BODY,
                        env={'LOWCMD_SUB': self.build_lowcmd_subscriber()})

    # 5. Low-level path undisturbed: published low states keep round-tripping across 20 calls.
    LOWSTATE_BODY = '''
r = robot()
n = r.get_num_motors()
if RESPONDER_ON:
    r.enable_motion_switcher_responder()
for k in range(1, 21):
    state = make_state(n, k)
    r.publish_low_state(state)
    call(r, NEW[k % 3], timeout_s=0.2)
    deadline = time.monotonic() + 3.0
    while True:
        try:
            read_back(r, n, k, timeout=0.2)
            break
        except AssertionError:
            # Republish: a single DDS sample may be lost before matching; the read path is under test.
            if time.monotonic() > deadline:
                raise
            r.publish_low_state(state)
'''

    def test_req5_low_state_round_trips_during_calls_without_responder(self):
        self.run_native('RESPONDER_ON = False\n' + self.LOWSTATE_BODY)

    def test_req5_low_state_round_trips_during_calls_with_responder(self):
        self.run_native('RESPONDER_ON = True\n' + self.LOWSTATE_BODY)

    # 6. Works on every G1/HG interface.
    def test_req6_methods_work_on_g1_hg_interfaces(self):
        self.run_native('''
robots = {
    "create_robot": u.create_robot("lo", G1, HG),
    "UnitreeInterface(type)": u.UnitreeInterface("lo", G1, HG),
    "UnitreeInterface(config)": u.UnitreeInterface("lo", u.G1_HG_CONFIG),
    "create_g1": u.UnitreeInterface.create_g1("lo"),
    "create_robot_with_config": u.create_robot_with_config("lo", u.G1_HG_CONFIG),
}
for label, r in robots.items():
    for name in NEW:
        res, elapsed = call(r, name, timeout_s=0.2)
        need(elapsed < 1.2, f"{label}.{name}: {elapsed:.2f}s")
''')


if __name__ == '__main__':
    unittest.main()
