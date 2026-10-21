"""Native binding tests for the read-only G1 mode check; run only in network isolation.

Written from g1-health/docs/BINDING_MODE_CHECK_SPEC.md (reqs 2, 3 and 6 revised 2026-10-21) alone, without
reading the change under test. Test names carry the spec's "Required behaviour" number (test_reqN_...);
test_extra_... checks the test harness itself (helper processes cannot leak).

Each case runs in a fresh process because ChannelFactory owns process-wide DDS.
No mocks: the real extension on loopback ("lo", default DDS config, as in test_low_state_fields.py).
Some cases also start helper processes on the same DDS domain: a second Python process running the
binding's own MotionSwitcher responder (req 1), and a small C++ rt/lowcmd subscriber compiled once
per run from the repository SDK (req 4). The binding has no documented Python rt/lowcmd subscriber.

Helpers cannot outlive their test: each runs in its own process group with a pipe as stdin, exits
as soon as that pipe closes (so also when the test child is killed or times out), and is killed by
process group on every exit path. Every helper carries a per-run marker on its command line; each
test starts by checking no marked helper is alive, and the class teardown checks it again.

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
  Q6. Reqs 2 and 3 (revised) exclude the first call per method from their bounds; that first call
      must still return nonzero/empty, without raising, within timeout_s + 10 s. Every later call must
      return within timeout_s + 1.5 s. Req 2 is tested with the default and timeout_s = 0.5, 1.0 and 3.0;
      3.0 is included because the stock SDK's own overshoot grows with the timeout: with no responder
      MotionSwitcherClient alone, SetTimeout(t), returns after about t + min(0.54 t, 1.08) s (measured
      2026-10-21, aarch64 container), which the revised 1.5 s margin covers for every timeout_s.
  Q7. Req 3 ValueError: 0, 0.0, -0.2, -1, NaN, +inf, -inf, 60.001, 61 and 1e9, passed by keyword.
      Exactly 60 is allowed by the wording but is not called (with no responder it would block 60 s).
  Q8. Req 4 "no new way to change the mode": the public UnitreeInterface names may only be the dev
      build's names (taken from dir() of the compiled dev module, which has more methods than the dev
      stub, e.g. publish_low_state, read_incoming_command) plus the three new methods. Every public
      name in the module, and on every class in it, is also checked against mode/loco-changing words.
  Q9. Req 4 rt/lowcmd: the dev binding publishes nothing on rt/lowcmd until write_low_command() is
      called, and publishes continuously after it. So the test never calls write_low_command() before
      the mode-check calls, waits 1 s after construction for discovery (loopback), and requires zero
      messages from construction through the calls plus 0.5 s. A final write_low_command() must then be
      seen, proving the subscriber was matched; matching is assumed to have completed within that 1 s.
  Q10. Req 5 is tested sequentially (publish, call, read back) for 20 calls, with and without the
      responder. Concurrent callers are covered by req 6's two-thread tests.
  Q11. Req 6: "created for the G1 (MessageType.HG)" is taken as every G1/HG constructor in the dev
      stub (create_robot, UnitreeInterface with type or config, create_g1, create_robot_with_config).
  Q12. Settled 2026-10-21: the rule is the message type, not the robot type (req 6 now reads "non-HG
      message types"). RuntimeError is required for every method on the GO2-message interfaces
      create_go2, create_h1 and create_g1(MessageType.GO2). HG robots other than the G1 (create_h1_2,
      create_go2(HG), create_custom(29, HG)) talk to the same mode switcher, so making the RPC call is
      correct for them; their outcome is printed and must be RuntimeError or a well-formed result
      (no other exception, no crash).
  Q13. GIL: with no responder, while get_silent(timeout_s=1) waits, a Python ticker thread sleeping
      5 ms per step must never be starved for 0.25 s or more. The first call is made beforehand so
      client creation is not inside the window.
  Q14. Two threads calling at once: each result must have the right shape; with the responder every
      result must be exactly the req 1 answer, with none, nonzero/empty within 0.2 + 1.5 s per call.
      Calls may be serialised inside the binding; only results and the bound are checked.
"""
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
HELPER_MARK = f'modecheck-helper-{os.getpid()}'

PRELUDE = '''
import math
import os
import signal
import subprocess
import sys
import threading
import time
import unitree_interface as u
G1, HG = u.RobotType.G1, u.MessageType.HG
NEW = ("check_motion_mode", "get_silent", "get_loco_fsm_id")
EMPTY = {"check_motion_mode": ("", ""), "get_silent": (None,), "get_loco_fsm_id": (None,)}
VALUE_TYPES = {"check_motion_mode": (str, str), "get_silent": (bool,), "get_loco_fsm_id": (int,)}
ANSWER = {"check_motion_mode": (0, "", ""), "get_silent": (0, False)}
BOUND = 0.2 + 1.5
HELPER_MARK = os.environ["HELPER_MARK"]
# Exits as soon as its stdin closes, i.e. when the process that started it goes away.
RESPONDER = (
    "import os, sys, unitree_interface as u\\n"
    "r = u.create_robot('lo', u.RobotType.G1, u.MessageType.HG)\\n"
    "r.enable_motion_switcher_responder()\\n"
    "print('ready', flush=True)\\n"
    "sys.stdin.read()\\n"
    "os._exit(0)\\n")


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


def fails_cleanly(r, name, bound, **kw):
    """No responder: nonzero code and empty values within `bound` seconds."""
    res, elapsed = call(r, name, **kw)
    need(res[0] != 0, f"{name}({kw}) with no responder returned code 0: {res!r}")
    need(elapsed < bound, f"{name}({kw}) took {elapsed:.3f}s, bound {bound}s")
    return elapsed


def warm(r, timeout_s=0.2):
    """Q6: the untimed first call per method (client creation and discovery)."""
    for name in NEW:
        fails_cleanly(r, name, timeout_s + 10.0, timeout_s=timeout_s)


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


def expect_responder_answers(r, repeat=6):
    for name, want in ANSWER.items():
        res = until_ok(r, name)
        for i in range(repeat):
            need(exactly(res, want), f"{name} with responder, call {i}: {res!r}, want {want!r}")
            res, _ = call(r, name)
    call(r, "get_loco_fsm_id", timeout_s=0.2)  # Q3: shape only


class Helper:
    """A helper process in its own process group; it exits when its stdin closes."""

    def __init__(self, argv, what):
        self.what = what
        self.p = subprocess.Popen(argv + [HELPER_MARK], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  text=True, start_new_session=True)
        lines = []
        for line in self.p.stdout:
            if line.strip() == "ready":
                return
            lines.append(line)
        self.stop()
        raise AssertionError(f"{what} did not start, rc={self.p.returncode}:\\n" + "".join(lines))

    def stop(self):
        try:
            os.killpg(self.p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.p.wait()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()


def responder_process():
    return Helper([sys.executable, "-c", RESPONDER], "responder process")


def lowcmd_counter():
    """C++ rt/lowcmd subscriber; returns (helper, count()), count = messages seen so far."""
    h = Helper([os.environ["LOWCMD_SUB"]], "rt/lowcmd subscriber")
    seen = {"n": 0}
    def pump():
        for out in h.p.stdout:
            if out.strip().isdigit():
                seen["n"] = int(out)
    threading.Thread(target=pump, daemon=True).start()
    return h, lambda: seen["n"]


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


def in_threads(count, work):
    """Run work(i) in `count` threads at once; re-raise the first failure."""
    errors, start = [], threading.Barrier(count)
    def run(i):
        try:
            start.wait()
            work(i)
        except BaseException as exc:
            errors.append(f"thread {i}: {type(exc).__name__}: {exc}")
    threads = [threading.Thread(target=run, args=(i,), daemon=True) for i in range(count)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=60)
        need(not t.is_alive(), "a calling thread did not finish within 60 s (deadlock?)")
    need(not errors, "\\n".join(errors))
'''

# Public UnitreeInterface names of the compiled dev module (Q8).
DEV_METHODS = [
    'create_custom', 'create_g1', 'create_go2', 'create_h1', 'create_h1_2', 'create_zero_command',
    'enable_motion_switcher_responder', 'get_config', 'get_control_mode', 'get_default_kd',
    'get_default_kp', 'get_num_motors', 'get_robot_name', 'publish_low_state', 'publish_odom_state',
    'publish_wireless_controller', 'read_incoming_command', 'read_low_state', 'read_odom_state',
    'read_wireless_controller', 'set_control_mode', 'write_low_command',
]

# Exits as soon as its stdin closes, i.e. when the process that started it goes away.
LOWCMD_SUBSCRIBER = r'''
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <unistd.h>
static std::atomic<long> count{0};
int main() {
  std::thread([] {
    char buf[64];
    while (read(0, buf, sizeof buf) > 0) {}
    std::_Exit(0);
  }).detach();
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


def live_helpers():
    """PIDs of processes whose command line carries this run's helper marker."""
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            cmdline = (entry / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if HELPER_MARK.encode() in cmdline:
            found.append(int(entry.name))
    return found


def no_helpers_left(grace=3.0):
    """Wait up to `grace` s for marked helpers to exit; kill and report any that remain."""
    deadline = time.monotonic() + grace
    while live_helpers() and time.monotonic() < deadline:
        time.sleep(0.05)
    left = live_helpers()
    for pid in left:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if left:
        raise AssertionError(f'helper processes still running {grace}s after their test: {left}')


class ModeCheckNativeTest(unittest.TestCase):
    lowcmd_sub = None

    def setUp(self):
        no_helpers_left()

    @classmethod
    def tearDownClass(cls):
        if cls.lowcmd_sub:
            shutil.rmtree(Path(cls.lowcmd_sub).parent, ignore_errors=True)
        no_helpers_left()

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
                                env=dict(os.environ, HELPER_MARK=HELPER_MARK, **(env or {})))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    # Helpers die with the process that started them, even when it is killed outright.
    def test_extra_responder_exits_when_its_parent_is_killed(self):
        child = subprocess.Popen([sys.executable, '-c', PRELUDE + '''
h = responder_process()
print("started", h.p.pid, flush=True)
time.sleep(60)
'''], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                                 env=dict(os.environ, HELPER_MARK=HELPER_MARK))
        try:
            line = child.stdout.readline().split()
            if line[:1] != ['started']:
                raise AssertionError(f'child did not start a responder: {line!r}')
            if int(line[1]) not in live_helpers():
                raise AssertionError(f'responder {line[1]} not found by its marker; the leak check is blind')
        finally:
            child.kill()
            child.wait()
            child.stdout.close()
        no_helpers_left()

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
with responder_process():
    expect_responder_answers(robot())
''')

    # 2. No responder: nonzero code, empty value, no exception; after the first call, within timeout_s + 1.5 s.
    def test_req2_no_responder_fails_cleanly_within_bound(self):
        self.run_native('''
r = robot()
warm(r, timeout_s=1.0)
for name in NEW:
    for kw in ({}, {"timeout_s": 1.0}, {"timeout_s": 0.5}, {"timeout_s": 3.0}):
        fails_cleanly(r, name, kw.get("timeout_s", 1.0) + 1.5, **kw)
''')

    # 3. timeout_s is honoured; invalid timeout_s raises ValueError.
    def test_req3_short_timeout_returns_quickly(self):
        self.run_native('''
r = robot()
warm(r)
worst = 0.0
for name in NEW:
    for i in range(3):
        worst = max(worst, fails_cleanly(r, name, BOUND, timeout_s=0.2))
print(f"slowest timed call {worst:.3f}s")
''')

    def test_req3_invalid_timeout_raises_value_error(self):
        self.run_native('''
r = robot()
for name in NEW:
    need(hasattr(r, name), f"UnitreeInterface has no {name}()")
    for bad in (0, 0.0, -0.2, -1, math.nan, math.inf, -math.inf, 60.001, 61, 1e9):
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
h, count = lowcmd_counter()
with h:
    r = robot()
    n = r.get_num_motors()
    if RESPONDER_ON:
        r.enable_motion_switcher_responder()
    time.sleep(1.0)  # Q9: discovery
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

    # 6. Works on every G1/HG interface; other robot types raise RuntimeError (Q12); the GIL is released,
    # repeated calls work, and two threads calling at once both get sane results.
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
        call(r, name, timeout_s=0.2)
''')

    def test_req6_non_hg_interface_raises_runtime_error(self):
        self.run_native('''
robots = {"create_go2": u.UnitreeInterface.create_go2("lo"), "create_h1": u.UnitreeInterface.create_h1("lo"),
          "create_g1(GO2)": u.UnitreeInterface.create_g1("lo", u.MessageType.GO2)}
for label, r in robots.items():
    need(r.get_config().message_type == u.MessageType.GO2, f"{label}: not a GO2-message interface")
    for name in NEW:
        rejected(lambda: getattr(r, name)(timeout_s=0.2), RuntimeError, f"{label}.{name}")
        rejected(lambda: getattr(r, name)(), RuntimeError, f"{label}.{name}() again")
''')

    def test_req6_non_g1_hg_interfaces_raise_runtime_error_or_answer(self):
        # Q12: outcome printed, not prescribed.
        self.run_native('''
robots = {"create_h1_2": u.UnitreeInterface.create_h1_2("lo"),
          "create_go2(HG)": u.UnitreeInterface.create_go2("lo", HG),
          "create_custom(29, HG)": u.UnitreeInterface.create_custom("lo", 29, HG)}
for label, r in robots.items():
    for name in NEW:
        try:
            res, _ = call(r, name, timeout_s=0.2)
        except AssertionError as exc:
            if "raised RuntimeError" not in str(exc):
                raise
            print(f"{label}.{name}: RuntimeError")
        else:
            print(f"{label}.{name}: returned {res!r}")
''')

    def test_req6_gil_released_during_blocking_call(self):
        self.run_native('''
r = robot()
warm(r)
ticks, stop = [], threading.Event()
def ticker():
    while not stop.is_set():
        ticks.append(time.monotonic())
        time.sleep(0.005)
t = threading.Thread(target=ticker, daemon=True)
t.start()
time.sleep(0.1)
t0 = time.monotonic()
res, elapsed = call(r, "get_silent", timeout_s=1.0)
t1 = time.monotonic()
stop.set()
t.join()
need(res[0] != 0, f"get_silent with no responder returned code 0: {res!r}")
inside = [t0] + [x for x in ticks if t0 < x < t1] + [t1]
gap = max(b - a for a, b in zip(inside, inside[1:]))
print(f"call {elapsed:.3f}s, {len(inside) - 2} ticks inside, longest gap {gap:.3f}s")
need(gap < 0.25, f"ticker thread starved for {gap:.3f}s during a {elapsed:.3f}s get_silent call: "
                 "the GIL is held while the call waits")
''')

    def test_req6_repeated_calls_reuse_clients(self):
        self.run_native('''
r = robot()
warm(r)
for name in NEW:
    for i in range(10):
        fails_cleanly(r, name, BOUND, timeout_s=0.2)
r.enable_motion_switcher_responder()
for name, want in ANSWER.items():
    until_ok(r, name)
    for i in range(50):
        res, _ = call(r, name)
        need(exactly(res, want), f"{name} call {i} after 10 failed calls: {res!r}, want {want!r}")
''')

    def test_req6_two_threads_calling_at_once_without_responder(self):
        self.run_native('''
r = robot()
warm(r)
def work(i):
    for k in range(3):
        for name in NEW:
            fails_cleanly(r, name, BOUND, timeout_s=0.2)
in_threads(2, work)
''')

    def test_req6_two_threads_calling_at_once_with_responder(self):
        self.run_native('''
r = robot()
r.enable_motion_switcher_responder()
for name in ANSWER:
    until_ok(r, name)
def work(i):
    for k in range(20):
        for name, want in ANSWER.items():
            res, _ = call(r, name)
            need(exactly(res, want), f"{name} call {k}: {res!r}, want {want!r}")
        call(r, "get_loco_fsm_id", timeout_s=0.2)
in_threads(2, work)
''')


if __name__ == '__main__':
    unittest.main()
