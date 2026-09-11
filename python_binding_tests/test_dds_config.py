"""Native binding tests; run only in network isolation (python_binding/DDS_CONFIG.md).

Each case runs in a fresh process because ChannelFactory owns process-wide DDS.
No mocks: imports the staged extension linked to the repository SDK archive.
"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


def xml(trace=None):
    tracing = (f'<Tracing><Verbosity>config</Verbosity><OutputFile>{trace}</OutputFile></Tracing>'
               if trace else '')
    return ('<CycloneDDS><Domain Id="0"><General><Interfaces>'
            '<NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast>'
            '</General><Discovery><Peers><Peer address="127.0.0.1"/>'
            '<Peer address="127.0.0.2"/></Peers></Discovery>'
            + tracing + '</Domain></CycloneDDS>')


PRELUDE = '''
import unitree_interface as u
from unitree_interface import unitree_interface as native
assert u.DDS_CONFIG_API_VERSION == native.DDS_CONFIG_API_VERSION == 1
assert "DDS_CONFIG_API_VERSION" in u.__all__
G1, HG = u.RobotType.G1, u.MessageType.HG

def rejected(call, error, text=None):
    try:
        call()
    except error as exc:
        if text is not None:
            assert text in str(exc), str(exc)
    else:
        raise AssertionError("expected rejection")
'''


class DdsConfigNativeTest(unittest.TestCase):
    def run_native(self, body, env=None):
        result = subprocess.run([sys.executable, '-c', PRELUDE + body],
                                capture_output=True, text=True, timeout=25,
                                env=dict(os.environ, **(env or {})))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def test_constructor_and_factory_compatibility(self):
        # Each first-init entry point must accept XML, and each can reuse it.
        constructors = [
            'u.UnitreeInterface("ignored-nic", G1, HG, **kw)',
            'u.UnitreeInterface("ignored-nic", u.G1_HG_CONFIG, **kw)',
            'u.UnitreeInterface("ignored-nic", G1, HG, 29, **kw)',
            'u.create_robot("ignored-nic", G1, HG, **kw)',
            'u.create_robot_with_config("ignored-nic", u.G1_HG_CONFIG, **kw)',
            'u.UnitreeInterface.create_g1("ignored-nic", **kw)',
            'u.UnitreeInterface.create_h1_2("ignored-nic", **kw)',
            'u.UnitreeInterface.create_custom("ignored-nic", 29, HG, **kw)',
            'u.UnitreeInterface.create_h1("ignored-nic", **kw)',
            'u.UnitreeInterface.create_go2("ignored-nic", **kw)',
            'u.UnitreeInterface.create_h1("ignored-nic", HG, **kw)',
            'u.UnitreeInterface.create_go2("ignored-nic", HG, **kw)',
        ]
        for constructor in constructors:
            with self.subTest(constructor=constructor):
                self.run_native(f'kw = {{"dds_config": {xml()!r}}}\n'
                                f'a = {constructor}\nb = {constructor}\n'
                                'assert a.get_num_motors() == b.get_num_motors()\n')

    def test_legacy_none_and_conflicts(self):
        self.run_native(f'''
a = u.create_robot("lo", G1, HG)
b = u.UnitreeInterface("lo", u.G1_HG_CONFIG, dds_config=None)
c = u.UnitreeInterface("lo", G1, HG, 29)
rejected(lambda: u.create_robot("lo", G1, dds_config={xml()!r}), RuntimeError, "incompatible")
rejected(lambda: u.create_robot("other-nic", G1), RuntimeError, "incompatible")
''')

    def test_explicit_reuse_and_conflicts(self):
        self.run_native(f'''
config = {xml()!r}
a = u.create_robot("ignored", G1, dds_config=config)
b = u.create_robot_with_config("different-ignored-nic", u.G1_HG_CONFIG, dds_config=config)
rejected(lambda: u.UnitreeInterface("lo", G1, HG), RuntimeError, "incompatible")
rejected(lambda: u.UnitreeInterface("lo", G1, HG, dds_config=None), RuntimeError, "incompatible")
rejected(lambda: u.create_robot("lo", G1, dds_config=config.replace("127.0.0.2", "127.0.0.3")), RuntimeError, "incompatible")
del a, b
rejected(lambda: u.create_robot("lo", G1), RuntimeError, "incompatible")
''')

    def test_invalid_xml_retry_before_native_initialization(self):
        invalid = ['', '  ', '/tmp/profile.xml', 'file:///tmp/profile.xml',
                   '<CycloneDDS>', '<Other/>', '<CycloneDDS/>',
                   xml().replace('Id="0"', 'Id="42"'),
                   xml().replace('Id="0"', 'Id="0" Id="0"'),
                   xml().replace('name="lo"', 'name="lo" name="lo"'),
                   xml().replace('address="127.0.0.1"',
                                 'address="127.0.0.1" address="127.0.0.1"'),
                   xml().replace('<CycloneDDS>', '<CycloneDDS xmlns="" xmlns="">'),
                   xml().replace('Id="0"', ''),
                   xml().replace('Id="0"', 'Id="${ROS_DOMAIN_ID}"'),
                   xml().replace('<General>', '<Id>42</Id><General>'),
                   xml().replace('</CycloneDDS>', '<Domain Id="42"/></CycloneDDS>'),
                   '\ufeff' + xml(), ' \ufeff' + xml(),
                   xml() + '\x00junk', xml() + 'junk']
        self.run_native(f'''
for bad in {invalid!r}:
    rejected(lambda: u.create_robot("lo", G1, dds_config=bad), ValueError, "dds_config")
a = u.create_robot("ignored", G1, dds_config={xml()!r})
''')

    def test_attribute_names_are_scoped_to_each_element(self):
        # Sibling Peer elements legitimately reuse the address attribute.
        self.run_native(f'a = u.create_robot("ignored", G1, dds_config={xml()!r})\n')
        # These nested/sibling names are well-formed XML, but not Cyclone schema.
        # RuntimeError (not ValueError) proves they pass pre-init validation;
        # the sticky failure confirms the native initializer was reached.
        config = xml().replace('<General>', '<General name="outer">'
                               '<Unknown name="inner"/><Unknown name="sibling"/>')
        self.run_native(f'''
rejected(lambda: u.create_robot("lo", G1, dds_config={config!r}), RuntimeError)
rejected(lambda: u.create_robot("lo", G1, dds_config={xml()!r}), RuntimeError, "previously failed")
''')

    def test_create_zero_command_stub_matches_no_arg_native_utility(self):
        self.run_native(f'''
import ast
from pathlib import Path
stub = ast.parse(Path(u.__file__).with_name("unitree_interface.pyi").read_text())
interface = next(node for node in stub.body
                 if isinstance(node, ast.ClassDef) and node.name == "UnitreeInterface")
method = next(node for node in interface.body
              if isinstance(node, ast.FunctionDef) and node.name == "create_zero_command")
assert [arg.arg for arg in method.args.args] == ["self"]
assert not method.args.posonlyargs and not method.args.kwonlyargs
assert method.args.vararg is None and method.args.kwarg is None
assert not method.args.defaults
robot = u.create_robot("ignored", G1, dds_config={xml()!r})
assert isinstance(robot.create_zero_command(), u.MotorCommand)
rejected(lambda: robot.create_zero_command(dds_config=None), TypeError)
''')

    def test_native_schema_failure_is_not_silently_retried(self):
        bad = xml().replace('<AllowMulticast>false', '<AllowMulticast>not-a-policy')
        self.run_native(f'''
rejected(lambda: u.create_robot("lo", G1, dds_config={bad!r}), RuntimeError)
rejected(lambda: u.create_robot("lo", G1, dds_config={xml()!r}), RuntimeError, "previously failed")
rejected(lambda: u.create_robot("lo", G1), RuntimeError, "previously failed")
''')

    def test_native_nic_failure_is_not_ignored(self):
        bad = xml().replace('name="lo"', 'name="missing-interface"')
        self.run_native(f'''
rejected(lambda: u.create_robot("lo", G1, dds_config={bad!r}), RuntimeError)
rejected(lambda: u.create_robot("lo", G1), RuntimeError, "previously failed")
''')

    def test_keyword_only_and_types(self):
        self.run_native(f'''
rejected(lambda: u.create_robot("lo", G1, HG, {xml()!r}), TypeError)
rejected(lambda: u.UnitreeInterface("lo", G1, HG, dds_config=123), TypeError)
a = u.create_robot("lo", G1)
''')

    def test_effective_config_and_hostile_ros_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / 'cyclone.log'
            # ROS config must not contribute its peer, multicast policy, or domain.
            hostile = ('<CycloneDDS><Domain Id="42"><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast>'
                       '</General><Discovery><Peers><Peer address="127.0.0.99"/></Peers>'
                       '</Discovery></Domain></CycloneDDS>')
            indented = ' \t\r\n' + xml(trace)
            for ros_xml in (hostile, hostile.replace('Id="42"', 'Id="any"')):
                self.run_native(f'a = u.create_robot("ignored-nic", G1, dds_config={indented!r})\n',
                                {'CYCLONEDDS_URI': ros_xml, 'ROS_DOMAIN_ID': '42'})
                text = trace.read_text()
                self.assertIn('NetworkInterface[@name]: lo', text)
                self.assertIn('AllowMulticast/#text: false', text)
                self.assertIn('Peer[@Address]: 127.0.0.1', text)
                self.assertIn('Peer[@Address]: 127.0.0.2', text)
                self.assertNotIn('127.0.0.99', text)
                self.assertIn('[0]', text)
            # Also preserve evidence outside the temporary directory when requested.
            if os.environ.get('DDS_TEST_EVIDENCE_DIR'):
                Path(os.environ['DDS_TEST_EVIDENCE_DIR'], 'effective-config.log').write_text(text)

    def test_repeated_construction_and_immediate_destruction(self):
        for config in (None, xml()):
            with self.subTest(explicit=config is not None):
                self.run_native(f'''
for _ in range(10):
    a = u.UnitreeInterface("lo", G1, HG, dds_config={config!r})
    b = u.UnitreeInterface("lo", G1, HG, dds_config={config!r})
    del a, b
''')

    def test_any_domain_and_xml_declaration(self):
        config = ' \t\r\n<?xml version="1.0"?>' + xml().replace('Id="0"', 'Id="any"')
        self.run_native(f'''
a = u.create_robot("ignored", G1, dds_config={config!r})
b = u.UnitreeInterface("another-nic", G1, HG, dds_config={config!r})
rejected(lambda: u.create_robot("lo", G1, dds_config={config.lstrip()!r}), RuntimeError, "incompatible")
''')

    def test_motion_switcher_responder_remains_opt_in_and_idempotent(self):
        self.run_native(f'''
a = u.create_robot("ignored", G1, dds_config={xml()!r})
a.enable_motion_switcher_responder()
a.enable_motion_switcher_responder()
''')


if __name__ == '__main__':
    unittest.main()
