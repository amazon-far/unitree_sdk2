"""Minimal setup.py whose only job is to mark this wheel as platform-specific.

All package metadata lives in pyproject.toml. The compiled pybind11 extension
(`unitree_interface.cpython-*.so`) is produced out-of-band by
scripts/cibw_before_build.sh and packaged via [tool.setuptools.package-data],
so setuptools does not see a real ext_module and would otherwise build a
pure-Python `py3-none-any` wheel. That tag is wrong: the .so is specific to one
CPython ABI and architecture.

Forcing Distribution.has_ext_modules() to return True makes setuptools tag the
wheel with the active interpreter's ABI + platform (e.g. cp311-cp311-linux_x86_64),
which auditwheel then converts to a manylinux tag during `auditwheel repair`.
"""

from setuptools import setup
from setuptools.dist import Distribution


class BinaryDistribution(Distribution):
    """A Distribution that always reports binary (platform-specific) content."""

    def has_ext_modules(self):  # noqa: D401 - setuptools hook
        return True

    def is_pure(self):
        return False


setup(distclass=BinaryDistribution)
