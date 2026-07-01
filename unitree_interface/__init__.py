# Unitree Interface SDK

# In an installed wheel the compiled extension is always present, so an
# ImportError here means a *real* load failure (e.g. a mis-RPATH'd DDS .so or a
# glibc/ABI mismatch) — not an "optional" extension. Swallowing it would make
# `import unitree_interface` succeed but expose an empty module, which is a
# confusing failure mode. Re-raise with the underlying cause attached.
#
# The only case where the extension legitimately does not exist is an in-tree
# source checkout that has not been built yet; we still raise, but with a
# message that points at the build step.
try:
    from .unitree_interface import *  # noqa: F401,F403
    from . import unitree_interface as _ext

    __all__ = getattr(_ext, "__all__", [name for name in dir(_ext) if not name.startswith("_")])
except ImportError as exc:
    raise ImportError(
        "Failed to load the compiled 'unitree_interface' extension. If you "
        "installed from a wheel this indicates a broken install (e.g. a "
        "missing/incompatible shared library); please report it. If you are "
        "working from a source checkout, build the extension first "
        "(see README.md)."
    ) from exc
