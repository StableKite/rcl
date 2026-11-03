"""Locate and activate the actual SDK shipped with the rcl wheel."""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
from threading import RLock
from typing import Protocol


class _DirectoryHandle(Protocol):
    def close(self) -> None: ...


_lock = RLock()
_active = False
_handles: tuple[_DirectoryHandle, ...] = ()
_native: ctypes.CDLL | None = None


def get_prefix() -> Path:
    """Return the installed prefix containing real headers, libraries and resources."""
    prefix = Path(__file__).resolve().parent / 'prefix'
    if not prefix.is_dir():
        raise FileNotFoundError(f'The packaged ROS SDK prefix is missing: {prefix}')
    return prefix


def _prepend(name: str, values: list[Path]) -> None:
    previous = os.environ.get(name, '')
    paths = [str(path) for path in values]
    paths.extend(value for value in previous.split(os.pathsep) if value and value not in paths)
    os.environ[name] = os.pathsep.join(paths)


def configure_environment() -> None:
    """Make actual installed resources discoverable without loading native libraries."""
    with _lock:
        prefix = get_prefix()
        site = prefix.parents[1]
        prefixes = [path for path in (site / 'rosidl_generator_py' / '_sdk',
                    site / 'rclpy' / '_sdk', prefix) if path.is_dir()]
        _prepend('AMENT_PREFIX_PATH', prefixes)
        _prepend('CMAKE_PREFIX_PATH', prefixes)
        if os.name == 'nt':
            _prepend('PATH', [prefix / 'bin'])


def activate() -> None:
    """Initialize this process's SDK discovery once, retaining native directory owners."""
    global _active, _handles, _native
    with _lock:
        if _active:
            return
        configure_environment()
        prefix = get_prefix()
        handles: list[_DirectoryHandle] = []
        try:
            if os.name == 'nt':
                handles.append(os.add_dll_directory(str(prefix / 'bin')))
                native = ctypes.WinDLL(str(prefix / 'bin' / 'rcl.dll'))
            else:
                native = ctypes.CDLL(str(prefix / 'lib' / 'librcl.so'), mode=ctypes.RTLD_GLOBAL)
        except BaseException:
            for handle in reversed(handles):
                handle.close()
            raise
        _native = native
        _handles = tuple(handles)
        _active = True
