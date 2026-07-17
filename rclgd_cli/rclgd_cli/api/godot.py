"""Access to the Godot engine binary installed by rclgd.

The rclgd build downloads the engine at build time and installs it as
lib/rclgd/godot-bin — next to librclgd.so, which is where GDExtension
resolves the extension library. The targeted version is recorded in
share/rclgd/godot_version (one line, e.g. "4.7.1"). That's the whole
mechanism: no cache, no environment overrides; the binary ships with the
package.
"""

import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError


class GodotRuntimeError(RuntimeError):
    """A problem with the Godot runtime setup, described for the user."""


def rclgd_prefix():
    try:
        return Path(get_package_prefix('rclgd'))
    except PackageNotFoundError:
        raise GodotRuntimeError(
            "package 'rclgd' not found on AMENT_PREFIX_PATH — "
            'source your workspace first')


def pinned_version(prefix):
    """The Godot version this rclgd build targets, e.g. '4.7.1'."""
    version_file = prefix / 'share' / 'rclgd' / 'godot_version'
    if not version_file.is_file():
        raise GodotRuntimeError(
            f'{version_file} not found — rebuild rclgd '
            '(it is generated at build time)')
    version = version_file.read_text().strip()
    if not version:
        raise GodotRuntimeError(f'{version_file} is empty — rebuild rclgd')
    return version


def launcher_path(prefix):
    """The installed lib/rclgd/godot launcher script."""
    return prefix / 'lib' / 'rclgd' / 'godot'


def binary_path(prefix):
    """The engine binary installed by the rclgd build."""
    return prefix / 'lib' / 'rclgd' / 'godot-bin'


def run_headless_import(prefix, project_dir):
    """Run Godot's headless asset import on a project; returns the exit code."""
    cmd = [str(launcher_path(prefix)),
           '--editor', '--headless', '--import', '--path', str(project_dir)]
    return subprocess.run(cmd).returncode
