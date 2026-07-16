"""Provisioning of the Godot editor binary for rclgd.

The rclgd build records the engine version it targets in
share/rclgd/godot_version (one line, e.g. "4.7.1"). `ros2 rclgd setup`
downloads exactly that release from the official GitHub mirror into
lib/rclgd/godot-bin — next to librclgd.so, which is where GDExtension
resolves the extension library. That's the whole mechanism: no cache, no
environment overrides; the binary either sits in the prefix or `setup`
fetches it.
"""

import hashlib
import subprocess
import tempfile
import urllib.request
import zipfile
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError

URL_BASE = 'https://github.com/godotengine/godot/releases/download'


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
    """Where the engine binary lives once downloaded."""
    return prefix / 'lib' / 'rclgd' / 'godot-bin'


def platform_slug():
    """Platform part of the official release archive name."""
    import platform
    system = platform.system()
    if system != 'Linux':
        raise GodotRuntimeError(
            f'unsupported platform: {system} (rclgd currently targets Linux)')
    machine = platform.machine()
    if machine == 'x86_64':
        return 'linux.x86_64'
    if machine in ('aarch64', 'arm64'):
        return 'linux.arm64'
    raise GodotRuntimeError(f'unsupported architecture: {machine}')


def run_headless_import(prefix, project_dir):
    """Run Godot's headless asset import on a project; returns the exit code."""
    cmd = [str(launcher_path(prefix)),
           '--editor', '--headless', '--import', '--path', str(project_dir)]
    return subprocess.run(cmd).returncode


def download_godot(prefix, *, force=False, skip_checksum=False, log=print):
    """Download the pinned Godot release into lib/rclgd/godot-bin."""
    dest = binary_path(prefix)
    if dest.exists() and not force:
        return dest

    release = f'{pinned_version(prefix)}-stable'
    zip_name = f'Godot_v{release}_{platform_slug()}.zip'
    url = f'{URL_BASE}/{release}/{zip_name}'

    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=dest.parent) as tmp:
        tmp = Path(tmp)
        zip_path = tmp / zip_name
        log(f'downloading {url}')
        _fetch(url, zip_path, log)

        if skip_checksum:
            log('checksum verification skipped (--skip-checksum)')
        else:
            expected = _expected_sha512(f'{URL_BASE}/{release}/SHA512-SUMS.txt', zip_name)
            actual = _sha512(zip_path)
            if actual != expected:
                raise GodotRuntimeError(
                    f'sha512 mismatch for {zip_name}:\n'
                    f'  expected {expected}\n  got      {actual}')
            log('sha512 verified')

        with zipfile.ZipFile(zip_path) as zf:
            names = [n for n in zf.namelist()
                     if n.startswith('Godot_v') and not n.endswith('/')]
            if len(names) != 1:
                raise GodotRuntimeError(
                    f'unexpected archive layout in {zip_name}: {zf.namelist()}')
            zf.extract(names[0], tmp)
        binary = tmp / names[0]
        binary.chmod(0o755)
        # tmp lives inside the destination directory, so this replace stays
        # on one filesystem (atomic)
        binary.replace(dest)

    return dest


def _fetch(url, dest, log):
    try:
        with urllib.request.urlopen(url) as resp, open(dest, 'wb') as out:
            done = 0
            while True:
                chunk = resp.read(1 << 16)
                if not chunk:
                    break
                out.write(chunk)
                done += len(chunk)
        log(f'downloaded {done / (1 << 20):.0f} MB')
    except OSError as e:
        raise GodotRuntimeError(f'download of {url} failed: {e}')


def _expected_sha512(sums_url, file_name):
    try:
        with urllib.request.urlopen(sums_url) as resp:
            sums = resp.read().decode()
    except OSError as e:
        raise GodotRuntimeError(
            f'could not fetch checksums from {sums_url}: {e}\n'
            'use --skip-checksum to proceed without verification')
    for line in sums.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[-1].lstrip('*') == file_name:
            return parts[0]
    raise GodotRuntimeError(f'{file_name} not listed in {sums_url}')


def _sha512(path):
    digest = hashlib.sha512()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()
