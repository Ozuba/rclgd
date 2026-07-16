import os
import subprocess
from pathlib import Path

from ros2cli.verb import VerbExtension

from rclgd_cli.api import godot


class Report:
    def __init__(self):
        self.failures = 0

    def ok(self, msg):
        print(f'✓ {msg}')

    def warn(self, msg, hint=None):
        print(f'! {msg}')
        if hint:
            print(f'    hint: {hint}')

    def fail(self, msg, hint=None):
        self.failures += 1
        print(f'✗ {msg}')
        if hint:
            print(f'    hint: {hint}')


class DoctorVerb(VerbExtension):
    """Check the rclgd installation and report problems."""

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        report = Report()

        # ROS environment
        distro = os.environ.get('ROS_DISTRO')
        if distro:
            rmw = os.environ.get('RMW_IMPLEMENTATION', 'default rmw')
            report.ok(f'ROS_DISTRO={distro}, {rmw}')
        else:
            report.fail('ROS_DISTRO not set',
                        'source /opt/ros/<distro>/setup.bash')

        # rclgd install
        try:
            prefix = godot.rclgd_prefix()
            report.ok(f'rclgd sourced from {prefix}')
        except godot.GodotRuntimeError as e:
            report.fail(str(e), 'build rclgd and source your workspace')
            return self._finish(report)

        lib = prefix / 'lib' / 'rclgd' / 'librclgd.so'
        if lib.is_file():
            report.ok(f'extension library {lib}')
        else:
            report.fail(f'librclgd.so not found in {lib.parent}',
                        'colcon build --packages-select rclgd')

        launcher = prefix / 'lib' / 'rclgd' / 'godot'
        if os.access(launcher, os.X_OK):
            report.ok('godot launcher installed (ros2 run rclgd godot)')
        else:
            report.fail(f'launcher missing: {launcher}',
                        'rebuild rclgd (older builds installed the binary directly)')

        # Version pin + engine binary
        try:
            wanted = godot.pinned_version(prefix)
        except godot.GodotRuntimeError as e:
            report.fail(str(e))
            return self._finish(report)
        report.ok(f'version pin: Godot {wanted}')

        binary = godot.binary_path(prefix)
        if not binary.exists():
            report.fail(f'Godot {wanted} not installed ({binary} missing)',
                        'ros2 rclgd setup')
        elif not os.access(binary, os.X_OK):
            report.fail(f'{binary} is not executable')
        else:
            version = self._godot_version(binary)
            if version is None:
                report.fail(f'{binary} did not run',
                            'ros2 rclgd setup --force')
            elif not version.startswith(wanted):
                report.warn(
                    f'binary reports {version} but rclgd was built for {wanted}',
                    'ros2 rclgd setup --force')
            else:
                report.ok(f'godot {version} at {binary}')

        # Project-level checks (only when run inside a Godot project)
        cwd = Path.cwd()
        if (cwd / 'project.godot').is_file():
            gdext = cwd / 'addons' / 'rclgd' / 'bin' / 'rclgd.gdextension'
            if gdext.is_file():
                report.ok(f'project {cwd.name}: rclgd.gdextension present')
            else:
                report.fail(
                    f'project {cwd.name}: no addons/rclgd/bin/rclgd.gdextension',
                    'the addon ships with rclgd_cli — copy addons/rclgd from '
                    'a freshly created package (ros2 rclgd create)')
            if not (cwd / '.godot').is_dir():
                report.warn(
                    f'project {cwd.name}: assets never imported (.godot/ missing)',
                    'ros2 rclgd editor --import-only')

        return self._finish(report)

    def _finish(self, report):
        if report.failures:
            print(f'{report.failures} issue(s) found')
        else:
            print('all checks passed')
        return report.failures

    def _godot_version(self, binary):
        try:
            out = subprocess.run(
                [str(binary), '--version'],
                capture_output=True, text=True, timeout=20)
        except (OSError, subprocess.TimeoutExpired):
            return None
        version = out.stdout.strip().splitlines()[-1] if out.stdout.strip() else None
        return version
