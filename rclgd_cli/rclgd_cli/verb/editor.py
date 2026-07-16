import os
from pathlib import Path

from ros2cli.verb import VerbExtension

from rclgd_cli.api import godot
from rclgd_cli.api import packages


def _package_name_completer(prefix, parsed_args, **kwargs):
    """Tab-complete built rclgd package names."""
    try:
        return sorted(packages.rclgd_packages())
    except Exception:
        return []


class EditorVerb(VerbExtension):
    """Open the Godot editor on an rclgd package's source project."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name', nargs='?',
            help='rclgd package to open (default: the project in the '
                 'current directory)')
        arg.completer = _package_name_completer
        parser.add_argument(
            '--import-only', action='store_true',
            help='run the headless asset import and exit (no editor window; '
                 'requires a project)')

    def main(self, *, args):
        try:
            rclgd_prefix = godot.rclgd_prefix()
            version = godot.pinned_version(rclgd_prefix)
        except godot.GodotRuntimeError as e:
            return str(e)

        # Resolve the project directory, preferring the *source* project — an
        # edit made to the installed copy is silently lost on the next
        # colcon build (unless --symlink-install).
        if args.package_name:
            known = packages.rclgd_packages()
            if args.package_name not in known:
                listing = ', '.join(sorted(known)) or '(none built yet)'
                return (f"unknown rclgd package '{args.package_name}'. "
                        f'built rclgd packages: {listing}')
            info = known[args.package_name]
            project = info['source']
            if project is None:
                project = info['prefix'] / 'share' / args.package_name
                print(f'warning: source project not found; opening the INSTALLED '
                      f'copy at {project} — edits there are overwritten by the '
                      'next colcon build')
        else:
            # No package given: use the project in the current directory, or
            # fall back to Godot's project selection screen.
            project = Path.cwd()
            if not (project / 'project.godot').is_file():
                project = None

        if not godot.binary_path(rclgd_prefix).exists():
            return (f'Godot {version} is not installed. '
                    "hint: run 'ros2 rclgd setup' to download it")

        if args.import_only:
            if project is None:
                return ('--import-only needs a project — pass a package name '
                        'or run inside one')
            code = godot.run_headless_import(rclgd_prefix, project)
            return f'asset import failed (exit {code})' if code != 0 else 0

        launcher = godot.launcher_path(rclgd_prefix)
        if project is None:
            print(f'opening the Godot {version} project manager')
            godot_args = ['--project-manager']
        else:
            print(f'opening {project} with Godot {version}')
            godot_args = ['--editor', '--path', str(project)]
        # Replace the CLI process; the launcher exec's the engine binary.
        os.execv(str(launcher), [str(launcher), *godot_args])
