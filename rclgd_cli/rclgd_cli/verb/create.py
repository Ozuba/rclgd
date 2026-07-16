import os
import re
import shutil
import subprocess
from importlib import resources
from pathlib import Path
from string import Template

from ros2cli.verb import VerbExtension

from rclgd_cli.api import godot

# Files in the template tree that go through string.Template substitution;
# everything else is copied verbatim.
TEMPLATED_FILES = ('package.xml', 'project.godot', 'main.gd')

# (feature tag, rendering_method) per --renderer choice; forward_plus is the
# project default and needs no [rendering] section.
RENDERERS = {
    'forward_plus': ('Forward Plus', None),
    'mobile': ('Mobile', 'mobile'),
    'gl_compatibility': ('GL Compatibility', 'gl_compatibility'),
}

RENDERING_SECTION = """
[rendering]

renderer/rendering_method="{method}"
"""


def _git_config(key, default):
    try:
        out = subprocess.run(
            ['git', 'config', '--get', key],
            capture_output=True, text=True, timeout=5)
        return out.stdout.strip() or default
    except (OSError, subprocess.TimeoutExpired):
        return default


class CreateVerb(VerbExtension):
    """Create a new rclgd (Godot) package."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'package_name',
            help='lower_snake_case name of the new package')
        parser.add_argument(
            '--destination-directory', default=os.curdir,
            help='directory to create the package in (default: .)')
        parser.add_argument(
            '--description', default='TODO: Package description',
            help='package description for package.xml')
        parser.add_argument(
            '--license', default='MIT',
            help='license for package.xml (default: MIT)')
        parser.add_argument(
            '--maintainer-name', default=None,
            help='maintainer name (default: from git config user.name)')
        parser.add_argument(
            '--maintainer-email', default=None,
            help='maintainer email (default: from git config user.email)')
        parser.add_argument(
            '--dependencies', nargs='*', default=[],
            help='exec_depend entries, e.g. message packages the project uses')
        parser.add_argument(
            '--renderer', choices=sorted(RENDERERS),
            default='forward_plus',
            help='Godot rendering method (default: forward_plus; use '
                 'gl_compatibility for headless CI or weak GPUs)')
        parser.add_argument(
            '--dry-run', action='store_true',
            help='print what would be created without writing anything')

    def main(self, *, args):
        name = args.package_name
        if not re.fullmatch(r'[a-z][a-z0-9_]*', name):
            return ('package name must be lower_snake_case ([a-z][a-z0-9_]*) '
                    'to satisfy ROS 2 naming rules')

        dest = Path(args.destination_directory).resolve() / name
        if dest.exists():
            return f"'{dest}' already exists"

        template_root = Path(str(resources.files('rclgd_cli') / 'template'))
        if not (template_root / 'project.godot').is_file():
            return f'project template not found at {template_root} — broken rclgd_cli install'

        godot_version = '4.7'
        try:
            godot_version = godot.pinned_version(godot.rclgd_prefix())
        except godot.GodotRuntimeError:
            pass

        renderer_feature, rendering_method = RENDERERS[args.renderer]
        substitutions = {
            'name': name,
            'description': args.description,
            'license': args.license,
            'maintainer_name': args.maintainer_name or _git_config(
                'user.name', os.environ.get('USER', 'user')),
            'maintainer_email': args.maintainer_email or _git_config(
                'user.email', 'user@todo.todo'),
            'depends': ''.join(
                f'  <exec_depend>{dep}</exec_depend>\n'
                for dep in args.dependencies),
            'godot_version': godot_version,
            'renderer_feature': renderer_feature,
            'rendering_section': (
                RENDERING_SECTION.format(method=rendering_method)
                if rendering_method else ''),
        }

        print(f'creating rclgd package at {dest}')
        for path in sorted(template_root.rglob('*')):
            if path.is_file():
                print(f'  {name}/{path.relative_to(template_root)}')

        if args.dry_run:
            print('dry run: nothing written')
            return 0

        shutil.copytree(template_root, dest)
        for rel in TEMPLATED_FILES:
            f = dest / rel
            f.write_text(Template(f.read_text()).substitute(substitutions))

        print('next steps:')
        print(f'  colcon build --packages-select {name} && source install/setup.bash')
        print(f'  ros2 rclgd editor {name}')
        print(f'  ros2 run {name} {name}')
        return 0
