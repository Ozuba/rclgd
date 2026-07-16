from ros2cli.verb import VerbExtension

from rclgd_cli.api import godot


class SetupVerb(VerbExtension):
    """Download the Godot editor binary this rclgd build targets."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--force', action='store_true',
            help='re-download even if the binary already exists')
        parser.add_argument(
            '--skip-checksum', action='store_true',
            help='do not verify the archive against the official SHA512 sums')

    def main(self, *, args):
        try:
            prefix = godot.rclgd_prefix()
            version = godot.pinned_version(prefix)

            binary = godot.binary_path(prefix)
            if binary.exists() and not args.force:
                print(f'Godot {version} already installed: {binary}')
                print('use --force to re-download')
                return 0

            print(f'rclgd requires Godot {version} ({godot.platform_slug()})')
            dest = godot.download_godot(
                prefix, force=args.force, skip_checksum=args.skip_checksum)
        except godot.GodotRuntimeError as e:
            return str(e)

        print(f'installed to {dest}')
        print('try it: ros2 run rclgd godot --version')
        return 0
