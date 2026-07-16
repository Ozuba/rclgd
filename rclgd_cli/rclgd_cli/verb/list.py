import sys

from ros2cli.verb import VerbExtension

from rclgd_cli.api import packages


class ListVerb(VerbExtension):
    """List built rclgd packages."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--names-only', action='store_true',
            help='print package names only (script friendly)')

    def main(self, *, args):
        pkgs = packages.rclgd_packages()
        if not pkgs:
            print('no rclgd packages found — build one and source your '
                  'workspace', file=sys.stderr)
            return 0
        for name in sorted(pkgs):
            if args.names_only:
                print(name)
                continue
            info = pkgs[name]
            source = info['source'] or '(source not found)'
            print(f'{name}\t{info["prefix"]}\t{source}')
        return 0
