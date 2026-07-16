from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class RclgdCommand(CommandExtension):
    """Tools for rclgd (Godot) packages."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        # Get verb extensions and let them add their arguments
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'rclgd_cli.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            # In case no verb was passed
            self._subparser.print_help()
            return 0
        extension = getattr(args, '_verb')
        return extension.main(args=args)
