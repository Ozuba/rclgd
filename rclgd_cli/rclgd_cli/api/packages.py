"""Discovery of built rclgd packages via the ament index.

colcon_rclgd registers every rclgd package under the 'rclgd_project' resource
type, with the package's source directory as the marker content so tooling can
open the editable project instead of the installed copy.
"""

from pathlib import Path

from ament_index_python.resources import get_resource
from ament_index_python.resources import get_resources

RESOURCE_TYPE = 'rclgd_project'


def rclgd_packages():
    """Return {name: {'prefix': Path, 'source': Path | None}}.

    'source' is the package's source directory as recorded at build time, or
    None if it no longer contains a Godot project (moved/deleted, or the
    package was built by an older colcon_rclgd that didn't record it).
    """
    result = {}
    for name, prefix in get_resources(RESOURCE_TYPE).items():
        content, _ = get_resource(RESOURCE_TYPE, name)
        source = Path(content.strip()) if content.strip() else None
        if source is not None and not (source / 'project.godot').is_file():
            source = None
        result[name] = {'prefix': Path(prefix), 'source': source}
    return result
