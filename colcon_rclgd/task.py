import os
import shutil
import stat
from pathlib import Path
from colcon_core.task import TaskExtensionPoint, run
# New Imports needed for shell script generation
from colcon_core.environment import create_environment_hooks, create_environment_scripts
from colcon_core.shell import create_environment_hook
from colcon_core.plugin_system import satisfies_version

class RclgdBuildTask(TaskExtensionPoint):
    def __init__(self):
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def build(self, *, additional_hooks=None):
        pkg = self.context.pkg
        args = self.context.args
        additional_hooks = [] if additional_hooks is None else additional_hooks
        
        # Standard ROS 2 install structure
        package_install_base = Path(args.install_base)
        install_share = package_install_base / 'share' / pkg.name
        install_lib = package_install_base / 'lib' / pkg.name

        # 1. Synchronize Project Files
        if args.symlink_install:
            self._create_symlink(pkg.path, install_share)
        else:
            if install_share.exists():
                shutil.rmtree(install_share)
            shutil.copytree(pkg.path, install_share)

        # 2. Deploy Ament Index Markers
        self._deploy_markers(package_install_base, pkg.name)

        # 3. Create Godot Launcher Shim
        self._create_shim(install_lib, pkg.name)

        # 4. Export godots assets
        await self._export_resources(package_install_base, install_share, pkg.name)

        # 4. Generate Shell Sourcing Files
        # First, create the specific ROS hook for this package
        additional_hooks.extend(
            create_environment_hook(
                'ament_prefix_path', package_install_base, 
                pkg.name, 'AMENT_PREFIX_PATH', '', mode='prepend'
            )
        )

        # Gather default hooks (PATH, etc.)
        default_hooks = create_environment_hooks(args.install_base, pkg.name)

        # TRIGGER: This creates package.sh, package.bash, package.zsh, etc.
        create_environment_scripts(
            pkg, args, default_hooks=default_hooks, additional_hooks=additional_hooks
        )

        return 0

    def _deploy_markers(self, prefix, pkg_name):
        marker_dir = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
        marker_dir.mkdir(parents=True, exist_ok=True)
        (marker_dir / pkg_name).touch()

    def _create_shim(self, lib_dir, pkg_name):
        lib_dir.mkdir(parents=True, exist_ok=True)
        shim_path = lib_dir / pkg_name
        content = f"""#!/bin/bash
RCLGD_PREFIX=$(ros2 pkg prefix rclgd 2>/dev/null || echo "{lib_dir.parent.parent}/rclgd")
PROJECT_PREFIX=$(ros2 pkg prefix {pkg_name} 2>/dev/null || echo "{lib_dir.parent.parent}/{pkg_name}")
GODOT_EXE="$RCLGD_PREFIX/lib/rclgd/godot"
PROJECT_PATH="$PROJECT_PREFIX/share/{pkg_name}"
exec "$GODOT_EXE" --path "$PROJECT_PATH" "$@"
"""
        with open(shim_path, 'w') as f:
            f.write(content)
        os.chmod(shim_path, os.stat(shim_path).st_mode | stat.S_IEXEC)

    def _create_symlink(self, target, link):
        if os.path.islink(link) or os.path.exists(link):
            os.remove(link) if os.path.islink(link) else shutil.rmtree(link)
        os.makedirs(link.parent, exist_ok=True)
        os.symlink(target, link)

    async def _export_resources(self, prefix, project_path, pkg_name):
        """Headless import of Godot assets into the install directory."""

        # Locate the Godot binary. 
        # We check the current install prefix (if rclgd is in the same workspace)
        # and fall back to searching the ROS 2 environment.
        godot_bin = prefix.parent / 'rclgd' / 'lib' / 'rclgd' / 'godot'

        # Fallback: check if 'rclgd' is already installed in the underlay
        if not godot_bin.exists():
            import subprocess
            try:
                # Use 'ros2 pkg prefix' to find where rclgd is installed
                res = subprocess.check_output(['ros2', 'pkg', 'prefix', 'rclgd'], text=True).strip()
                godot_bin = Path(res) / 'lib' / 'rclgd' / 'godot'
            except Exception:
                pass

        if not godot_bin.exists():
            print(f"[{pkg_name}] Warning: Godot binary not found. Skipping resource export.")
            return

        print(f"[{pkg_name}] Exporting/Importing Godot resources headlessly...")

        # Build the command. 
        # Using --editor + --quit-after is the standard way to trigger 
        # the initial import and shader compilation in Godot 4.
        cmd = [
            str(godot_bin),
            '--editor',
            '--headless',
            '--path', str(project_path),
            '--quit-after', '20' # Increase this if your project has many 3D assets
        ]

        try:
            # We use colcon's 'run' to ensure the output is captured 
            # in the colcon logs and respects the build context.
            await run(self.context, cmd, cwd=str(project_path))
        except Exception as e:
            print(f"[{pkg_name}] Resource export encountered an error: {e}")