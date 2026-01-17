import os
import xml.etree.ElementTree as ET
from colcon_core.package_identification import PackageIdentificationExtensionPoint
from colcon_core.plugin_system import satisfies_version

class RclgdPackageIdentification(PackageIdentificationExtensionPoint):
    def __init__(self):
        super().__init__()
        # Ensure compatibility with the colcon-core identification API
        satisfies_version(PackageIdentificationExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def identify(self, desc):
        # 1. Locate the package.xml file
        manifest_path = desc.path / 'package.xml'
        if not manifest_path.exists():
            return None

        try:
            # 2. Parse the XML manifest
            tree = ET.parse(manifest_path)
            root = tree.getroot()
            
            # 3. Extract the Package Name (Critical for colcon to recognize the pkg)
            name_node = root.find('name')
            if name_node is not None:
                desc.name = name_node.text.strip()
            
            # 4. Check for the custom build_type in the <export> section
            # This follows the colcon-ros-cargo logic: only claim it if specified
            export_node = root.find('export')
            if export_node is not None:
                build_type_node = export_node.find('build_type')
                if build_type_node is not None:
                    build_type = build_type_node.text.strip()
                    
                    # Only assign if it matches our registered prefix
                    if build_type == 'ros.rclgd':
                        desc.type = 'ros.rclgd'
                        return
                        
        except Exception as e:
            # Silently fail for malformed XML to let other identification plugins try
            return None