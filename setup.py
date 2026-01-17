from setuptools import setup, find_packages

setup(
    name='colcon-rclgd',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'colcon-core',
        'colcon-ros',
    ],
    data_files=[
    # This is the "marker" that tells ROS this package exists
    ('share/ament_index/resource_index/packages', ['resource/colcon-rclgd']),
    # This installs the package.xml so colcon-ros can read dependencies
    ('share/colcon-rclgd', ['package.xml']),
    ],
    entry_points={
    'colcon_core.package_identification': [
        'rclgd = colcon_rclgd.package_identification:RclgdPackageIdentification',
    ],
    'colcon_core.task.build': [
        'ros.rclgd = colcon_rclgd.task:RclgdBuildTask',
    ],
},
)