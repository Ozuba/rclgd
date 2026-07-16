from setuptools import find_packages, setup

package_name = 'colcon_rclgd'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ozuba',
    maintainer_email='miguelozubas@gmail.com',
    description='colcon build extension for rclgd (Godot Engine) packages',
    license='MIT',
    entry_points={
        'colcon_core.package_identification': [
            'ros.rclgd = colcon_rclgd.package_identification:RclgdPackageIdentification',
        ],
        'colcon_core.task.build': [
            'ros.rclgd = colcon_rclgd.task:RclgdBuildTask',
        ],
    },
)
