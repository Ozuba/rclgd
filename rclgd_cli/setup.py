from setuptools import find_packages, setup

package_name = 'rclgd_cli'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test']),
    # Minimal project layout copied by `ros2 rclgd create` (globs must name
    # each directory level and skip dotfiles, so .gitignore is listed
    # explicitly)
    package_data={'rclgd_cli': [
        'template/*',
        'template/.gitignore',
        'template/addons/rclgd/*',
        'template/addons/rclgd/bin/*',
    ]},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='ozuba',
    maintainer_email='miguelozubas@gmail.com',
    description='ros2cli extension for rclgd: create/edit Godot packages and '
                'manage the Godot runtime',
    license='MIT',
    # Entry points are declared in setup.cfg
)
