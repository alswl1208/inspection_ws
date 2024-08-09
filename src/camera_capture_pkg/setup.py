from setuptools import setup

package_name = 'camera_capture_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bt_executor.launch.xml', 'launch/nav2_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minji',
    maintainer_email='your-email@example.com',
    description='Camera capture node for ROS 2',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinate_service_node = camera_capture_pkg.coordinate_service_node:main',
            'camera_node = camera_capture_pkg.camera_node:main',
        ],
    },
)
