from setuptools import setup

package_name = 'elite_robot_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Elite CS Robot Control Example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = elite_robot_example.robot_controller:main',
            'status_monitor = elite_robot_example.status_monitor:main',
            'robot_basic_control = elite_robot_example.robot_basic_control:main',
            'robot_cartesian_control = elite_robot_example.robot_cartesian_control:main',
        ],
    },
)
