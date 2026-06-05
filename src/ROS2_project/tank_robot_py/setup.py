from setuptools import find_packages, setup

package_name = 'tank_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='horvathbd',
    maintainer_email='horvathbd@todo.todo',
    description='Tank robot python scripts for aiming and anomaly detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turret_controller = tank_robot_py.turret_controller:main',
            'gun_controller = tank_robot_py.gun_controller:main',
            'shoot_controller = tank_robot_py.shoot_controller:main',
            'turret_cam=tank_robot_py.turret_cam:main',
        ],
    },
)