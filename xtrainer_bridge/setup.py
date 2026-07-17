from setuptools import find_packages, setup

package_name = 'xtrainer_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kira',
    maintainer_email='kira@todo.todo',
    description='XTrainer bridge: joint state merger + FollowJointTrajectory action server for MoveIt',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xtrainer_bridge_node = xtrainer_bridge.xtrainer_bridge_node:main',
            'xtrainer_joint_states = xtrainer_bridge.xtrainer_joint_states:main',
            'xtrainer_controller = xtrainer_bridge.xtrainer_controller:main',
        ],
    },
)