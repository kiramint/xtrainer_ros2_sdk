from setuptools import find_packages, setup

package_name = 'xtrainer_gripper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/gripper.launch.py',
            'launch/gripper_open.launch.py',
            'launch/gripper_close.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kira',
    maintainer_email='kira@todo.todo',
    description='XTrainer standalone gripper driver',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = xtrainer_gripper.gripper_node:main',
            'gripper_open_arms = xtrainer_gripper.gripper_open_arms:main',
            'gripper_close_arms = xtrainer_gripper.gripper_close_arms:main',
            'gripper_open_close_demo = demo.gripper_open_close_demo:main',
            'gripper_constant_force_demo = demo.gripper_constant_force_demo:main',
            'scan_servo_id = demo.scan_servo_id:main',
        ],
    },
)