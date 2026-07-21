from setuptools import find_packages, setup

package_name = 'xtrainer_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start.launch.py']),
        ('share/' + package_name + '/launch', ['launch/read_pose.launch.py']),
        ('share/' + package_name + '/config', ['config/moveit_cpp.yaml',
                                                 'config/xtrainer.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kira',
    maintainer_email='kira@todo.todo',
    description='XTrainer task-level operations using MoveIt',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'start = xtrainer_task.start:main',
            'dino_test = xtrainer_task.dino_test:main',
            'read_pose = xtrainer_task.read_pose:main',
        ],
    },
)