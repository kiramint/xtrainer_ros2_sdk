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
        ('share/' + package_name + '/launch', ['launch/start_open_bottle.launch.py']),
        ('share/' + package_name + '/launch', ['launch/start_insert_straw.launch.py']),
        ('share/' + package_name + '/launch', ['launch/start_grasp_garbage.launch.py']),
        ('share/' + package_name + '/launch', ['launch/start_grasp_plane.launch.py']),
        ('share/' + package_name + '/launch', ['launch/read_pose.launch.py']),
        ('share/' + package_name + '/launch', ['launch/goto_pose.launch.py']),
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
            'start_open_bottle = xtrainer_task.start_open_bottle:main',
            'start_insert_straw = xtrainer_task.start_insert_straw:main',
            'start_grasp_garbage = xtrainer_task.start_grasp_garbage:main',
            'start_grasp_garbage = xtrainer_task.start_grasp_plane:main',
            'dino_test = xtrainer_task.dino_test:main',
            'dino_pose_test = xtrainer_task.dino_pose_test:main',
            'dino_contour_test = xtrainer_task.dino_contour_test:main',
            'detect_desk_object = xtrainer_task.detect_desk_object:main',
            'detect_desk_object2 = xtrainer_task.detect_desk_object2:main',
            'read_pose = xtrainer_task.read_pose:main',
            'goto_pose = xtrainer_task.goto_pose:main',
            'graspnet_test = xtrainer_task.graspnet_test:main',
            'graspnet_sam_test = xtrainer_task.graspnet_sam_test:main',
            'graspnet_sam_region_test = xtrainer_task.graspnet_sam_region_test:main',
        ],
    },
)