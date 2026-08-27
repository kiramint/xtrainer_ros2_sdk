from setuptools import find_packages, setup

package_name = 'xtrainer_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start.launch.py']),
        ('share/' + package_name + '/launch', ['launch/calibrate_evaluate.launch.py']),
        ('share/' + package_name + '/launch', ['launch/calibrate_top.launch.py']),
        ('share/' + package_name + '/launch', ['launch/calibrate_top_435.launch.py']),
        ('share/' + package_name + '/launch', ['launch/calibrate_left.launch.py']),
        ('share/' + package_name + '/launch', ['launch/calibrate_right.launch.py']),
        ('share/' + package_name + '/launch', ['launch/enable.launch.py']),
        ('share/' + package_name + '/launch', ['launch/disable.launch.py']),
        ('share/' + package_name + '/launch', ['launch/enable_and_drag.launch.py']),
        ('share/' + package_name + '/launch', ['launch/clear_error.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kira',
    maintainer_email='kira@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'enable_arms = xtrainer_control.enable_arms:main',
            'disable_arms = xtrainer_control.disable_arms:main',
            'enable_and_drag = xtrainer_control.enable_and_drag:main',
            'clear_error = xtrainer_control.clear_error:main',
        ],
    },
)
