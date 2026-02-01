from setuptools import find_packages, setup

package_name = 'turtlebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/turtlebot_control']),
    ('share/turtlebot_control', ['package.xml']),
    ('share/turtlebot_control/launch', ['launch/system.launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	       'control_node = turtlebot_control.control_node:main',
        ],
    },
)
