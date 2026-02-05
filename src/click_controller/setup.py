from setuptools import find_packages, setup

package_name = 'click_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/click_controller']),
    ('share/click_controller', ['package.xml']),
    ('share/click_controller/launch',
        ['launch/demo.launch.py']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-student',
    maintainer_email='ros-student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'gui_controller = click_controller.gui_controller:main',
    ],
},

)
