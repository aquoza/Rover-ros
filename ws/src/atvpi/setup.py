from setuptools import find_packages, setup

package_name = 'atvpi'

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
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub = atvpi.sub_node:main',
            'talker = atvpi.pub:main',
            'gamepad_websocket_node = atvpi.gamepad_websocket_node:main',
            'thermal_camera_node = atvpi.thermal_camera:main',
            'sub2 = atvpi.sub_node2:main',

        ],
    },
)
