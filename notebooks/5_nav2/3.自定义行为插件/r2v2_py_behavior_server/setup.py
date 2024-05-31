from setuptools import find_packages, setup

package_name = 'r2v2_py_behavior_server'

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
    maintainer='wtr2023',
    maintainer_email='quan.2003@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'putball_server = r2v2_py_behavior_server.putball_server:main',
            'sensors_listener = r2v2_py_behavior_server.sensors_listener:main',
            'chassis_move_server=r2v2_py_behavior_server.chassis_node:main'
        ],
    },
)
