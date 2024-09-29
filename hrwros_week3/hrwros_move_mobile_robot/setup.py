import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hrwros_move_mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', "*.py"))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammed',
    maintainer_email='elyamani.muhammed.eng@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_mobile_robot=hrwros_move_mobile_robot.move_mobile_robot:main',
            'create_unknown_obstacles=hrwros_move_mobile_robot.create_unknown_obstacles:main',
        ],
    },
)