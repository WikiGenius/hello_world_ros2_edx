from setuptools import find_packages, setup
from glob import glob

package_name = 'hrwros_week1_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammed',
    maintainer_email='elyamani.muhammed.eng@gmail.com',
    description='The hrwros_week1_assignment package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'week1_assignment1_part1 = hrwros_week1_assignment.week1_assignment1_part1:main',
            'week1_assignment1_part3 = hrwros_week1_assignment.week1_assignment1_part3:main',
            'week1_assignment2 = hrwros_week1_assignment.week1_assignment2:main',
            'week1_assignment3 = hrwros_week1_assignment.week1_assignment3:main',

        ],
    },
)
