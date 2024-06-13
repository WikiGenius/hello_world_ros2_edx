from setuptools import find_packages, setup
import glob
package_name = 'hrwros_week3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob.glob('launch/*.launch.py')
         ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammed',
    maintainer_email='elyamani.muhammed.eng@gmail.com',
    description='The hrwros_week3 package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'script = hrwros_week3.script:main',
            # Add other scripts here
        ],
    },
)
