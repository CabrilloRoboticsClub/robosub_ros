import os
from glob import glob
from setuptools import setup

package_name = 'lazershark'

setup(
    name=package_name,
    version='0.0.1',
    packages=['lazershark'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resources/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'resources'), glob(os.path.join('resources', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maximus',
    maintainer_email='matera@lifealgorithmic.com',
    description='TODO: Package description',
    license='AGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "thrust=lazershark.thrust:main",
            "pilot_input=lazershark.pilot_input:main", 
        ],
    },
)