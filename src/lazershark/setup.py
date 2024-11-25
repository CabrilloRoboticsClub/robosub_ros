import os
from glob import glob
from setuptools import setup

package_name = 'lazershark'

setup(
    name=package_name,
    version='0.0.1',
    packages=["gazebo_test", "lazershark_sim"],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'lazershark_sim'), glob(os.path.join('lazershark_sim', '*'))),
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
            "thrust=gazebo_test.thrust:main",
            "pilot_input=gazebo_test.pilot_input:main", 
            "imu_converter=gazebo_test.imu_converter:main"
        ],
    },
)