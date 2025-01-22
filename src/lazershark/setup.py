import os
from glob import glob
from setuptools import setup

package_name = 'lazershark'

def generate_data_files(share_path, dir):
    data_files = []
    
    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=["gazebo_test", "lazershark_sim", "auv", "auv.pi", "auv.jetson"],
    include_package_data=True,
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'lazershark_sim', 'config'), glob(os.path.join('lazershark_sim', 'config', '*'))),
        (os.path.join('share', package_name, 'lazershark_sim', 'worlds'), glob(os.path.join('lazershark_sim', 'worlds', '*'))),
        (os.path.join('share', package_name, 'lazershark_sim', 'hooks'), glob(os.path.join('lazershark_sim', 'hooks', '*'))),
        (os.path.join('share', package_name, 'lazershark_sim', 'robot', 'meshes'), glob(os.path.join('lazershark_sim', 'robot', 'meshes', '*'))),
        (os.path.join('share', package_name, 'lazershark_sim', 'robot'), glob(os.path.join('lazershark_sim', 'robot', '*.*'))),

    ] + generate_data_files('share/' + package_name + '/', 'robot'),
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
            "imu_converter=gazebo_test.imu_converter:main",
            "naviguider=auv.pi.naviguider:main",
            "targetpoint=auv.pi.targetpoint:main",
            "a50=auv.pi.a50:main",
        ],
    },
)