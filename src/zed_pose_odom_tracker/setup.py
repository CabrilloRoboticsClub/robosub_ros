from setuptools import find_packages, setup

package_name = 'zed_pose_odom_tracker'

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
    maintainer='fxwllms',
    maintainer_email='fxwllms1@gmail.com',
    description='Setup for transform node of the ZED Pose Odom Tracker',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_pose_odom_node = zed_pose_odom_tracker.zed_pose_odom_tracker:main',
        ],
    },
)
