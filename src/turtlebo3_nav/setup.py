from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtlebot3_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name),
        glob('launch/*launch.[pxy][yma]*')),
        # maps
        (os.path.join('share', package_name, 'maps/'),
        glob('maps/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gxz',
    maintainer_email='gxz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amcl_init_pose_publisher = turtlebot3_nav.set_init_amcl_pose:main',
        ],
    },
)
