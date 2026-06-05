from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'w102_path_planning'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CTI One',
    maintainer_email='w102@cti.com',
    description='W102 graph-based path planning simulation in a living room.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'w102_path_sim     = w102_path_planning.w102_path_sim:main',
            'w102_gazebo_nav   = w102_path_planning.w102_gazebo_nav:main',
            'w102_viz_markers  = w102_path_planning.w102_viz_markers:main',
        ],
    },
)
