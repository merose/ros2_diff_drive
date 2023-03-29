import os
from glob import glob

from setuptools import setup

package_name = 'diff_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='merose',
    maintainer_email='markrose@acm.org',
    description='Differential drive robot notes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample = diff_drive.sample:main',
            'diff_drive_odometry = diff_drive.diff_drive_odometry:main',
            'diff_drive_mock_robot = diff_drive.diff_drive_mock_robot:main',
            'diff_drive_controller = diff_drive.diff_drive_controller:main',
            'diff_drive_go_to_goal = diff_drive.diff_drive_go_to_goal:main'
        ],
    },
)
