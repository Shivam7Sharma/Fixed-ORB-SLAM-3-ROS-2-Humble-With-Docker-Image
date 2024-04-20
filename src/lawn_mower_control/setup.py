from setuptools import setup
import os
from glob import glob

package_name = 'lawn_mower_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhooi',
    maintainer_email='mhooi3@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['diff_drive = lawn_mower_control.diff_drive:main',
            'manual_control = lawn_mower_control.manual_control:main',
            'mowing_planner = lawn_mower_control.mowing_planner:main',
        ],
    },
)
