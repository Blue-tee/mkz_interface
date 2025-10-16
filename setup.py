from setuptools import setup
from glob import glob
import os

package_name = 'mkz_interface'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Lincoln MKZ interface: Autoware ↔ Dataspeed DBW1 bridge (ROS 2 Humble)',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'autoware_to_dbw_can.py = mkz_interface.autoware_to_dbw_can:main',
            'dbw_can_to_autoware.py = mkz_interface.dbw_can_to_autoware:main',
        ],
    },
)

