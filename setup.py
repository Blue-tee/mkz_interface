from setuptools import setup

package_name = 'mkz_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trc',
    maintainer_email='zillur991@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'dbw_can_to_autoware = mkz_interface.dbw_can_to_autoware:main',
                'autoware_to_dbw_can = mkz_interface.autoware_to_dbw_can:main'
        ],
    },
)
