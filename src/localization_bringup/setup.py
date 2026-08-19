import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'localization_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hectorla',
    maintainer_email='hectorla@espol.edu.ec',
    description='Bringup de localización (nav2 AMCL) para F1TENTH: launch + config + nodo scan_throttle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_throttle = localization_bringup.scan_throttle:main',
        ],
    },
)
