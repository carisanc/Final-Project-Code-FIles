import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'path_planning'

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
        (os.path.join('share', package_name, 'racelines'), glob('racelines/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hectorla',
    maintainer_email='hectorla@espol.edu.ec',
    description='Raceline de mínima curvatura para F1TENTH: generador offline (mapa → CSV) + nodo publicador',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_raceline = path_planning.generate_raceline:main',
            'raceline_publisher = path_planning.raceline_publisher:main',
        ],
    },
)
