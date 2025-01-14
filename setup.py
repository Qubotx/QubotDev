import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tortubot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share/' + package_name, 'description'), glob(os.path.join('description', '*.*'))),
        (os.path.join('share/' + package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmoralesd',
    maintainer_email='claudio.morales55@inacapmail.cl',
    description='Paquete base para el control de Tortubot en entorno de simulación',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
