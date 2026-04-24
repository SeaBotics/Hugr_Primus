import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_primus'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Idsoe',
    maintainer_email='kontakt@seaboticsuia.com',
    description='Package for the control and distribution of forces from NAV2',
    license='APACHE 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'velocity_controller_node = control_primus.velocity_controller_node:main',
            'thrust_allocation_node = control_primus.thrust_allocation_node:main',
        ],
    },
)
