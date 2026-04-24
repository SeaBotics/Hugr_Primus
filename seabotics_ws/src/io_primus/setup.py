from setuptools import find_packages, setup

package_name = 'io_primus'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='idsoj',
    maintainer_email='kontakt@seaboticsuia.com',
    description='Bridge between the Nvidia Jetson Orin Nano 8GB used by SeaBotics, on the Hugr Primus, to interact with a Arduino based MCU',
    license='FREE USE',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arduino_bridge = io_primus.arduino_bridge_node:main',
        ],
    },
)
