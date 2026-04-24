from setuptools import find_packages, setup

package_name = 'battery_monitoring'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'bleak'],
    zip_safe=True,
    maintainer='SeaBotics',
    maintainer_email='kontakt@seaboticsuia.com',
    description='BMS monitoring for SeaBotics ASV over Bluetooth',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bms_driver = battery_monitoring.bms_driver_node:main',
            'bms_analytics = battery_monitoring.battery_analytics_node:main',
        ],
    },
)
