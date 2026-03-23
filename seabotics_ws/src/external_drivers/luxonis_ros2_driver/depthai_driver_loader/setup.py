<<<<<<< Updated upstream
from setuptools import find_packages, setup

package_name = 'depthai_driver_loader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='carlos.mauricio.leal@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'download = depthai_driver_loader.download_depthai:main'
	],
    },
)
=======

from setuptools import find_packages, setup

package_name = 'depthai_driver_loader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
	'setuptools',
	'depthai>=2.22.0',
	],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='carlos.mauricio.leal@outlook.com',
    description='Utility package that downloads, builds and loads Luxonis DepthAI ROS-2 drivers',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'download = depthai_driver_loader.download_depthai:main'
	],
    },
)
>>>>>>> Stashed changes
