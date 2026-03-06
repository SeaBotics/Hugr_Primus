from setuptools import setup

package_name = 'hugr_fake_status'

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
    maintainer='seabotics',
    maintainer_email='example@example.com',
    description='Fake status publishers for Hugr Primus GUI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_status = hugr_fake_status.fake_status_node:main',
            'gyro_tf = hugr_fake_status.gyro_tf_node:main',
            'fake_leak = hugr_fake_status.fake_leak:main',
        ],
    },
)
