from setuptools import setup

package_name = 'hugr_fake_status'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ornevikanders',
    maintainer_email='none@none.com',
    description='Fake status publisher + gyro TF for Hugr RViz panel',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_status = hugr_fake_status.fake_status_node:main',
            'gyro_tf = hugr_fake_status.gyro_tf_node:main',
        ],
    },
)
