from setuptools import find_packages, setup

package_name = 'teleoperation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Standard ROS2-filer (La stå - disse er kritiske for at pakken finnes)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # TEKNIKER-NOTAT: Legg til nye mapper her (f.eks. launch eller config)
        # Hvis du legger til en launch-fil senere, fjern # foran linjen under:
        # ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'], # TEKNIKER-NOTAT: Python-biblioteker (pip) listes her.
    zip_safe=True,
    maintainer='SeaBotics',
    maintainer_email='deg@seabotics.com',
    description='Teleoperation package for SeaBotics Hugr Primus',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # HER ER KOBLINGEN (Tenningsnøkkelen):
            # 'kommando_i_terminal = mappe.filnavn:hovedfunksjon'
            'teleoperation_node = teleoperation.teleoperation_node:main',
            
            # TEKNIKER-NOTAT: Legg til nye linjer her for hver nye .py-fil du vil kjøre.
            # Husk komma på slutten av linjen!
        ],
    },
)
