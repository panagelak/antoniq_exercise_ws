from setuptools import find_packages, setup

package_name = 'antoniq_mission_status_rqt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Panagiotis Angelakis',
    maintainer_email='panagiotis.angelakis.robot@gmail.com',
    description='rqt GUI plugin that displays antoniq_interfaces/MissionStatus.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'antoniq_mission_status_rqt = antoniq_mission_status_rqt.main:main',
        ],
    },
)
