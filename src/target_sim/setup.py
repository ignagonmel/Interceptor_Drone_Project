from setuptools import find_packages, setup

package_name = 'target_sim'

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
    maintainer='ignagonmel',
    maintainer_email='ignagonmelgmail.com',
    description='Simulacion de trayectoria de objetivo evasivo 3D',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'shahed_flight_controller = target_sim.shahed_flight_controller:main'
        ],
    },
)