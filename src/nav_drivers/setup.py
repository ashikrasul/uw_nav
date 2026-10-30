from setuptools import find_packages, setup

package_name = 'nav_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashik_rasul',
    maintainer_email='ashik.rasul@outlook.com',
    description='Sensor drivers: GPS, BNO055 IMU, NM3 acoustic modem',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_fix = nav_drivers.gps_fix:main',
            'imu_node = nav_drivers.imu_node:main',
            'acoustic_node = nav_drivers.acoustic_node:main',
        ],
    },
)
