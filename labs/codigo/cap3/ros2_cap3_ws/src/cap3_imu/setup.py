from setuptools import find_packages, setup

package_name = 'cap3_imu'

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
    maintainer='Tomas Rocha',
    maintainer_email='',
    description='Simulador de IMU e nodo integrador de orientación con TF2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_simulator = cap3_imu.imu_simulator:main',
            'imu_integrator = cap3_imu.imu_integrator:main',
        ],
    },
)
