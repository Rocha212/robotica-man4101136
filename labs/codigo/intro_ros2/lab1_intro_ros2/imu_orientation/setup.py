from setuptools import find_packages, setup

package_name = 'imu_orientation'

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
    description='Mini INS: integración de orientación a partir de una IMU.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_integrator = imu_orientation.imu_integrator:main',
            'imu_simulator = imu_orientation.imu_simulator:main',
        ],
    },
)
