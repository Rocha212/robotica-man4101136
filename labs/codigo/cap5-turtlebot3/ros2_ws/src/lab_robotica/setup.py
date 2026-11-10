from setuptools import find_packages, setup

package_name = 'lab_robotica'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Estudiantes de Robotica',
    maintainer_email='student@example.com',
    description='Controladores de TurtleBot3 para el laboratorio de robotica.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_goal = lab_robotica.go_to_goal:main',
            'line_following = lab_robotica.line_following:main',
            'pure_pursuit = lab_robotica.pure_pursuit:main',
        ],
    },
)
