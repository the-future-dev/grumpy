from setuptools import find_packages, setup

package_name = 'drive_control'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_path = drive_control.drive_path:main',
            'local_obstacle_avoidance = drive_control.local_obstacle_avoidance:main',
            'drive_2 = drive_control.drive_2:main'
        ],
    },
)
