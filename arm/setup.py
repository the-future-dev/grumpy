from setuptools import find_packages, setup

package_name = 'arm_manipulation'

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
    maintainer='phlinde',
    maintainer_email='phlinde@kth.se',
    description='Instructions and calculaitons for the hiwonder servo arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = arm_manipulation.servo_manipulator:main',
        ],
    },
)