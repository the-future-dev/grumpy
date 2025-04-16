from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'models'),
         glob(os.path.join('weights', '05.pth'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group-5',
    maintainer_email='andre.ritossa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'detection_node = {package_name}.detection_node:main'
        ],
    },
)
