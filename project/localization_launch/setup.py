from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'localization_launch'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Malte Sundgren',
    maintainer_email='maltesu@kth.se',
    description='',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={},
)
