from setuptools import find_packages, setup

package_name = 'planner'

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
    maintainer_email='nhoog@kth.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_exploration = planner.planner_exploration:main',
            'planner_collection = planner.planner_collection:main',
            'a_star_algorithm = planner.a_star_algorithm:main'
        ],
    },
)
