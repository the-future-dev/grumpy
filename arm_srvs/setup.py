from setuptools import find_packages, setup

package_name = 'arm_srvs'

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
    maintainer='Philip Linderoth',
    maintainer_email='phlinde@kth.se',
    description='Services for the hiwonder servo arm to pick up and drop objects',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_service = arm_srvs.pick_service:main',
            'drop_service = arm_srvs.drop_service:main',
            'arm_camera_service = arm_srvs.arm_camera_service:main',
            'position_service = arm_srvs.positioning_service:main'
        ],
    },
)
