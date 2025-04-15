from setuptools import find_packages, setup

package_name = 'detection_arm_cam'

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
    description='A package for the arm camera to detect if an object is in the gripper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_arm_cam = detection_arm_cam.detection_arm_cam:main'
        ],
    },
)
