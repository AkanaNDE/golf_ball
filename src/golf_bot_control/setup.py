from setuptools import find_packages, setup

package_name = 'golf_bot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadeem',
    maintainer_email='nackawat@gmail.com',
    description='Xbox teleop and wheel odometry for the golf ball collector robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_teleop = golf_bot_control.xbox_teleop:main',
            'wheel_odometry = golf_bot_control.wheel_odometry:main',
        ],
    },
)
