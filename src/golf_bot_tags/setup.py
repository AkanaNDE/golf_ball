from setuptools import find_packages, setup

package_name = 'golf_bot_tags'

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
    description='AprilTag tour mission for the golf ball collector robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tag_mission = golf_bot_tags.tag_mission:main',
            'tag_focal_calibration = golf_bot_tags.tag_focal_calibration:main',
        ],
    },
)
