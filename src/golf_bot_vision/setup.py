from glob import glob

from setuptools import find_packages, setup

package_name = 'golf_bot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadeem',
    maintainer_email='nackawat@gmail.com',
    description='YOLO golf ball detection and ball-chasing controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'golf_ball_detector = golf_bot_vision.golf_ball_detector:main',
            'ball_chaser = golf_bot_vision.ball_chaser:main',
        ],
    },
)
