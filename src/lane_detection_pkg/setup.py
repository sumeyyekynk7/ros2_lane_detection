from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'lane_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['videos/*.mp4']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lviv',
    maintainer_email='lviv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lane_node = lane_detection_pkg.lane_node:main',
            'direction_publisher = '
            'lane_detection_pkg.direction_publisher:main',
            'direction_subscriber = '
            'lane_detection_pkg.direction_subscriber:main',
            'lane_controller = lane_detection_pkg.lane_controller:main',
            'video_reader = lane_detection_pkg.video_reader:main',
        ],
    },
)
