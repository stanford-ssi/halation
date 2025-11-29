from setuptools import setup
import os
from glob import glob

package_name = 'avoidance_rerouting'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- ADD THIS LINE ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hiroki Kimiwada',
    maintainer_email='hiroki.kimiwada@masason.org',
    description='Mock LiDAR and local rerouting package for rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_lidar_publisher = avoidance_rerouting.mock_lidar_publisher:main',
            'lidar_detection = avoidance_rerouting.lidar_detection:main',
            'rerouting_node = avoidance_rerouting.rerouting_node:main',
        ],
    },
)