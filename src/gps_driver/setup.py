from setuptools import find_packages, setup

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/gps.launch.py', 'launch/gps_imu.launch.py']),
        ('share/' + package_name + '/config',
            ['config/gps_params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Bowen Liu',
    maintainer_email='bliu45@stanford.edu',
    description='Adafruit Ultimate GPS driver for ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_node = gps_driver.gps_node:main',
        ],
    },
)
