from setuptools import find_packages, setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/imu.launch.py']),
        ('share/' + package_name + '/config',
            ['config/imu_params.yaml']),
    ],
    install_requires=['setuptools', 'adafruit-circuitpython-bno055'],
    zip_safe=True,
    maintainer='Bowen Liu',
    maintainer_email='bliu45@stanford.edu',
    description='Adafruit BNO055 9-DOF IMU driver for ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_node = imu_driver.imu_node:main',
        ],
    },
)
