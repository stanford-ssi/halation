from setuptools import find_packages, setup

package_name = 'rover_station_sync'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'fastapi-proxy-lib',
    ],
    zip_safe=True,
    maintainer='Hinson Chan',
    maintainer_email='hinson@stanford.edu',
    description='package for bidirectional communication between rover and station',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fastapi_proxy_node = rover_station_sync.proxy_node:main',
        ],
    },
)
