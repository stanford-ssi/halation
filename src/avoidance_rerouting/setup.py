from setuptools import setup
import os
from glob import glob

package_name = "avoidance_rerouting"

setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hiroki Kimiwada",
    maintainer_email="hiroki.kimiwada@masason.org",
    description="GPS + Vision navigation for SSI rover.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rover_nav        = avoidance_rerouting.rover_nav:main",
            "vision_detection = avoidance_rerouting.grounding_detect_node:main",
            "mock_gps_imu     = avoidance_rerouting.mock_gps_imu:main",
            "mock_vision      = avoidance_rerouting.mock_vision:main",
        ],
    },
)