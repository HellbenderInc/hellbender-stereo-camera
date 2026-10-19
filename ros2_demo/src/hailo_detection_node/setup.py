from setuptools import setup

package_name = "hailo_detection_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hailo_detection.launch.py"]),
        ("share/" + package_name + "/config", ["config/node_config.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hellbender Inc",
    maintainer_email="support@hellbender.com",
    description="ROS 2 node for person detection via Hailo-8 on a stereo depth+colour image stream.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hailo_detection_node = hailo_detection_node.hailo_detection_node:main",
        ],
    },
)
