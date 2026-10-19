from setuptools import setup

package_name = "imu_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/imu.launch.py"]),
        ("share/" + package_name + "/config", ["config/imu_node_config.json"]),
    ],
    install_requires=["setuptools", "eclipse-zenoh"],
    zip_safe=True,
    maintainer="Hellbender Inc",
    maintainer_email="support@hellbender.com",
    description="ROS 2 node that consumes IMU data published as JSON over Zenoh and republishes it as sensor_msgs/Imu plus the raw JSON payload.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_node = imu_node.imu_node:main",
        ],
    },
)
