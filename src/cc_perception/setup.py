from glob import glob

from setuptools import find_packages, setup

package_name = "cc_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="CorridorCourier",
    maintainer_email="peterimanishkumar@gmail.com",
    description="RGB-D object detection and 3D projection for the CorridorCourier costmap layer.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = cc_perception.detector_node:main",
            "benchmark_latency = cc_perception.benchmark:main",
        ],
    },
)
