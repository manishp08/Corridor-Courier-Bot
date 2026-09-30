from setuptools import find_packages, setup

package_name = "cc_eval"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy", "matplotlib"],
    zip_safe=True,
    maintainer="CorridorCourier",
    maintainer_email="peterimanishkumar@gmail.com",
    description="Evaluation harness, metrics and report generation for CorridorCourier.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "harness = cc_eval.harness_node:main",
            "run_ablation = cc_eval.run_ablation:main",
            "sensor_sanity = cc_eval.sensor_sanity:main",
            "drift_test = cc_eval.drift_test:main",
            "report = cc_eval.report:main",
        ],
    },
)
