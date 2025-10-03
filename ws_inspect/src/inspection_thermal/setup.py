from setuptools import setup

package_name = "inspection_thermal"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/params", ["params/thermal.yaml"]),
        (f"lib/{package_name}", ["scripts/thermal_alarm_node"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vincent",
    maintainer_email="vincent@example.com",
    description="Thermal alarm node for inspection targets.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "thermal_alarm_node = inspection_thermal.thermal_alarm_node:main",
        ],
    },
)
