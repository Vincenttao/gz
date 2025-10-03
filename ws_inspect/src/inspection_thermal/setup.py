from setuptools import setup

package_name = "inspection_thermal"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    py_modules=["thermal_alarm_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
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
            "thermal_alarm_node = thermal_alarm_node:main",
        ],
    },
)
