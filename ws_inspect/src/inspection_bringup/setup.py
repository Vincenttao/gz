from glob import glob
from setuptools import setup

package_name = "inspection_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vincent",
    maintainer_email="vincent@example.com",
    description="Launch and configuration entry points for the inspection simulation stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},
)
