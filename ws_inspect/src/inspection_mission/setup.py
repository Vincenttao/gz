from setuptools import setup

package_name = "inspection_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/params", ["params/mission.yaml"]),
        (f"lib/{package_name}", ["scripts/mission_node"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vincent",
    maintainer_email="vincent@example.com",
    description="Mission control node for the inspection robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_node = inspection_mission.mission_node:main",
        ],
    },
)
