from glob import glob
from setuptools import find_packages, setup


package_name = "pick_place_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EveningStudy",
    maintainer_email="eveningstudy@example.com",
    description="RGB-D object localization for a pick-and-place workcell.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "object_localizer = pick_place_perception.object_localizer:main",
        ],
    },
)
