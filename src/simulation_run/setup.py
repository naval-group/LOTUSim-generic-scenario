import os
from glob import glob

from setuptools import setup

package_name = "simulation_run"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juliette",
    maintainer_email="jgrosset10@gmail.com",
    description="Description de ton package",
    license="SPDX-License-Identifier: EPL-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = simulation_run.main:main",
            "clean_simulation = simulation_run.simulation_runner:stop_simulation",
        ],
    },
)
