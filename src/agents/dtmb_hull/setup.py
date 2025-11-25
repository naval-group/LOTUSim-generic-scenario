from setuptools import find_packages, setup

package_name = "dtmb_hull"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juliette",
    maintainer_email="jgrosset10@gmail.com",
    description="Commando agent for LOTUSim simulation",
    license="SPDX-License-Identifier: EPL-2.0",
    tests_require=["pytest"],
    entry_points={
        "lotusim.agents": ["dtmb_hull = dtmb_hull.dtmb_hull:DtmbHull"],
        "console_scripts": [],
    },
)
