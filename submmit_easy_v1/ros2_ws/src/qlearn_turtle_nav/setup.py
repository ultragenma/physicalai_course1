from setuptools import find_packages, setup

package_name = "qlearn_turtle_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ultragenma",
    maintainer_email="40537061+ultragenma@users.noreply.github.com",
    description="Q-learning goal navigation node for ROS 2 turtlesim.",
    license="MIT",
    entry_points={"console_scripts": ["navigator = qlearn_turtle_nav.navigator:main"]},
)
