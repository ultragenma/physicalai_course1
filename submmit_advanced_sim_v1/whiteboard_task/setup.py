from setuptools import find_packages, setup

package_name = "whiteboard_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/whiteboard_task"]),
                (f"share/{package_name}", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["whiteboard_bridge = whiteboard_task.ros_bridge:main"]},
)
