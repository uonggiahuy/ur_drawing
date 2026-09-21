from glob import glob

from setuptools import setup

package_name = "ur_drawing"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ur_drawing"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UR Drawing Student",
    maintainer_email="student@example.com",
    description="MoveIt Cartesian drawing demonstrations for a simulated UR arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "letter_drawer = ur_drawing.letter_drawer:main",
            "circle_drawer = ur_drawing.circle_drawer:main",
        ],
    },
)
