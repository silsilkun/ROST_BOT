from setuptools import setup

package_name = "estimation_add"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Combine coordinates and type ids into a packed list.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "estimation_add_node = estimation_add.add_node:main",
        ],
    },
)
