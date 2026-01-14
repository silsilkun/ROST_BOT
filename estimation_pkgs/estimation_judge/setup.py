from setuptools import setup

package_name = "estimation_judge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "python-dotenv"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Estimate object types from images using Gemini.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "estimation_judge_node = estimation_judge.judge_node:main",
        ],
    },
)
