from setuptools import find_packages, setup

package_name = 'estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eipih',
    maintainer_email='2001sonickim@gmail.comm',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "main = estimation.nodes.main_node:main",
            "test = estimation.nodes.standalone_test_node:main",
            "add = estimation.utils.add_service:main",
            "judge = estimation.utils.judge_service:main",
            "estimation = estimation.nodes.estimation_node:main",
            "estimation_test = estimation.test_client:main",
        ],
    },
)
