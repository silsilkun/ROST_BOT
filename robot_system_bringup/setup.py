from setuptools import setup

package_name = 'robot_system_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eipih',
    maintainer_email='2001sonickim@gmail.comm',
    description='System bringup launch package for Doosan + control + estimation.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
