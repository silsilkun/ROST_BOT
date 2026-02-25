from setuptools import find_packages, setup

package_name = 'control'

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
            'recycle = control.recycle:main',
            'renew = control.recycle_260220:main',
            'control_node = control.nodes.control_node:main',
            'recycle_new = control.recycle_new:main',
        ],
    },
)
