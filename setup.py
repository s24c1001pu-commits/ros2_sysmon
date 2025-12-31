from setuptools import find_packages, setup

package_name = 'ros2_sysmon'

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
    maintainer='takuto',
    maintainer_email='s24C1001pu@s.chibakoudai.jp',
    description='System monitoring node for ROS 2 (CPU and memory usage)',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sysmon = ros2_sysmon.sysmon:main',
        ],
    },
)

