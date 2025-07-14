from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himel',
    maintainer_email='63048839+Ashfaqul-Awal-Himel@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "figure8_driver = turtle_control.figure8_driver:main",
            "trace_toggle_service = turtle_control.trace_toggle_service:main",
        ],
    },
)
