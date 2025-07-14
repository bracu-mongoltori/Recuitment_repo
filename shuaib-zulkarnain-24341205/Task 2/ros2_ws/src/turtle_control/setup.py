from setuptools import find_packages, setup
from setuptools import setup
import os
from glob import glob



package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/turtle_control']),
    ('share/turtle_control', ['package.xml']),
    (os.path.join('share', 'turtle_control', 'launch'), glob('launch/*.py')),],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hp',
    maintainer_email='hp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'figure8_driver = turtle_control.figure8_driver:main',
		'trace_toggle = turtle_control.trace_toggle:main',

        ],
    },
)
