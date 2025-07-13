from setuptools import setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),  # we'll add this file soon
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashra',
    maintainer_email='ashra@todo.todo',
    description='Turtle control package with figure-eight driver and pen toggle service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'figure8_driver = turtle_control.figure8_driver:main',
            'trace_toggle = turtle_control.trace_toggle:main',
        ],
    },
       data_files=[
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ...
    ],
)



