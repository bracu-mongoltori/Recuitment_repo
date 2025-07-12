from setuptools import setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='TurtleSim Figure8 and Trace Toggle',
    license='MIT',
    entry_points={
        'console_scripts': [
            'figure8_driver = turtle_control.figure8_driver:main',
            'trace_toggle = turtle_control.trace_toggle:main',
        ],
    },
)