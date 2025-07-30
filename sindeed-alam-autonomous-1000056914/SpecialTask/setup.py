from setuptools import setup

package_name = 'sindeed_alam_autonomous'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='Sindeed Ibti',
    maintainer_email='you@example.com',
    description='Path planning with A* and Dijkstra in ROS 2 using matplotlib',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = sindeed_alam_autonomous.planner_node:main'
        ],
    },
)
