from setuptools import setup

package_name = 'plum_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bambinojr',
    maintainer_email='nathan.busi00@gmail.com',
    description='Custom ROS2 nodes for Dusty robot, including coverage path planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner_node = plum_nodes.coverage_planner_node:main',
        ],
    },
)
