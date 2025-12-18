from setuptools import setup

setup(
    name='turtlebot3_smart_navigator',
    version='0.1.1',
    description='A small ROS2 example package for TurtleBot3 smart navigation',
    packages=['turtlebot3_smart_navigator'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'input_node = turtlebot3_smart_navigator.input_node:main',
            'decision_node = turtlebot3_smart_navigator.decision_node:main',
            'navigator_node = turtlebot3_smart_navigator.navigator_node:main',
        ],
    },
)