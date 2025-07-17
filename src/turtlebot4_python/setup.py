from setuptools import find_packages, setup

package_name = 'turtlebot4_python'

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
    maintainer='ister',
    maintainer_email='ister@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_python_node = turtlebot4_python.turtlebot4_python_node:main',
            'robot_gui = turtlebot4_python.robot_gui:main',
            'move_robot = turtlebot4_python.move_robot:main'
        ],
    },
)
