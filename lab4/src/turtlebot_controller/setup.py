from setuptools import find_packages, setup

package_name = 'turtlebot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='rw435',
    description='Proportional controller for TurtleBot to follow an AR tag.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'turtlebot_control = turtlebot_controller.turtlebot_control:main',
        ],
    },
)