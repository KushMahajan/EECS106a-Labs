from setuptools import find_packages, setup

package_name = 'my_chatter'

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
    maintainer='ee106a-ahk',
    maintainer_email='ee106a-ahk@todo.todo',
    description='Publisher and subscriber for user input and timestamp',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_talker = my_chatter.my_talker:main',
            'my_listener = my_chatter.my_listener:main',
        ],
    },
)
