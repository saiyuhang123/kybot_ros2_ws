from setuptools import setup
from glob import glob
import os

package_name = 'ros_arduino_python'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Patrick Goebel',
    maintainer_email='patrick@pirobot.org',
    description='ROS Arduino Python.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_node = ros_arduino_python.arduino_node:main',
            'save_image = ros_arduino_python.save_image:main',
        ],
    },
)
