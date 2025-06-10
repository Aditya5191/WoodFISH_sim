from setuptools import setup
import os
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # Install model files
        ('share/' + package_name + '/model', glob('model/*')),

        ('share/' + package_name + '/meshes', glob('meshes/*')),

        # ('share/' + package_name + '/model/meshes', glob('model/meshes/*')),

        # Install parameter files (including bridge_params.yaml)
        ('share/' + package_name + '/parameters', glob('parameters/*')),
        
        
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Optionally, include other directories or files as needed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='aditya.jemshetty@gmail.com',
    description='ROS 2 package for robot description and simulation setup',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
