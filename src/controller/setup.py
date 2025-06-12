from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

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
    maintainer='pranav',
    maintainer_email='pranavu8406@gmail.com',
    description='Thruster controller for AUV with individual and combined thruster control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_controller = controller.thruster_controller:main',
            'teleop_auv_keyboard = controller.teleop_auv_keyboard:main',  
        ],
    },
)
