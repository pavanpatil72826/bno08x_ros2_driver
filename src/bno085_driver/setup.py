from setuptools import setup
import os
from glob import glob

package_name = 'bno085_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavan Patil',
    maintainer_email='pavan.patil@gahanai.com',
    description='BNO085 (via ESP32 serial) IMU driver',
    license='Gahan AI Non-Commercial Software License 1.0',
    entry_points={
        'console_scripts': ['bno085_node = bno085_driver.bno085_node:main'],
    },
)
