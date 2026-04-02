from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uart_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karisora',
    maintainer_email='152140541+SoraKarimata@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = uart_control.publisher:main',
            'serial_publiasher = uart_control.serial_publiasher:main',
            'serial_subscriber = uart_control.serial_subscriber:main'
        ],
    },
)
