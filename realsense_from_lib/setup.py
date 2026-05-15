from setuptools import find_packages, setup
from glob import glob

package_name = 'realsense_from_lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ares',
    maintainer_email='152140541+SoraKarimata@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publish_realsense = realsense_from_lib.publish_realsense:main',
            'encode_image = realsense_from_lib.encode_image:main',
        ],
    },
)
