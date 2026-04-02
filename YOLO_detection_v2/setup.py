from setuptools import find_packages, setup

package_name = 'YOLO_detection_v2'

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
    maintainer='jin',
    maintainer_email='nagao.jin.r6@dc.tohoku.ac.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'subscribe_realsense = YOLO_detection_v2.subscribe_realsense:main',
            'publish_object_detection = YOLO_detection_v2.object_detection:main',
            'publish_direction = YOLO_detection_v2.make_direction:main',
            'publish_YOLOtf = YOLO_detection_v2.YOLOtf:main',
            'subscribe_detection = YOLO_detection_v2.subscribe_detection:main',
            'subscribe_direction = YOLO_detection_v2.subscribe_direction:main',
            'subscribe_depth = YOLO_detection_v2.subscribe_depth:main',
            
        ],
    },
)
