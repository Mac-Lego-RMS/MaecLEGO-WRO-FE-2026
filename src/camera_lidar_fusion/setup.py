import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_lidar_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macjetson',
    maintainer_email='maeclegorms@gmail.com',
    description='Fusion of the horizontally mounted 360 degree fisheye camera with the 2D lidar: '
                'colour per lidar point (CSV) and calibration of the camera rotation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_pixel_mapper = camera_lidar_fusion.lidar_pixel_mapper:main',
            'rotation_calibration = camera_lidar_fusion.rotation_calibration:main',
            'camera_exposure_calib = camera_lidar_fusion.camera_exposure_calib:main',
            'csi_camera = camera_lidar_fusion.csi_camera:main',
        ],
    },
)
