from setuptools import find_packages, setup
from glob import glob

package_name = 'igv_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.pth')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igv',
    maintainer_email='igv@todo.todo',
    description='IGV perception package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'segformer_lane_node = igv_perception.segformer_lane_node:main',
            'obstacle_detector_node = igv_perception.obstacle_detector_node:main',
	    'cloud_debug_filter_node = igv_perception.cloud_debug_filter_node:main',
        ],
    },
)
