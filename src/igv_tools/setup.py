from setuptools import find_packages, setup

package_name = 'igv_tools'

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
    maintainer='igv',
    maintainer_email='igv@todo.todo',
    description='IGV tools package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_lane_publisher = igv_tools.fake_lane_publisher:main',
            'fake_obstacle_publisher = igv_tools.fake_obstacle_publisher:main',
        ],
    },
)
