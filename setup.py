from setuptools import setup
import os
from glob import glob

package_name = 'surgical_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zara Shariff',
    maintainer_email='zshariff@example.com',
    description='Surgical instrument positioning simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'surgical_controller = surgical_sim.surgical_controller:main',
            'pose_publisher     = surgical_sim.pose_publisher:main',
            'metrics_logger     = surgical_sim.metrics_logger:main',
        ],
    },
)
