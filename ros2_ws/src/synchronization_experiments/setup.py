from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'synchronization_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/thesis'), glob('synchronization_experiments/launch/thesis/*')),
        (os.path.join('share', package_name, 'launch/paper'), glob('synchronization_experiments/launch/paper/*')),
        (os.path.join('share', package_name, 'config/thesis'), glob('synchronization_experiments/config/thesis/*')),
        (os.path.join('share', package_name, 'config/paper'), glob('synchronization_experiments/config/paper/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davide',
    maintainer_email='davide.peron19@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'try = synchronization_experiments.nodes.try:main',
            'rrc = synchronization_experiments.nodes.rrc_sim:main',
            'data_collector = synchronization_experiments.nodes.data_collector:main',
        ],
    },
)
