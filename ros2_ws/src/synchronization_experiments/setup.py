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
        (os.path.join('share', package_name, 'launch'), glob('synchronization_experiments/launch/*')),
        (os.path.join('share', package_name, 'config'), glob('synchronization_experiments/config/*')),
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
        ],
    },
)
