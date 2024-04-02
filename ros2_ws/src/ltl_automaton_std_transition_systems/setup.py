from setuptools import setup
from setuptools import find_packages
import os
from glob import glob


package_name = 'ltl_automaton_std_transition_systems'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('ltl_automaton_std_transition_systems/launch/*')),
        (os.path.join('share', package_name, 'config'), glob('ltl_automaton_std_transition_systems/config/*')),
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
            'ltl_2D_pose_monitor_node = ltl_automaton_std_transition_systems.nodes.region_2d_pose_monitor:main'
        ],
    },
)
