from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros2_stamp_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/sim.launch.py']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew-Papesh',
    maintainer_email='55628349+Matthew-Papesh@users.noreply.github.com',
    description='A Stochastic Topological Adaptive Motion Planner (STAMP) that constructs a 2D navigation trajectory from c-space optimized splines. Splines are tuned by simulated annealing and stochastic exploration of the local c-space.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'quintic_test_node = ros2_stamp_nav.quintic_test:main', 
            'stamp_nav = ros2_stamp_nav.navigator:main',
            'stamp_spline = ros2_stamp_nav.spline:main',
        ],
    },

)
