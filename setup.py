from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'acid_techno'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arnas Biesevicius',
    maintainer_email='a.biesevicius@student.tue.nl',
    description='DTAS to map out ocean acidity levels',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Add all nodes here 
            'path_find_node = acid_techno.path_find_node:main',
            'read_acidity_node = acid_techno.read_acidity_node:main'
        ],
    },
)
