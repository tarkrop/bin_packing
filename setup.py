from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bin_packing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takrop',
    maintainer_email='takrop@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'packing_node = bin_packing.packing_node:main'
        ],
    },
)
