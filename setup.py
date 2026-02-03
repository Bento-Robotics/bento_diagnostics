import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bento_diagnostics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'launch', 'pyyaml'],
    zip_safe=True,
    maintainer='Sam (snaens) Pelz',
    maintainer_email='snaens@snaens.net',
    description='Diagnostics for nodes used on bento robots',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bento_diagnostics_node = bento_diagnostics.diagnostics:main',
        ],
    },
)
