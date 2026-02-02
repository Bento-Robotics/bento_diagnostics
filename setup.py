from setuptools import find_packages, setup

package_name = 'bento_diagnostics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
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
