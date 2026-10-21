from glob import glob

from setuptools import setup

package_name = 'particle_filter'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santimontiel',
    maintainer_email='santimontieleu@gmail.com',
    description='Particle-filter localization (MCL) with RangeLibc, ported from '
                'f1tenth/particle_filter.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter = particle_filter.particle_filter_node:main',
        ],
    },
)
