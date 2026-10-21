from glob import glob

from setuptools import setup

package_name = 'rbsf_path_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santimontiel',
    maintainer_email='santimontieleu@gmail.com',
    description='Pure Pursuit and MAP tracking of the planned trajectory.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_tracker = rbsf_path_tracking.path_tracker_node:main',
        ],
    },
)
