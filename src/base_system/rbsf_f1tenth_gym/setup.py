from glob import glob

from setuptools import setup

package_name = 'rbsf_f1tenth_gym'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/rviz', glob('config/rviz/*.rviz')),
        ('share/' + package_name + '/config/foxglove', glob('config/foxglove/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santimontiel',
    maintainer_email='santimontieleu@gmail.com',
    description='Launches the f1tenth_gym simulator (f1tenth_gym_ros bridge) with RBSF naming.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_bridge = rbsf_f1tenth_gym.gym_bridge:main',
        ],
    },
)
