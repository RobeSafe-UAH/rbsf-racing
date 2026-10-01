from glob import glob

from setuptools import setup

package_name = 'rbsf_dual_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santimontiel',
    maintainer_email='santimontieleu@gmail.com',
    description='DualSLAM mapping slot: SLAM Toolbox and Cartographer side by side, map saver.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_overlay = rbsf_dual_slam.map_overlay_node:main',
            'map_saver = rbsf_dual_slam.map_saver_node:main',
        ],
    },
)
