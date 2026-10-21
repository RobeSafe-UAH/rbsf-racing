from glob import glob

from setuptools import setup

package_name = 'rbsf_identity'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santimontiel',
    maintainer_email='santimontieleu@gmail.com',
    description='Identity implementations of the functional slots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_relay = rbsf_identity.map_relay:main',
            'odom_relay = rbsf_identity.odom_relay:main',
            'reference_line = rbsf_identity.reference_line:main',
        ],
    },
)
