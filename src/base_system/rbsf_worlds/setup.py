from glob import glob
import os

from setuptools import setup

package_name = 'rbsf_worlds'

# One data_files entry per track directory, so tracks/<name>/* lands in share/tracks/<name>/.
tracks = [
    (os.path.join('share', package_name, 'tracks', os.path.basename(d)), glob(f'{d}/*'))
    for d in sorted(glob('tracks/*')) if os.path.isdir(d)
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/tracks', ['tracks/README.md']),
        *tracks,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santimontiel',
    maintainer_email='santimontieleu@gmail.com',
    description='Tracks and world tools shared by the RBSF Racing simulators and stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
