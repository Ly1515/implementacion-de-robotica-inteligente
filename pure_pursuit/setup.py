import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'config'),
        glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lymc',
    maintainer_email='lymc@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main',
            'odometria_node = pure_pursuit.odometria_node:main',
            'pura_persecucion_node = pure_pursuit.pura_persecucion_node:main'
        ],
    },
)
