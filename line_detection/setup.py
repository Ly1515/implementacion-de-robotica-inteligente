import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'line_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lymc',
    maintainer_email='lymc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seguidor_linea_node = line_detection.seguidor_linea_node:main',
            'identificador_color_node = line_detection.identificador_color_node:main',
            'alto_nivel_node = line_detection.alto_nivel_node:main'
        ],
    },
)
