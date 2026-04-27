import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'proyecto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'),
         glob('data/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devuser',
    maintainer_email='sinfonia@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation = proyecto.navigation_node:main',
            'spawner = proyecto.gazebo_spawner_node:main',
        ],
    },
)
