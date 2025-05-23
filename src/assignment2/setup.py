from setuptools import setup
from glob import glob

package_name = 'assignment2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raffaeleperri',
    maintainer_email='raffaeleperri70@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = assignment2.controller_node:main',
            'controller_node_wall = assignment2.controller_node_wall:main',
            'mapping_node = assignment2.mapping_node:main',
        ],
    },
)
