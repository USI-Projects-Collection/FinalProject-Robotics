from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paolodeidda',
    maintainer_email='paolodeidda.97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # formato: <script-name> = <python_module>:<function>
            'path_planner_node = path_planner.path_planner_node:main',
            'current_pos = path_planner.current_pos:main',
        ],
    },
)
