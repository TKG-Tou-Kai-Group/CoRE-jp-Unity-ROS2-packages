from setuptools import find_packages, setup

package_name = 'core_sim_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Masaaki Hijikata',
    maintainer_email='hijimasa@gmail.com',
    description='Client nodes that drive Unity_ROS2_Robot_Simulator for the CoRE stage',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_world = core_sim_utils.load_world:main',
            'spawn_flying_discs = core_sim_utils.spawn_flying_discs:main',
        ],
    },
)
