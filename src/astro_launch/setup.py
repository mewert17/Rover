from setuptools import find_packages, setup

package_name = 'astro_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/astro_launch/urdf', ['urdf/rover.urdf']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/rover_launch.py']),
        ('share/' + package_name + '/params',
            ['params/nav2_params.yaml']),


    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='astrobotics',
    maintainer_email='astrobotics@todo.todo',
    description='Launch nodes (navigation, localization, opc, routine)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_command_node = astro_launch.motor_command_node:main',
            'beacon_localization = astro_launch.beacon_localization:main',
            'localization_testing = astro_launch.localization_testing:main',
        ],
    },
)
