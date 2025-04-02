from setuptools import find_packages, setup

package_name = 'open_cv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'tf2_sensor_msgs',  # Add this
        'tf2_ros',  # Add this as well
        'numpy' ,
        'transforms3d' ,
    ],
    zip_safe=True,
    maintainer='mewert',
    maintainer_email='mewert@todo.todo',
    description='Package for object detection and occupancy grid generation from point cloud',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = open_cv_pkg.object_detection:main',
            'filtering = open_cv_pkg.filtering:main',
            'markers = open_cv_pkg.markers:main',
            'grid = open_cv_pkg.grid:main',
            'og = open_cv_pkg.og:main',
        ],
    },
)
