from setuptools import find_packages, setup

package_name = 'communications'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opcua'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='lilmewert@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opcua_client = communications.opcua_client:main',
            'routine_node  = communications.routine_node:main', 
        ],
    },
)
