from setuptools import find_packages, setup
from glob import glob

package_name = 'JIB'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a Project Code Demos',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boolean_publisher = JIB.boolean_publisher:main',
            'float_publisher   = JIB.float_publisher:main',
            'point_publisher   = JIB.point_publisher:main',
            'pose_publisher    = JIB.pose_publisher:main',
            'subscriber        = JIB.subscriber:main',
            'balldemo          = JIB.balldemo:main',
            'fencedemo         = JIB.fencedemo:main',
            'interactivedemo   = JIB.interactivedemo:main',
            'pirouette         = JIB.pirouette:main',
            'pirouetteandwave  = JIB.pirouetteandwave:main',
        ],
    },
)
