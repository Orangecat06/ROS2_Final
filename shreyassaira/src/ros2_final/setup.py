from setuptools import find_packages, setup

package_name = 'ros2_final'

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
    maintainer='controlslab',
    maintainer_email='sairaninja@gmail.com',
    description='ROS2 final',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'pub_sub = ros2_final.map_pub:main'
        ],
    },
)
