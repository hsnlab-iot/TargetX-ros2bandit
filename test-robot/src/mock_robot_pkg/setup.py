from setuptools import setup, find_packages

package_name = 'mock_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='A mock robot ROS 2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = mock_robot_pkg.robot:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch',
            ['launch/robot_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ], 
)