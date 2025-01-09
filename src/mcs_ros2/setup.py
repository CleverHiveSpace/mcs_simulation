from setuptools import find_packages, setup

package_name = 'mcs_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        # 'aiortc',
        'numpy',
        'opencv-python',
        'python3-socketio',
        'python3-websocket',
        'setuptools',
    ],
    zip_safe=True,
    maintainer='adrians',
    maintainer_email='adrianstahl.2@icloud.com',
    description='MCS ros2 package, which listens to webots-docker simulation and sends data to MCS using socket.io',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webots_node = mcs_ros2.webots_listener:main',
            'camera_listener = mcs_ros2.camera_listener:main',
        ],
    },
)
